// See cs_texture_palette.hpp. Code follows Orca 2.5.0's GLGizmoTextureDisplacement.cpp line for line
// (constants, sampling, greedy ranking, lookup cube); only the GUI state it read from (preset bundle,
// plater) is passed in instead.
#include "cs_texture_palette.hpp"

#include <libslic3r/FilamentMixer.hpp>
#include <libslic3r/PrintConfig.hpp>
#include <libslic3r/TriangleSelector.hpp>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>

namespace cs_tdc {

using namespace Slic3r;

namespace {
constexpr int    PALETTE_LUT_EDGE   = 24;
constexpr float  PREFER_PURE_DE     = 2.f;
constexpr size_t MIX_TARGET_SAMPLES = size_t(1) << 20;
constexpr size_t MIX_TARGET_BINS    = 256;
constexpr float  MIN_MIX_GAIN       = 0.05f;
constexpr int    EXTRUDER_MAX       = int(EnforcerBlockerType::ExtruderMax);
} // namespace

Vec3f srgb_to_lab(const Vec3f &c)
{
    const auto lin = [](float v) { return v > 0.04045f ? std::pow((v + 0.055f) / 1.055f, 2.4f) : v / 12.92f; };
    const float r = lin(c.x()), g = lin(c.y()), b = lin(c.z());
    const float x = (0.4124f * r + 0.3576f * g + 0.1805f * b) / 0.95047f;
    const float y = (0.2126f * r + 0.7152f * g + 0.0722f * b);
    const float z = (0.0193f * r + 0.1192f * g + 0.9505f * b) / 1.08883f;
    const auto f = [](float t) { return t > 0.008856f ? std::cbrt(t) : 7.787f * t + 16.f / 116.f; };
    const float fx = f(x), fy = f(y), fz = f(z);
    return Vec3f(116.f * fy - 16.f, 500.f * (fx - fy), 200.f * (fy - fz));
}

// slic3r/Utils/ColorSpaceConvert.cpp (that file also needs wxWidgets, so it is not compiled here).
float DeltaE00(float l1, float a1, float b1, float l2, float a2, float b2)
{
    auto rad2deg = [](float rad) { return 360.0 * rad / (2.0 * M_PI); };
    auto deg2rad = [](float deg) { return (2.0 * M_PI * deg) / 360.0; };

    float avgL = (l1 + l2) / 2.0;
    float c1 = std::sqrt(std::pow(a1, 2) + std::pow(b1, 2));
    float c2 = std::sqrt(std::pow(a2, 2) + std::pow(b2, 2));
    float avgC = (c1 + c2) / 2.0;
    float g = (1.0 - std::sqrt(std::pow(avgC, 7) / (std::pow(avgC, 7) + std::pow(25.0, 7)))) / 2.0;

    float a1p = a1 * (1.0 + g);
    float a2p = a2 * (1.0 + g);
    float c1p = std::sqrt(std::pow(a1p, 2) + std::pow(b1, 2));
    float c2p = std::sqrt(std::pow(a2p, 2) + std::pow(b2, 2));
    float avgCp = (c1p + c2p) / 2.0;

    float h1p = rad2deg(std::atan2(b1, a1p));
    if (h1p < 0.0) h1p = h1p + 360.0;
    float h2p = rad2deg(std::atan2(b2, a2p));
    if (h2p < 0.0) h2p = h2p + 360;

    float avghp = std::abs(h1p - h2p) > 180.0 ? (h1p + h2p + 360.0) / 2.0 : (h1p + h2p) / 2.0;
    float t = 1.0 - 0.17 * std::cos(deg2rad(avghp - 30.0)) + 0.24 * std::cos(deg2rad(2.0 * avghp)) +
              0.32 * std::cos(deg2rad(3.0 * avghp + 6.0)) - 0.2 * std::cos(deg2rad(4.0 * avghp - 63.0));

    float deltahp = h2p - h1p;
    if (std::abs(deltahp) > 180.0) {
        if (h2p <= h1p) deltahp += 360.0;
        else deltahp -= 360.0;
    }
    float deltalp = l2 - l1;
    float deltacp = c2p - c1p;
    deltahp = 2.0 * std::sqrt(c1p * c2p) * std::sin(deg2rad(deltahp) / 2.0);

    float sl = 1.0 + ((0.015 * std::pow(avgL - 50.0, 2)) / std::sqrt(20.0 + std::pow(avgL - 50.0, 2)));
    float sc = 1.0 + 0.045 * avgCp;
    float sh = 1.0 + 0.015 * avgCp * t;
    float deltaro = 30.0 * std::exp(-(std::pow((avghp - 275.0) / 25.0, 2)));
    float rc = 2.0 * std::sqrt(std::pow(avgCp, 7) / (std::pow(avgCp, 7) + std::pow(25.0, 7)));
    float rt = -rc * std::sin(2.0 * deg2rad(deltaro));
    float kl = 1, kc = 1, kh = 1;
    return std::sqrt(std::pow(deltalp / (kl * sl), 2) + std::pow(deltacp / (kc * sc), 2) + std::pow(deltahp / (kh * sh), 2) +
                     rt * (deltacp / (kc * sc)) * (deltahp / (kh * sh)));
}

std::vector<MixTarget> mix_targets(const std::vector<TextureDisplacementLayer> &layers)
{
    std::vector<std::vector<MixTarget>> per_layer;
    for (const TextureDisplacementLayer &layer : layers) {
        if (!layer.color_enabled || layer.empty() || analyze_texture_detail(layer).flat_colors)
            continue;
        TextureDisplacementLayer raw = layer;
        raw.smoothing                = 0.f;
        const DecodedHeightTexture tex = decode_height_texture(raw);
        if (!tex.has_color())
            continue;
        struct Bin { double r = 0., g = 0., b = 0.; uint32_t n = 0; };
        std::vector<Bin> bins(size_t(16 * 16 * 16));
        const size_t npx     = size_t(tex.width) * size_t(tex.height);
        const size_t stride  = std::max<size_t>(1, npx / MIX_TARGET_SAMPLES);
        uint64_t     state   = 0x9E3779B97F4A7C15ull;
        size_t       sampled = 0;
        for (size_t start = 0; start < npx; start += stride) {
            state          = state * 6364136223846793005ull + 1442695040888963407ull;
            const size_t i = start + size_t((uint64_t(uint32_t(state >> 32)) * uint64_t(stride)) >> 32);
            if (i >= npx)
                break;
            const uint8_t *px  = &tex.rgb[i * 3];
            Bin           &bin = bins[size_t(px[0] >> 4) * 256 + size_t(px[1] >> 4) * 16 + size_t(px[2] >> 4)];
            bin.r += px[0];
            bin.g += px[1];
            bin.b += px[2];
            ++bin.n;
            ++sampled;
        }
        std::vector<MixTarget> targets;
        for (const Bin &bin : bins)
            if (bin.n > 0) {
                const double inv = 1. / (255. * double(bin.n));
                targets.push_back({ srgb_to_lab(Vec3f(float(bin.r * inv), float(bin.g * inv), float(bin.b * inv))),
                                    float(double(bin.n) / double(sampled)) });
            }
        per_layer.push_back(std::move(targets));
    }
    std::vector<MixTarget> out;
    for (std::vector<MixTarget> &targets : per_layer)
        for (MixTarget &t : targets) {
            t.weight /= float(per_layer.size());
            out.push_back(t);
        }
    if (out.size() > MIX_TARGET_BINS) {
        std::partial_sort(out.begin(), out.begin() + MIX_TARGET_BINS, out.end(),
                          [](const MixTarget &l, const MixTarget &r) { return l.weight > r.weight; });
        out.resize(MIX_TARGET_BINS);
    }
    return out;
}

std::vector<PaletteEntry> rank_mixes(const std::vector<ColorRGBA> &filaments, const std::vector<MixTarget> &targets, int limit)
{
    const int n = int(filaments.size());
    if (n < 2 || targets.empty() || limit <= 0)
        return {};
    std::vector<std::string> hex(filaments.size());
    for (int i = 0; i < n; ++i)
        hex[size_t(i)] = encode_color(filaments[size_t(i)]);
    std::vector<PaletteEntry> candidates;
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j)
            for (int den = 2; den <= 6; ++den)
                for (int num = 1; num < den; ++num) {
                    if (std::gcd(num, den) != 1)
                        continue;
                    PaletteEntry e{ Vec3f::Zero(), i, j, num, den };
                    ColorRGB     blended;
                    if (!decode_color(blend_color_multi({ hex[size_t(i)], hex[size_t(j)] }, { e.a_percent(), 100 - e.a_percent() }), blended))
                        continue;
                    e.rgb = Vec3f(blended.r(), blended.g(), blended.b());
                    candidates.push_back(e);
                }
    const size_t       nt = targets.size(), nc = candidates.size();
    std::vector<Vec3f> filament_lab(size_t(n), Vec3f::Zero());
    for (int i = 0; i < n; ++i)
        filament_lab[size_t(i)] = srgb_to_lab(Vec3f(filaments[size_t(i)].r(), filaments[size_t(i)].g(), filaments[size_t(i)].b()));
    const auto de = [](const Vec3f &l, const Vec3f &r) { return DeltaE00(l.x(), l.y(), l.z(), r.x(), r.y(), r.z()); };
    std::vector<float> pure_d(nt, std::numeric_limits<float>::max());
    for (size_t t = 0; t < nt; ++t)
        for (const Vec3f &lab : filament_lab)
            pure_d[t] = std::min(pure_d[t], de(targets[t].lab, lab));
    std::vector<float> dist(nc * nt);
    tbb::parallel_for(tbb::blocked_range<size_t>(0, nc), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t c = range.begin(); c < range.end(); ++c) {
            const Vec3f lab = srgb_to_lab(candidates[c].rgb);
            for (size_t t = 0; t < nt; ++t)
                dist[c * nt + t] = de(targets[t].lab, lab);
        }
    });
    std::vector<float>        current = pure_d;
    std::vector<char>         taken(nc, 0);
    std::vector<PaletteEntry> out;
    const auto counts = [&](size_t c, size_t t) {
        const float d = dist[c * nt + t];
        return d < current[t] && d <= pure_d[t] - PREFER_PURE_DE;
    };
    while (int(out.size()) < limit) {
        size_t best      = nc;
        double best_gain = 0.;
        for (size_t c = 0; c < nc; ++c) {
            if (taken[c])
                continue;
            double gain = 0.;
            for (size_t t = 0; t < nt; ++t)
                if (counts(c, t))
                    gain += double(targets[t].weight) * double(current[t] - dist[c * nt + t]);
            if (gain > best_gain) {
                best_gain = gain;
                best      = c;
            }
        }
        if (best == nc || best_gain < double(MIN_MIX_GAIN))
            break;
        taken[best] = 1;
        out.push_back(candidates[best]);
        for (size_t t = 0; t < nt; ++t)
            if (counts(best, t))
                current[t] = dist[best * nt + t];
    }
    return out;
}

std::vector<PaletteEntry> pick_mixes(const std::vector<PaletteEntry> &ranking, int count, int free_slots,
                                     const std::function<bool(const PaletteEntry &)> &reusable)
{
    std::vector<PaletteEntry> out;
    for (const PaletteEntry &e : ranking) {
        if (int(out.size()) >= count)
            break;
        if (reusable && reusable(e)) {
            out.push_back(e);
        } else if (free_slots > 0) {
            out.push_back(e);
            --free_slots;
        }
    }
    return out;
}

std::vector<PaletteEntry> make_palette(const std::vector<ColorRGBA> &filaments, const std::vector<PaletteEntry> &mixes)
{
    std::vector<PaletteEntry> out;
    out.reserve(filaments.size() + mixes.size());
    for (int i = 0; i < int(filaments.size()); ++i)
        out.push_back({ Vec3f(filaments[size_t(i)].r(), filaments[size_t(i)].g(), filaments[size_t(i)].b()), i, i, 1, 1 });
    out.insert(out.end(), mixes.begin(), mixes.end());
    return out;
}

std::vector<int> palette_filaments(const std::vector<PaletteEntry> &palette, const std::vector<uint8_t> &triangle_color,
                                   const std::function<int(const PaletteEntry &)> &slot_for_mix)
{
    std::vector<char> used(palette.size(), 0);
    for (const uint8_t v : triangle_color)
        if (v > 0 && size_t(v) <= palette.size())
            used[size_t(v) - 1] = 1;
    std::vector<int> out(palette.size(), -1);
    for (size_t i = 0; i < palette.size(); ++i) {
        const PaletteEntry &e = palette[i];
        if (!e.is_mix()) {
            out[i] = e.a;
        } else if (used[i]) {
            const int slot = slot_for_mix ? slot_for_mix(e) : -1;
            out[i] = slot >= 0 ? slot : (e.num * 2 >= e.den ? e.a : e.b);
        }
    }
    return out;
}

ColorQuantizeFn make_palette_quantizer(const std::vector<PaletteEntry> &palette)
{
    if (palette.empty())
        return nullptr;
    struct Lab { float l, a, b; };
    std::vector<Lab> palette_lab(palette.size());
    for (size_t i = 0; i < palette.size(); ++i) {
        const Vec3f lab = srgb_to_lab(palette[i].rgb);
        palette_lab[i]  = { lab.x(), lab.y(), lab.z() };
    }
    constexpr int E   = PALETTE_LUT_EDGE;
    auto          lut = std::make_shared<std::vector<uint8_t>>(size_t(E) * E * E, 0);
    tbb::parallel_for(tbb::blocked_range<int>(0, E), [&](const tbb::blocked_range<int> &range) {
        for (int r = range.begin(); r < range.end(); ++r)
            for (int g = 0; g < E; ++g)
                for (int b = 0; b < E; ++b) {
                    const Vec3f lab0 = srgb_to_lab(Vec3f((r + 0.5f) / E, (g + 0.5f) / E, (b + 0.5f) / E));
                    const float l0 = lab0.x(), a0 = lab0.y(), b0 = lab0.z();
                    int   best = 0, best_pure = -1;
                    float best_d = std::numeric_limits<float>::max(), best_pure_d = best_d;
                    for (size_t i = 0; i < palette_lab.size(); ++i) {
                        const float d = DeltaE00(l0, a0, b0, palette_lab[i].l, palette_lab[i].a, palette_lab[i].b);
                        if (d < best_d) {
                            best_d = d;
                            best   = int(i);
                        }
                        if (!palette[i].is_mix() && d < best_pure_d) {
                            best_pure_d = d;
                            best_pure   = int(i);
                        }
                    }
                    if (best_pure >= 0 && palette[size_t(best)].is_mix() && best_pure_d - best_d < PREFER_PURE_DE)
                        best = best_pure;
                    (*lut)[(size_t(r) * E + size_t(g)) * E + size_t(b)] = uint8_t(best);
                }
    });
    return [lut](const Vec3f &rgb) -> int {
        constexpr int E = PALETTE_LUT_EDGE;
        const int r = std::clamp(int(rgb.x() * E), 0, E - 1);
        const int g = std::clamp(int(rgb.y() * E), 0, E - 1);
        const int b = std::clamp(int(rgb.z() * E), 0, E - 1);
        return int((*lut)[(size_t(r) * E + size_t(g)) * E + size_t(b)]);
    };
}

ProjectFilaments project_filaments(const DynamicPrintConfig &config)
{
    ProjectFilaments out;
    out.config             = &config;
    const auto *colours    = config.option<ConfigOptionStrings>("filament_colour");
    const auto *is_mixed   = config.option<ConfigOptionBools>("filament_is_mixed");
    const size_t total     = colours ? colours->values.size() : 0;
    out.total_slots        = int(total);
    for (size_t i = 0; i < total; ++i) {
        if (is_mixed != nullptr && i < is_mixed->values.size() && is_mixed->values[i])
            continue;
        ColorRGBA c;
        if (!decode_color(colours->values[i], c))
            c = ColorRGBA(0.5f, 0.5f, 0.5f, 1.f);
        out.physical.push_back(c);
        out.physical_hex.push_back(encode_color(ColorRGB(c.r(), c.g(), c.b())));
    }
    if (out.physical.size() > size_t(EXTRUDER_MAX)) {
        out.physical.resize(size_t(EXTRUDER_MAX));
        out.physical_hex.resize(size_t(EXTRUDER_MAX));
    }
    return out;
}

PaletteResult build_palette(const std::vector<TextureDisplacementLayer> &layers, const TextureDisplacementOptions &options,
                            const ProjectFilaments &filaments)
{
    PaletteResult out;
    out.max_mixes = std::max(1, EXTRUDER_MAX - int(filaments.physical.size()));
    for (const TextureDisplacementLayer &layer : layers)
        if (layer.color_enabled && !layer.empty() && height_texture_has_color(layer))
            out.any_color = true;
    if (!out.any_color || filaments.physical.empty())
        return out;
    const bool mixing    = options.color_mix_enabled;
    const int  mix_count = std::max(0, options.color_mix_count);
    std::vector<PaletteEntry> mixes;
    if (mixing) {
        const std::vector<PaletteEntry> ranking = rank_mixes(filaments.physical, mix_targets(layers), EXTRUDER_MAX - int(filaments.physical.size()));
        const int free_slots = std::max(0, EXTRUDER_MAX - filaments.total_slots);
        const DynamicPrintConfig *cfg = filaments.config;
        mixes = pick_mixes(ranking, mix_count, free_slots, [cfg](const PaletteEntry &e) {
            return cfg != nullptr &&
                   find_fixed_mixed_filament(*cfg, { unsigned(e.a + 1), unsigned(e.b + 1) }, { e.a_percent(), 100 - e.a_percent() }) >= 0;
        });
    }
    out.settings.palette          = make_palette(filaments.physical, mixes);
    out.settings.palette_pure     = make_palette(filaments.physical, {});
    out.settings.despeckle_passes = options.color_despeckle;
    return out;
}

} // namespace cs_tdc
