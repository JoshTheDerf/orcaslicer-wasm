// Texture-displacement colour palette: the GUI half of Orca 2.5's "Colors" option, ported from
// slic3r/GUI/Gizmos/GLGizmoTextureDisplacement.cpp (cached_palette, mix_targets, rank_mixes, pick_mixes,
// make_palette, palette_filaments, make_palette_quantizer) and slic3r/Utils/ColorSpaceConvert.cpp
// (DeltaE00). libslic3r's build_texture_displacement() decides *where* colour changes; this decides
// *which* filament (or two-filament mix) each colour prints in, exactly as the desktop gizmo does.
#pragma once

#include <libslic3r/Color.hpp>
#include <libslic3r/PrintConfig.hpp>
#include <libslic3r/TextureDisplacement.hpp>

#include <functional>
#include <string>
#include <vector>

namespace cs_tdc {

using Slic3r::Vec3f;
using PaletteEntry = Slic3r::PrintableColor;

struct MixTarget
{
    Vec3f lab    = Vec3f::Zero();
    float weight = 0.f;
};

Vec3f srgb_to_lab(const Vec3f &c);
float DeltaE00(float l1, float a1, float b1, float l2, float a2, float b2);

std::vector<MixTarget>    mix_targets(const std::vector<Slic3r::TextureDisplacementLayer> &layers);
std::vector<PaletteEntry> rank_mixes(const std::vector<Slic3r::ColorRGBA> &filaments, const std::vector<MixTarget> &targets, int limit);
std::vector<PaletteEntry> pick_mixes(const std::vector<PaletteEntry> &ranking, int count, int free_slots,
                                     const std::function<bool(const PaletteEntry &)> &reusable);
std::vector<PaletteEntry> make_palette(const std::vector<Slic3r::ColorRGBA> &filaments, const std::vector<PaletteEntry> &mixes);
std::vector<int>          palette_filaments(const std::vector<PaletteEntry> &palette, const std::vector<uint8_t> &triangle_color,
                                            const std::function<int(const PaletteEntry &)> &slot_for_mix);
Slic3r::ColorQuantizeFn   make_palette_quantizer(const std::vector<PaletteEntry> &palette);

// The project's filaments as the gizmo sees them (Plater::get_extruders_colors() + project_config):
// `config` carries filament_colour and the filament_is_mixed / filament_mixed_* keys.
struct ProjectFilaments
{
    const Slic3r::DynamicPrintConfig *config = nullptr;
    std::vector<Slic3r::ColorRGBA>    physical;    // filament_palette(): physical only, capped at ExtruderMax
    std::vector<std::string>          physical_hex;
    int                               total_slots = 0; // every slot, physical + mixed
};
ProjectFilaments project_filaments(const Slic3r::DynamicPrintConfig &config);

// GLGizmoTextureDisplacement::cached_palette() + color_settings_for(): empty settings when no layer is colouring.
struct PaletteResult
{
    Slic3r::TextureColorSettings settings;
    bool                         any_color = false; // any_layer_colors()
    int                          max_mixes = 1;     // the "Mixed colors" slider's upper bound
};
PaletteResult build_palette(const std::vector<Slic3r::TextureDisplacementLayer> &layers,
                            const Slic3r::TextureDisplacementOptions &options, const ProjectFilaments &filaments);

} // namespace cs_tdc
