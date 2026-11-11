// Texture import module (texture.mjs / texture.wasm): OrcaSlicer 2.5's import of textured and coloured
// models, as a separate, lazily loaded module next to the slicer engine (wasm/texture.cmake). It links
// what the slicer leaves out: Assimp 5.4.3 (glTF/GLB, FBX; Format/AssimpImport.cpp) and OpenCV 4.14.0
// core/imgproc/imgcodecs (TexturePainting.cpp, TextureToColor/*).
//
// One stateless-ABI dispatcher, the cs_tool conventions:
//   int tex_call(req_json, req_len, blob, blob_len, &out_json, &out_len, &out_blob, &out_blob_len)
//   request  { op, args }  + a blob (input files / data, addressed by {offset, length} in args)
//   response { ok, error, result, meshes:[{vertexOffset, vertexCount, indexOffset, triangleCount}],
//              blobs:{ name:{offset, length} } } + one output blob (meshes first: float32 xyz then
//              uint32 triangles, then the named blobs, every part 4-aligned)
// Model sessions (a loaded TexturedMesh and its last painted result) live across calls in one
// instance, keyed by `handle`; `close` frees one.
//
// Ops (desktop source in brackets; full request/response in cubby-slicer docs/ENGINE-CONTRACT.md):
//   load        file bytes (+ companions) -> object mesh, textured mesh, materials, textures, colours
//               [Model::read_from_file textured branches, TextureImportDialog ctor]
//   analyze     colour count / smoothing / repair choice -> painted mesh + clusters
//               [TextureImportDialog::start_computation: texture_to_painting / face_colors_to_painting]
//   match       clusters + project filaments -> dialog filament entries + cluster->filament matches,
//               auto-merge, CMYW / RYBW auto-mix [do_auto_match, compact_used_virtual_filaments,
//               apply_auto_standard_mix]
//   add_mixed   one row -> a mixed filament from a colour-decompose recipe [add_decomposed_mixed_filament]
//   apply       entries + matches -> project filaments to add, final indices, colour paint strings,
//               painted mesh, base extruder [on_ok_clicked, Plater::apply_textured_mesh_import_result,
//               apply_painted_mesh_to_volume]
//   close
#include "common/cs_common.hpp"

#include <libslic3r/libslic3r.h>
#include <libslic3r/ColorDecomposeRecipe.hpp>
#include <libslic3r/FilamentMixer.hpp>
#include <libslic3r/Format/AssimpImport.hpp>
#include <libslic3r/Format/OBJ.hpp>
#include <libslic3r/MeshRepair.hpp>
#include <libslic3r/Model.hpp>
#include <libslic3r/TextureDisplacement.hpp>
#include <libslic3r/TexturePainting.hpp>
#include <libslic3r/TriangleMesh.hpp>
#include <libslic3r/TriangleSelector.hpp>
#include <libslic3r/Utils.hpp>

#include <assimp/version.h>
#include <opencv2/core/version.hpp>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <sys/stat.h>
#include <vector>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#else
#define EMSCRIPTEN_KEEPALIVE
#endif

using namespace Slic3r;
using cs::json;

namespace {

constexpr int    MAX_FILAMENTS = int(EnforcerBlockerType::ExtruderMax); // TextureImportDialog::max_filament_count()
constexpr size_t MAX_INPUT_FILES = 256;
constexpr const char* DEFAULT_VIRTUAL_FILAMENT_BASIC_TYPE = "PLA Basic";
constexpr const char* DEFAULT_VIRTUAL_FILAMENT_SHORT_TYPE = "PLA";
constexpr const char* DEFAULT_VIRTUAL_FILAMENT_NAME       = "Bambu PLA Basic";

void ensure_runtime_dirs()
{
    static std::once_flag once;
    std::call_once(once, [] {
        ::mkdir("/tmp", 0777);
        ::mkdir("/data", 0777);
        ::mkdir("/import", 0777);
        set_resources_dir("/resources"); // resources/filament_mixing (CMYW / RYBW recipe tables)
        set_data_dir("/data");
        set_temporary_dir("/tmp");
        set_logging_level(1);
    });
}

struct ToolOut {
    json result = json::object();
    std::vector<indexed_triangle_set> meshes;
    std::vector<std::pair<std::string, std::string>> blobs;
};

template<class T> std::string bytes_of(const std::vector<T>& v)
{
    return std::string(reinterpret_cast<const char*>(v.data()), v.size() * sizeof(T));
}

indexed_triangle_set its_of(const std::vector<std::array<float, 3>>& v, const std::vector<std::array<int, 3>>& f)
{
    indexed_triangle_set its;
    its.vertices.reserve(v.size());
    for (const auto& p : v) its.vertices.emplace_back(p[0], p[1], p[2]);
    its.indices.reserve(f.size());
    for (const auto& t : f) its.indices.emplace_back(t[0], t[1], t[2]);
    return its;
}

// ---- colours (TextureImportDialog helpers) ---------------------------------------------------
std::string rgb_to_hex(const std::array<std::size_t, 3>& c)
{
    char b[8];
    std::snprintf(b, sizeof b, "#%02X%02X%02X", unsigned(c[0] & 0xFF), unsigned(c[1] & 0xFF), unsigned(c[2] & 0xFF));
    return b;
}

// texture_normalize_color_hex + decompose_normalize_color_hex
std::string normalize_hex(std::string hex)
{
    if (hex.empty()) return "#808080";
    if (hex.front() != '#') hex = "#" + hex;
    if (hex.size() >= 7) hex = hex.substr(0, 7);
    std::transform(hex.begin(), hex.end(), hex.begin(), [](unsigned char c) { return char(std::toupper(c)); });
    return hex;
}

std::array<float, 4> parse_color_string(const std::string& hex)
{
    std::array<float, 4> c = {1.f, 1.f, 1.f, 1.f};
    if (hex.size() >= 7 && hex[0] == '#') {
        unsigned long val = std::strtoul(hex.c_str() + 1, nullptr, 16);
        c[0] = ((val >> 16) & 0xFF) / 255.f;
        c[1] = ((val >> 8) & 0xFF) / 255.f;
        c[2] = ((val) & 0xFF) / 255.f;
    }
    return c;
}

std::array<std::size_t, 3> parse_rgb(const std::string& hex)
{
    const auto c = parse_color_string(normalize_hex(hex));
    return {std::size_t(std::lround(c[0] * 255.f)), std::size_t(std::lround(c[1] * 255.f)), std::size_t(std::lround(c[2] * 255.f))};
}

// ---- model sessions ----------------------------------------------------------------------------
struct Session {
    std::string           format;
    TexturedMesh          textured;
    indexed_triangle_set  object_its;   // the geometry the plater adds as the object
    std::vector<Vec2f>    object_uv;    // OBJ: the model's own UV channel (3 per object triangle), V flipped
    PaintedMesh           painted;
    bool                  has_painted = false;
};
std::map<int, std::unique_ptr<Session>> g_sessions;
int g_next_session = 1;

Session& session(const json& args)
{
    auto it = g_sessions.find(args.value("handle", 0));
    if (it == g_sessions.end()) throw cs::JobError("unknown model handle");
    return *it->second;
}

// Plater.cpp has_importable_texture
bool has_importable_texture(const TexturedMesh& tm)
{
    if (tm.vertices.empty() || tm.indices.empty()) return false;
    if (!tm.precomputed_face_colors.empty()) return true;
    return std::any_of(tm.textures.begin(), tm.textures.end(), [](const TextureImage& t) { return !t.data.empty(); });
}

// A relative file name inside the import directory: no absolute path, no "..", backslashes as '/'.
std::string safe_relative(std::string name)
{
    std::replace(name.begin(), name.end(), '\\', '/');
    while (!name.empty() && name.front() == '/') name.erase(name.begin());
    if (name.empty() || name.size() > 1024) throw cs::JobError("load: bad file name");
    for (const std::string& part : [&] { std::vector<std::string> p; size_t s = 0; for (size_t i = 0; i <= name.size(); ++i) if (i == name.size() || name[i] == '/') { p.push_back(name.substr(s, i - s)); s = i + 1; } return p; }())
        if (part.empty() || part == "." || part == "..") throw cs::JobError("load: bad file name '" + name + "'");
    return name;
}

void remove_tree(const std::string& dir)
{
    boost::system::error_code ec;
    boost::filesystem::remove_all(dir, ec);
}

// load: args { files: [{ name, offset, length }] } — files[0] is the model (.glb .gltf .fbx .obj), the rest are
// what it references (.bin buffers, .mtl, texture images), written beside it under their (relative) names.
void op_load(const json& args, const uint8_t* blob, int blob_len, ToolOut& out)
{
    const json files = args.value("files", json::array());
    if (!files.is_array() || files.empty() || files.size() > MAX_INPUT_FILES) throw cs::JobError("load: files must list 1..256 files");
    const int handle = g_next_session++;
    const std::string dir = "/import/" + std::to_string(handle);
    remove_tree(dir);
    std::string model_path;
    for (const json& f : files) {
        if (!f.is_object()) throw cs::JobError("load: file entry must be an object");
        const std::string rel = safe_relative(f.value("name", std::string()));
        const uint64_t off = cs::get_u64(f, "offset"), len = cs::get_u64(f, "length");
        if (!cs::checked_range(off, len, 1, uint64_t(blob_len < 0 ? 0 : blob_len))) throw cs::JobError("load: file '" + rel + "' out of bounds");
        const boost::filesystem::path p = boost::filesystem::path(dir) / rel;
        boost::system::error_code ec;
        boost::filesystem::create_directories(p.parent_path(), ec);
        std::ofstream os(p.string(), std::ios::binary);
        if (len) os.write(reinterpret_cast<const char*>(blob + off), std::streamsize(len));
        if (!os) { remove_tree(dir); throw cs::JobError("load: could not stage '" + rel + "'"); }
        if (model_path.empty()) model_path = p.string();
    }

    auto s = std::make_unique<Session>();
    TexturedMesh& tm = s->textured;
    std::string message;
    bool ok = false;
    const auto ends = [&](const char* e) { return boost::algorithm::iends_with(model_path, e); };
    try {
        if (ends(".glb") || ends(".gltf") || ends(".fbx")) {
            // Model::read_from_file: the textured import path; the geometry becomes a plain object
            // (add_textured_mesh_to_model), the texture goes to the import dialog.
            s->format = ends(".fbx") ? "fbx" : ends(".gltf") ? "gltf" : "glb";
            ok = load_assimp_textured_model(model_path, tm, &message);
            if (!ok) {
                if (message.empty()) message = "The file format is incompatible and cannot be parsed.";
            } else {
                indexed_triangle_set its = its_of(tm.vertices, tm.indices);
                its_merge_vertices(its);
                its_remove_degenerate_faces(its);
                its_compactify_vertices(its);
                s->object_its = std::move(its);
            }
        } else if (ends(".obj")) {
            s->format = "obj";
            Model model;
            ObjInfo obj_info;
            ObjParser::MtlData mtl_data;
            ok = load_obj(model_path.c_str(), &model, obj_info, message, nullptr, &mtl_data);
            if (ok && (model.objects.empty() || model.objects.back()->volumes.empty())) { ok = false; message = "The OBJ file has no geometry."; }
            if (ok) {
                const indexed_triangle_set& its = model.objects.back()->volumes[0]->mesh().its;
                s->object_its = its;
                if (!obj_info.uvs.empty() && obj_info.uvs.size() == its.indices.size()) {
                    s->object_uv.resize(its.indices.size() * 3);
                    for (size_t fi = 0; fi < obj_info.uvs.size(); ++fi)
                        for (int k = 0; k < 3; ++k)
                            s->object_uv[fi * 3 + size_t(k)] = Vec2f(obj_info.uvs[fi][size_t(k)].x(), 1.f - obj_info.uvs[fi][size_t(k)].y());
                }
                auto build_geometry = [&]() {
                    tm.vertices.resize(its.vertices.size());
                    for (size_t i = 0; i < its.vertices.size(); ++i) tm.vertices[i] = {its.vertices[i].x(), its.vertices[i].y(), its.vertices[i].z()};
                    tm.indices.resize(its.indices.size());
                    for (size_t i = 0; i < its.indices.size(); ++i) tm.indices[i] = {its.indices[i][0], its.indices[i][1], its.indices[i][2]};
                };
                if (obj_info.has_uv_png && !obj_info.uvs.empty()) {
                    TexturedMesh t;
                    if (obj_to_textured_mesh(obj_info, its, mtl_data, boost::filesystem::path(model_path).parent_path().string(), t))
                        tm = std::move(t);
                } else if (!obj_info.vertex_colors.empty()) {
                    build_geometry();
                    tm.precomputed_face_colors.resize(its.indices.size());
                    for (size_t i = 0; i < its.indices.size(); ++i) {
                        const auto& f = its.indices[i];
                        auto avg = [&](int ch) -> std::size_t {
                            float v = (obj_info.vertex_colors[f[0]][ch] + obj_info.vertex_colors[f[1]][ch] + obj_info.vertex_colors[f[2]][ch]) / 3.0f * 255.0f;
                            return (std::size_t) std::clamp(v, 0.0f, 255.0f);
                        };
                        tm.precomputed_face_colors[i] = {avg(0), avg(1), avg(2)};
                    }
                    tm.precomputed_vertex_colors = obj_info.vertex_colors;
                } else if (!obj_info.face_colors.empty() && !obj_info.has_uv_png) {
                    build_geometry();
                    const size_t nf = tm.indices.size();
                    tm.precomputed_face_colors.resize(nf);
                    for (size_t i = 0; i < nf; ++i) {
                        if (i < obj_info.face_colors.size()) {
                            const auto& c = obj_info.face_colors[i];
                            tm.precomputed_face_colors[i] = {(std::size_t) std::clamp(c[0] * 255.0f, 0.0f, 255.0f),
                                                             (std::size_t) std::clamp(c[1] * 255.0f, 0.0f, 255.0f),
                                                             (std::size_t) std::clamp(c[2] * 255.0f, 0.0f, 255.0f)};
                        } else {
                            tm.precomputed_face_colors[i] = {128, 128, 128};
                        }
                    }
                }
            }
        } else {
            message = "load: unsupported model file (expected .glb, .gltf, .fbx or .obj)";
        }
    } catch (const std::exception& e) {
        ok = false;
        message = e.what();
    }
    remove_tree(dir); // everything the loaders need is in memory now (textures as encoded bytes)
    if (!ok) throw cs::JobError(message.empty() ? "The file could not be loaded." : message);
    if (s->object_its.indices.empty()) throw cs::JobError("The model has no triangles.");

    const bool importable = has_importable_texture(tm);
    json materials = json::array();
    for (size_t m = 0; m < tm.material_colors.size(); ++m) {
        const auto& c = tm.material_colors[m];
        materials.push_back({{"rgba", {c[0], c[1], c[2], c[3]}},
                             {"texture", m < tm.material_texture_map.size() ? tm.material_texture_map[m] : -1}});
    }
    out.meshes.push_back(s->object_its);
    out.result = {{"handle", handle}, {"format", s->format}, {"importable", importable},
                  {"source", !importable ? json(nullptr) : json(tm.precomputed_face_colors.empty() ? "texture" : "faceColors")},
                  {"objectTriangles", s->object_its.indices.size()}, {"materials", materials}, {"hasModelUv", !s->object_uv.empty()}};
    if (!s->object_uv.empty()) out.blobs.emplace_back("modelUv", bytes_of(s->object_uv));
    if (importable) {
        // The dialog's preview data (TextureImportDialog ctor): the textured mesh, its original per-face
        // colours (precomputed, or sampled from the textures / material colours), and for a textured model
        // the decoded images with per-face UVs and texture ids, so the host can draw it textured.
        out.meshes.push_back(its_of(tm.vertices, tm.indices));
        const size_t nf = tm.indices.size();
        std::vector<std::array<std::size_t, 3>> orig;
        if (!tm.precomputed_face_colors.empty()) orig = tm.precomputed_face_colors;
        else sample_original_face_colors(tm, orig);
        std::string rgb(nf * 3, '\0');
        for (size_t f = 0; f < nf && f < orig.size(); ++f)
            for (int k = 0; k < 3; ++k) rgb[f * 3 + size_t(k)] = char(std::min<std::size_t>(orig[f][size_t(k)], 255));
        out.blobs.emplace_back("originalColors", std::move(rgb));
        json textures = json::array();
        if (tm.precomputed_face_colors.empty() && !tm.textures.empty()) {
            const bool with_pixels = args.value("texturePixels", true);
            for (size_t ti = 0; ti < tm.textures.size(); ++ti) {
                std::vector<unsigned char> bgr;
                int w = 0, h = 0;
                json tj = {{"width", 0}, {"height", 0}, {"encodedBytes", tm.textures[ti].data.size()}};
                if (decode_texture_to_pixels(tm.textures[ti], bgr, w, h) && !bgr.empty()) {
                    tj["width"] = w;
                    tj["height"] = h;
                    if (with_pixels) {
                        std::string rgba(size_t(w) * size_t(h) * 4, '\0');
                        for (size_t p = 0, n = size_t(w) * size_t(h); p < n; ++p) {
                            rgba[p * 4 + 0] = char(bgr[p * 3 + 2]);
                            rgba[p * 4 + 1] = char(bgr[p * 3 + 1]);
                            rgba[p * 4 + 2] = char(bgr[p * 3 + 0]);
                            rgba[p * 4 + 3] = char(255);
                        }
                        const std::string name = "texture" + std::to_string(ti);
                        out.blobs.emplace_back(name, std::move(rgba));
                        tj["pixels"] = name;
                    }
                }
                textures.push_back(tj);
            }
            std::vector<float> uv(nf * 6, 0.f);
            std::vector<int32_t> tex_id(nf, -1);
            const bool has_mapping = !tm.material_texture_map.empty();
            for (size_t fi = 0; fi < nf; ++fi) {
                for (int vi = 0; vi < 3; ++vi) {
                    std::array<float, 2> c = {0.f, 0.f};
                    if (tm.has_face_uvs()) {
                        const int idx = tm.uv_indices[fi][size_t(vi)];
                        if (idx >= 0 && size_t(idx) < tm.uv_coords.size()) c = tm.uv_coords[size_t(idx)];
                    } else if (!tm.uvs.empty()) {
                        const int idx = tm.indices[fi][size_t(vi)];
                        if (idx >= 0 && size_t(idx) < tm.uvs.size()) c = tm.uvs[size_t(idx)];
                    }
                    uv[fi * 6 + size_t(vi) * 2] = c[0];
                    uv[fi * 6 + size_t(vi) * 2 + 1] = c[1];
                }
                const int mat = fi < tm.material_ids.size() ? tm.material_ids[fi] : -1;
                if (has_mapping && mat >= 0 && size_t(mat) < tm.material_texture_map.size()) tex_id[fi] = tm.material_texture_map[size_t(mat)];
                else tex_id[fi] = 0;
            }
            out.blobs.emplace_back("uv", bytes_of(uv));
            out.blobs.emplace_back("faceTexture", bytes_of(tex_id));
        }
        out.result["texturedTriangles"] = nf;
        out.result["textures"] = textures;
    }
    g_sessions[handle] = std::move(s);
    if (g_sessions.size() > 16) g_sessions.erase(g_sessions.begin()); // a forgotten close must not grow without bound
}

// analyze: args { handle, colorCount: 0 = automatic | 1..32, smooth: 0..10 (default 5), repair: "ask" | "none" | "repair" }
void op_analyze(const json& args, ToolOut& out)
{
    Session& s = session(args);
    if (!has_importable_texture(s.textured)) throw cs::JobError("analyze: the model carries no texture or colours");
    TexturePaintingSettings settings;
    const int count = args.value("colorCount", 0);
    if (count < 0 || count > MAX_FILAMENTS) throw cs::JobError("analyze: colorCount must be 0 (automatic) or 1..32");
    settings.target_colors_num = size_t(count);
    settings.smooth_weight = std::clamp(args.value("smooth", 5), 0, 10) / 10.0;
    const std::string repair = args.value("repair", std::string("ask"));
    settings.mesh_repair_decision = repair == "repair" ? TexturePaintingSettings::MeshRepairDecision::RepairAndImport
                                  : repair == "none"   ? TexturePaintingSettings::MeshRepairDecision::ImportWithoutRepair
                                                       : TexturePaintingSettings::MeshRepairDecision::Ask;
    // The dialog's repair callback: libslic3r's CGAL repair_mesh (as "Fix model").
    settings.mesh_repair_callback = [](const indexed_triangle_set& mesh, indexed_triangle_set& repaired_mesh,
                                       std::function<void(const char*, unsigned)> progress_callback, std::function<bool()>,
                                       std::string* error_message) -> bool {
        if (progress_callback) progress_callback("Repairing mesh", 0);
        TriangleMesh tm(mesh);
        if (!repair_mesh(tm, nullptr, error_message, {})) return false;
        repaired_mesh = tm.its;
        if (progress_callback) progress_callback("Repairing mesh", 100);
        return true;
    };
    bool decision_required = false;
    settings.mesh_repair_decision_required = &decision_required;
    PaintedMesh painted;
    auto progress = [](int percent, const char*) { cs::progress(percent, "Computing texture colors..."); };
    const bool ok = !s.textured.precomputed_face_colors.empty() ? face_colors_to_painting(s.textured, painted, settings, progress, nullptr)
                                                                : texture_to_painting(s.textured, painted, settings, progress, nullptr);
    if (!ok && decision_required) {
        out.result = {{"needsRepairDecision", true}};
        return;
    }
    if (!ok || painted.face_colors.empty() || painted.cluster_colors.empty()) throw cs::JobError("Computation failed. Please adjust parameters and retry.");
    s.painted = std::move(painted);
    s.has_painted = true;
    const PaintedMesh& p = s.painted;
    std::map<std::array<std::size_t, 3>, int> cluster_of;
    json clusters = json::array();
    for (size_t i = 0; i < p.cluster_colors.size(); ++i) {
        cluster_of[p.cluster_colors[i]] = int(i);
        clusters.push_back(rgb_to_hex(p.cluster_colors[i]));
    }
    std::string face_cluster(p.face_colors.size(), '\0');
    std::vector<size_t> counts(p.cluster_colors.size(), 0);
    for (size_t f = 0; f < p.face_colors.size(); ++f) {
        const auto it = cluster_of.find(p.face_colors[f]);
        const int c = it == cluster_of.end() ? 255 : it->second;
        face_cluster[f] = char(uint8_t(c));
        if (c != 255) ++counts[size_t(c)];
    }
    out.meshes.push_back(its_of(p.vertices, p.indices));
    out.blobs.emplace_back("faceCluster", std::move(face_cluster));
    out.result = {{"needsRepairDecision", false}, {"clusters", clusters}, {"clusterTriangles", counts}, {"colorCount", p.cluster_colors.size()},
                  {"triangles", p.indices.size()}};
}

// ---- filament matching (TextureImportDialog state, ported) ---------------------------------------
enum class Kind { ExistingPhysical, ExistingMixed, NewPhysical, NewMixed };
const char* kind_name(Kind k)
{
    switch (k) {
    case Kind::ExistingPhysical: return "existing";
    case Kind::ExistingMixed: return "existingMixed";
    case Kind::NewPhysical: return "new";
    default: return "newMixed";
    }
}
Kind kind_of(const std::string& s)
{
    if (s == "existing") return Kind::ExistingPhysical;
    if (s == "existingMixed") return Kind::ExistingMixed;
    if (s == "new") return Kind::NewPhysical;
    if (s == "newMixed") return Kind::NewMixed;
    throw cs::JobError("unknown filament entry kind '" + s + "'");
}
bool is_physical(Kind k) { return k == Kind::ExistingPhysical || k == Kind::NewPhysical; }
bool is_mixed(Kind k) { return k == Kind::ExistingMixed || k == Kind::NewMixed; }

struct Entry {
    Kind                      kind = Kind::ExistingPhysical;
    int                       dialog_index = -1;
    int                       project_index = -1;
    std::string               color_hex, name, type, preset_name;
    std::vector<unsigned int> mixed_components; // 1-based dialog (= project for existing) indices
    std::vector<int>          mixed_ratios;     // percent weights
};

bool entry_is_pla_basic(const Entry& e)
{
    return e.type == DEFAULT_VIRTUAL_FILAMENT_SHORT_TYPE || e.type == DEFAULT_VIRTUAL_FILAMENT_BASIC_TYPE ||
           e.name.find(DEFAULT_VIRTUAL_FILAMENT_BASIC_TYPE) != std::string::npos ||
           e.preset_name.find(DEFAULT_VIRTUAL_FILAMENT_BASIC_TYPE) != std::string::npos;
}
// ColorDecomposeSupport official_basic_type_from_preset_name
bool official_basic(const std::string& preset_name)
{
    return preset_name.find("Bambu PLA Basic") != std::string::npos || preset_name.find("Bambu PETG Basic") != std::string::npos;
}
bool entry_official_basic(const Entry& e)
{
    if (!is_physical(e.kind)) return false;
    if (e.kind == Kind::NewPhysical) return official_basic(e.name);
    return official_basic(e.preset_name);
}

struct Dialog {
    std::vector<Entry>                 entries;
    std::vector<std::array<float, 4>>  colors_rgba;   // m_filament_colors_rgba
    std::vector<std::string>           color_strs;    // m_filament_color_strs
    std::vector<std::string>           names;         // m_filament_names
    size_t                             existing = 0;  // m_existing_filament_count
    std::vector<FilamentMatch>         matches;       // m_current_matches
    std::vector<std::array<std::size_t, 3>> clusters; // m_painted.cluster_colors
    std::string                        default_preset;
    bool                               auto_merge = true;
    bool                               dropped = false;
    bool                               auto_mix_applied = false;

    size_t max_filament_count() const { return size_t(MAX_FILAMENTS); }
    bool can_add_virtual() const { return colors_rgba.size() < max_filament_count(); }

    void push(const Entry& e)
    {
        entries.push_back(e);
        entries.back().dialog_index = int(entries.size() - 1);
        color_strs.push_back(e.color_hex);
        names.push_back(e.name);
        colors_rgba.push_back(parse_color_string(e.color_hex));
    }

    int find_closest_filament_index(const std::array<std::size_t, 3>& color) const
    {
        int best_idx = -1;
        double best_delta = std::numeric_limits<double>::max();
        const size_t n = std::min(colors_rgba.size(), max_filament_count());
        for (size_t i = 0; i < n; ++i) {
            const double d = compute_delta_e(color, colors_rgba[i]);
            if (d < best_delta) { best_delta = d; best_idx = int(i); }
        }
        return best_idx;
    }

    int add_virtual_filament(const std::array<float, 4>& rgba, const std::string& hex, const std::string& preset_name = {})
    {
        if (!can_add_virtual()) { dropped = true; return -1; }
        Entry e;
        e.kind = Kind::NewPhysical;
        e.color_hex = normalize_hex(hex);
        e.name = DEFAULT_VIRTUAL_FILAMENT_NAME;
        e.preset_name = preset_name.empty() ? default_preset : preset_name;
        const int idx = int(colors_rgba.size());
        entries.push_back(e);
        entries.back().dialog_index = idx;
        colors_rgba.push_back(rgba);
        color_strs.push_back(hex);
        names.push_back(DEFAULT_VIRTUAL_FILAMENT_NAME);
        return idx;
    }

    int add_virtual_mixed_filament(const std::string& color_hex, const std::vector<int>& comps, const std::vector<int>& ratios)
    {
        if (comps.size() < 2 || comps.size() != ratios.size()) return -1;
        for (int idx : comps)
            if (idx < 0 || idx >= int(entries.size()) || !is_physical(entries[size_t(idx)].kind)) return -1;
        if (!can_add_virtual()) { dropped = true; return -1; }
        Entry e;
        e.kind = Kind::NewMixed;
        e.color_hex = normalize_hex(color_hex);
        e.name = DEFAULT_VIRTUAL_FILAMENT_NAME;
        e.mixed_ratios = ratios;
        for (int idx : comps) e.mixed_components.push_back(unsigned(idx + 1));
        push(e);
        return int(entries.size() - 1);
    }

    void set_match_target(FilamentMatch& m, int idx)
    {
        m.filament_index = idx;
        if (idx >= 0 && idx < int(colors_rgba.size())) {
            m.filament_color = colors_rgba[size_t(idx)];
            m.delta_e = compute_delta_e(m.cluster_color, m.filament_color);
            if (idx >= int(existing)) m.delta_e = 0.0;
        }
    }

    // TextureImportDialog::compact_used_virtual_filaments
    void compact()
    {
        if (matches.empty()) return;
        const std::vector<std::array<float, 4>> old_colors = colors_rgba;
        const std::vector<std::string> old_strs = color_strs, old_names = names;
        const std::vector<Entry> old_entries = entries;
        auto valid_new_mixed = [&](const Entry& e) {
            if (e.kind != Kind::NewMixed) return true;
            if (e.mixed_components.size() < 2 || e.mixed_components.size() != e.mixed_ratios.size()) return false;
            for (unsigned c : e.mixed_components) {
                const int ci = c >= 1 ? int(c) - 1 : -1;
                if (ci < 0 || ci >= int(old_entries.size()) || ci >= int(old_colors.size()) || !is_physical(old_entries[size_t(ci)].kind)) return false;
            }
            return true;
        };
        std::set<int> used;
        for (const auto& m : matches)
            if (m.filament_index >= int(existing) && m.filament_index < int(old_colors.size())) {
                if (m.filament_index < int(old_entries.size()) && old_entries[size_t(m.filament_index)].kind == Kind::NewMixed &&
                    !valid_new_mixed(old_entries[size_t(m.filament_index)]))
                    continue;
                used.insert(m.filament_index);
            }
        for (bool added = true; added;) {
            added = false;
            const std::vector<int> cur(used.begin(), used.end());
            for (int u : cur) {
                if (u < 0 || u >= int(old_entries.size()) || old_entries[size_t(u)].kind != Kind::NewMixed || !valid_new_mixed(old_entries[size_t(u)])) continue;
                for (unsigned c : old_entries[size_t(u)].mixed_components) {
                    const int ci = c >= 1 ? int(c) - 1 : -1;
                    if (ci >= int(existing) && ci < int(old_entries.size()) && used.insert(ci).second) added = true;
                }
            }
        }
        std::vector<std::array<float, 4>> nc;
        std::vector<std::string> ns, nn;
        std::vector<Entry> ne;
        const size_t ex = std::min(existing, old_colors.size());
        for (size_t i = 0; i < ex; ++i) {
            nc.push_back(old_colors[i]);
            ns.push_back(i < old_strs.size() ? old_strs[i] : "");
            nn.push_back(i < old_names.size() ? old_names[i] : "Filament " + std::to_string(i + 1));
            Entry e = i < old_entries.size() ? old_entries[i] : Entry{};
            e.dialog_index = int(i);
            e.color_hex = normalize_hex(ns.back());
            e.name = nn.back();
            ne.push_back(e);
        }
        std::map<int, int> old_to_new;
        for (int oi : used) {
            old_to_new[oi] = int(nc.size());
            nc.push_back(old_colors[size_t(oi)]);
            ns.push_back(oi < int(old_strs.size()) ? old_strs[size_t(oi)] : "");
            nn.push_back(oi < int(old_names.size()) ? old_names[size_t(oi)] : DEFAULT_VIRTUAL_FILAMENT_NAME);
            Entry e = oi < int(old_entries.size()) ? old_entries[size_t(oi)] : Entry{};
            e.dialog_index = int(ne.size());
            e.color_hex = normalize_hex(ns.back());
            e.name = nn.back();
            if (e.kind == Kind::NewPhysical && e.preset_name.empty()) e.preset_name = default_preset;
            ne.push_back(e);
        }
        colors_rgba = std::move(nc);
        color_strs = std::move(ns);
        names = std::move(nn);
        entries = std::move(ne);
        std::set<int> invalid_mixed;
        for (Entry& e : entries) {
            if (e.kind != Kind::NewMixed) continue;
            std::vector<int> comps;
            bool valid = e.mixed_components.size() >= 2 && e.mixed_components.size() == e.mixed_ratios.size();
            for (unsigned c : e.mixed_components) {
                const int oc = c >= 1 ? int(c) - 1 : -1;
                if (oc < 0) { valid = false; break; }
                auto it = old_to_new.find(oc);
                const int nci = it != old_to_new.end() ? it->second : oc;
                if (nci < 0 || nci >= int(entries.size()) || !is_physical(entries[size_t(nci)].kind)) { valid = false; break; }
                comps.push_back(nci);
            }
            if (!valid) { invalid_mixed.insert(e.dialog_index); continue; }
            e.mixed_components.clear();
            for (int c : comps) e.mixed_components.push_back(unsigned(c + 1));
        }
        auto closest_physical = [this](const std::array<std::size_t, 3>& color) {
            int best = -1;
            double bd = std::numeric_limits<double>::max();
            const size_t n = std::min(colors_rgba.size(), max_filament_count());
            for (size_t i = 0; i < n && i < entries.size(); ++i) {
                if (!is_physical(entries[i].kind)) continue;
                const double d = compute_delta_e(color, colors_rgba[i]);
                if (d < bd) { bd = d; best = int(i); }
            }
            return best;
        };
        for (auto& m : matches) {
            auto it = old_to_new.find(m.filament_index);
            if (it != old_to_new.end()) m.filament_index = it->second;
            else if (m.filament_index >= int(existing)) m.filament_index = find_closest_filament_index(m.cluster_color);
            if (invalid_mixed.count(m.filament_index) > 0) {
                const int fb = closest_physical(m.cluster_color);
                m.filament_index = fb >= 0 ? fb : find_closest_filament_index(m.cluster_color);
            }
            if (m.filament_index >= 0 && m.filament_index < int(colors_rgba.size())) set_match_target(m, m.filament_index);
        }
    }

    // TextureImportDialog::do_auto_match
    void auto_match()
    {
        if (clusters.empty()) return;
        auto_mix_applied = false;
        dropped = false;
        compact();
        const auto previous = matches;
        std::map<std::array<std::size_t, 3>, int> prev_virtual;
        for (const auto& m : previous)
            if (m.filament_index >= int(existing) && m.filament_index < int(entries.size()) && is_physical(entries[size_t(m.filament_index)].kind))
                prev_virtual[m.cluster_color] = m.filament_index;
        auto find_virtual_by_color = [this](const std::array<std::size_t, 3>& color) -> int {
            const std::string hex = rgb_to_hex(color);
            for (size_t i = existing; i < color_strs.size(); ++i)
                if (color_strs[i] == hex && i < entries.size() && is_physical(entries[i].kind)) return int(i);
            return -1;
        };
        auto get_or_add_virtual = [&](const std::array<std::size_t, 3>& color) -> int {
            auto pit = prev_virtual.find(color);
            if (pit != prev_virtual.end() && pit->second >= int(existing) && pit->second < int(colors_rgba.size())) return pit->second;
            const int ex = find_virtual_by_color(color);
            if (ex >= 0) return ex;
            return add_virtual_filament({color[0] / 255.f, color[1] / 255.f, color[2] / 255.f, 1.f}, rgb_to_hex(color));
        };
        if (auto_merge) {
            std::vector<std::string> nm;
            for (size_t i = 0; i < existing; ++i) nm.push_back(i < names.size() ? names[i] : "Filament " + std::to_string(i + 1));
            std::vector<std::array<float, 4>> ex_colors(colors_rgba.begin(), colors_rgba.begin() + std::min(existing, colors_rgba.size()));
            matches = match_clusters_to_filaments(clusters, ex_colors, nm);
            constexpr double NEW_FILAMENT_THRESHOLD = 5.0;
            std::map<std::array<std::size_t, 3>, int> virtual_index;
            for (auto& m : matches) {
                if (m.delta_e <= NEW_FILAMENT_THRESHOLD) continue;
                auto it = virtual_index.find(m.cluster_color);
                if (it != virtual_index.end()) {
                    m.filament_index = it->second;
                } else {
                    const int ni = get_or_add_virtual(m.cluster_color);
                    if (ni >= 0) virtual_index[m.cluster_color] = ni;
                    m.filament_index = ni >= 0 ? ni : find_closest_filament_index(m.cluster_color);
                }
                if (m.filament_index >= 0 && m.filament_index < int(colors_rgba.size())) set_match_target(m, m.filament_index);
            }
        } else {
            matches.clear();
            std::map<std::array<std::size_t, 3>, int> vmap;
            for (size_t i = 0; i < clusters.size(); ++i) {
                FilamentMatch fm;
                fm.cluster_index = int(i);
                fm.cluster_color = clusters[i];
                auto it = vmap.find(fm.cluster_color);
                if (it != vmap.end()) {
                    fm.filament_index = it->second;
                } else {
                    const int idx = get_or_add_virtual(fm.cluster_color);
                    if (idx >= 0) vmap[fm.cluster_color] = idx;
                    fm.filament_index = idx >= 0 ? idx : find_closest_filament_index(fm.cluster_color);
                }
                if (fm.filament_index >= 0 && fm.filament_index < int(colors_rgba.size())) set_match_target(fm, fm.filament_index);
                matches.push_back(fm);
            }
        }
    }

    // TextureImportDialog::sort_current_matches_by_filament_index
    void sort_matches()
    {
        std::stable_sort(matches.begin(), matches.end(), [](const FilamentMatch& l, const FilamentMatch& r) {
            const bool lv = l.filament_index >= 0, rv = r.filament_index >= 0;
            if (lv != rv) return lv;
            if (!lv) return false;
            return l.filament_index < r.filament_index;
        });
    }

    // TextureImportDialog::apply_auto_standard_mix (rows = the current matches, in order)
    bool auto_mix(ColorDecomposeRecipeMode mode)
    {
        if (matches.empty()) return false;
        dropped = false;
        auto find_or_add_base = [this](const std::string& color_hex) -> int {
            const std::string n = normalize_hex(color_hex);
            for (const Entry& e : entries)
                if (is_physical(e.kind) && normalize_hex(e.color_hex) == n && entry_is_pla_basic(e)) return e.dialog_index;
            const int idx = add_virtual_filament(parse_color_string(n), n, default_preset);
            if (idx >= 0 && idx < int(entries.size())) {
                entries[size_t(idx)].type = DEFAULT_VIRTUAL_FILAMENT_BASIC_TYPE;
                entries[size_t(idx)].name = DEFAULT_VIRTUAL_FILAMENT_NAME;
            }
            return idx;
        };
        auto find_existing_mixed = [this](const std::vector<int>& comps, const std::vector<int>& ratios) -> int {
            for (const Entry& e : entries) {
                if (!is_mixed(e.kind) || e.mixed_components.size() != comps.size() || e.mixed_ratios.size() != ratios.size()) continue;
                bool same = true;
                for (size_t i = 0; i < comps.size() && same; ++i)
                    same = e.mixed_components[i] == unsigned(comps[i] + 1) && e.mixed_ratios[i] == ratios[i];
                if (same) return e.dialog_index;
            }
            return -1;
        };
        bool changed = false;
        for (size_t row = 0; row < matches.size(); ++row) {
            ColorDecomposeRgb target;
            if (!color_decompose_hex_to_rgb(rgb_to_hex(matches[row].cluster_color), target)) continue;
            const auto recipe = lookup_standard_recipe(target, mode, DEFAULT_VIRTUAL_FILAMENT_BASIC_TYPE);
            if (!recipe.valid || recipe.components.size() < 2) continue;
            std::vector<int> comps, ratios;
            for (const auto& c : recipe.components) {
                const int ci = find_or_add_base(c.color_hex);
                if (ci < 0) { comps.clear(); break; }
                comps.push_back(ci);
                ratios.push_back(c.ratio);
            }
            if (comps.size() < 2 || comps.size() != ratios.size()) continue;
            int mi = find_existing_mixed(comps, ratios);
            if (mi < 0) mi = add_virtual_mixed_filament(recipe.matched_color_hex, comps, ratios);
            if (mi < 0) continue;
            set_match_target(matches[row], mi);
            changed = true;
        }
        if (!changed) return false;
        auto_mix_applied = true;
        compact();
        return true;
    }
};

std::vector<unsigned int> json_uints(const json& j)
{
    std::vector<unsigned int> v;
    if (j.is_string()) return parse_mixed_components(j.get<std::string>());
    if (j.is_array()) for (const json& x : j) if (x.is_number()) v.push_back(unsigned(std::max(0, int(std::lround(x.get<double>())))));
    return v;
}

// The project's filament slots (Plater::run_textured_mesh_import_dialog's filament_entries).
void load_project_filaments(Dialog& d, const json& fils)
{
    if (!fils.is_array() || fils.empty()) throw cs::JobError("filaments: the project's filament slots are required");
    if (fils.size() > size_t(MAX_FILAMENTS)) throw cs::JobError("filaments: at most 32");
    int i = 0;
    for (const json& f : fils) {
        if (!f.is_object()) throw cs::JobError("filaments: entries must be objects");
        Entry e;
        e.kind = f.value("mixed", false) ? Kind::ExistingMixed : Kind::ExistingPhysical;
        e.project_index = i;
        e.color_hex = normalize_hex(f.value("color", std::string("#808080")));
        e.type = f.value("type", std::string());
        e.name = f.value("name", std::string());
        if (e.name.empty()) e.name = "Filament " + std::to_string(i + 1);
        e.preset_name = f.value("presetName", std::string());
        if (e.kind == Kind::ExistingMixed) {
            e.mixed_components = json_uints(f.value("components", json::array()));
            const json r = f.value("ratios", json::array());
            if (r.is_string()) {
                for (double x : parse_mixed_ratios(r.get<std::string>(), e.mixed_components.size())) e.mixed_ratios.push_back(int(std::lround(x * 100.0)));
            } else if (r.is_array()) {
                for (const json& x : r) e.mixed_ratios.push_back(x.is_number() ? int(std::lround(x.get<double>())) : 0);
            }
        }
        d.push(e);
        ++i;
    }
    d.existing = d.entries.size();
}

// Host-held dialog state (what match / add_mixed return) back into a Dialog.
void load_state(Dialog& d, const json& args)
{
    const json ents = args.value("entries", json::array());
    if (!ents.is_array() || ents.empty() || ents.size() > size_t(MAX_FILAMENTS)) throw cs::JobError("entries: 1..32 filament entries required");
    for (const json& j : ents) {
        if (!j.is_object()) throw cs::JobError("entries: entries must be objects");
        Entry e;
        e.kind = kind_of(j.value("kind", std::string("existing")));
        e.project_index = j.value("project", -1);
        e.color_hex = normalize_hex(j.value("color", std::string("#808080")));
        e.name = j.value("name", std::string());
        e.type = j.value("type", std::string());
        e.preset_name = j.value("presetName", std::string());
        e.mixed_components = json_uints(j.value("components", json::array()));
        for (const json& x : j.value("ratios", json::array())) e.mixed_ratios.push_back(x.is_number() ? int(std::lround(x.get<double>())) : 0);
        if ((e.kind == Kind::ExistingPhysical || e.kind == Kind::ExistingMixed) && e.project_index < 0) throw cs::JobError("entries: an existing entry needs its project index");
        d.push(e);
    }
    d.existing = 0;
    while (d.existing < d.entries.size() && (d.entries[d.existing].kind == Kind::ExistingPhysical || d.entries[d.existing].kind == Kind::ExistingMixed)) ++d.existing;
    for (size_t i = d.existing; i < d.entries.size(); ++i)
        if (d.entries[i].kind == Kind::ExistingPhysical || d.entries[i].kind == Kind::ExistingMixed) throw cs::JobError("entries: existing entries come first");
    for (const json& m : args.value("matches", json::array())) {
        if (!m.is_object()) continue;
        FilamentMatch fm;
        fm.cluster_index = m.value("cluster", -1);
        fm.cluster_color = parse_rgb(m.value("color", std::string("#808080")));
        const int fi = m.value("filament", -1);
        if (fi >= int(d.entries.size())) throw cs::JobError("matches: filament index out of range");
        fm.filament_index = -1;
        if (fi >= 0) d.set_match_target(fm, fi);
        d.matches.push_back(fm);
    }
}

json state_json(const Dialog& d)
{
    json ents = json::array();
    for (const Entry& e : d.entries) {
        json j = {{"kind", kind_name(e.kind)}, {"color", e.color_hex}, {"name", e.name}, {"type", e.type}};
        if (e.project_index >= 0 && (e.kind == Kind::ExistingPhysical || e.kind == Kind::ExistingMixed)) j["project"] = e.project_index;
        if (!e.preset_name.empty()) j["presetName"] = e.preset_name;
        if (is_mixed(e.kind)) { j["components"] = e.mixed_components; j["ratios"] = e.mixed_ratios; }
        ents.push_back(j);
    }
    json ms = json::array();
    for (const FilamentMatch& m : d.matches)
        ms.push_back({{"cluster", m.cluster_index}, {"color", rgb_to_hex(m.cluster_color)}, {"filament", m.filament_index}, {"deltaE", m.delta_e}});
    return {{"entries", ents}, {"matches", ms}, {"dropped", d.dropped}, {"autoMixApplied", d.auto_mix_applied}};
}

// match: args { handle | clusters:[hex], filaments:[{color, name?, type?, presetName?, mixed?, components?, ratios?}],
//               autoMerge?: true, autoMix?: "none" | "cmyw" | "rybw", defaultPresetName? }
void op_match(const json& args, ToolOut& out)
{
    Dialog d;
    d.default_preset = args.value("defaultPresetName", std::string());
    d.auto_merge = args.value("autoMerge", true);
    if (auto c = args.find("clusters"); c != args.end() && c->is_array()) {
        for (const json& h : *c) d.clusters.push_back(parse_rgb(h.is_string() ? h.get<std::string>() : std::string()));
    } else {
        Session& s = session(args);
        if (!s.has_painted) throw cs::JobError("match: analyze the model first");
        d.clusters = s.painted.cluster_colors;
    }
    if (d.clusters.empty() || d.clusters.size() > 255) throw cs::JobError("match: 1..255 clusters");
    load_project_filaments(d, args.value("filaments", json::array()));
    // on_computation_complete: baseline match, compacted, rows in filament order.
    d.auto_match();
    d.compact();
    d.sort_matches();
    const std::string mix = args.value("autoMix", std::string("none"));
    if (mix == "cmyw" || mix == "rybw") d.auto_mix(mix == "cmyw" ? ColorDecomposeRecipeMode::CMYW : ColorDecomposeRecipeMode::RYBW);
    else if (mix != "none") throw cs::JobError("match: autoMix is none, cmyw or rybw");
    out.result = state_json(d);
}

// add_mixed: args { entries, matches, row, components:[{ filament?: dialog index of a physical entry, color? }],
//                   ratios:[percent], color: matched "#hex", defaultPresetName? }
void op_add_mixed(const json& args, ToolOut& out)
{
    Dialog d;
    d.default_preset = args.value("defaultPresetName", std::string());
    load_state(d, args);
    const int row = args.value("row", -1);
    if (row < 0 || row >= int(d.matches.size())) throw cs::JobError("add_mixed: row out of range");
    const json comps = args.value("components", json::array());
    std::vector<int> ratios;
    for (const json& r : args.value("ratios", json::array())) ratios.push_back(r.is_number() ? int(std::lround(r.get<double>())) : 0);
    if (!comps.is_array() || comps.size() < 2 || comps.size() != ratios.size()) throw cs::JobError("add_mixed: 2+ components with one ratio each");
    std::vector<int> idx;
    for (const json& c : comps) {
        const int fi = c.value("filament", -1);
        if (fi >= 0) {
            if (fi >= int(d.entries.size()) || d.entries[size_t(fi)].kind != Kind::ExistingPhysical) throw cs::JobError("add_mixed: component filament must be an existing physical filament");
            idx.push_back(fi);
            continue;
        }
        const std::string hex = normalize_hex(c.value("color", std::string()));
        int ex = -1;
        for (const Entry& e : d.entries)
            if (is_physical(e.kind) && normalize_hex(e.color_hex) == hex && entry_official_basic(e)) { ex = e.dialog_index; break; }
        if (ex < 0) ex = d.add_virtual_filament(parse_color_string(hex), hex);
        if (ex < 0) throw cs::JobError("add_mixed: no room for another filament");
        idx.push_back(ex);
    }
    const int mi = d.add_virtual_mixed_filament(normalize_hex(args.value("color", std::string("#808080"))), idx, ratios);
    if (mi < 0) throw cs::JobError("add_mixed: the mixed filament could not be added");
    d.matches[size_t(row)].filament_index = mi;
    out.result = state_json(d);
}

// apply: args { handle, entries, matches } -> the project changes (Plater::apply_textured_mesh_import_result)
void op_apply(const json& args, ToolOut& out)
{
    Session& s = session(args);
    if (!s.has_painted) throw cs::JobError("apply: analyze the model first");
    const PaintedMesh& painted = s.painted;
    Dialog d;
    load_state(d, args);
    d.clusters = painted.cluster_colors;
    for (const FilamentMatch& m : d.matches)
        if (m.filament_index < 0 || m.filament_index >= int(d.entries.size())) throw cs::JobError("apply: every colour needs a filament");
    // on_ok_clicked
    d.compact();

    std::vector<int> remap(d.entries.size(), -1);
    int existing_physical = 0, new_physical = 0, existing_total = 0;
    for (const Entry& e : d.entries) {
        if (e.kind == Kind::ExistingPhysical) ++existing_physical;
        else if (e.kind == Kind::NewPhysical) ++new_physical;
        if (e.kind == Kind::ExistingPhysical || e.kind == Kind::ExistingMixed) ++existing_total;
    }
    for (const Entry& e : d.entries) {
        if (e.kind == Kind::ExistingPhysical) remap[size_t(e.dialog_index)] = e.project_index;
        else if (e.kind == Kind::ExistingMixed) remap[size_t(e.dialog_index)] = e.project_index + new_physical;
    }
    json new_fils = json::array();
    int order = 0;
    std::vector<std::string> physical_colors; // project physical colours after the additions, by final index
    for (const Entry& e : d.entries)
        if (e.kind == Kind::ExistingPhysical) {
            if (int(physical_colors.size()) <= e.project_index) physical_colors.resize(size_t(e.project_index) + 1, "#808080");
            physical_colors[size_t(e.project_index)] = e.color_hex;
        }
    physical_colors.resize(size_t(existing_physical), "#808080");
    for (const Entry& e : d.entries) {
        if (e.kind != Kind::NewPhysical) continue;
        const int final_idx = existing_physical + order++;
        remap[size_t(e.dialog_index)] = final_idx;
        physical_colors.push_back(e.color_hex);
        new_fils.push_back({{"index", final_idx}, {"color", e.color_hex}, {"presetName", e.preset_name}, {"name", e.name}});
    }
    json new_mixed = json::array();
    int next_slot = existing_total + new_physical;
    for (const Entry& e : d.entries) {
        if (e.kind != Kind::NewMixed) continue;
        std::vector<unsigned int> comps;
        bool valid = e.mixed_components.size() >= 2 && e.mixed_components.size() == e.mixed_ratios.size();
        for (unsigned c : e.mixed_components) {
            const int di = int(c) - 1;
            if (di < 0 || di >= int(remap.size()) || remap[size_t(di)] < 0) { valid = false; break; }
            comps.push_back(unsigned(remap[size_t(di)] + 1));
        }
        if (!valid || next_slot >= MAX_FILAMENTS) continue;
        std::vector<std::string> hex;
        for (unsigned c : comps) hex.push_back(c - 1 < physical_colors.size() ? physical_colors[c - 1] : std::string("#808080"));
        const int final_idx = next_slot++;
        remap[size_t(e.dialog_index)] = final_idx;
        new_mixed.push_back({{"index", final_idx}, {"components", comps}, {"ratios", e.mixed_ratios}, {"color", blend_color_multi(hex, e.mixed_ratios)}});
    }

    // Cluster colour -> final filament (0-based), the base extruder (lowest filament painted), the paint.
    std::map<std::array<std::size_t, 3>, int> color_to_filament;
    json final_matches = json::array();
    for (const FilamentMatch& m : d.matches) {
        const int f = m.filament_index >= 0 && m.filament_index < int(remap.size()) ? remap[size_t(m.filament_index)] : -1;
        if (m.cluster_index >= 0 && m.cluster_index < int(painted.cluster_colors.size()) && f >= 0)
            color_to_filament[painted.cluster_colors[size_t(m.cluster_index)]] = f;
        final_matches.push_back({{"cluster", m.cluster_index}, {"color", rgb_to_hex(m.cluster_color)}, {"filament", f}});
    }
    indexed_triangle_set its = its_of(painted.vertices, painted.indices);
    Model model;
    ModelVolume* vol = model.add_object()->add_volume(TriangleMesh(its));
    TriangleSelector selector(vol->mesh());
    std::string states(painted.face_colors.size(), '\0');
    int min_used = -1;
    for (size_t fi = 0; fi < painted.face_colors.size() && fi < its.indices.size(); ++fi) {
        const auto it = color_to_filament.find(painted.face_colors[fi]);
        if (it == color_to_filament.end()) continue;
        const int st = int(EnforcerBlockerType::Extruder1) + it->second;
        if (st > int(EnforcerBlockerType::ExtruderMax)) continue;
        selector.set_facet(int(fi), EnforcerBlockerType(st));
        states[fi] = char(uint8_t(st));
        if (min_used < 0 || it->second + 1 < min_used) min_used = it->second + 1;
    }
    vol->mmu_segmentation_facets.set(selector);
    json paint = json::array();
    for (const auto& m : vol->mmu_segmentation_facets.get_data().triangles_to_split)
        paint.push_back({m.triangle_idx, vol->mmu_segmentation_facets.get_triangle_as_string(m.triangle_idx)});

    out.meshes.push_back(its);
    out.blobs.emplace_back("faceFilament", std::move(states));
    if (!s.object_uv.empty()) {
        const std::vector<Vec2f> uv = transfer_uv_channel(s.object_its, s.object_uv, its);
        if (!uv.empty()) out.blobs.emplace_back("modelUv", bytes_of(uv));
    }
    out.result = {{"newFilaments", new_fils}, {"newMixedFilaments", new_mixed}, {"remap", remap}, {"matches", final_matches},
                  {"extruder", min_used > 0 ? json(min_used) : json(nullptr)}, {"paint", {{"color", paint}}},
                  {"triangles", its.indices.size()}};
}

int call_impl(const char* req, int req_len, const uint8_t* blob, int blob_len, std::string& out_json, std::string& out_blob)
{
    ToolOut out;
    json rep = {{"ok", false}, {"error", nullptr}};
    try {
        ensure_runtime_dirs();
        if (!req || req_len <= 0) throw cs::JobError("empty request");
        const json j = json::parse(req, req + req_len);
        const std::string op = j.value("op", std::string());
        const json args = j.value("args", json::object());
        if (op == "load") op_load(args, blob, blob_len, out);
        else if (op == "analyze") op_analyze(args, out);
        else if (op == "match") op_match(args, out);
        else if (op == "add_mixed") op_add_mixed(args, out);
        else if (op == "apply") op_apply(args, out);
        else if (op == "close") g_sessions.erase(args.value("handle", 0));
        else throw cs::JobError("unknown op '" + op + "'");
        rep["ok"] = true;
    } catch (const std::bad_alloc&) {
        rep["error"] = "out of memory";
    } catch (const std::exception& e) {
        rep["error"] = e.what();
    }
    json mj = json::array();
    size_t size = 0;
    for (const auto& m : out.meshes) size += m.vertices.size() * 12 + m.indices.size() * 12;
    out_blob.assign(size, '\0');
    size_t off = 0;
    for (const auto& m : out.meshes) {
        const size_t vb = m.vertices.size() * 12, ib = m.indices.size() * 12;
        if (vb) std::memcpy(out_blob.data() + off, m.vertices.data(), vb);
        if (ib) std::memcpy(out_blob.data() + off + vb, m.indices.data(), ib);
        mj.push_back({{"vertexOffset", off}, {"vertexCount", m.vertices.size()}, {"indexOffset", off + vb}, {"triangleCount", m.indices.size()}});
        off += vb + ib;
    }
    json bj = json::object();
    for (const auto& [name, bytes] : out.blobs) {
        off = out_blob.size();
        out_blob.resize(off + ((bytes.size() + 3) & ~size_t(3)), '\0');
        if (!bytes.empty()) std::memcpy(out_blob.data() + off, bytes.data(), bytes.size());
        bj[name] = {{"offset", off}, {"length", bytes.size()}};
    }
    rep["result"] = out.result;
    rep["meshes"] = mj;
    rep["blobs"] = bj;
    out_json = rep.dump();
    return rep["ok"].get<bool>() ? 0 : 1;
}

std::string g_version;

} // namespace

extern "C" {

EMSCRIPTEN_KEEPALIVE const char* tex_version(void)
{
    if (g_version.empty())
        g_version = json{{"module", "orca-texture"}, {"abi", 1}, {"orca", "2.5.0"},
                         {"assimp", std::to_string(aiGetVersionMajor()) + "." + std::to_string(aiGetVersionMinor()) + "." + std::to_string(aiGetVersionPatch())},
                         {"opencv", CV_VERSION},
#ifdef NDEBUG
                         {"build", "release"}
#else
                         {"build", "debug"}
#endif
                        }.dump();
    return g_version.c_str();
}

EMSCRIPTEN_KEEPALIVE int tex_call(const char* req_json, int req_len, const uint8_t* blob, int blob_len,
                                  char** out_json, int* out_json_len, uint8_t** out_blob, int* out_blob_len)
{
    if (out_json) *out_json = nullptr;
    if (out_json_len) *out_json_len = 0;
    if (out_blob) *out_blob = nullptr;
    if (out_blob_len) *out_blob_len = 0;
    std::string js, bl;
    int rc = 1;
    try {
        rc = call_impl(req_json, req_len, blob, blob_len, js, bl);
    } catch (...) {
        js = R"({"ok":false,"error":"engine error"})";
        rc = 1;
    }
    if (cs::emit(js, out_json, out_json_len) != 0) return -1;
    if (!bl.empty() && out_blob && out_blob_len) {
        uint8_t* p = static_cast<uint8_t*>(std::malloc(bl.size()));
        if (!p) return -1;
        std::memcpy(p, bl.data(), bl.size());
        *out_blob = p;
        *out_blob_len = int(bl.size());
    }
    return rc;
}

EMSCRIPTEN_KEEPALIVE void tex_free(void* p) { std::free(p); }

} // extern "C"
