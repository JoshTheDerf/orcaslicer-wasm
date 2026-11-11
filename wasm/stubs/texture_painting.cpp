// Texture → colour painting without OpenCV (replaces TexturePainting.cpp,
// TextureToColor/* and ObjColorUtils.cpp, see wasm/CMakeLists.txt). These turn
// a textured / vertex-coloured import (OBJ, GLB) into filament painting; the
// engine receives meshes from the host, so they are not on the slicing path.
// Each entry point reports failure.
#include "libslic3r/TexturePainting.hpp"

namespace Slic3r {

bool texture_to_painting(const TexturedMesh &, PaintedMesh &, const TexturePaintingSettings &, PaintProgressCallback, PaintCancelCallback) { return false; }
bool face_colors_to_painting(const TexturedMesh &, PaintedMesh &, const TexturePaintingSettings &, PaintProgressCallback, PaintCancelCallback) { return false; }
std::vector<FilamentMatch> match_clusters_to_filaments(const std::vector<std::array<std::size_t, 3>> &, const std::vector<std::array<float, 4>> &,
                                                       const std::vector<std::string> &) { return {}; }
double compute_delta_e(const std::array<std::size_t, 3> &, const std::array<float, 4> &) { return 0.; }
bool apply_painted_mesh_to_volume(const PaintedMesh &, const std::vector<FilamentMatch> &, ModelVolume &) { return false; }
bool decode_texture_to_pixels(const TextureImage &, std::vector<unsigned char> &, int &, int &) { return false; }
bool sample_original_face_colors(const TexturedMesh &, std::vector<std::array<std::size_t, 3>> &) { return false; }

} // namespace Slic3r
