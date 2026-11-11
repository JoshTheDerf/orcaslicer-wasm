// Model-file formats whose libraries are not linked in the WASM engine. Same
// symbols as the upstream sources they replace (see wasm/CMakeLists.txt); every
// entry point fails cleanly. The host (Cubby Slicer) imports meshes itself and
// hands the engine triangles, so none of these is on the slicing path.
//
//   Format/STEP.cpp          OCCT     (STEP.hpp is patched to build with SLIC3R_NO_OCCT)
//   Format/svg.cpp           OCCT
//   Format/DRC.cpp           Draco
//   Format/AssimpImport.cpp  Assimp   (FBX/GLB/DAE/... textured import)
#include "libslic3r/Format/STEP.hpp"
#include "libslic3r/Format/svg.hpp"
#include "libslic3r/Format/DRC.hpp"
#include "libslic3r/Format/AssimpImport.hpp"

#ifndef SLIC3R_NO_OCCT
#error "stubs/formats.cpp is only for builds without OCCT"
#endif

namespace Slic3r {

// ---- STEP (OCCT) ----
bool load_step(const char *, Model *, bool &is_cancel, double, double, bool, ImportStepProgressFn, StepIsUtf8Fn, long &mesh_face_num)
{
    is_cancel     = false;
    mesh_face_num = 0;
    return false;
}

bool StepPreProcessor::preprocess(const char *path, std::string &output_path)
{
    output_path = path ? path : "";
    return true;
}
bool StepPreProcessor::isUtf8File(const char *) { return true; }
bool StepPreProcessor::isUtf8(const std::string) { return true; }
bool StepPreProcessor::isGBK(const std::string) { return false; }
int  StepPreProcessor::preNum(const unsigned char) { return 0; }

Step::Step(boost::filesystem::path path, ImportStepProgressFn stepFn, StepIsUtf8Fn isUtf8Fn)
    : Step(path.string(), std::move(stepFn), std::move(isUtf8Fn)) {}
Step::Step(std::string path, ImportStepProgressFn stepFn, StepIsUtf8Fn isUtf8Fn)
    : m_path(std::move(path)), m_stepFn(std::move(stepFn)), m_utf8Fn(std::move(isUtf8Fn)) {}
Step::~Step() = default;
Step::Step_Status Step::load() { return Step_Status::LOAD_ERROR; }
unsigned int Step::get_triangle_num(double, double) { return 0; }
unsigned int Step::get_triangle_num_tbb(double, double) { return 0; }
void Step::clean_mesh_data() {}
Step::Step_Status Step::mesh(Model *, bool &is_cancel, bool, double, double)
{
    is_cancel = false;
    return Step_Status::MESH_ERROR;
}
void Step::update_process(int, int, int, bool &) {}

// ---- SVG as a 3D model (OCCT) ----
bool load_svg(const char *, Model *, std::string &message)
{
    message = "SVG import is not available in this build.";
    return false;
}

// ---- Draco ----
bool load_drc(const char *, TriangleMesh *) { return false; }
bool load_drc(const char *, Model *, const char *) { return false; }
bool store_drc(const char *, TriangleMesh *, int, int) { return false; }
bool store_drc(const char *, ModelObject *, int, int) { return false; }
bool store_drc(const char *, Model *, int, int) { return false; }

// ---- Assimp ----
bool load_assimp_textured_model(const std::string &, TexturedMesh &, std::string *error_message)
{
    if (error_message)
        *error_message = "This file format is not available in this build.";
    return false;
}

} // namespace Slic3r
