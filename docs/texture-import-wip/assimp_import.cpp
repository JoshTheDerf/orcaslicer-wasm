// -> wasm/stubs/assimp_import.cpp when resuming (the Assimp stand-in split out of stubs/formats.cpp, so the
// texture module's real AssimpImport.o replaces it without a duplicate symbol).
#include "libslic3r/Format/AssimpImport.hpp"

#include <string>

namespace Slic3r {

bool load_assimp_textured_model(const std::string &, TexturedMesh &, std::string *error_message)
{
    if (error_message)
        *error_message = "This file format is not available in this build.";
    return false;
}

} // namespace Slic3r
