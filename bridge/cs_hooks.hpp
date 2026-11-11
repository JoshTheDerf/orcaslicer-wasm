// Plugin hook points of OrcaSlicer 2.5's libslic3r, driven from JavaScript.
//
// Desktop Orca's Python plugin layer (src/slic3r/plugin, GUI only) installs
// dispatchers into seams libslic3r exposes, all inert by default:
//
//   Print::set_slicing_pipeline_hook_fn   12 pipeline steps inside Print::process(), active only
//                                         while the config's `slicing_pipeline_plugin` is non-empty
//   set_lifecycle_hook_fn                 SliceStarted / SliceGeometryFinished /
//                                         GCodeExportStarted / GCodeExportFinished (from libslic3r)
//   ConfigBase::set_resolve_capability_fn manifest resolution (GUI preset saving; not needed here)
//   psGCodePostProcess                    fired by the GUI's PostProcessor, not by libslic3r: the
//                                         host runs it on its copy of the G-code at export time.
//
// For one cs_slice, Scope installs:
//   * a pipeline dispatcher that calls the host's `Module.csHook("pipeline", beginJson, null)`
//     SYNCHRONOUSLY (proxied to the main runtime thread in the pthreads build). While that call
//     is on the stack the host reads and edits the live slice graph through the re-entrant
//     export cs_hook_query() below, then returns the capability result.
//   * a lifecycle recorder: events go to report.events, never into JS during the slice.
// Wire format: cubby-slicer/docs/ENGINE-CONTRACT.md, "Plugin hooks".
#pragma once

#include <nlohmann/json.hpp>

#include <cstdint>

namespace Slic3r { class Print; }

namespace cs::hooks {

using json = nlohmann::json;

class Scope {
public:
    explicit Scope(Slic3r::Print& print);
    ~Scope();
    Scope(const Scope&) = delete;
    Scope& operator=(const Scope&) = delete;
    // Moves recorded lifecycle events (report.events) and hook messages (report.warnings) into the report.
    void finish(json& report);
};

} // namespace cs::hooks

extern "C" {
// Re-entrant graph access, valid only while Module.csHook("pipeline", ...) is running (returns -1
// otherwise). Same buffer conventions as cs_tool: request JSON + blob in, JSON + blob out
// (engine-allocated, release with cs_free). rc 0 = every op succeeded, 1 = an op failed (see JSON).
int cs_hook_query(const char* req_json, int req_len, const uint8_t* blob, int blob_len,
                  char** out_json, int* out_json_len, uint8_t** out_blob, int* out_blob_len);
}
