// The two sketch-tolerance functions of libslic3r/CAD/SketchEngine.cpp that the
// OCCT-free CAD sources (SketchSolver, SketchInference) call. SketchEngine.cpp
// itself is the OCCT B-rep kernel and is not built; same definitions as upstream.
#include "libslic3r/CAD/SketchEngine.hpp"

#include <atomic>

namespace Slic3r {

static std::atomic<bool> s_auto_close{true};
double sketch_join_tol() { return s_auto_close.load() ? kSketchJoinTol : 0.0; }
void   set_sketch_auto_close(bool on) { s_auto_close.store(on); }

} // namespace Slic3r
