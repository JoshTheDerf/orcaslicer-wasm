// SLA hollowing / drain holes without OpenVDB (replaces SLA/Hollowing.cpp, see
// wasm/CMakeLists.txt). The engine slices FFF only; SLAPrint is compiled but
// never run, so hollowing is a no-op and drain holes have no geometry.
#include "libslic3r/SLA/Hollowing.hpp"

namespace Slic3r { namespace sla {

struct Interior { indexed_triangle_set mesh; };

void InteriorDeleter::operator()(Interior *p) { delete p; }
indexed_triangle_set &get_mesh(Interior &interior) { return interior.mesh; }
const indexed_triangle_set &get_mesh(const Interior &interior) { return interior.mesh; }

InteriorPtr generate_interior(const TriangleMesh &, const HollowingConfig &, const JobController &) { return {}; }
void hollow_mesh(TriangleMesh &, const HollowingConfig &, int) {}
void hollow_mesh(TriangleMesh &, const Interior &, int) {}
void remove_inside_triangles(TriangleMesh &, const Interior &, const std::vector<bool> &) {}
double get_distance(const Vec3f &, const Interior &) { return 0.; }
void cut_drainholes(std::vector<ExPolygons> &, const std::vector<float> &, float, const DrainHoles &, std::function<void(void)>) {}

bool DrainHole::operator==(const DrainHole &sp) const
{
    return pos == sp.pos && normal == sp.normal && is_approx(radius, sp.radius) && is_approx(height, sp.height);
}
bool DrainHole::is_inside(const Vec3f &) const { return false; }
bool DrainHole::get_intersections(const Vec3f &, const Vec3f &, std::array<std::pair<float, Vec3d>, 2> &) const { return false; }
indexed_triangle_set DrainHole::to_mesh() const { return {}; }

}} // namespace Slic3r::sla
