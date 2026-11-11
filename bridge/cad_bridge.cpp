// Design-tab CAD kernel bridge (cad.mjs / cad.wasm) — OrcaSlicer 2.5's libslic3r/CAD on
// OpenCASCADE 8.0.1, driven from a web UI through ONE stateful JSON dispatcher:
//
//   int cad_call(method, req_json, blob) -> out_json (+ out_blob)
//
// Contract (every method, its JSON, the blob layouts, id semantics, error codes):
//   cubby-slicer/docs/CAD-KERNEL-CONTRACT.md
//
// Documents live in this module between calls. Method names and argument shapes follow Orca's
// own control surface (src/slic3r/GUI/CAD/McpControl.cpp, whose handlers are ported here with the
// GUI panel replaced by the document), plus what that surface lacks: generic feature editing,
// history edits, constraints, meshes with face/edge ids, recipe I/O that keeps unknown bytes.
//
// Build invariants: cubby-slicer/docs/ENGINE-CONTRACT.md (native Wasm EH, single-threaded, no
// function-pointer-cast emulation). OCCT's thread pool is pinned to the calling thread in
// cad_init() so its "parallel" loops run inline.
#include "cs_common.hpp"

#include <libslic3r/CAD/CadDocument.hpp>
#include <libslic3r/CAD/GeometryEngine.hpp>
#include <libslic3r/CAD/SketchEngine.hpp>
#include <libslic3r/CAD/SketchInference.hpp>
#include <libslic3r/CAD/SketchSolver.hpp>
#include <libslic3r/CAD/ThreadStandards.hpp>
#include <libslic3r/BoundingBox.hpp>
#include <libslic3r/TriangleMesh.hpp>

#include <cereal/archives/binary.hpp>
#include <cereal/types/array.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepGProp.hxx>
#include <BRepLib_ToolTriangulatedShape.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_IndexedMap.hxx>
#include <NCollection_List.hxx>
#include <OSD_ThreadPool.hxx>
#include <Poly_PolygonOnTriangulation.hxx>
#include <Poly_Triangulation.hxx>
#include <STEPControl_StepModelType.hxx>
#include <STEPControl_Writer.hxx>
#include <Standard_Failure.hxx>
#include <Standard_Version.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <gp_Pln.hxx>

#include <algorithm>
#include <deque>
#include <set>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unistd.h>
#include <vector>

#ifndef __EMSCRIPTEN__
#define EMSCRIPTEN_KEEPALIVE
#endif

using namespace Slic3r;
using json = nlohmann::json;

// Rebuild progress for the host: Module.cadProgress(featureIndex, featureCount, name, cancellable)
// before every replayed feature and once with featureIndex == featureCount before the finishing
// passes and tessellation. A truthy return asks to cancel (honoured only when `cancellable`).
EM_JS(int, cad_progress_js, (int index, int count, const char* name, int cancellable), {
    var f = Module["cadProgress"];
    if (typeof f !== "function") return 0;
    try { return f(index, count, UTF8ToString(name), !!cancellable) ? 1 : 0; } catch (e) { return 0; }
});

namespace {

// ---- errors ---------------------------------------------------------------------------------
enum ErrCode {
    kUnknownMethod = -32601,
    kInvalidParams = -32602,
    kFailure       = -32000,
    kStale         = -32010,
    kReadOnly      = -32020,
    kNoDocument    = -32021,
    kSketchState   = -32022,
    kUnreadable    = -32030,
    kNotARecipe    = -32031,
};
struct CadError : std::runtime_error {
    int code;
    CadError(int c, const std::string& m) : std::runtime_error(m), code(c) {}
};
[[noreturn]] void fail(int code, const std::string& msg) { throw CadError(code, msg); }
[[noreturn]] void bad_param(const std::string& msg) { throw CadError(kInvalidParams, msg); }

using ShapeIndexMap = NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher>;
using AncestorMap   = NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

// ---- enum names (ordinal order = Orca's wire format; append-only upstream) ------------------
const char* const kFeatureTypes[] = {
    "Sketch", "Extrude", "Fillet", "Chamfer", "Hole", "Thread", "Shell", "Revolve", "Sweep", "Pattern",
    "Plane", "Loft", "Draft", "Import", "Boolean", "Cut", "Mirror", "Axis", "CoordSys", "Helix",
    "Transform", "Thicken", "Project", "DeleteFace", "Rib", "SurfaceExtrude", "SurfaceRevolve",
    "ThickenSurface", "SurfaceOffset", "SurfaceLoft", "SurfaceFill", "Mate"};
const char* const kShapes[]      = {"Rectangle", "Circle"};
const char* const kPlaneTypes[]  = {"Offset", "Angle", "Midplane", "Tangent", "TwoEdges", "Coincident"};
const char* const kAxisTypes[]   = {"TwoPoints", "FaceNormal", "CylinderCenterline", "PlaneIntersection", "AlongEdge"};
const char* const kCsTypes[]     = {"PointWorld", "FaceAndDirection"};
const char* const kBoolModes[]   = {"New", "Add", "Cut", "Intersect"};
const char* const kExtrudeEnds[] = {"Blind", "Symmetric", "TwoSided", "ThroughAll", "UpToFace", "UpToVertex"};
const char* const kFaceGroups[]  = {"Top", "Bottom", "Lateral", "All"};
const char* const kEntityTypes[] = {"line", "arc", "circle", "point", "ellipse", "ellipse_arc", "spline"};
const char* const kRoles[]       = {"p0", "p1", "center"};
const char* const kConstraintTypes[] = {
    "fix", "coincident", "horizontal", "vertical", "distance", "lock_x", "lock_y", "equal_length", "parallel",
    "perpendicular", "concentric", "tangent", "midpoint", "symmetric", "angle", "radius", "diameter",
    "point_on_line", "point_on_object", "equal_radius", "collinear", "distance_x", "distance_y",
    "symmetric_about_y", "symmetric_about_x"};
// The slicer engine's cs_tool "sketch" spellings, accepted on input.
const char* const kConstraintTypesCamel[] = {
    "fix", "coincident", "horizontal", "vertical", "distance", "lockX", "lockY", "equalLength", "parallel",
    "perpendicular", "concentric", "tangent", "midpoint", "symmetric", "angle", "radius", "diameter",
    "pointOnLine", "pointOnObject", "equalRadius", "collinear", "distanceX", "distanceY",
    "symmetricAboutY", "symmetricAboutX"};
constexpr int kNumFeatureTypes = int(sizeof(kFeatureTypes) / sizeof(kFeatureTypes[0]));
static_assert(kNumFeatureTypes == int(CadFeatureType::Mate) + 1, "feature type table out of date");
static_assert(int(sizeof(kConstraintTypes) / sizeof(kConstraintTypes[0])) == int(SketchConstraintType::SymmetricAboutX) + 1,
              "constraint type table out of date");

template<size_t N> int name_index(const char* const (&names)[N], const json& j, const char* what)
{
    if (j.is_number_integer()) {
        const long long v = j.get<long long>();
        if (v >= 0 && size_t(v) < N) return int(v);
    } else if (j.is_string()) {
        const std::string s = j.get<std::string>();
        for (size_t i = 0; i < N; ++i) if (s == names[i]) return int(i);
        // case-insensitive second chance ("extrude", "new", "blind")
        for (size_t i = 0; i < N; ++i) {
            const char* n = names[i];
            if (s.size() != std::strlen(n)) continue;
            bool eq = true;
            for (size_t k = 0; k < s.size() && eq; ++k) eq = std::tolower((unsigned char)s[k]) == std::tolower((unsigned char)n[k]);
            if (eq) return int(i);
        }
    }
    bad_param(std::string("unknown ") + what + " " + j.dump());
}
int constraint_type_index(const json& j)
{
    if (j.is_string()) {
        const std::string s = j.get<std::string>();
        for (size_t i = 0; i < std::size(kConstraintTypesCamel); ++i) if (s == kConstraintTypesCamel[i]) return int(i);
    }
    return name_index(kConstraintTypes, j, "constraint type");
}

const char* feature_type_name(CadFeatureType t) { return kFeatureTypes[int(t)]; }

// ---- small JSON helpers -----------------------------------------------------------------------
json vec2_json(const Vec2d& v) { return json::array({v.x(), v.y()}); }
json vec3_json(const Vec3d& v) { return json::array({v.x(), v.y(), v.z()}); }
inline json vec3(const Vec3d& v) { return vec3_json(v); }

double num(const json& j, const char* what)
{
    if (!j.is_number()) bad_param(std::string(what) + " must be a number");
    const double v = j.get<double>();
    if (!std::isfinite(v)) bad_param(std::string(what) + " must be finite");
    return v;
}
Vec2d vec2_from(const json& j, const char* what)
{
    if (!j.is_array() || j.size() != 2) bad_param(std::string(what) + " must be [x, y]");
    return Vec2d(num(j[0], what), num(j[1], what));
}
Vec3d vec3_from(const json& j, const char* what)
{
    if (!j.is_array() || j.size() != 3) bad_param(std::string(what) + " must be [x, y, z]");
    return Vec3d(num(j[0], what), num(j[1], what), num(j[2], what));
}
int int_param(const json& p, const char* key)
{
    auto it = p.find(key);
    if (it == p.end()) bad_param(std::string("missing '") + key + "'");
    if (!it->is_number_integer()) bad_param(std::string("'") + key + "' must be an integer");
    return it->get<int>();
}
template<class T> T opt(const json& p, const char* key, T dflt)
{
    auto it = p.find(key);
    if (it == p.end() || it->is_null()) return dflt;
    try { return it->get<T>(); }
    catch (const json::exception&) { bad_param(std::string("'") + key + "' has the wrong type"); }
}

json plane_json(const SketchPlane& p)
{
    return json{{"origin", vec3_json(p.origin)}, {"normal", vec3_json(p.normal)},
                {"x_axis", vec3_json(p.x_axis)}, {"y_axis", vec3_json(p.y_axis)}};
}
SketchPlane plane_from_json(const json& j, const Vec3d& origin)
{
    if (j.is_string()) {
        const std::string n = j.get<std::string>();
        SketchPlane pl;
        if (n == "XY") pl = SketchPlane::XY();
        else if (n == "XZ") pl = SketchPlane::XZ();
        else if (n == "YZ") pl = SketchPlane::YZ();
        else bad_param("plane must be XY, XZ, YZ or a plane frame object");
        pl.origin = origin;   // the default planes pass through the modeling origin (McpControl plane_from)
        return pl;
    }
    if (!j.is_object()) bad_param("plane must be XY, XZ, YZ or {origin, normal, x_axis, y_axis}");
    SketchPlane pl;
    pl.origin = vec3_from(j.value("origin", json::array({0, 0, 0})), "plane.origin");
    pl.normal = vec3_from(j.value("normal", json::array({0, 0, 1})), "plane.normal");
    pl.x_axis = vec3_from(j.value("x_axis", json::array({1, 0, 0})), "plane.x_axis");
    pl.y_axis = vec3_from(j.value("y_axis", json::array({0, 1, 0})), "plane.y_axis");
    return pl;
}

// ---- sketch entity / constraint JSON ----------------------------------------------------------
json entity_json(const SketchEntity& e)
{
    json ctrl = json::array();
    for (const Vec2d& c : e.ctrl) ctrl.push_back(vec2_json(c));
    return json{{"type", kEntityTypes[int(e.type)]}, {"p0", vec2_json(e.p0)}, {"p1", vec2_json(e.p1)},
                {"center", vec2_json(e.center)}, {"radius", e.radius}, {"start_angle", e.start_angle},
                {"end_angle", e.end_angle}, {"construction", e.construction}, {"rminor", e.rminor},
                {"rotation", e.rotation}, {"ctrl", ctrl}};
}
json entities_json(const std::vector<SketchEntity>& es)
{
    json a = json::array();
    for (const SketchEntity& e : es) a.push_back(entity_json(e));
    return a;
}
// Lossless when every field is given (our own output); McpControl's shorthands otherwise:
// point {p}, circle/arc {centre|center|p0, radius}, arc ends derived from centre+angles.
SketchEntity entity_from_json(const json& j)
{
    if (!j.is_object()) bad_param("entity must be an object");
    SketchEntity e;
    json t = j.value("type", json("line"));
    if (t.is_string()) {
        const std::string s = t.get<std::string>();
        if (s == "bspline" || s == "BSpline") t = "spline";
        else if (s == "ellipseArc" || s == "EllipseArc") t = "ellipse_arc";
    }
    e.type = SketchEntity::Type(name_index(kEntityTypes, t, "entity type"));
    auto v2 = [&](const char* k, Vec2d d) { auto it = j.find(k); return it == j.end() ? d : vec2_from(*it, k); };
    e.p0 = v2("p0", Vec2d::Zero());
    e.p1 = v2("p1", Vec2d::Zero());
    e.center = v2("center", v2("centre", Vec2d::Zero()));
    e.radius = j.contains("radius") ? num(j["radius"], "radius") : 0.0;
    e.start_angle = j.contains("start_angle") ? num(j["start_angle"], "start_angle") : j.value("startAngle", 0.0);
    e.end_angle = j.contains("end_angle") ? num(j["end_angle"], "end_angle") : j.value("endAngle", 0.0);
    e.construction = opt<bool>(j, "construction", false);
    e.rminor = j.contains("rminor") ? num(j["rminor"], "rminor") : 0.0;
    e.rotation = j.contains("rotation") ? num(j["rotation"], "rotation") : 0.0;
    if (auto it = j.find("ctrl"); it != j.end()) {
        if (!it->is_array()) bad_param("ctrl must be an array of [x, y]");
        for (const json& c : *it) e.ctrl.push_back(vec2_from(c, "ctrl"));
    }
    const bool has_center = j.contains("center") || j.contains("centre");
    switch (e.type) {
    case SketchEntity::Type::Point:
        if (j.contains("p")) e.p0 = vec2_from(j["p"], "p");
        break;
    case SketchEntity::Type::Circle:
        if (!has_center && j.contains("p0")) e.center = e.p0;
        if (!has_center && !j.contains("p0")) bad_param("circle needs a 'center' (or 'p0')");
        if (!(e.radius > 0)) bad_param("circle needs a positive 'radius'");
        e.p0 = e.center;
        break;
    case SketchEntity::Type::Arc:
        if (!has_center) bad_param("arc needs a 'center'");
        if (!(e.radius > 0)) bad_param("arc needs a positive 'radius'");
        if (!j.contains("p0")) e.p0 = e.center + e.radius * Vec2d(std::cos(e.start_angle), std::sin(e.start_angle));
        if (!j.contains("p1")) e.p1 = e.center + e.radius * Vec2d(std::cos(e.end_angle), std::sin(e.end_angle));
        break;
    case SketchEntity::Type::Ellipse:
    case SketchEntity::Type::EllipseArc:
        if (!has_center) bad_param("ellipse needs a 'center'");
        if (!(e.radius > 0) || !(e.rminor > 0)) bad_param("ellipse needs positive 'radius' and 'rminor'");
        if (e.type == SketchEntity::Type::Ellipse) e.p0 = e.center;
        else {   // v1.2: ends derived from centre / radii / rotation / angles, as for plain arcs
            auto at = [&](double t) { const double c = std::cos(e.rotation), sn = std::sin(e.rotation);
                const Vec2d q(e.radius * std::cos(t), e.rminor * std::sin(t)); return Vec2d(e.center.x() + c * q.x() - sn * q.y(), e.center.y() + sn * q.x() + c * q.y()); };
            if (!j.contains("p0")) e.p0 = at(e.start_angle);
            if (!j.contains("p1")) e.p1 = at(e.end_angle);
        }
        break;
    case SketchEntity::Type::BSpline:
        if (e.ctrl.size() < 2) bad_param("spline needs at least 2 'ctrl' poles");
        e.p0 = e.ctrl.front(); e.p1 = e.ctrl.back();
        break;
    case SketchEntity::Type::Line:
        break;
    }
    return e;
}
std::vector<SketchEntity> entities_from_json(const json& a)
{
    if (!a.is_array()) bad_param("entities must be an array");
    std::vector<SketchEntity> out;
    for (const json& j : a) out.push_back(entity_from_json(j));
    return out;
}
// Angle constraints: the model (and Orca's solver, SketchSolver.cpp) stores radians; the wire is
// degrees (v1.2, the contract's unit; every other angle a user types is degrees).
double angle_wire(const SketchEntityConstraintDef& c) { return c.type == SketchConstraintType::Angle ? c.value * 180.0 / M_PI : c.value; }
json constraint_json(const SketchEntityConstraintDef& c)
{
    return json{{"type", kConstraintTypes[int(c.type)]}, {"ea", c.ea}, {"ra", kRoles[int(c.ra)]},
                {"eb", c.eb}, {"rb", kRoles[int(c.rb)]}, {"value", angle_wire(c)}, {"ec", c.ec}, {"rc", kRoles[int(c.rc)]}};
}
json constraints_json(const std::vector<SketchEntityConstraintDef>& cs)
{
    json a = json::array();
    for (const auto& c : cs) a.push_back(constraint_json(c));
    return a;
}
SketchEntityConstraintDef constraint_from_json(const json& j)
{
    if (!j.is_object()) bad_param("constraint must be an object");
    SketchEntityConstraintDef c;
    c.type = SketchConstraintType(constraint_type_index(j.value("type", json("coincident"))));
    c.ea = opt<int>(j, "ea", -1); c.eb = opt<int>(j, "eb", -1); c.ec = opt<int>(j, "ec", -1);
    c.ra = SketchPointRole(name_index(kRoles, j.value("ra", json("p0")), "role"));
    c.rb = SketchPointRole(name_index(kRoles, j.value("rb", json("p0")), "role"));
    c.rc = SketchPointRole(name_index(kRoles, j.value("rc", json("p0")), "role"));
    c.value = j.contains("value") ? num(j["value"], "value") : 0.0;
    if (c.type == SketchConstraintType::Angle) c.value *= M_PI / 180.0;   // wire degrees → model radians
    return c;
}
std::vector<SketchEntityConstraintDef> constraints_from_json(const json& a)
{
    if (!a.is_array()) bad_param("constraints must be an array");
    std::vector<SketchEntityConstraintDef> out;
    for (const json& j : a) out.push_back(constraint_from_json(j));
    return out;
}
void check_constraint_refs(const std::vector<SketchEntityConstraintDef>& cs, size_t n_ents)
{
    for (size_t i = 0; i < cs.size(); ++i)
        for (int e : {cs[i].ea, cs[i].eb, cs[i].ec})
            if (e >= int(n_ents) || (e < 0 && e != -1 && !is_sketch_ref(e)))
                bad_param("constraint " + std::to_string(i) + " references entity " + std::to_string(e) +
                          " (have " + std::to_string(n_ents) + ")");
}

// ---- CadFeature <-> JSON: one visitor over every serialized field, by its C++ name ------------
// Order = CadFeature::save() (so describe_tools lists them in recipe order).
template<class V> void visit_fields(CadFeature& f, V&& v)
{
    v("type", f.type); v("name", f.name); v("enabled", f.enabled); v("shape", f.shape); v("plane", f.plane);
    v("width", f.width); v("height", f.height); v("radius", f.radius);
    v("profile", f.profile); v("entities", f.entities); v("constraints", f.constraints);
    v("entity_constraints", f.entity_constraints); v("imported_regions", f.imported_regions);
    v("import_offset", f.import_offset); v("import_scale_x", f.import_scale_x); v("import_scale_y", f.import_scale_y);
    v("import_on_face", f.import_on_face); v("import_face_body", f.import_face_body);
    v("sketch_ref", f.sketch_ref); v("distance", f.distance); v("symmetric", f.symmetric); v("mode", f.mode);
    v("extrude_end", f.extrude_end); v("distance2", f.distance2); v("taper_deg", f.taper_deg); v("flip", f.flip);
    v("up_to_face", f.up_to_face); v("extrude_src_face", f.extrude_src_face); v("up_to_point", f.up_to_point);
    v("target_body", f.target_body);
    v("dressup_size", f.dressup_size); v("face_group", f.face_group); v("dressup_edge", f.dressup_edge);
    v("hole_diameter", f.hole_diameter); v("hole_depth", f.hole_depth); v("hole_through", f.hole_through);
    v("hole_x", f.hole_x); v("hole_y", f.hole_y);
    v("thread_radius", f.thread_radius); v("thread_pitch", f.thread_pitch); v("thread_height", f.thread_height);
    v("thread_depth", f.thread_depth); v("thread_internal", f.thread_internal); v("thread_x", f.thread_x);
    v("thread_y", f.thread_y);
    v("shell_thickness", f.shell_thickness); v("shell_face", f.shell_face);
    v("draft_face", f.draft_face); v("draft_angle", f.draft_angle);
    v("revolve_angle", f.revolve_angle); v("revolve_axis", f.revolve_axis);
    v("sweep_path_ref", f.sweep_path_ref); v("loft_profile_refs", f.loft_profile_refs); v("loft_ruled", f.loft_ruled);
    v("pattern_circular", f.pattern_circular); v("pattern_count", f.pattern_count);
    v("pattern_spacing", f.pattern_spacing); v("pattern_dir", f.pattern_dir); v("pattern_angle", f.pattern_angle);
    v("plane_base", f.plane_base); v("plane_offset", f.plane_offset); v("plane_angle_tilt", f.plane_angle_tilt);
    v("plane_axis", f.plane_axis);
    v("bool_tool_body", f.bool_tool_body); v("bool_keep_tool", f.bool_keep_tool); v("bool_tolerance", f.bool_tolerance);
    v("bool_target_face", f.bool_target_face); v("bool_tool_face", f.bool_tool_face);
    v("cut_offset", f.cut_offset); v("cut_flip", f.cut_flip); v("cut_keep_upper", f.cut_keep_upper);
    v("cut_keep_lower", f.cut_keep_lower);
    v("plane_type", f.plane_type); v("plane_face_body", f.plane_face_body); v("plane_face", f.plane_face);
    v("plane_face2_body", f.plane_face2_body); v("plane_face2", f.plane_face2); v("plane_edge_body", f.plane_edge_body);
    v("plane_edge", f.plane_edge); v("plane_edge2_body", f.plane_edge2_body); v("plane_edge2", f.plane_edge2);
    v("plane_u_size", f.plane_u_size); v("plane_v_size", f.plane_v_size);
    v("mirror_keep_original", f.mirror_keep_original);
    v("axis_type", f.axis_type); v("axis_p1", f.axis_p1); v("axis_p2", f.axis_p2); v("axis_body", f.axis_body);
    v("axis_face", f.axis_face); v("axis_edge", f.axis_edge); v("axis_plane_a", f.axis_plane_a);
    v("axis_plane_b", f.axis_plane_b);
    v("coordsys_type", f.coordsys_type); v("coordsys_point", f.coordsys_point); v("coordsys_body", f.coordsys_body);
    v("coordsys_face", f.coordsys_face); v("coordsys_edge", f.coordsys_edge); v("coordsys_x_hint", f.coordsys_x_hint);
    v("helix_radius", f.helix_radius); v("helix_pitch", f.helix_pitch); v("helix_height", f.helix_height);
    v("helix_left_handed", f.helix_left_handed); v("helix_taper_deg", f.helix_taper_deg);
    v("xf_translate", f.xf_translate); v("xf_axis", f.xf_axis); v("xf_pivot", f.xf_pivot);
    v("xf_angle_deg", f.xf_angle_deg); v("xf_copy", f.xf_copy);
    v("thicken_face", f.thicken_face); v("thicken_thickness", f.thicken_thickness); v("thicken_flip", f.thicken_flip);
    v("cut_face_body", f.cut_face_body); v("cut_face", f.cut_face);
    v("project_source_body", f.project_source_body); v("project_edges", f.project_edges); v("project_face", f.project_face);
    v("delete_faces", f.delete_faces);
    v("hole_style", f.hole_style); v("hole_cbore_diameter", f.hole_cbore_diameter); v("hole_cbore_depth", f.hole_cbore_depth);
    v("hole_csink_diameter", f.hole_csink_diameter); v("hole_csink_angle", f.hole_csink_angle);
    v("hole_standard", f.hole_standard);
    v("rib_sketch_ref", f.rib_sketch_ref); v("rib_entity", f.rib_entity); v("rib_thickness", f.rib_thickness);
    v("rib_depth", f.rib_depth);
    v("pattern_curve_sketch", f.pattern_curve_sketch); v("pattern_curve_entity", f.pattern_curve_entity);
    v("expr", f.expr);
    v("mate_kind", f.mate_kind); v("mate_cs_a", f.mate_cs_a); v("mate_cs_b", f.mate_cs_b);
    v("mate_offset", f.mate_offset); v("mate_angle", f.mate_angle); v("mate_flip", f.mate_flip);
    v("coordsys_face_kind", f.coordsys_face_kind); v("coordsys_face_edges", f.coordsys_face_edges);
    v("thread_major_nominal", f.thread_major_nominal); v("pattern_inclusive", f.pattern_inclusive);
    v("dressup_edges", f.dressup_edges);
    v("text_string", f.text_string); v("text_font", f.text_font); v("text_height", f.text_height);
    v("revolve_axis_entity", f.revolve_axis_entity);
}

// Names of an enum's values, for the visitors.
template<class E> struct EnumNames;
#define CAD_ENUM_NAMES(E, TABLE) template<> struct EnumNames<E> { static constexpr const auto& names = TABLE; };
CAD_ENUM_NAMES(CadFeatureType, kFeatureTypes)
CAD_ENUM_NAMES(SketchShape, kShapes)
CAD_ENUM_NAMES(PlaneType, kPlaneTypes)
CAD_ENUM_NAMES(AxisType, kAxisTypes)
CAD_ENUM_NAMES(CoordSysType, kCsTypes)
CAD_ENUM_NAMES(BooleanMode, kBoolModes)
CAD_ENUM_NAMES(ExtrudeEnd, kExtrudeEnds)
CAD_ENUM_NAMES(FaceGroup, kFaceGroups)
#undef CAD_ENUM_NAMES

json regions_json(const std::vector<std::vector<std::vector<Vec2d>>>& rs)
{
    json a = json::array();
    for (const auto& r : rs) {
        json loops = json::array();
        for (const auto& loop : r) { json l = json::array(); for (const Vec2d& p : loop) l.push_back(vec2_json(p)); loops.push_back(std::move(l)); }
        a.push_back(std::move(loops));
    }
    return a;
}


struct FieldToJson {
    json& out;
    void operator()(const char* k, int& v) { out[k] = v; }
    void operator()(const char* k, double& v) { out[k] = v; }
    void operator()(const char* k, bool& v) { out[k] = v; }
    void operator()(const char* k, std::string& v) { out[k] = v; }
    void operator()(const char* k, Vec2d& v) { out[k] = vec2_json(v); }
    void operator()(const char* k, Vec3d& v) { out[k] = vec3_json(v); }
    void operator()(const char* k, SketchPlane& v) { out[k] = plane_json(v); }
    void operator()(const char* k, std::vector<int>& v) { out[k] = v; }
    void operator()(const char* k, std::map<std::string, std::string>& v) { out[k] = v; }
    void operator()(const char* k, SketchProfile& v)
    {
        json pts = json::array();
        for (const Vec2d& p : v.points) pts.push_back(vec2_json(p));
        out[k] = json{{"points", pts}, {"closed", v.closed}};
    }
    void operator()(const char* k, std::vector<SketchEntity>& v) { out[k] = entities_json(v); }
    void operator()(const char* k, std::vector<SketchConstraintDef>& v)
    {
        json a = json::array();
        for (const auto& c : v) a.push_back(json{{"type", kConstraintTypes[int(c.type)]}, {"a", c.a}, {"b", c.b},
                                                 {"c", c.c}, {"d", c.d}, {"value", c.value}});
        out[k] = a;
    }
    void operator()(const char* k, std::vector<SketchEntityConstraintDef>& v) { out[k] = constraints_json(v); }
    void operator()(const char* k, std::vector<std::vector<std::vector<Vec2d>>>& v) { out[k] = regions_json(v); }
    template<class E, class = std::enable_if_t<std::is_enum_v<E>>> void operator()(const char* k, E& v)
    {
        out[k] = EnumNames<E>::names[int(v)];
    }
};

struct FieldFromJson {
    const json& in;
    size_t used{0};
    const json* get(const char* k) { auto it = in.find(k); if (it == in.end()) return nullptr; ++used; return &*it; }
    void operator()(const char* k, int& v)
    {
        if (const json* j = get(k)) {
            if (j->is_number_integer()) v = j->get<int>();
            else if (j->is_number() && std::floor(j->get<double>()) == j->get<double>()) v = int(j->get<double>());
            else bad_param(std::string(k) + " must be an integer");
        }
    }
    void operator()(const char* k, double& v) { if (const json* j = get(k)) v = num(*j, k); }
    void operator()(const char* k, bool& v)
    {
        if (const json* j = get(k)) { if (!j->is_boolean()) bad_param(std::string(k) + " must be a boolean"); v = j->get<bool>(); }
    }
    void operator()(const char* k, std::string& v)
    {
        if (const json* j = get(k)) { if (!j->is_string()) bad_param(std::string(k) + " must be a string"); v = j->get<std::string>(); }
    }
    void operator()(const char* k, Vec2d& v) { if (const json* j = get(k)) v = vec2_from(*j, k); }
    void operator()(const char* k, Vec3d& v) { if (const json* j = get(k)) v = vec3_from(*j, k); }
    void operator()(const char* k, SketchPlane& v) { if (const json* j = get(k)) v = plane_from_json(*j, origin); }
    void operator()(const char* k, std::vector<int>& v)
    {
        if (const json* j = get(k)) {
            if (!j->is_array()) bad_param(std::string(k) + " must be an array of integers");
            v.clear();
            for (const json& e : *j) { if (!e.is_number_integer()) bad_param(std::string(k) + " must be an array of integers"); v.push_back(e.get<int>()); }
        }
    }
    void operator()(const char* k, std::map<std::string, std::string>& v)
    {
        if (const json* j = get(k)) {
            if (!j->is_object()) bad_param(std::string(k) + " must be an object of strings");
            v.clear();
            for (auto it = j->begin(); it != j->end(); ++it) {
                if (!it->is_string()) bad_param(std::string(k) + " must be an object of strings");
                v[it.key()] = it->get<std::string>();
            }
        }
    }
    void operator()(const char* k, SketchProfile& v)
    {
        if (const json* j = get(k)) {
            if (!j->is_object()) bad_param("profile must be {points, closed}");
            v.points.clear();
            for (const json& p : j->value("points", json::array())) v.points.push_back(vec2_from(p, "profile.points"));
            v.closed = j->value("closed", false);
        }
    }
    void operator()(const char* k, std::vector<SketchEntity>& v) { if (const json* j = get(k)) v = entities_from_json(*j); }
    void operator()(const char* k, std::vector<SketchConstraintDef>& v)
    {
        if (const json* j = get(k)) {
            if (!j->is_array()) bad_param("constraints must be an array");
            v.clear();
            for (const json& c : *j) {
                SketchConstraintDef d;
                d.type = SketchConstraintType(constraint_type_index(c.value("type", json("coincident"))));
                d.a = c.value("a", -1); d.b = c.value("b", -1); d.c = c.value("c", -1); d.d = c.value("d", -1);
                d.value = c.value("value", 0.0);
                v.push_back(d);
            }
        }
    }
    void operator()(const char* k, std::vector<SketchEntityConstraintDef>& v) { if (const json* j = get(k)) v = constraints_from_json(*j); }
    void operator()(const char* k, std::vector<std::vector<std::vector<Vec2d>>>& v)
    {
        if (const json* j = get(k)) {
            if (!j->is_array()) bad_param("imported_regions must be [region[loop[[x,y]…]]]");
            v.clear();
            for (const json& r : *j) {
                std::vector<std::vector<Vec2d>> region;
                for (const json& l : r) { std::vector<Vec2d> loop; for (const json& p : l) loop.push_back(vec2_from(p, "imported_regions")); region.push_back(std::move(loop)); }
                v.push_back(std::move(region));
            }
        }
    }
    template<class E, class = std::enable_if_t<std::is_enum_v<E>>> void operator()(const char* k, E& v)
    {
        if (const json* j = get(k)) v = E(name_index(EnumNames<E>::names, *j, k));
    }
    Vec3d origin{Vec3d::Zero()};
};

struct FieldDescribe {
    json& out;
    void put(const char* k, const char* t, json extra = json()) { json f{{"name", k}, {"type", t}}; if (!extra.is_null()) f["enum"] = extra; out.push_back(f); }
    void operator()(const char* k, int&) { put(k, "int"); }
    void operator()(const char* k, double&) { put(k, "number"); }
    void operator()(const char* k, bool&) { put(k, "bool"); }
    void operator()(const char* k, std::string&) { put(k, "string"); }
    void operator()(const char* k, Vec2d&) { put(k, "vec2"); }
    void operator()(const char* k, Vec3d&) { put(k, "vec3"); }
    void operator()(const char* k, SketchPlane&) { put(k, "plane"); }
    void operator()(const char* k, std::vector<int>&) { put(k, "int[]"); }
    void operator()(const char* k, std::map<std::string, std::string>&) { put(k, "map<string,string>"); }
    void operator()(const char* k, SketchProfile&) { put(k, "profile"); }
    void operator()(const char* k, std::vector<SketchEntity>&) { put(k, "entity[]"); }
    void operator()(const char* k, std::vector<SketchConstraintDef>&) { put(k, "profile_constraint[]"); }
    void operator()(const char* k, std::vector<SketchEntityConstraintDef>&) { put(k, "constraint[]"); }
    void operator()(const char* k, std::vector<std::vector<std::vector<Vec2d>>>&) { put(k, "regions"); }
    template<class E, class = std::enable_if_t<std::is_enum_v<E>>> void operator()(const char* k, E&)
    {
        json names = json::array();
        for (const char* n : EnumNames<E>::names) names.push_back(n);
        put(k, "enum", names);
    }
};

json feature_json(const CadFeature& cf, bool include_brep)
{
    CadFeature& f = const_cast<CadFeature&>(cf);   // the visitor is shared with the writer; read only here
    json out = json::object();
    visit_fields(f, FieldToJson{out});
    std::string brep;
    if (f.type == CadFeatureType::Import) brep = brep_to_string(f.imported_solid);
    out["brep_bytes"] = brep.size();
    if (include_brep) out["brep"] = brep;
    return out;
}

// Apply `fields` onto `f`. `type` cannot change here (a different feature is a new feature).
void apply_fields(CadFeature& f, const json& fields, const Vec3d& origin, bool allow_type)
{
    if (!fields.is_object()) bad_param("fields must be an object");
    const CadFeatureType before = f.type;
    FieldFromJson r{fields};
    r.origin = origin;
    visit_fields(f, r);
    size_t extra = 0;
    if (fields.contains("brep")) {
        if (f.type != CadFeatureType::Import) bad_param("'brep' is only valid on an Import feature");
        if (!fields["brep"].is_string()) bad_param("brep must be a string (BRepTools text)");
        f.imported_solid = brep_from_string(fields["brep"].get<std::string>());
        if (f.imported_solid.IsNull()) bad_param("brep could not be read");
        ++extra;
    }
    if (fields.contains("brep_bytes")) ++extra;   // our own output echoed back: ignored
    if (r.used + extra != fields.size()) {
        json probe = json::object();
        CadFeature tmp; visit_fields(tmp, FieldToJson{probe});
        for (auto it = fields.begin(); it != fields.end(); ++it)
            if (!probe.contains(it.key()) && it.key() != "brep" && it.key() != "brep_bytes")
                bad_param("unknown feature field '" + it.key() + "'");
    }
    if (!allow_type && f.type != before) bad_param("a feature's type cannot be changed (delete it and add a new one)");
}

// ---- recipe framing: unknown bytes a newer Orca appended ----------------------------------------
struct Tails {
    std::vector<std::string> feat;   // per feature: frame bytes after the last field this build reads
    std::string vars, names, extras; // same for the three document blocks
    std::string trailing;            // bytes after the last known block
    size_t feature_bytes() const { size_t n = 0; for (const auto& s : feat) n += s.size(); return n; }
    size_t block_bytes() const { return vars.size() + names.size() + extras.size(); }
    bool any() const { return feature_bytes() + block_bytes() + trailing.size() > 0; }
};

struct ByteReader {
    const std::string& s; size_t pos{0};
    bool u32(uint32_t& v) { if (pos + 4 > s.size()) return false; std::memcpy(&v, s.data() + pos, 4); pos += 4; return true; }
    bool take(size_t n, std::string& out) { if (pos + n > s.size()) return false; out.assign(s, pos, n); pos += n; return true; }
};

template<class F> size_t consumed_by(const std::string& buf, F&& read)
{
    std::istringstream is(buf);
    try {
        cereal::BinaryInputArchive ar(is);
        read(ar);
    } catch (...) {
        return buf.size();   // ran out: an OLDER layout, nothing unknown in it
    }
    const auto p = is.tellg();
    return p < 0 ? buf.size() : size_t(p);
}

// Mirror of CadDocument::deserialize_recipe's framed path (v5/v6), recording what it skips.
void scan_tails(const std::string& blob, Tails& t)
{
    t = Tails{};
    ByteReader r{blob};
    uint32_t v = 0, n = 0;
    if (!r.u32(v) || v < 5 || !r.u32(n)) return;
    for (uint32_t i = 0; i < n; ++i) {
        uint32_t len; std::string frame;
        if (!r.u32(len) || !r.take(len, frame)) return;
        const size_t used = consumed_by(frame, [](cereal::BinaryInputArchive& ar) { CadFeature f; ar(f); });
        t.feat.push_back(used < frame.size() ? frame.substr(used) : std::string());
    }
    auto block = [&](std::string& tail, auto&& read) {
        uint32_t len; std::string b;
        if (!r.u32(len) || !r.take(len, b)) return false;
        const size_t used = consumed_by(b, read);
        tail = used < b.size() ? b.substr(used) : std::string();
        return true;
    };
    if (!block(t.vars, [](cereal::BinaryInputArchive& ar) { std::map<std::string, std::string> m; ar(m); })) return;
    if (!block(t.names, [](cereal::BinaryInputArchive& ar) { std::map<uint32_t, std::string> m; ar(m); })) return;
    if (!block(t.extras, [](cereal::BinaryInputArchive& ar) {
            double x, y, z; std::map<uint32_t, std::array<float, 4>> c; bool w; ar(x, y, z, c, w); })) return;
    if (r.pos < blob.size()) t.trailing = blob.substr(r.pos);
}

// CadDocument::serialize_recipe() plus the unknown tails. Without tails the bytes are identical
// to Orca's writer (the tests check that).
bool has_any_triangulation(const TopoDS_Shape& shape)
{
    for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
        TopLoc_Location loc;
        if (!BRep_Tool::Triangulation(TopoDS::Face(ex.Current()), loc).IsNull()) return true;
    }
    return false;
}
// The B-rep an Import feature will hold, as a save → reopen gives it back: written without its
// tessellation (as write_recipe saves it) and read again. A fresh shape (STEP / mesh reader,
// BRepPrimAPI) and its text round trip differ in the last bits, so their tessellations differ: without
// this the first edit after a reopen would hand every linked plate object of an Import body a new
// mesh (and reset its paint). Idempotent: a reopened shape round-trips to itself.
TopoDS_Shape as_saved(const TopoDS_Shape& s)
{
    if (s.IsNull()) return s;
    TopoDS_Shape r = brep_from_string(brep_to_string(BRepBuilderAPI_Copy(s, /*copyGeom*/ false, /*copyMesh*/ false).Shape()));
    return r.IsNull() ? s : r;
}
std::string write_recipe(const CadDocument& doc, const Tails& tails)
{
    std::ostringstream oss;
    {
        cereal::BinaryOutputArchive ar(oss);
        uint32_t v = CadDocument::ORCA_CAD_RECIPE_VERSION;
        ar(v);
        uint32_t n = uint32_t(doc.features.size());
        ar(n);
        auto frame = [&](const std::string& body) {
            uint32_t len = uint32_t(body.size());
            ar(len);
            ar(cereal::binary_data(body.data(), body.size()));
        };
        for (size_t i = 0; i < doc.features.size(); ++i) {
            std::ostringstream fos;
            const CadFeature& f = doc.features[i];
            if (f.type == CadFeatureType::Import && !f.imported_solid.IsNull() && has_any_triangulation(f.imported_solid)) {
                // The rebuild adopts the Import B-rep as the body itself (same TShape), so the body's
                // display tessellation (0.003 mm) sits on its faces and BRepTools::Write would save
                // it too (a 20 mm sphere: 2 MB). Save the same B-rep without it (topology copied in
                // order, geometry shared): reads back to the same faces and ids, ~100x smaller.
                CadFeature lean = f;
                lean.imported_solid = BRepBuilderAPI_Copy(f.imported_solid, /*copyGeom*/ false, /*copyMesh*/ false).Shape();
                cereal::BinaryOutputArchive fa(fos);
                fa(lean);
            } else {
                cereal::BinaryOutputArchive fa(fos);
                fa(f);
            }
            std::string fb = fos.str();
            if (i < tails.feat.size()) fb += tails.feat[i];
            frame(fb);
        }
        std::ostringstream vos;
        { cereal::BinaryOutputArchive va(vos); va(doc.variables); }
        frame(vos.str() + tails.vars);
        std::map<uint32_t, std::string> named;
        for (uint32_t i = 0; i < doc.bodies.size(); ++i)
            if (doc.bodies[i].has_user_name && !doc.bodies[i].user_name.empty()) named[i] = doc.bodies[i].user_name;
        std::ostringstream bos;
        { cereal::BinaryOutputArchive ba(bos); ba(named); }
        frame(bos.str() + tails.names);
        std::map<uint32_t, std::array<float, 4>> colours;
        for (uint32_t i = 0; i < doc.bodies.size(); ++i)
            if (doc.bodies[i].has_color)
                colours[i] = {doc.bodies[i].color.r(), doc.bodies[i].color.g(), doc.bodies[i].color.b(), doc.bodies[i].color.a()};
        std::ostringstream xos;
        {
            cereal::BinaryOutputArchive xa(xos);
            xa(doc.modeling_origin.x(), doc.modeling_origin.y(), doc.modeling_origin.z(), colours, doc.auto_close_loops);
        }
        frame(xos.str() + tails.extras);
    }
    return oss.str() + tails.trailing;
}

// Carry per-feature tails across a history change by aligning (type, name) sequences (LCS).
std::vector<std::string> realign_tails(const std::vector<std::pair<int, std::string>>& before,
                                       const std::vector<std::string>& tails,
                                       const std::vector<CadFeature>& after)
{
    std::vector<std::string> out(after.size());
    if (tails.empty()) return out;
    bool anything = false;
    for (const auto& s : tails) anything |= !s.empty();
    if (!anything) return out;
    const size_t n = before.size(), m = after.size();
    std::vector<std::vector<int>> L(n + 1, std::vector<int>(m + 1, 0));
    auto same = [&](size_t i, size_t j) { return before[i].first == int(after[j].type) && before[i].second == after[j].name; };
    for (size_t i = n; i-- > 0;)
        for (size_t j = m; j-- > 0;)
            L[i][j] = same(i, j) ? L[i + 1][j + 1] + 1 : std::max(L[i + 1][j], L[i][j + 1]);
    for (size_t i = 0, j = 0; i < n && j < m;) {
        if (same(i, j)) { if (i < tails.size()) out[j] = tails[i]; ++i; ++j; }
        else if (L[i + 1][j] >= L[i][j + 1]) ++i;
        else ++j;
    }
    return out;
}

// ---- sketch session --------------------------------------------------------------------------
struct SketchSession {
    SketchPlane plane;
    int feature{-1};                       // editing features[feature]; -1 = a new sketch
    std::vector<SketchEntity> ents;
    std::vector<SketchEntityConstraintDef> cons;
    std::vector<int> sel;
    SketchSolveResult last;                // of the last solve (dof -1 / ok when no constraints)
    bool solved_any{false};
    struct State { std::vector<SketchEntity> ents; std::vector<SketchEntityConstraintDef> cons; };
    std::vector<State> undo, redo;
    void checkpoint() { undo.push_back({ents, cons}); redo.clear(); if (undo.size() > 200) undo.erase(undo.begin()); }
};

// ---- document state --------------------------------------------------------------------------
struct BodyMeta { bool has_name{false}; std::string name; bool has_color{false}; ColorRGBA color; };
struct Snapshot {
    std::vector<CadFeature> features;
    std::map<std::string, std::string> variables;
    std::vector<std::string> tails;
    std::vector<BodyMeta> bodies;
    Vec3d origin; bool auto_close;
};

// ---- replay record and cache (CubbySlicer) ---------------------------------------------------
// Every rebuild is recorded per feature: the feature's state right before its replay (after the
// parametric pass), right after it, and the running body list around it (shape handles: cheap).
// The record answers faces_made_by without a replay, and it is the cache a later rebuild resumes
// from: if features[0..k) have byte-identical states (and reference no feature at or past k), the
// replay of those k features would repeat exactly, so features[0..k) are put back to their recorded
// post-replay states and the loop starts at k from a COPY of the recorded body list (topology
// copied, geometry shared, no triangulation: the same inputs a full replay would hand feature k).
struct FeatureKey {
    std::string bytes;          // the cereal recipe frame of the feature, minus the fields below
    TopoDS_Shape solid;         // Import's embedded B-rep, compared by identity (kept alive here)
    bool operator==(const FeatureKey& o) const { return bytes == o.bytes && solid.IsEqual(o.solid); }
};
FeatureKey feature_key(const CadFeature& f)
{
    CadFeature k = f;
    k.imported_solid.Nullify();      // compared by identity instead of serialising the B-rep
    // Read only by recompute()'s finishing pass (and mate_options), never by a feature's replay.
    k.coordsys_face_kind = -1;
    k.coordsys_face_edges = -1;
    std::ostringstream os;
    { cereal::BinaryOutputArchive ar(os); ar(k); }
    FeatureKey key{os.str(), f.imported_solid};
    auto put = [&](const auto& v) { key.bytes.append(reinterpret_cast<const char*>(&v), sizeof v); };
    // Transient body-reference state: read by the replay only while a re-point is pending.
    put(f.body_refs_pending);
    if (f.body_refs_pending) {
        put(f.body_ref_ids.size());
        for (const CadFeature::BodyId& id : f.body_ref_ids) { put(id.src); put(id.ord); }
    }
    return key;
}
// Highest feature index a feature's replay reads (Orca's for_each_feature_ref fields).
int max_feature_ref(const CadFeature& f)
{
    int m = -1;
    for (int r : {f.sketch_ref, f.sweep_path_ref, f.pattern_curve_sketch, f.rib_sketch_ref, f.mate_cs_a, f.mate_cs_b})
        m = std::max(m, r);
    for (int r : f.loft_profile_refs) m = std::max(m, r);
    return m;
}
// Fresh topology for a resumed replay: one copy of all bodies together (sharing between bodies
// survives), geometry handles shared, triangulation dropped.
std::vector<CadBody> copy_bodies(const std::vector<CadBody>& in)
{
    std::vector<CadBody> out = in;
    TopoDS_Compound comp;
    BRep_Builder bb;
    bb.MakeCompound(comp);
    int n = 0;
    for (const CadBody& b : in)
        if (!b.shape.IsNull()) { bb.Add(comp, b.shape); ++n; }
    if (n == 0) return out;
    BRepBuilderAPI_Copy cp(comp, /*copyGeom*/ false, /*copyMesh*/ false);
    TopoDS_Iterator it(cp.Shape());
    for (CadBody& b : out) {
        if (b.shape.IsNull()) continue;
        if (!it.More()) throw std::runtime_error("replay cache: body copy failed");
        b.shape = it.Value();
        it.Next();
    }
    return out;
}

struct ReplayStep {
    FeatureKey key;                       // features[i] before its replay
    CadFeature post;                      // features[i] right after it
    std::vector<CadBody> before, after;   // the running body list around it
};
using StepPtr = std::shared_ptr<const ReplayStep>;
struct ReplayRun {
    std::string global;                   // document inputs outside the features
    std::vector<StepPtr> steps;           // steps[i] = feature i (a prefix may be shared with the run it resumed)
    size_t resumed_at{0};                 // first feature this run replayed itself
    bool complete{false};                 // every feature replayed
    bool cancelled{false};
};
using RunPtr = std::shared_ptr<ReplayRun>;
constexpr size_t kCachedRuns = 4;

std::string global_key(const CadDocument& d)
{
    std::string g;
    auto put = [&](const auto& v) { g.append(reinterpret_cast<const char*>(&v), sizeof v); };
    put(d.auto_close_loops); put(d.modeling_origin.x()); put(d.modeling_origin.y()); put(d.modeling_origin.z());
    put(d.linear_deflection); put(d.angular_deflection);
    return g;
}

// What failed: a feature index (-1 = not one feature) and its name.
struct FailInfo { int index{-1}; std::string name; };
FailInfo fail_info(const CadDocument& d)
{
    FailInfo f;
    f.index = d.failed_feature;
    if (f.index >= 0 && f.index < int(d.features.size())) f.name = d.features[f.index].name;
    else f.index = -1;
    return f;
}

// Per-call bookkeeping, reset by cad_call. The first recompute of a call is the attempt; once
// one fails or is cancelled, the rest of the call is restoring and is never cancelled.
struct CallState {
    bool cancellable{true};
    bool cancelled{false};
    bool have_fail{false};
    FailInfo fail;            // the call's first failed rebuild
    int replayed_from{-1};    // where the call's first rebuild started replaying (-1 = no rebuild)
    int recomputes{0};
    bool no_cache{false};
};
CallState g_call;

struct DocState;
struct DocObserver : CadDocument::ReplayObserver {
    DocState& s;
    RunPtr cur;
    std::vector<FeatureKey> keys;
    std::vector<CadBody> pending_before;
    explicit DocObserver(DocState& st) : s(st) {}
    size_t begin(CadDocument& doc, std::vector<CadBody>& built) override;
    bool before_feature(CadDocument& doc, size_t fi, const std::vector<CadBody>& built) override;
    void after_feature(CadDocument& doc, size_t fi, const std::vector<CadBody>& built) override;
    bool replayed(CadDocument& doc, const std::vector<CadBody>& built) override;
    void finished(CadDocument& doc, bool ok) override;
};

struct DocState {
    CadDocument doc;
    std::string original;        // bytes as opened (returned verbatim until dirty)
    uint32_t version{CadDocument::ORCA_CAD_RECIPE_VERSION};
    bool dirty{true};            // a new document has no original bytes
    bool read_only{false};
    std::string read_only_reason;
    Tails tails;
    std::vector<Snapshot> undo, redo;
    std::optional<SketchSession> sketch;
    double last_rebuild_ms{0};
    bool state_ok{true};         // the current history rebuilt (its last rebuild succeeded)
    FailInfo state_fail;         // else: the feature it fails on
    RunPtr run;                  // the rebuild that produced the current bodies
    std::deque<RunPtr> runs;     // recent rebuilds = the replay cache, newest last
    bool replay_cache{true};
    DocObserver obs{*this};
    DocState() { doc.replay_observer = &obs; }
    DocState(const DocState&) = delete;
    DocState& operator=(const DocState&) = delete;
};

size_t DocObserver::begin(CadDocument& doc, std::vector<CadBody>& built)
{
    cur = std::make_shared<ReplayRun>();
    cur->global = global_key(doc);
    pending_before.clear();
    const size_t n = doc.features.size();
    keys.clear();
    keys.reserve(n);
    for (const CadFeature& f : doc.features) keys.push_back(feature_key(f));
    size_t best = 0;
    const ReplayRun* from = nullptr;
    if (s.replay_cache && !g_call.no_cache) {
        // valid[k]: features[0..k) read no feature at or past k (their replay depends on the prefix only)
        std::vector<char> valid(n + 1, 1);
        int mx = -1;
        for (size_t k = 1; k <= n; ++k) { mx = std::max(mx, max_feature_ref(doc.features[k - 1])); valid[k] = mx < int(k); }
        for (auto it = s.runs.rbegin(); it != s.runs.rend(); ++it) {
            const ReplayRun& r = **it;
            if (r.global != cur->global) continue;
            size_t m = 0;
            while (m < n && m < r.steps.size() && r.steps[m]->key == keys[m]) ++m;
            while (m > best && !valid[m]) --m;
            if (m > best) { best = m; from = &r; }
        }
    }
    if (g_call.replayed_from < 0) g_call.replayed_from = int(best);
    if (!from) return 0;
    for (size_t i = 0; i < best; ++i) {
        CadFeature f = from->steps[i]->post;
        // not part of the key: the document's own value is the one a full replay would keep
        f.coordsys_face_kind = doc.features[i].coordsys_face_kind;
        f.coordsys_face_edges = doc.features[i].coordsys_face_edges;
        doc.features[i] = std::move(f);
        cur->steps.push_back(from->steps[i]);
    }
    built = copy_bodies(from->steps[best - 1]->after);
    cur->resumed_at = best;
    return best;
}
bool DocObserver::before_feature(CadDocument& doc, size_t fi, const std::vector<CadBody>& built)
{
    pending_before = built;
    if (cad_progress_js(int(fi), int(doc.features.size()), doc.features[fi].name.c_str(), g_call.cancellable) && g_call.cancellable) {
        if (cur) cur->cancelled = true;
        return false;
    }
    return true;
}
void DocObserver::after_feature(CadDocument& doc, size_t fi, const std::vector<CadBody>& built)
{
    if (!cur || fi != cur->steps.size() || fi >= keys.size()) return;   // (cannot happen: the loop runs in order)
    auto st = std::make_shared<ReplayStep>();
    st->key = keys[fi];
    st->post = doc.features[fi];
    st->before = std::move(pending_before);
    st->after = built;
    cur->steps.push_back(std::move(st));
}
bool DocObserver::replayed(CadDocument& doc, const std::vector<CadBody>& built)
{
    (void)built;
    if (cur) cur->complete = true;
    const int n = int(doc.features.size());
    if (cad_progress_js(n, n, "", g_call.cancellable) && g_call.cancellable) {
        if (cur) cur->cancelled = true;
        return false;
    }
    return true;
}
void DocObserver::finished(CadDocument& doc, bool ok)
{
    if (!cur) { cur = std::make_shared<ReplayRun>(); cur->global = global_key(doc); }   // failed before the loop
    ++g_call.recomputes;
    if (cur->cancelled) {
        doc.failed_feature = -1;
        doc.error = "rebuild cancelled";
        g_call.cancelled = true;
    }
    if (!ok) {
        if (!g_call.have_fail) { g_call.have_fail = true; g_call.fail = fail_info(doc); }
        g_call.cancellable = false;
    }
    s.state_ok = ok;
    s.state_fail = ok ? FailInfo{} : fail_info(doc);
    s.run = cur;
    if (!s.replay_cache) s.runs.clear();
    else if (!cur->steps.empty()) {
        // drop older records this one contains (same leading steps)
        for (auto it = s.runs.begin(); it != s.runs.end();) {
            const ReplayRun& r = **it;
            bool sub = r.global == cur->global && r.steps.size() <= cur->steps.size();
            for (size_t i = 0; sub && i < r.steps.size(); ++i) sub = r.steps[i] == cur->steps[i];
            it = sub ? s.runs.erase(it) : it + 1;
        }
        s.runs.push_back(cur);
        while (s.runs.size() > kCachedRuns) s.runs.pop_front();
    }
    cur.reset();
    keys.clear();
    pending_before.clear();
}

std::map<int, std::unique_ptr<DocState>> g_docs;
int g_next_doc = 1;

std::vector<BodyMeta> body_meta(const CadDocument& d)
{
    std::vector<BodyMeta> m;
    for (const CadBody& b : d.bodies) m.push_back({b.has_user_name, b.user_name, b.has_color, b.color});
    return m;
}
void apply_body_meta(CadDocument& d, const std::vector<BodyMeta>& m)
{
    for (size_t i = 0; i < m.size() && i < d.bodies.size(); ++i) {
        d.bodies[i].has_user_name = m[i].has_name; d.bodies[i].user_name = m[i].name;
        d.bodies[i].has_color = m[i].has_color; d.bodies[i].color = m[i].color;
    }
}
Snapshot take_snapshot(const DocState& s)
{
    return Snapshot{s.doc.features, s.doc.variables, s.tails.feat, body_meta(s.doc), s.doc.modeling_origin, s.doc.auto_close_loops};
}

// Everything a rebuild derives, so a rejected or cancelled edit puts the document back exactly
// as it was without replaying the history again (and keeps its generation: the ids it handed
// out are valid again).
struct Derived {
    std::vector<CadFeature> features;
    std::vector<CadBody> bodies;
    TopoDS_Shape body;
    TriangleMesh display_mesh;
    std::vector<TriangleMesh> body_meshes;
    std::vector<int> tri_face, tri_body;
    std::string error;
    std::vector<std::pair<int, std::string>> mate_conflicts;
    uint64_t generation{0};
    int failed_feature{-1};
    bool state_ok{true};
    FailInfo state_fail;
    RunPtr run;
    double ms{0};
};
Derived take_derived(const DocState& s)
{
    const CadDocument& d = s.doc;
    return Derived{d.features, d.bodies, d.body, d.display_mesh, d.display_body_meshes, d.display_tri_face, d.display_tri_body,
                   d.error, d.mate_conflicts, d.topo_generation, d.failed_feature, s.state_ok, s.state_fail, s.run, s.last_rebuild_ms};
}
void restore_derived(DocState& s, Derived& v)
{
    CadDocument& d = s.doc;
    d.features = std::move(v.features);
    d.bodies = std::move(v.bodies);
    d.body = v.body;
    d.display_mesh = std::move(v.display_mesh);
    d.display_body_meshes = std::move(v.body_meshes);
    d.display_tri_face = std::move(v.tri_face);
    d.display_tri_body = std::move(v.tri_body);
    d.error = std::move(v.error);
    d.mate_conflicts = std::move(v.mate_conflicts);
    d.topo_generation = v.generation;
    d.failed_feature = v.failed_feature;
    s.state_ok = v.state_ok;
    s.state_fail = v.state_fail;
    s.run = v.run;
    s.last_rebuild_ms = v.ms;
}

bool rebuild(DocState& s)
{
    const auto t0 = std::chrono::steady_clock::now();
    bool ok = false;
    try {
        ok = s.doc.recompute();
    } catch (const Standard_Failure& e) {
        s.doc.error = std::string("OCCT: ") + (e.GetMessageString() ? e.GetMessageString() : "failure");
    } catch (const std::exception& e) {
        s.doc.error = e.what();
    }
    // A history of sketches and datums only is a valid state, not a failure (recompute's own rule).
    if (!ok) {
        const bool any_solid = std::any_of(s.doc.features.begin(), s.doc.features.end(),
            [](const CadFeature& f) { return f.enabled && CadDocument::produces_body(f.type); });
        if (!any_solid && s.doc.error.empty()) { ok = true; s.state_ok = true; s.state_fail = {}; }
    }
    if (!ok) {
        // A failed replay leaves no bodies (recompute() returns before replacing the old ones).
        s.doc.bodies.clear();
        s.doc.body = TopoDS_Shape();
        s.doc.display_mesh = TriangleMesh{};
        s.doc.display_body_meshes.clear();
        s.doc.display_tri_face.clear();
        s.doc.display_tri_body.clear();
    }
    s.last_rebuild_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return ok;
}
void restore_snapshot(DocState& s, Snapshot& snap)
{
    s.doc.features = std::move(snap.features);
    s.doc.variables = std::move(snap.variables);
    s.tails.feat = std::move(snap.tails);
    s.doc.modeling_origin = snap.origin;
    s.doc.auto_close_loops = snap.auto_close;
    s.doc.bodies.clear();
    rebuild(s);
    apply_body_meta(s.doc, snap.bodies);
}

DocState& doc_of(const json& p)
{
    auto it = p.find("doc");
    if (it == p.end() || !it->is_number_integer()) bad_param("missing 'doc' (document handle)");
    auto d = g_docs.find(it->get<int>());
    if (d == g_docs.end()) fail(kNoDocument, "no document " + std::to_string(it->get<int>()));
    return *d->second;
}
DocState& writable(const json& p)
{
    DocState& s = doc_of(p);
    if (s.read_only) fail(kReadOnly, "the document is read-only: " + s.read_only_reason);
    return s;
}
void check_generation(const DocState& s, const json& p)
{
    auto it = p.find("generation");
    if (it == p.end() || it->is_null()) return;
    if (!it->is_number_integer()) bad_param("'generation' must be an integer");
    if (it->get<uint64_t>() != s.doc.topo_generation)
        fail(kStale, "stale ids: the model was rebuilt since generation " + std::to_string(it->get<uint64_t>()) +
                     " (now " + std::to_string(s.doc.topo_generation) + "); re-read the topology");
}
// add_feature / set_feature: `fields` that hold face / edge ids (valid for one generation).
// A request whose fields set any of them to an id (>= 0, or a non-empty list) has its
// `generation` checked like every other id-taking method; requests without ids, or without
// `generation`, are not checked.
void check_generation_of_fields(const DocState& s, const json& p, const json& fields)
{
    static const char* const kTopoFields[] = {
        "up_to_face", "extrude_src_face", "dressup_edge", "dressup_edges", "shell_face", "draft_face",
        "plane_face", "plane_face2", "plane_edge", "plane_edge2", "bool_target_face", "bool_tool_face",
        "axis_face", "axis_edge", "coordsys_face", "coordsys_edge", "thicken_face", "cut_face",
        "project_edges", "project_face", "delete_faces"};
    if (!fields.is_object()) return;
    bool ids = false;
    for (const char* k : kTopoFields) {
        auto it = fields.find(k);
        if (it == fields.end()) continue;
        if (it->is_number_integer() && it->get<long long>() >= 0) ids = true;
        else if (it->is_array() && !it->empty()) ids = true;
    }
    if (ids) check_generation(s, p);
}

// One mutating action = one undo step: snapshot, mutate, rebuild, and either keep it (and the
// undo step) or put the document back exactly as it was (no replay: the derived state is kept
// aside) with the failure's reason in doc.error. `rolled_back` = the kernel already restored the
// history itself (Orca's transactional edits); the derived state is put back all the same.
struct Mutation {
    DocState& s;
    Snapshot snap;
    Derived derived;
    std::vector<std::pair<int, std::string>> keys;
    explicit Mutation(DocState& st) : s(st), snap(take_snapshot(st)), derived(take_derived(st))
    {
        for (const CadFeature& f : st.doc.features) keys.emplace_back(int(f.type), f.name);
    }
    // Put everything back (history, variables, tails, document fields, derived state).
    void restore()
    {
        s.doc.variables = snap.variables;
        s.tails.feat = snap.tails;
        s.doc.modeling_origin = snap.origin;
        s.doc.auto_close_loops = snap.auto_close;
        restore_derived(s, derived);
    }
    bool finish(bool ok, bool rolled_back = false)
    {
        (void)rolled_back;
        if (ok) {
            s.tails.feat = realign_tails(keys, snap.tails, s.doc.features);
            s.undo.push_back(std::move(snap));
            if (s.undo.size() > 200) s.undo.erase(s.undo.begin());
            s.redo.clear();
            s.dirty = true;
            return true;
        }
        const std::string why = s.doc.error.empty() ? "the edit could not be rebuilt" : s.doc.error;
        restore();
        s.doc.error = why;
        return false;
    }
};

json bodies_summary(const CadDocument& d);

// The result tail every mutating McpControl verb returns. CubbySlicer: `failed_feature` /
// `failed_feature_name` (the feature the attempted rebuild failed on, -1 / "" when it did not or
// failed outside one feature), `replayed_from` (first feature the call's rebuild replayed; the
// features before it came from the replay cache; -1 = no rebuild), `cancelled`.
json build_result(DocState& s, bool ok, json extra = json::object())
{
    extra["ok"] = ok;
    extra["bodies"] = int(s.doc.bodies.size());
    extra["error"] = ok ? std::string() : s.doc.error;
    extra["generation"] = s.doc.topo_generation;
    const FailInfo f = (!ok && g_call.have_fail) ? g_call.fail : FailInfo{};
    extra["failed_feature"] = f.index;
    extra["failed_feature_name"] = f.name;
    extra["replayed_from"] = g_call.replayed_from;
    extra["cancelled"] = g_call.cancelled;
    return extra;
}

// CubbySlicer: a Shell whose replay "succeeded" without hollowing anything. Orca's Shell
// (MakeThickSolidByJoin) only checks IsDone() and a non-null shape, and OCCT reports both when
// (a) the open face borders fillets/blends it cannot offset: the body comes back unchanged, and
// (b) no face is opened (closed hollow): BRepOffset_MakeOffset builds the solid wall only when
// faces are removed, so the result is the inward offset alone — a smaller filled solid (a
// 20x20x10 box at 1.5 mm gives a 17x17x7 solid) or, around fillets, a sheet with no solid.
// Returns why the Shell feature `fi` of the current (just rebuilt) state is such a no-op, or "".
// Uses the replay record (the body list right before and after the feature): no extra replay.
std::string shell_problem(const DocState& s, int fi)
{
    if (fi < 0 || fi >= int(s.doc.features.size())) return {};
    const CadFeature& f = s.doc.features[fi];
    if (f.type != CadFeatureType::Shell || !f.enabled) return {};
    if (!s.run || int(s.run->steps.size()) <= fi) return {};
    const ReplayStep& st = *s.run->steps[fi];
    if (st.before.size() != st.after.size()) return {};
    auto props = [](const TopoDS_Shape& sh, double& vol, int& faces, int& shells) {
        vol = 0; faces = 0; shells = 0;
        for (TopExp_Explorer e(sh, TopAbs_FACE); e.More(); e.Next()) ++faces;
        bool solid = false;
        for (TopExp_Explorer e(sh, TopAbs_SOLID); e.More(); e.Next()) {
            solid = true;
            for (TopExp_Explorer se(e.Current(), TopAbs_SHELL); se.More(); se.Next()) ++shells;
        }
        if (!solid) return false;
        GProp_GProps g;
        BRepGProp::VolumeProperties(sh, g);
        vol = std::abs(g.Mass());
        return true;
    };
    bool any = false;
    for (size_t i = 0; i < st.after.size(); ++i) {
        const TopoDS_Shape& a = st.before[i].shape, & b = st.after[i].shape;
        if (a.IsNull() || b.IsNull() || a.IsSame(b)) continue;
        any = true;
        double va, vb; int fa, fb, sa, sb;
        const bool solid_a = props(a, va, fa, sa);
        const bool solid_b = props(b, vb, fb, sb);
        if (!solid_a) continue;   // not a solid before: nothing to compare against
        if (!solid_b)
            return "the shell produced a sheet with no solid (OCCT could not build the wall" +
                   std::string(f.shell_face < 0 ? "; a closed hollow needs an open face in this kernel" : "") +
                   ") — the edit was not applied";
        if (fa == fb && std::abs(va - vb) <= 1e-7 * std::max(1.0, va))
            return "the shell changed nothing: OCCT could not hollow the body with that open face (its edges "
                   "meet fillets or blends it cannot offset) — try another face or a thinner wall";
        if (f.shell_face < 0 && sb < 2)
            return "a closed hollow (no open face) is not supported: Orca 2.5.0's Shell returns only the inward "
                   "offset of the body, not a solid with a cavity — pick a face to open";
    }
    if (!any) return "the shell changed nothing — the edit was not applied";
    return {};
}
// rebuild(), then refuse a no-op Shell at `fi` (see shell_problem) as a failure of that feature.
bool rebuild_checked(DocState& s, int fi)
{
    if (!rebuild(s)) return false;
    const std::string why = shell_problem(s, fi);
    if (why.empty()) return true;
    s.doc.error = why;
    if (!g_call.have_fail) { g_call.have_fail = true; g_call.fail = FailInfo{fi, s.doc.features[fi].name}; }
    return false;
}

// Run "add features + rebuild" as one action (McpControl's checkpoint → add → recompute → undo).
template<class F> bool add_and_rebuild(DocState& s, F&& add, int check_feature = -1)
{
    Mutation m(s);
    try {
        add();
    } catch (...) {
        // a refusal half-way through an add (bad argument) must leave nothing behind
        if (s.doc.features.size() != m.snap.features.size()) m.restore();
        throw;
    }
    return m.finish(check_feature >= 0 ? rebuild_checked(s, check_feature) : rebuild(s));
}

int target_body_arg(const json& params, const CadDocument& doc)
{
    int bi = opt<int>(params, "body", -1);
    if (bi >= int(doc.bodies.size()))
        bad_param("body index out of range (have " + std::to_string(doc.bodies.size()) + ")");
    return bi;
}
const TopoDS_Shape& body_shape(const DocState& s, const json& params, const char* key = "body", int dflt = 0)
{
    const int idx = opt<int>(params, key, dflt);
    if (idx < 0 || idx >= int(s.doc.bodies.size()))
        bad_param("body index out of range (have " + std::to_string(s.doc.bodies.size()) + ")");
    if (s.doc.bodies[idx].shape.IsNull()) fail(kFailure, "body has no shape");
    return s.doc.bodies[idx].shape;
}
BooleanMode bool_from(const std::string& s)
{
    if (s == "union" || s == "add" || s == "Add") return BooleanMode::Add;
    if (s == "subtract" || s == "cut" || s == "Cut") return BooleanMode::Cut;
    if (s == "intersect" || s == "common" || s == "Intersect") return BooleanMode::Intersect;
    return BooleanMode::New;
}
SketchPlane plane_arg(const json& params, const CadDocument& doc, const char* key = "plane", const char* dflt = "XY")
{
    auto it = params.find(key);
    return plane_from_json(it == params.end() ? json(dflt) : *it, doc.modeling_origin);
}

// ---- output blob -----------------------------------------------------------------------------
struct BlobOut {
    std::vector<uint8_t> bytes;
    json put(const void* p, size_t elem, size_t count)
    {
        while (bytes.size() % 4) bytes.push_back(0);
        const size_t off = bytes.size();
        bytes.resize(off + elem * count);
        if (count) std::memcpy(bytes.data() + off, p, elem * count);
        return json{{"offset", off}, {"count", count}};
    }
    template<class T> json put(const std::vector<T>& v) { return put(v.data(), sizeof(T), v.size()); }
};

struct Ctx {
    const json& params;
    const uint8_t* blob;
    size_t blob_len;
    BlobOut& out_blob;
    std::string blob_string() const { return std::string(reinterpret_cast<const char*>(blob), blob_len); }
};

// ---- meshes ----------------------------------------------------------------------------------
bool has_triangulation(const TopoDS_Shape& shape)
{
    for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
        TopLoc_Location loc;
        if (BRep_Tool::Triangulation(TopoDS::Face(ex.Current()), loc).IsNull()) return false;
    }
    return true;
}

// Display mesh of one body: per-face vertices (crisp normals at B-rep edges), triangle -> face id
// (TopExp_Explorer ordinal, = Orca's display_tri_face), and every edge as a polyline keyed by its
// TopExp::MapShapes ordinal (= Orca's edge id).
json display_mesh(const TopoDS_Shape& shape, int body, BlobOut& b, bool with_edges, double lin, double ang)
{
    if (!has_triangulation(shape)) BRepMesh_IncrementalMesh(shape, lin, false, ang, false);
    std::vector<float> pos, nrm;
    std::vector<uint32_t> idx;
    std::vector<int32_t> tri_face;
    int face_id = -1;
    for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
        ++face_id;
        const TopoDS_Face& face = TopoDS::Face(ex.Current());
        TopLoc_Location loc;
        Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
        if (tri.IsNull() || tri->NbTriangles() == 0) continue;
        if (!tri->HasNormals()) BRepLib_ToolTriangulatedShape::ComputeNormals(face, tri);
        const gp_Trsf trsf = loc.Transformation();
        const bool rev = face.Orientation() == TopAbs_REVERSED;
        const uint32_t base = uint32_t(pos.size() / 3);
        for (int i = 1; i <= tri->NbNodes(); ++i) {
            gp_Pnt p = tri->Node(i);
            p.Transform(trsf);
            pos.push_back(float(p.X())); pos.push_back(float(p.Y())); pos.push_back(float(p.Z()));
            gp_Dir n = tri->HasNormals() ? tri->Normal(i) : gp_Dir(0, 0, 1);
            n.Transform(trsf);
            if (rev) n.Reverse();
            nrm.push_back(float(n.X())); nrm.push_back(float(n.Y())); nrm.push_back(float(n.Z()));
        }
        for (int i = 1; i <= tri->NbTriangles(); ++i) {
            int a, c, d;
            tri->Triangle(i).Get(a, c, d);
            if (rev) std::swap(c, d);
            if (a == c || c == d || a == d) continue;
            idx.push_back(base + a - 1); idx.push_back(base + c - 1); idx.push_back(base + d - 1);
            tri_face.push_back(face_id);
        }
    }
    json m{{"body", body}, {"faces", face_id + 1}, {"positions", b.put(pos)}, {"normals", b.put(nrm)},
           {"indices", b.put(idx)}, {"triFace", b.put(tri_face)}};
    if (with_edges) {
        ShapeIndexMap edges;
        TopExp::MapShapes(shape, TopAbs_EDGE, edges);
        AncestorMap faces_of_edge;
        TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, faces_of_edge);
        std::vector<int32_t> ids;
        std::vector<uint32_t> starts{0};
        std::vector<float> pts;
        json flags = json::array();
        for (int i = 1; i <= edges.Extent(); ++i) {
            const TopoDS_Edge& e = TopoDS::Edge(edges(i));
            bool seam = false, done = false;
            const bool degen = BRep_Tool::Degenerated(e);
            if (const NCollection_List<TopoDS_Shape>* fl = faces_of_edge.Seek(e)) {
                for (NCollection_List<TopoDS_Shape>::Iterator it(*fl); it.More(); it.Next()) {
                    const TopoDS_Face& f = TopoDS::Face(it.Value());
                    seam = seam || BRep_Tool::IsClosed(e, f);
                    if (done) continue;
                    TopLoc_Location loc;
                    Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(f, loc);
                    if (tri.IsNull()) continue;
                    Handle(Poly_PolygonOnTriangulation) poly = BRep_Tool::PolygonOnTriangulation(e, tri, loc);
                    if (poly.IsNull() || poly->NbNodes() < 2) continue;
                    const gp_Trsf trsf = loc.Transformation();
                    for (int k = 1; k <= poly->NbNodes(); ++k) {
                        gp_Pnt p = tri->Node(poly->Node(k));
                        p.Transform(trsf);
                        pts.push_back(float(p.X())); pts.push_back(float(p.Y())); pts.push_back(float(p.Z()));
                    }
                    done = true;
                }
            }
            if (!done && !degen) {
                for (const Vec3d& p : GeometryEngine::sample_edge_world(e, std::max(lin, 0.01))) {
                    pts.push_back(float(p.x())); pts.push_back(float(p.y())); pts.push_back(float(p.z()));
                }
            }
            ids.push_back(i - 1);
            starts.push_back(uint32_t(pts.size() / 3));
            if (seam || degen) flags.push_back(json{{"edge", i - 1}, {"seam", seam}, {"degenerate", degen}});
        }
        m["edges"] = int(ids.size());
        m["edgeIds"] = b.put(ids);
        m["edgeStarts"] = b.put(starts);
        m["edgePoints"] = b.put(pts);
        m["edgeFlags"] = flags;
    }
    return m;
}

json plate_mesh(const TriangleMesh& tm, const std::vector<int>* tri_face, int body, BlobOut& b)
{
    std::vector<float> pos;
    pos.reserve(tm.its.vertices.size() * 3);
    for (const auto& v : tm.its.vertices) { pos.push_back(v.x()); pos.push_back(v.y()); pos.push_back(v.z()); }
    std::vector<uint32_t> idx;
    idx.reserve(tm.its.indices.size() * 3);
    for (const auto& t : tm.its.indices) { idx.push_back(uint32_t(t[0])); idx.push_back(uint32_t(t[1])); idx.push_back(uint32_t(t[2])); }
    json m{{"body", body}, {"vertices", tm.its.vertices.size()}, {"triangles", tm.its.indices.size()},
           {"positions", b.put(pos)}, {"indices", b.put(idx)}};
    if (tri_face) {
        std::vector<int32_t> tf(tri_face->begin(), tri_face->end());
        m["triFace"] = b.put(tf);
    }
    return m;
}

// ---- descriptions ----------------------------------------------------------------------------
bool shape_is_solid(const TopoDS_Shape& s) { return !s.IsNull() && !CadDocument::is_sheet_shape(s); }

json bodies_summary(const CadDocument& d)
{
    json bodies = json::array();
    for (size_t i = 0; i < d.bodies.size(); ++i) {
        const CadBody& b = d.bodies[i];
        json j{{"index", int(i)}, {"name", b.has_user_name ? b.user_name : b.name}, {"user_name", b.has_user_name},
               {"has_color", b.has_color}, {"source_feature", b.source_feature}, {"is_solid", shape_is_solid(b.shape)}};
        if (b.has_color) j["color"] = json::array({b.color.r(), b.color.g(), b.color.b(), b.color.a()});
        if (!b.shape.IsNull()) {
            Bnd_Box bb;
            BRepBndLib::Add(b.shape, bb);
            if (!bb.IsVoid()) {
                double x0, y0, z0, x1, y1, z1;
                bb.Get(x0, y0, z0, x1, y1, z1);
                j["bbox"] = json{{"min", {x0, y0, z0}}, {"max", {x1, y1, z1}}};
                j["center"] = {0.5 * (x0 + x1), 0.5 * (y0 + y1), 0.5 * (z0 + z1)};
            }
            j["face_count"] = GeometryEngine::face_count(b.shape);
            j["edge_count"] = GeometryEngine::edge_count(b.shape);
        }
        bodies.push_back(std::move(j));
    }
    return bodies;
}
json mate_conflicts_json(const CadDocument& d)
{
    json a = json::array();
    for (const auto& [fi, why] : d.mate_conflicts) a.push_back(json{{"feature", fi}, {"reason", why}});
    return a;
}
void evaluate_each_variable(const std::map<std::string, std::string>& vars, json& values, json& errors);
json describe_scene(DocState& s)
{
    const CadDocument& doc = s.doc;
    json features = json::array();
    for (size_t i = 0; i < doc.features.size(); ++i) {
        const CadFeature& f = doc.features[i];
        features.push_back(json{{"index", int(i)}, {"type", feature_type_name(f.type)}, {"name", f.name}, {"enabled", f.enabled}});
    }
    json values, errors;
    evaluate_each_variable(doc.variables, values, errors);
    const FailInfo fail = s.state_ok ? FailInfo{} : s.state_fail;
    return json{{"modeling_origin", vec3_json(doc.modeling_origin)}, {"features", features}, {"bodies", bodies_summary(doc)},
                {"error", doc.error}, {"generation", doc.topo_generation}, {"read_only", s.read_only},
                {"dirty", s.dirty}, {"can_undo", !s.undo.empty()}, {"can_redo", !s.redo.empty()},
                {"variables", doc.variables}, {"variable_values", values}, {"variable_errors", errors},
                {"mate_conflicts", mate_conflicts_json(doc)},
                {"auto_close_loops", doc.auto_close_loops}, {"sketch_open", s.sketch.has_value()},
                {"rebuild_ms", s.last_rebuild_ms}, {"rebuild_ok", s.state_ok},
                {"failed_feature", fail.index}, {"failed_feature_name", fail.name},
                {"replay_cache", s.replay_cache}};
}

json surface_kind(GeomAbs_SurfaceType t)
{
    switch (t) {
    case GeomAbs_Plane: return "plane"; case GeomAbs_Cylinder: return "cylinder"; case GeomAbs_Cone: return "cone";
    case GeomAbs_Sphere: return "sphere"; case GeomAbs_Torus: return "torus"; case GeomAbs_BezierSurface: return "bezier";
    case GeomAbs_BSplineSurface: return "bspline"; case GeomAbs_SurfaceOfRevolution: return "revolution";
    case GeomAbs_SurfaceOfExtrusion: return "extrusion"; case GeomAbs_OffsetSurface: return "offset";
    default: return "other";
    }
}
json curve_kind(GeomAbs_CurveType t)
{
    switch (t) {
    case GeomAbs_Line: return "line"; case GeomAbs_Circle: return "circle"; case GeomAbs_Ellipse: return "ellipse";
    case GeomAbs_Hyperbola: return "hyperbola"; case GeomAbs_Parabola: return "parabola";
    case GeomAbs_BezierCurve: return "bezier"; case GeomAbs_BSplineCurve: return "bspline";
    case GeomAbs_OffsetCurve: return "offset"; default: return "other";
    }
}
json bbox_json(const TopoDS_Shape& s)
{
    Bnd_Box bb;
    BRepBndLib::Add(s, bb);
    if (bb.IsVoid()) return nullptr;
    double x0, y0, z0, x1, y1, z1;
    bb.Get(x0, y0, z0, x1, y1, z1);
    return json{{"min", {x0, y0, z0}}, {"max", {x1, y1, z1}}};
}

// ---- sketch helpers (ported from DesignSketchTool, GUI-free) -----------------------------------
constexpr int kArcFacets = 24;
std::vector<Vec2d> circle_polygon(const Vec2d& c, double r, int n = 48)
{
    std::vector<Vec2d> v;
    for (int i = 0; i < n; ++i) { const double a = 2.0 * M_PI * double(i) / double(n); v.emplace_back(c.x() + r * std::cos(a), c.y() + r * std::sin(a)); }
    return v;
}
Vec2d ellipse_point(const Vec2d& c, double a, double b, double phi, double t)
{
    const double cu = std::cos(phi), su = std::sin(phi), x = a * std::cos(t), y = b * std::sin(t);
    return Vec2d(c.x() + x * cu - y * su, c.y() + x * su + y * cu);
}
std::vector<Vec2d> ellipse_polyline(const Vec2d& c, double a, double b, double phi, double t0, double t1, int n = 48)
{
    std::vector<Vec2d> v;
    for (int i = 0; i <= n; ++i) v.push_back(ellipse_point(c, a, b, phi, t0 + (t1 - t0) * double(i) / double(n)));
    return v;
}
int bspline_degree(int n) { return n >= 4 ? 3 : (n >= 2 ? n - 1 : 0); }
std::vector<double> bspline_knots(int n, int p)
{
    std::vector<double> U;
    const int interior = n - p - 1;
    for (int i = 0; i <= p; ++i) U.push_back(0.0);
    for (int i = 1; i <= interior; ++i) U.push_back(double(i));
    for (int i = 0; i <= p; ++i) U.push_back(double(interior + 1));
    return U;
}
Vec2d bspline_eval(const std::vector<Vec2d>& P, const std::vector<double>& U, int p, double u)
{
    const int n = int(P.size());
    if (u <= U[p]) return P.front();
    if (u >= U[n]) return P.back();
    int k = p;
    while (k < n - 1 && U[k + 1] <= u) ++k;
    std::vector<Vec2d> d(p + 1);
    for (int j = 0; j <= p; ++j) d[j] = P[j + k - p];
    for (int r = 1; r <= p; ++r)
        for (int j = p; j >= r; --j) {
            const double denom = U[j + 1 + k - r] - U[j + k - p];
            const double a = denom > 1e-12 ? (u - U[j + k - p]) / denom : 0.0;
            d[j] = (1.0 - a) * d[j - 1] + a * d[j];
        }
    return d[p];
}
std::vector<Vec2d> bspline_polyline(const std::vector<Vec2d>& ctrl)
{
    const int n = int(ctrl.size());
    if (n < 2) return ctrl;
    const int p = bspline_degree(n);
    const std::vector<double> U = bspline_knots(n, p);
    const double umax = double(n - p);
    const int samples = std::max(24, 14 * (n - 1));
    std::vector<Vec2d> out;
    for (int i = 0; i <= samples; ++i) out.push_back(bspline_eval(ctrl, U, p, umax * double(i) / double(samples)));
    return out;
}
std::vector<Vec2d> entity_polyline(const SketchEntity& e)
{
    switch (e.type) {
    case SketchEntity::Type::Line: return {e.p0, e.p1};
    case SketchEntity::Type::Circle: return circle_polygon(e.center, e.radius);
    case SketchEntity::Type::Arc: {
        std::vector<Vec2d> pts;
        for (int i = 0; i <= kArcFacets; ++i) {
            const double a = e.start_angle + (e.end_angle - e.start_angle) * double(i) / double(kArcFacets);
            pts.emplace_back(e.center.x() + e.radius * std::cos(a), e.center.y() + e.radius * std::sin(a));
        }
        return pts;
    }
    case SketchEntity::Type::Ellipse: return ellipse_polyline(e.center, e.radius, e.rminor, e.rotation, 0.0, 2.0 * M_PI);
    case SketchEntity::Type::EllipseArc: return ellipse_polyline(e.center, e.radius, e.rminor, e.rotation, e.start_angle, e.end_angle);
    case SketchEntity::Type::BSpline: return bspline_polyline(e.ctrl);
    case SketchEntity::Type::Point: return {e.p0};
    }
    return {};
}

struct RegionLoop { std::vector<Vec2d> poly; std::vector<int> ents; std::vector<int> holes; bool defect{false}; Vec2d defect_at{0, 0}; };

// DesignSketchTool::region_loops: closed loops (circles/ellipses alone, line/arc/spline chains
// walked end to end), each assigned as a hole of the smallest loop containing it.
std::vector<RegionLoop> region_loops(const std::vector<SketchEntity>& ents)
{
    std::vector<RegionLoop> regions;
    const double eps2 = sketch_join_tol() * sketch_join_tol();
    auto is_near = [&](const Vec2d& a, const Vec2d& b) { return (a - b).squaredNorm() < eps2; };
    struct Seg { std::vector<Vec2d> pts; int ent{-1}; bool used{false}; };
    std::vector<Seg> segs;
    for (int i = 0; i < int(ents.size()); ++i) {
        const SketchEntity& e = ents[i];
        if (e.construction) continue;
        if (e.type == SketchEntity::Type::Circle || e.type == SketchEntity::Type::Ellipse) {
            regions.push_back({entity_polyline(e), {i}, {}});
        } else if (e.type == SketchEntity::Type::Line || e.type == SketchEntity::Type::Arc ||
                   e.type == SketchEntity::Type::EllipseArc || e.type == SketchEntity::Type::BSpline) {
            std::vector<Vec2d> p = entity_polyline(e);
            if (p.size() >= 2) segs.push_back({std::move(p), i, false});
        }
    }
    for (size_t s = 0; s < segs.size(); ++s) {
        if (segs[s].used) continue;
        segs[s].used = true;
        std::vector<Vec2d> loop = segs[s].pts;
        std::vector<int> loop_ents = {segs[s].ent};
        const Vec2d start = loop.front();
        Vec2d cur = loop.back();
        bool extended = true;
        while (extended && !is_near(cur, start)) {
            extended = false;
            for (size_t t = 0; t < segs.size(); ++t) {
                if (segs[t].used) continue;
                const std::vector<Vec2d>& q = segs[t].pts;
                if (is_near(q.front(), cur)) { for (size_t k = 1; k < q.size(); ++k) loop.push_back(q[k]); cur = q.back(); }
                else if (is_near(q.back(), cur)) { for (int k = int(q.size()) - 2; k >= 0; --k) loop.push_back(q[k]); cur = q.front(); }
                else continue;
                segs[t].used = true;
                loop_ents.push_back(segs[t].ent);
                extended = true;
                break;
            }
        }
        if (is_near(cur, start) && loop.size() >= 4) {
            loop.pop_back();
            regions.push_back({std::move(loop), std::move(loop_ents), {}});
        }
    }
    auto poly_area = [](const std::vector<Vec2d>& q) {
        double a2 = 0.0;
        for (size_t i = 0, j = q.size() - 1; i < q.size(); j = i++) a2 += (q[j].x() + q[i].x()) * (q[j].y() - q[i].y());
        return std::abs(a2) * 0.5;
    };
    auto point_in = [](const Vec2d& pt, const std::vector<Vec2d>& q) {
        bool in = false;
        for (size_t i = 0, j = q.size() - 1; i < q.size(); j = i++) {
            const Vec2d& A = q[i]; const Vec2d& B = q[j];
            if (((A.y() > pt.y()) != (B.y() > pt.y())) && (pt.x() < (B.x() - A.x()) * (pt.y() - A.y()) / (B.y() - A.y()) + A.x())) in = !in;
        }
        return in;
    };
    auto interior_point = [](const std::vector<Vec2d>& q) {
        size_t k = 0;
        for (size_t i = 1; i < q.size(); ++i) if (q[i].y() < q[k].y() || (q[i].y() == q[k].y() && q[i].x() < q[k].x())) k = i;
        const Vec2d& v = q[k];
        Vec2d a = q[(k + q.size() - 1) % q.size()] - v, b = q[(k + 1) % q.size()] - v;
        const double la = a.norm(), lb = b.norm();
        if (la < 1e-12 || lb < 1e-12) return v;
        a /= la; b /= lb;
        Vec2d bis = a + b;
        if (bis.norm() < 1e-12) return v;
        bis.normalize();
        return Vec2d(v + bis * (1e-3 * std::min(la, lb)));
    };
    std::vector<Vec2d> probe(regions.size());
    for (size_t i = 0; i < regions.size(); ++i)
        if (regions[i].poly.size() >= 3) probe[i] = interior_point(regions[i].poly);
        else if (!regions[i].poly.empty()) probe[i] = regions[i].poly.front();
    for (size_t i = 0; i < regions.size(); ++i) {
        if (regions[i].poly.empty()) continue;
        int best = -1; double best_area = 0.0;
        for (size_t j = 0; j < regions.size(); ++j) {
            if (i == j || regions[j].poly.size() < 3 || !point_in(probe[i], regions[j].poly)) continue;
            const double a2 = poly_area(regions[j].poly);
            if (best < 0 || a2 < best_area) { best = int(j); best_area = a2; }
        }
        if (best >= 0) regions[best].holes.push_back(int(i));
    }
    for (RegionLoop& r : regions) r.defect = sketch_loop_defect(ents, r.ents, r.defect_at);
    return regions;
}

// DesignSketchTool::loop_report + McpControl sketch_report.
json loop_report(const std::vector<SketchEntity>& ents)
{
    json loops = json::array(), defects = json::array();
    const auto regs = region_loops(ents);
    for (const auto& r : regs) {
        double area = 0.0;
        if (r.ents.size() == 1 && (ents[r.ents[0]].type == SketchEntity::Type::Circle || ents[r.ents[0]].type == SketchEntity::Type::Ellipse)) {
            const SketchEntity& c = ents[r.ents[0]];
            area = c.type == SketchEntity::Type::Circle ? M_PI * c.radius * c.radius : M_PI * c.radius * c.rminor;
        } else {
            const double eps = 1e-6;
            double area2 = 0.0; bool exact = true; Vec2d cur(0, 0);
            for (size_t k = 0; k < r.ents.size() && exact; ++k) {
                const SketchEntity& e = ents[r.ents[k]];
                if (e.type != SketchEntity::Type::Line && e.type != SketchEntity::Type::Arc) { exact = false; break; }
                const Vec2d A = e.p0, B = e.p1;
                bool rev = false;
                if (k == 0) {
                    if (r.ents.size() > 1) { const SketchEntity& n = ents[r.ents[1]]; if ((A - n.p0).norm() < eps || (A - n.p1).norm() < eps) rev = true; }
                } else {
                    if ((B - cur).norm() < eps) rev = true;
                    else if ((A - cur).norm() >= eps) { exact = false; break; }
                }
                const Vec2d P = rev ? B : A, Q = rev ? A : B;
                if (e.type == SketchEntity::Type::Line) area2 += P.x() * Q.y() - Q.x() * P.y();
                else {
                    const double a0 = rev ? e.end_angle : e.start_angle, a1 = rev ? e.start_angle : e.end_angle;
                    area2 += e.center.x() * e.radius * (std::sin(a1) - std::sin(a0)) - e.center.y() * e.radius * (std::cos(a1) - std::cos(a0))
                           + e.radius * e.radius * (a1 - a0);
                }
                cur = Q;
            }
            if (exact) area = 0.5 * area2;
            else {
                double a = 0.0;
                for (size_t i = 0; i + 1 < r.poly.size(); ++i) a += r.poly[i].x() * r.poly[i + 1].y() - r.poly[i + 1].x() * r.poly[i].y();
                if (r.poly.size() > 2) a += r.poly.back().x() * r.poly.front().y() - r.poly.front().x() * r.poly.back().y();
                area = 0.5 * a;
            }
        }
        json l{{"entities", r.ents}, {"holes", r.holes}, {"closed", true}, {"area", area}, {"defect", r.defect}};
        if (r.defect) { l["defect_at"] = vec2_json(r.defect_at); defects.push_back(vec2_json(r.defect_at)); }
        loops.push_back(std::move(l));
    }
    std::vector<Vec2d> ends, open_ends;
    for (const SketchEntity& e : ents) {
        if (e.construction) continue;
        if (e.type == SketchEntity::Type::Line || e.type == SketchEntity::Type::Arc ||
            e.type == SketchEntity::Type::EllipseArc || e.type == SketchEntity::Type::BSpline) { ends.push_back(e.p0); ends.push_back(e.p1); }
    }
    const double eps = std::max(sketch_join_tol(), 1e-9);
    for (size_t i = 0; i < ends.size(); ++i) {
        int met = 0;
        for (size_t j = 0; j < ends.size(); ++j) if (i != j && (ends[i] - ends[j]).norm() < eps) ++met;
        if (met) continue;
        bool dup = false;
        for (const Vec2d& q : open_ends) if ((q - ends[i]).norm() < eps) { dup = true; break; }
        if (!dup) open_ends.push_back(ends[i]);
    }
    json oe = json::array();
    for (const Vec2d& p : open_ends) oe.push_back(vec2_json(p));
    const bool buildable = !regs.empty() && open_ends.empty() && defects.empty();
    return json{{"closed_loops", loops}, {"open_ends", oe}, {"defects", defects}, {"buildable", buildable}};
}

bool point_at(const std::vector<SketchEntity>& ents, int i, SketchPointRole r, Vec2d& out)
{
    if (i < 0 || i >= int(ents.size())) return false;
    const SketchEntity& e = ents[i];
    switch (r) {
    case SketchPointRole::P0: out = e.p0; return true;
    case SketchPointRole::P1: out = e.p1; return e.type != SketchEntity::Type::Point && e.type != SketchEntity::Type::Circle && e.type != SketchEntity::Type::Ellipse;
    case SketchPointRole::Center: out = e.center; return true;
    }
    return false;
}
int endpoint_roles(const SketchEntity& e, SketchPointRole out[2])
{
    switch (e.type) {
    case SketchEntity::Type::Line: case SketchEntity::Type::Arc: case SketchEntity::Type::BSpline: case SketchEntity::Type::EllipseArc:
        out[0] = SketchPointRole::P0; out[1] = SketchPointRole::P1; return 2;
    case SketchEntity::Type::Point: out[0] = SketchPointRole::P0; return 1;
    default: return 0;
    }
}
bool has_coincident(const std::vector<SketchEntityConstraintDef>& cons, int i, SketchPointRole ri, int j, SketchPointRole rj)
{
    for (const auto& c : cons)
        if (c.type == SketchConstraintType::Coincident &&
            ((c.ea == i && c.ra == ri && c.eb == j && c.rb == rj) || (c.ea == j && c.ra == rj && c.eb == i && c.rb == ri)))
            return true;
    return false;
}
bool entity_ref_point(const SketchEntity& e, Vec2d& out)
{
    switch (e.type) {
    case SketchEntity::Type::Point: out = e.p0; return true;
    case SketchEntity::Type::Circle: case SketchEntity::Type::Arc: case SketchEntity::Type::Ellipse: case SketchEntity::Type::EllipseArc:
        out = e.center; return true;
    default: return false;
    }
}
void translate_entity(SketchEntity& e, const Vec2d& d)
{
    e.p0 += d; e.p1 += d; e.center += d;
    for (auto& cp : e.ctrl) cp += d;
}

// Why constraint `c` cannot apply to the entities it names ("" = it can). libslvs ASSERTS (the
// module traps, every document lost) on a diameter of a line, equal-radius of lines, and similar
// kind mismatches, and Orca's solver checks only that the handles exist; so every solve here goes
// through safe_solve and every request through check_constraint_kinds (-32602).
std::string constraint_kind_problem(const SketchEntityConstraintDef& c, const std::vector<SketchEntity>& E)
{
    using T = SketchEntity::Type;
    using CT = SketchConstraintType;
    const int n = int(E.size());
    auto is = [&](int i, std::initializer_list<T> ts) { if (i < 0 || i >= n) return false; for (T t : ts) if (E[i].type == t) return true; return false; };
    auto line = [&](int i) { return i == -3 || i == -4 || is(i, {T::Line}); };   // -3/-4 = the sketch X/Y axes
    auto round = [&](int i) { return is(i, {T::Arc, T::Circle}); };
    const std::string name = kConstraintTypes[int(c.type)];
    switch (c.type) {
    case CT::Radius: case CT::Diameter:
        return round(c.ea) ? "" : name + " needs a circle or an arc (entity " + std::to_string(c.ea) + " is not one)";
    case CT::EqualRadius:
        return round(c.ea) && round(c.eb) ? "" : "equal_radius needs two circles / arcs";
    case CT::Tangent:
        if (!(line(c.ea) || round(c.ea)) || !(line(c.eb) || round(c.eb))) return "tangent needs lines, arcs or circles";
        return round(c.ea) || round(c.eb) ? "" : "tangent needs an arc or a circle (two lines are never tangent: use collinear or parallel)";
    case CT::Concentric:
        return is(c.ea, {T::Arc, T::Circle, T::Ellipse, T::EllipseArc}) && is(c.eb, {T::Arc, T::Circle, T::Ellipse, T::EllipseArc})
                   ? "" : "concentric needs two circles, arcs or ellipses";
    case CT::EqualLength: case CT::Parallel: case CT::Perpendicular: case CT::Angle: case CT::Collinear:
        return line(c.ea) && line(c.eb) ? "" : name + " needs two lines";
    case CT::PointOnLine: case CT::Midpoint:
        return line(c.eb) ? "" : name + " needs a line as its second entity (eb)";
    case CT::Symmetric:
        return line(c.ec) ? "" : "symmetric needs a line as its axis (ec)";
    default:
        return "";
    }
}
void check_constraint_kinds(const std::vector<SketchEntityConstraintDef>& cs, const std::vector<SketchEntity>& E)
{
    for (size_t i = 0; i < cs.size(); ++i) {
        const std::string why = constraint_kind_problem(cs[i], E);
        if (!why.empty()) bad_param("constraint " + std::to_string(i) + ": " + why);
    }
}
// sketch_solve / sketch_solve_drag without the constraints whose entity kinds do not fit (they are
// reported in `skipped`, like Orca's own unrepresentable references): never traps.
SketchSolveResult safe_solve(std::vector<SketchEntity>& E, const std::vector<SketchEntityConstraintDef>& cons,
                             int drag_ei = -1, SketchPointRole drag_role = SketchPointRole::P0)
{
    std::vector<SketchEntityConstraintDef> use;
    std::vector<int> map, dropped;
    for (size_t i = 0; i < cons.size(); ++i) {
        if (constraint_kind_problem(cons[i], E).empty()) { use.push_back(cons[i]); map.push_back(int(i)); }
        else dropped.push_back(int(i));
    }
    SketchSolveResult r = drag_ei >= 0 ? sketch_solve_drag(E, use, drag_ei, drag_role) : sketch_solve(E, use);
    for (int& b : r.bad) b = map[b];
    for (int& b : r.skipped) b = map[b];
    r.skipped.insert(r.skipped.end(), dropped.begin(), dropped.end());
    std::sort(r.skipped.begin(), r.skipped.end());
    return r;
}

void session_solve(SketchSession& k)
{
    if (k.cons.empty()) { k.last = SketchSolveResult{}; k.last.ok = true; k.last.dof = -1; return; }
    k.last = safe_solve(k.ents, k.cons);
    k.solved_any = true;
}
// Add a batch; keep it only if the system still solves (DesignSketchTool::try_add_constraints).
bool try_add_constraints(SketchSession& k, const std::vector<SketchEntityConstraintDef>& cands)
{
    if (cands.empty()) return true;
    const size_t mark = k.cons.size();
    k.cons.insert(k.cons.end(), cands.begin(), cands.end());
    SketchSolveResult r = safe_solve(k.ents, k.cons);   // writes back only on success
    if (r.ok) { k.last = r; return true; }
    k.cons.resize(mark);
    return false;
}
// DesignSketchTool::infer_auto_constraints for a scripted add (zero tolerances: only relations
// that are exactly true; relational inference only for gesture-sized batches).
void infer_auto_constraints(SketchSession& k, int base)
{
    const int n = int(k.ents.size());
    if (base < 0 || base >= n) return;
    std::vector<SketchEntityConstraintDef> coincs;
    for (int i = base; i < n; ++i) {
        SketchPointRole ir[2]; const int ni = endpoint_roles(k.ents[i], ir);
        for (int a = 0; a < ni; ++a) {
            Vec2d pa; if (!point_at(k.ents, i, ir[a], pa)) continue;
            for (int j = 0; j < n; ++j) {
                if (j == i) continue;
                SketchPointRole jr[2]; const int nj = endpoint_roles(k.ents[j], jr);
                for (int b = 0; b < nj; ++b) {
                    if (j >= base && j < i) continue;
                    Vec2d pb; if (!point_at(k.ents, j, jr[b], pb)) continue;
                    if ((pa - pb).squaredNorm() > 0.0) continue;
                    if (has_coincident(k.cons, i, ir[a], j, jr[b])) continue;
                    if (has_coincident(coincs, i, ir[a], j, jr[b])) continue;
                    SketchEntityConstraintDef c;
                    c.type = SketchConstraintType::Coincident; c.ea = i; c.ra = ir[a]; c.eb = j; c.rb = jr[b];
                    coincs.push_back(c);
                }
            }
        }
    }
    try_add_constraints(k, coincs);
    std::vector<SketchEntityConstraintDef> axes;
    for (int i = base; i < n; ++i) {
        if (k.ents[i].type != SketchEntity::Type::Line) continue;
        auto ax = infer_axis_constraint(k.ents[i].p0, k.ents[i].p1, 0.0);
        if (!ax) continue;
        SketchEntityConstraintDef c;
        c.type = *ax; c.ea = i; c.ra = SketchPointRole::P0; c.eb = i; c.rb = SketchPointRole::P1;
        axes.push_back(c);
    }
    if (!try_add_constraints(k, axes))
        for (const auto& c : axes) try_add_constraints(k, {c});
    if (n - base <= 16) {
        std::vector<SketchEntityConstraintDef> rels;
        for (int i = base; i < n; ++i) { auto r = infer_relations(k.ents, i, 0.0, 0.0); rels.insert(rels.end(), r.begin(), r.end()); }
        try_add_constraints(k, rels);
    }
}

json solve_json(const SketchSession& k)
{
    return json{{"solve_ok", k.last.ok}, {"dof", k.last.dof}, {"result", k.last.result}, {"bad", k.last.bad}, {"skipped", k.last.skipped}};
}

// =============================================================================================
// Methods
// =============================================================================================
using Handler = std::function<json(Ctx&)>;
std::map<std::string, Handler>& methods() { static std::map<std::string, Handler> m; return m; }
struct Reg { Reg(const char* n, Handler h) { methods()[n] = std::move(h); } };
#define CAD_METHOD(NAME) json m_##NAME(Ctx& c); Reg reg_##NAME(#NAME, m_##NAME); json m_##NAME(Ctx& c)

// The face/edge accessors take ids from the caller: refuse a stale generation first.
TopoDS_Face face_arg(const DocState& s, const json& p, const TopoDS_Shape& body, const char* key = "face")
{
    check_generation(s, p);
    const int id = int_param(p, key);
    TopoDS_Face f = GeometryEngine::face_by_index(body, id);
    if (f.IsNull()) bad_param(std::string(key) + " id " + std::to_string(id) + " out of range (have " +
                              std::to_string(GeometryEngine::face_count(body)) + ")");
    return f;
}
TopoDS_Edge edge_arg(const DocState& s, const json& p, const TopoDS_Shape& body, const char* key = "edge")
{
    check_generation(s, p);
    const int id = int_param(p, key);
    TopoDS_Edge e = GeometryEngine::edge_by_index(body, id);
    if (e.IsNull()) bad_param(std::string(key) + " id " + std::to_string(id) + " out of range (have " +
                              std::to_string(GeometryEngine::edge_count(body)) + ")");
    return e;
}
int feature_index_arg(const DocState& s, const json& p, const char* key = "index")
{
    const int i = int_param(p, key);
    if (i < 0 || i >= int(s.doc.features.size()))
        bad_param(std::string(key) + " " + std::to_string(i) + " out of range (have " + std::to_string(s.doc.features.size()) + " features)");
    return i;
}

// ---- module ----------------------------------------------------------------------------------
CAD_METHOD(describe_tools)
{
    (void)c;
    json names = json::array();
    for (const auto& kv : methods()) names.push_back(kv.first);
    json types = json::array();
    for (const char* t : kFeatureTypes) types.push_back(t);
    json fields = json::array();
    CadFeature tmp;
    visit_fields(tmp, FieldDescribe{fields});
    json bindable = json::array();
    for (const json& f : fields)
        if (CadDocument::is_bindable_field(f["name"].get<std::string>())) bindable.push_back(f["name"]);
    if (CadDocument::is_bindable_field("thread_diameter")) bindable.push_back("thread_diameter");
    return json{{"module", "orca-cad"}, {"methods", names}, {"feature_types", types}, {"fields", fields},
                {"bindable_fields", bindable}, {"entity_types", kEntityTypes}, {"constraint_types", kConstraintTypes},
                {"id_lifetime",
                 "Face and edge ids are TopExp ordinals of ONE body's current topology (faces: TopExp_Explorer order, "
                 "edges: TopExp::MapShapes order) and expire when the model rebuilds. Pass the 'generation' they came "
                 "with back on every id-taking call; a mismatch is refused (-32010)."}};
}

CAD_METHOD(thread_standards)
{
    (void)c;
    static const char* const series[] = {"metric_coarse", "metric_fine", "unc", "unf"};
    json a = json::array();
    for (const ThreadSpec& t : thread_standards())
        a.push_back(json{{"name", t.name}, {"major_diameter", t.major_diameter_mm}, {"pitch", t.pitch_mm},
                         {"series", series[int(t.series)]}, {"depth", t.thread_depth_mm()},
                         {"minor_diameter", t.minor_diameter_mm()}, {"internal_depth", t.internal_depth_mm()},
                         {"imperial", t.imperial()}});
    return json{{"threads", a}};
}

// Same table as CadDocument.cpp kHoleStdTable (add_hole_standard resolves designations there).
CAD_METHOD(hole_standards)
{
    (void)c;
    struct E { const char* d; double cl, cbd, cbdp, csd, csa; };
    static const E table[] = {
        {"M3", 3.4, 6.5, 3.4, 6.7, 90}, {"M4", 4.5, 8.0, 4.4, 9.0, 90}, {"M5", 5.5, 10.0, 5.4, 11.3, 90},
        {"M6", 6.6, 11.0, 6.4, 13.4, 90}, {"M8", 9.0, 15.0, 8.6, 17.9, 90}, {"M10", 11.0, 18.0, 10.6, 22.4, 90},
        {"#6-32 UNC", 3.7, 6.4, 3.8, 7.1, 82}, {"#8-32 UNC", 4.5, 7.9, 4.6, 8.4, 82}, {"#10-24 UNC", 5.1, 9.5, 5.3, 9.8, 82},
        {"1/4-20 UNC", 6.8, 11.1, 6.9, 12.9, 82}, {"5/16-18 UNC", 8.4, 13.5, 8.6, 16.1, 82}, {"3/8-16 UNC", 10.1, 15.9, 10.2, 19.4, 82}};
    json a = json::array();
    for (const E& e : table)
        a.push_back(json{{"designation", e.d}, {"clearance", e.cl}, {"cbore_diameter", e.cbd}, {"cbore_depth", e.cbdp},
                         {"csink_diameter", e.csd}, {"csink_angle", e.csa}});
    return json{{"holes", a}};
}

std::string write_temp(const std::string& name, const uint8_t* data, size_t len)
{
    const std::string path = "/tmp/" + name;
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    f.write(reinterpret_cast<const char*>(data), std::streamsize(len));
    if (!f) fail(kFailure, "cannot write " + path);
    return path;
}
std::vector<TopoDS_Shape> read_step_blob(const Ctx& c)
{
    if (c.blob_len == 0) bad_param("the STEP file goes in the blob");
    const std::string path = write_temp("cad_import.step", c.blob, c.blob_len);
    std::string err;
    std::vector<TopoDS_Shape> solids = GeometryEngine::read_step_solids(path, err);
    ::unlink(path.c_str());
    if (solids.empty()) fail(kFailure, err.empty() ? "no solids in STEP" : err);
    return solids;
}

CAD_METHOD(step_to_mesh)
{
    const double lin = opt<double>(c.params, "linear_deflection", 0.003), ang = opt<double>(c.params, "angular_deflection", 0.5);
    std::vector<TopoDS_Shape> solids = read_step_blob(c);
    json meshes = json::array();
    for (size_t i = 0; i < solids.size(); ++i) {
        std::vector<int> tf;
        TriangleMesh tm = SketchEngine::tessellate(solids[i], tf, lin, ang);
        meshes.push_back(plate_mesh(tm, &tf, int(i), c.out_blob));
    }
    return json{{"solids", solids.size()}, {"meshes", meshes}};
}

// ---- document lifecycle ----------------------------------------------------------------------
int register_doc(std::unique_ptr<DocState> s)
{
    const int id = g_next_doc++;
    g_docs[id] = std::move(s);
    return id;
}

CAD_METHOD(doc_new)
{
    auto s = std::make_unique<DocState>();
    if (c.params.contains("modeling_origin")) s->doc.modeling_origin = vec3_from(c.params["modeling_origin"], "modeling_origin");
    s->doc.auto_close_loops = opt<bool>(c.params, "auto_close_loops", true);
    s->replay_cache = opt<bool>(c.params, "replay_cache", true);
    rebuild(*s);
    const uint64_t gen = s->doc.topo_generation;
    return json{{"doc", register_doc(std::move(s))}, {"generation", gen}};
}

// A Design-tab recipe starts with its format number (1..6 so far) and a feature count; a framed
// one (v5+) holds a 4-byte length per feature after that. Anything else is some other file.
std::string not_a_recipe(const uint8_t* b, size_t n)
{
    uint32_t v = 0, count = 0;
    std::memcpy(&v, b, 4);
    if (v == 0 || v > 1000)
        return "not a Design-tab recipe (Metadata/orca_cad.bin): it starts with " + std::to_string(v) +
               ", not a recipe format number";
    if (n < 8) return "not a Design-tab recipe (Metadata/orca_cad.bin): " + std::to_string(n) + " bytes is too short";
    std::memcpy(&count, b + 4, 4);
    if (v >= 5 ? count > (n - 8) / 4 : count > n)
        return "not a Design-tab recipe (Metadata/orca_cad.bin): it claims " + std::to_string(count) +
               " features in " + std::to_string(n) + " bytes";
    return {};
}

CAD_METHOD(doc_open)
{
    if (c.blob_len < 4) fail(kUnreadable, "CAD data could not be read: the recipe is empty");
    if (const std::string why = not_a_recipe(c.blob, c.blob_len); !why.empty()) fail(kNotARecipe, why);
    auto s = std::make_unique<DocState>();
    s->replay_cache = opt<bool>(c.params, "replay_cache", true);
    if (c.params.contains("modeling_origin")) s->doc.modeling_origin = vec3_from(c.params["modeling_origin"], "modeling_origin");
    s->original = c.blob_string();
    std::memcpy(&s->version, s->original.data(), 4);
    s->dirty = false;
    std::string blob = s->original;
    if (s->version > CadDocument::ORCA_CAD_RECIPE_VERSION) {
        // Newer than this kernel: show it if the v5/v6 framing still reads it, never write it.
        s->read_only = true;
        s->read_only_reason = "saved with a newer version of the Design tab (format v" + std::to_string(s->version) +
                              ", this kernel reads up to v" + std::to_string(CadDocument::ORCA_CAD_RECIPE_VERSION) + ")";
        const uint32_t six = CadDocument::ORCA_CAD_RECIPE_VERSION;
        std::memcpy(blob.data(), &six, 4);
    }
    const auto t0 = std::chrono::steady_clock::now();
    bool ok = false;
    try {
        ok = s->doc.deserialize_recipe(blob);
    } catch (const Standard_Failure& e) {
        s->doc.error = std::string("OCCT: ") + (e.GetMessageString() ? e.GetMessageString() : "failure");
    }
    s->last_rebuild_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    if (!ok && s->doc.features.empty()) {
        // deserialize refused the blob itself (v1-v3, garbage), not a failed rebuild.
        if (!s->read_only || s->doc.error.find("could not be read") != std::string::npos) {
            const std::string why = s->read_only ? s->read_only_reason + "; " + s->doc.error : s->doc.error;
            fail(kUnreadable, why.empty() ? "CAD data could not be read" : why);
        }
    }
    if (!ok) {
        const bool any_solid = std::any_of(s->doc.features.begin(), s->doc.features.end(),
            [](const CadFeature& f) { return f.enabled && CadDocument::produces_body(f.type); });
        if (!any_solid && s->doc.error.empty()) ok = true;
    }
    if (s->version >= 5) scan_tails(blob, s->tails);
    s->tails.feat.resize(s->doc.features.size());
    if (!ok) {
        // as every failed rebuild: no bodies (deserialize_recipe's recompute returns before replacing them)
        s->doc.bodies.clear();
        s->doc.body = TopoDS_Shape();
    }
    if (ok) { s->state_ok = true; s->state_fail = {}; }
    const FailInfo fail = ok ? FailInfo{} : s->state_fail;
    json r{{"version", s->version}, {"read_only", s->read_only}, {"ok", ok}, {"error", s->doc.error},
           {"features", s->doc.features.size()}, {"bodies", s->doc.bodies.size()}, {"generation", s->doc.topo_generation},
           {"ms", s->last_rebuild_ms}, {"failed_feature", fail.index}, {"failed_feature_name", fail.name},
           {"cancelled", g_call.cancelled},
           {"unknown", {{"feature_tail_bytes", s->tails.feature_bytes()}, {"block_tail_bytes", s->tails.block_bytes()},
                        {"trailing_bytes", s->tails.trailing.size()}}}};
    if (s->read_only) r["read_only_reason"] = s->read_only_reason;
    r["doc"] = register_doc(std::move(s));
    return r;
}

CAD_METHOD(doc_save)
{
    DocState& s = doc_of(c.params);
    const bool canonical = opt<bool>(c.params, "canonical", false);
    std::string bytes;
    bool verbatim = false;
    if (opt<std::string>(c.params, "writer", "") == "orca") bytes = s.doc.serialize_recipe();   // Orca's own writer (drops unknown bytes): for tests
    else if (s.read_only || (!s.dirty && !canonical && !s.original.empty())) { bytes = s.original; verbatim = true; }
    else bytes = write_recipe(s.doc, s.tails);
    c.out_blob.put(bytes.data(), 1, bytes.size());
    uint32_t v = 0;
    if (bytes.size() >= 4) std::memcpy(&v, bytes.data(), 4);
    return json{{"bytes", bytes.size()}, {"version", v}, {"verbatim", verbatim}};
}

CAD_METHOD(doc_close)
{
    doc_of(c.params);
    g_docs.erase(c.params["doc"].get<int>());
    return json::object();
}

CAD_METHOD(doc_list)
{
    (void)c;
    json a = json::array();
    for (const auto& [id, s] : g_docs)
        a.push_back(json{{"doc", id}, {"features", s->doc.features.size()}, {"dirty", s->dirty}, {"read_only", s->read_only}});
    return json{{"docs", a}};
}

CAD_METHOD(describe_scene) { return describe_scene(doc_of(c.params)); }

CAD_METHOD(set_document)
{
    DocState& s = writable(c.params);
    Mutation m(s);
    if (c.params.contains("modeling_origin")) s.doc.modeling_origin = vec3_from(c.params["modeling_origin"], "modeling_origin");
    if (c.params.contains("auto_close_loops")) s.doc.auto_close_loops = opt<bool>(c.params, "auto_close_loops", true);
    if (c.params.contains("replay_cache")) {
        s.replay_cache = opt<bool>(c.params, "replay_cache", true);
        if (!s.replay_cache) s.runs.clear();
    }
    return build_result(s, m.finish(rebuild(s)));
}

// Undo / redo: move one snapshot from `from` to the current state (rebuilt), the current state to
// `to`. A cancelled rebuild puts everything back.
json history_step(DocState& s, std::vector<Snapshot>& from, std::vector<Snapshot>& to)
{
    bool ok = false;
    if (!from.empty()) {
        Snapshot cur = take_snapshot(s);
        Derived cur_derived = take_derived(s);
        Snapshot target = from.back();
        restore_snapshot(s, target);
        if (g_call.cancelled) {
            s.doc.variables = cur.variables;
            s.tails.feat = cur.tails;
            s.doc.modeling_origin = cur.origin;
            s.doc.auto_close_loops = cur.auto_close;
            restore_derived(s, cur_derived);
        } else {
            from.pop_back();
            to.push_back(std::move(cur));
            s.dirty = true;
            ok = true;
        }
    }
    const FailInfo f = s.state_ok ? FailInfo{} : s.state_fail;
    return json{{"ok", ok}, {"can_undo", !s.undo.empty()}, {"can_redo", !s.redo.empty()},
                {"generation", s.doc.topo_generation}, {"error", s.doc.error}, {"rebuild_ok", s.state_ok},
                {"failed_feature", f.index}, {"failed_feature_name", f.name},
                {"replayed_from", g_call.replayed_from}, {"cancelled", g_call.cancelled}};
}

CAD_METHOD(undo)
{
    DocState& s = writable(c.params);
    if (s.sketch) fail(kSketchState, "a sketch is open: use sketch_undo, or commit/cancel it first");
    return history_step(s, s.undo, s.redo);
}

CAD_METHOD(redo)
{
    DocState& s = writable(c.params);
    if (s.sketch) fail(kSketchState, "a sketch is open: use sketch_redo, or commit/cancel it first");
    return history_step(s, s.redo, s.undo);
}

// ---- features (generic) ----------------------------------------------------------------------
CAD_METHOD(list_features)
{
    DocState& s = doc_of(c.params);
    const bool full = opt<bool>(c.params, "full", false);
    json a = json::array();
    for (size_t i = 0; i < s.doc.features.size(); ++i) {
        const CadFeature& f = s.doc.features[i];
        json j = full ? feature_json(f, false) : json{{"type", feature_type_name(f.type)}, {"name", f.name}, {"enabled", f.enabled}};
        j["index"] = int(i);
        a.push_back(std::move(j));
    }
    return json{{"features", a}, {"generation", s.doc.topo_generation}};
}

CAD_METHOD(get_feature)
{
    DocState& s = doc_of(c.params);
    const int i = feature_index_arg(s, c.params);
    json f = feature_json(s.doc.features[i], opt<bool>(c.params, "include_brep", false));
    const size_t bytes = f["brep_bytes"].get<size_t>();
    return json{{"index", i}, {"feature", f}, {"brep_bytes", bytes}};
}

// Every feature-index field must name an EARLIER feature (history runs top to bottom).
void check_feature_refs(const CadDocument& doc, int fi)
{
    CadFeature& f = const_cast<CadFeature&>(doc.features[fi]);
    auto chk = [&](int ref, const char* what) {
        if (ref >= 0 && ref >= fi) bad_param(std::string(what) + " " + std::to_string(ref) + " must name an earlier feature than " + std::to_string(fi));
    };
    chk(f.sketch_ref, "sketch_ref"); chk(f.sweep_path_ref, "sweep_path_ref"); chk(f.pattern_curve_sketch, "pattern_curve_sketch");
    chk(f.rib_sketch_ref, "rib_sketch_ref"); chk(f.mate_cs_a, "mate_cs_a"); chk(f.mate_cs_b, "mate_cs_b");
    for (int r : f.loft_profile_refs) chk(r, "loft_profile_refs");
}

CAD_METHOD(add_feature)
{
    DocState& s = writable(c.params);
    CadFeature f;
    f.type = CadFeatureType(name_index(kFeatureTypes, c.params.value("type", json()), "feature type"));
    // New features use the current semantics (what Orca's add_* and GUI set for every new one).
    f.thread_major_nominal = true;
    f.pattern_inclusive = true;
    if (f.type == CadFeatureType::Plane || f.type == CadFeatureType::Pattern || f.type == CadFeatureType::Hole ||
        f.type == CadFeatureType::Thread || f.type == CadFeatureType::Sketch || f.type == CadFeatureType::Helix ||
        f.type == CadFeatureType::Cut || f.type == CadFeatureType::Mirror || f.type == CadFeatureType::Project)
        f.plane.origin = s.doc.modeling_origin;
    if (c.params.contains("fields")) apply_fields(f, c.params["fields"], s.doc.modeling_origin, false);
    f.name = opt<std::string>(c.params, "name", f.name.empty() ? std::string(feature_type_name(f.type)) + " " + std::to_string(s.doc.features.size() + 1) : f.name);
    if (f.type == CadFeatureType::Import && f.imported_solid.IsNull()) bad_param("an Import feature needs 'fields.brep' (or use import_step / import_mesh)");
    if ((f.type == CadFeatureType::Fillet || f.type == CadFeatureType::Chamfer) && !f.dressup_edges.empty() && f.dressup_edge < 0)
        f.dressup_edge = f.dressup_edges.front();
    if (c.params.contains("fields")) check_generation_of_fields(s, c.params, c.params["fields"]);
    Mutation m(s);
    s.doc.features.push_back(f);
    try { check_feature_refs(s.doc, int(s.doc.features.size()) - 1); }
    catch (...) { s.doc.features.pop_back(); throw; }
    const bool ok = m.finish(rebuild_checked(s, int(s.doc.features.size()) - 1));
    return build_result(s, ok, json{{"index", ok ? int(s.doc.features.size()) - 1 : -1}});
}

CAD_METHOD(set_feature)
{
    DocState& s = writable(c.params);
    const int i = feature_index_arg(s, c.params);
    if (!c.params.contains("fields")) bad_param("set_feature needs 'fields'");
    check_generation_of_fields(s, c.params, c.params["fields"]);
    CadFeature edited = s.doc.features[i];
    apply_fields(edited, c.params["fields"], s.doc.modeling_origin, false);
    if ((edited.type == CadFeatureType::Fillet || edited.type == CadFeatureType::Chamfer) && c.params["fields"].contains("dressup_edges"))
        edited.dressup_edge = edited.dressup_edges.empty() ? -1 : edited.dressup_edges.front();
    Mutation m(s);
    s.doc.features[i] = edited;
    try { check_feature_refs(s.doc, i); }
    catch (...) { s.doc.features[i] = m.snap.features[i]; throw; }
    return build_result(s, m.finish(rebuild_checked(s, i)), json{{"index", i}});
}

CAD_METHOD(rename_feature)
{
    DocState& s = writable(c.params);
    const int i = feature_index_arg(s, c.params);
    const std::string name = opt<std::string>(c.params, "name", "");
    if (name.empty()) bad_param("rename_feature needs a non-empty 'name'");
    Mutation m(s);
    s.doc.features[i].name = name;
    m.keys[i].second = name;    // same feature: carry its tail
    for (CadBody& b : s.doc.bodies) if (b.source_feature == i && !b.has_user_name) b.name = name;
    m.finish(true);
    return json{{"ok", true}, {"index", i}, {"name", name}};
}

CAD_METHOD(suppress_feature)
{
    DocState& s = writable(c.params);
    const bool suppressed = opt<bool>(c.params, "suppressed", true);
    if (c.params.contains("indices")) {
        // CubbySlicer batch: toggle several features with ONE rebuild (one undo step). Applied
        // as is (no body-reference re-pointing, like `force`); without `force` a failing rebuild
        // rolls the whole batch back.
        std::vector<int> idx;
        for (const json& v : c.params["indices"]) {
            if (!v.is_number_integer() || v.get<int>() < 0 || v.get<int>() >= int(s.doc.features.size()))
                bad_param("indices: feature index out of range");
            idx.push_back(v.get<int>());
        }
        Mutation m(s);
        for (int i : idx) s.doc.features[i].enabled = !suppressed;
        const bool rebuilt = rebuild(s);
        const bool force = opt<bool>(c.params, "force", false);
        // kept = the toggle is in the document now: always when the rebuild succeeds, and with
        // `force` also when it does not (then ok:false) — never after a cancelled rebuild.
        const bool kept = rebuilt || (force && !g_call.cancelled);
        m.finish(kept);
        if (!rebuilt && kept) {
            s.doc.error = s.doc.error.empty() ? "rebuild failed" : s.doc.error;
            g_call.have_fail = true; g_call.fail = s.state_fail;   // the kept state's failure
        }
        return build_result(s, rebuilt, json{{"indices", idx}, {"kept", kept}});
    }
    const int i = feature_index_arg(s, c.params);
    Mutation m(s);
    bool ok = false;
    try { ok = s.doc.set_feature_enabled(i, !suppressed); }
    catch (const Standard_Failure& e) { s.doc.error = std::string("OCCT: ") + e.GetMessageString(); }
    // force: keep the toggle even though the rebuild still fails (Orca's history edits are all
    // transactional, so a document that already fails to rebuild — e.g. two features that are each
    // too large — could otherwise never be repaired one suppression at a time). The toggle is
    // applied as is; body references are not re-pointed (Orca re-points them from the last good
    // build, which a failing document does not have).
    if (!ok && !g_call.cancelled && opt<bool>(c.params, "force", false) && s.doc.features[i].enabled == suppressed) {
        s.doc.features[i].enabled = !suppressed;
        const bool rebuilt = rebuild(s);
        m.finish(true);
        if (!rebuilt) { g_call.have_fail = true; g_call.fail = s.state_fail; }   // the kept state's failure
        return build_result(s, rebuilt, json{{"index", i}, {"enabled", s.doc.features[i].enabled}, {"kept", true}});
    }
    ok = m.finish(ok, true);
    return build_result(s, ok, json{{"index", i}, {"enabled", s.doc.features[i].enabled}, {"kept", ok}});
}

CAD_METHOD(move_feature)
{
    DocState& s = writable(c.params);
    const int i = feature_index_arg(s, c.params);
    int to = c.params.contains("to") ? int_param(c.params, "to") : i + int_param(c.params, "delta");
    to = std::max(0, std::min(int(s.doc.features.size()) - 1, to));
    Mutation m(s);
    bool ok = true;
    int at = i;
    while (ok && at != to) {
        const int step = to > at ? 1 : -1;
        ok = s.doc.move_feature(at, step);
        if (ok) at += step;
    }
    if (!ok) {
        // move_feature rolled back its own step; undo the steps before it.
        m.finish(false, true);
        return build_result(s, false, json{{"index", i}});
    }
    return build_result(s, m.finish(true), json{{"index", at}});
}

CAD_METHOD(delete_feature)
{
    DocState& s = writable(c.params);
    const int i = feature_index_arg(s, c.params);
    // Orca's cascade (remove_feature): a Sketch takes the features that consume it as a profile.
    std::vector<int> removed{i};
    if (s.doc.features[i].type == CadFeatureType::Sketch)
        for (int j = 0; j < int(s.doc.features.size()); ++j) {
            const CadFeature& f = s.doc.features[j];
            if (f.sketch_ref == i || f.sweep_path_ref == i || f.rib_sketch_ref == i) removed.push_back(j);
        }
    std::sort(removed.begin(), removed.end());
    removed.erase(std::unique(removed.begin(), removed.end()), removed.end());
    Mutation m(s);
    bool ok = false;
    try { ok = s.doc.remove_feature(i); }
    catch (const Standard_Failure& e) { s.doc.error = std::string("OCCT: ") + e.GetMessageString(); }
    ok = m.finish(ok, true);
    return build_result(s, ok, json{{"removed", ok ? json(removed) : json::array()}});
}

CAD_METHOD(feature_preview)
{
    DocState& s = doc_of(c.params);
    CadFeature f;
    f.type = CadFeatureType(name_index(kFeatureTypes, c.params.value("type", json()), "feature type"));
    f.thread_major_nominal = true;
    f.pattern_inclusive = true;
    f.plane.origin = s.doc.modeling_origin;
    if (c.params.contains("fields")) apply_fields(f, c.params["fields"], s.doc.modeling_origin, false);
    TriangleMesh merged;
    std::vector<TriangleMesh> per_body;
    std::string err;
    bool ok = false;
    try { ok = s.doc.preview(f, merged, per_body, err); }
    catch (const Standard_Failure& e) { err = std::string("OCCT: ") + e.GetMessageString(); }
    json meshes = json::array();
    if (ok) for (size_t i = 0; i < per_body.size(); ++i) meshes.push_back(plate_mesh(per_body[i], nullptr, int(i), c.out_blob));
    return json{{"ok", ok}, {"error", err}, {"meshes", meshes}};
}

CAD_METHOD(faces_made_by)
{
    DocState& s = doc_of(c.params);
    const int i = feature_index_arg(s, c.params);
    // From the record of the rebuild that produced the current bodies (no replay); Orca's own
    // scratch replay only when there is no complete record of the current history.
    // replay:true = always Orca's replay (tests compare the two)
    const bool recorded = !opt<bool>(c.params, "replay", false) && s.run && s.run->complete && s.state_ok &&
                          i < int(s.run->steps.size());
    const auto faces = recorded ? s.doc.faces_made_by(i, s.run->steps[i]->before, s.run->steps[i]->after)
                                : s.doc.faces_made_by(i);
    json a = json::array();
    for (const auto& [b, f] : faces) a.push_back(json::array({b, f}));
    return json{{"faces", a}, {"generation", s.doc.topo_generation}, {"replayed", !recorded}};
}

json mesh_all(DocState& s, const json& p, BlobOut& b)
{
    const bool edges = opt<bool>(p, "edges", true);
    std::vector<int> which;
    if (p.contains("bodies") && p["bodies"].is_array()) for (const json& j : p["bodies"]) which.push_back(j.get<int>());
    else for (int i = 0; i < int(s.doc.bodies.size()); ++i) which.push_back(i);
    json meshes = json::array();
    for (int i : which) {
        if (i < 0 || i >= int(s.doc.bodies.size())) bad_param("body " + std::to_string(i) + " out of range");
        if (s.doc.bodies[i].shape.IsNull()) continue;
        meshes.push_back(display_mesh(s.doc.bodies[i].shape, i, b, edges, s.doc.linear_deflection, s.doc.angular_deflection));
    }
    return meshes;
}

CAD_METHOD(regenerate)
{
    DocState& s = doc_of(c.params);
    if (c.params.contains("linear_deflection")) s.doc.linear_deflection = num(c.params["linear_deflection"], "linear_deflection");
    if (c.params.contains("angular_deflection")) s.doc.angular_deflection = num(c.params["angular_deflection"], "angular_deflection");
    const std::vector<BodyMeta> meta = body_meta(s.doc);
    // full: replay everything (ignore the replay cache; it is kept for later rebuilds)
    g_call.no_cache = opt<bool>(c.params, "full", false);
    Derived before = take_derived(s);
    bool ok = rebuild(s);
    if (g_call.cancelled) { restore_derived(s, before); ok = false; }   // nothing changed: back as it was
    apply_body_meta(s.doc, meta);
    const FailInfo f = ok ? FailInfo{} : (g_call.cancelled ? FailInfo{} : s.state_fail);
    json r{{"ok", ok}, {"error", g_call.cancelled ? std::string("rebuild cancelled") : s.doc.error},
           {"generation", s.doc.topo_generation}, {"bodies", bodies_summary(s.doc)},
           {"mate_conflicts", mate_conflicts_json(s.doc)}, {"ms", s.last_rebuild_ms},
           {"failed_feature", f.index}, {"failed_feature_name", f.name},
           {"replayed_from", g_call.replayed_from}, {"cancelled", g_call.cancelled}};
    if (opt<bool>(c.params, "mesh", true)) r["meshes"] = mesh_all(s, c.params, c.out_blob);
    return r;
}

CAD_METHOD(get_mesh)
{
    DocState& s = doc_of(c.params);
    return json{{"generation", s.doc.topo_generation}, {"meshes", mesh_all(s, c.params, c.out_blob)}};
}

// ---- variables / expressions (McpControl) ----------------------------------------------------
// The identifiers an expression names, scanned as Orca's evaluate_variables does (built-ins
// skipped; a number's digits are not an identifier, so "2x" names x and "1e3" names e3).
std::set<std::string> expr_identifiers(const std::string& e)
{
    static const std::set<std::string> builtin{"pi", "sqrt", "abs", "sin", "cos", "tan", "min", "max", "deg", "rad"};
    std::set<std::string> out;
    for (size_t i = 0; i < e.size();) {
        const unsigned char ch = static_cast<unsigned char>(e[i]);
        if (std::isalpha(ch) || ch == '_') {
            const size_t start = i;
            while (i < e.size() && (std::isalnum(static_cast<unsigned char>(e[i])) || e[i] == '_')) ++i;
            std::string id = e.substr(start, i - start);
            if (!builtin.count(id)) out.insert(std::move(id));
            continue;
        }
        ++i;
    }
    return out;
}
// The variables `names` reach through the variables' own expressions (including themselves).
std::set<std::string> variable_closure(const std::map<std::string, std::string>& vars, std::set<std::string> names)
{
    std::vector<std::string> todo(names.begin(), names.end());
    while (!todo.empty()) {
        const std::string n = todo.back();
        todo.pop_back();
        auto it = vars.find(n);
        if (it == vars.end()) continue;
        for (const std::string& id : expr_identifiers(it->second))
            if (names.insert(id).second) todo.push_back(id);
    }
    return names;
}
// Does any feature's expression binding (enabled or not: bound values are written into the
// fields, and saved) read variable `name`, directly or through other variables?
bool feature_exprs_read(const CadDocument& d, const std::string& name)
{
    std::set<std::string> direct;
    for (const CadFeature& f : d.features)
        for (const auto& [field, e] : f.expr) { (void)field; for (const std::string& id : expr_identifiers(e)) direct.insert(id); }
    return variable_closure(d.variables, direct).count(name) > 0;
}
// Each variable's value or why it does not evaluate (it, or a variable it uses, is unknown,
// cyclic or malformed), from Orca's evaluator on the variables it reaches.
void evaluate_each_variable(const std::map<std::string, std::string>& vars, json& values, json& errors)
{
    values = json::object();
    errors = json::object();
    for (const auto& [name, e] : vars) {
        (void)e;
        std::map<std::string, std::string> sub;
        for (const std::string& n : variable_closure(vars, {name}))
            if (auto it = vars.find(n); it != vars.end()) sub[n] = it->second;
        try {
            const auto v = CadDocument::evaluate_variable_values(sub);
            values[name] = v.at(name);
        } catch (const std::exception& ex) {
            errors[name] = ex.what();
        }
    }
}

CAD_METHOD(set_variable)
{
    DocState& s = writable(c.params);
    const std::string name = opt<std::string>(c.params, "name", "");
    if (name.empty()) bad_param("set_variable needs 'name'");
    const std::string expr = opt<std::string>(c.params, "expr", "");
    Mutation m(s);
    if (expr.empty()) s.doc.variables.erase(name);
    else s.doc.variables[name] = expr;
    // No feature expression reads it (directly or through other variables): the rebuild would
    // bind the same values into the same features and replay the same history. Skip it when
    // the document currently rebuilds and every variable still evaluates (an expression that
    // does not evaluate fails the rebuild even when unused, so that path still rebuilds).
    bool evaluates = true;
    try { CadDocument::evaluate_variable_values(s.doc.variables); } catch (const std::exception&) { evaluates = false; }
    const bool skip = s.state_ok && evaluates && !feature_exprs_read(s.doc, name);
    const bool ok = m.finish(skip ? true : rebuild(s));
    return build_result(s, ok, json{{"name", name}, {"rebuilt", !skip}});
}

CAD_METHOD(set_feature_expr)
{
    DocState& s = writable(c.params);
    const int fi = feature_index_arg(s, c.params, "feature");
    const std::string field = opt<std::string>(c.params, "field", "");
    const std::string expr = opt<std::string>(c.params, "expr", "");
    if (!expr.empty() && !CadDocument::is_bindable_field(field))
        bad_param("field '" + field + "' cannot be driven by an expression (describe_tools.bindable_fields)");
    Mutation m(s);
    if (expr.empty()) s.doc.features[fi].expr.erase(field);
    else s.doc.features[fi].expr[field] = expr;
    const bool ok = m.finish(rebuild(s));
    return build_result(s, ok, json{{"feature", fi}, {"field", field}});
}

// ---- build verbs (McpControl handlers, panel -> document) --------------------------------------
CAD_METHOD(extrude)
{
    DocState& s = writable(c.params);
    const json& p = c.params;
    const double d = opt<double>(p, "distance", 10.0);
    if (d <= 0) bad_param("distance must be > 0");
    BooleanMode mode = bool_from(opt<std::string>(p, "boolean", "new"));
    const std::string end = opt<std::string>(p, "end", "blind");
    int sk = -1, e = -1;
    const bool ok = add_and_rebuild(s, [&] {
        if (p.contains("face")) {                       // CubbySlicer: push/pull a body face
            check_generation(s, p);
            e = s.doc.add_extrude_face(int_param(p, "face"), d, false, mode, opt<std::string>(p, "name", "Extrude"));
            s.doc.features[e].target_body = target_body_arg(p, s.doc);
        } else {
            if (p.contains("sketch")) {                 // CubbySlicer: extrude an existing sketch
                sk = feature_index_arg(s, p, "sketch");
                if (s.doc.features[sk].type != CadFeatureType::Sketch) bad_param("'sketch' must be a Sketch feature");
            } else {
                SketchPlane pl = plane_arg(p, s.doc);
                SketchProfile prof;
                if (p.contains("profile")) {
                    for (const json& q : p["profile"]) prof.points.push_back(vec2_from(q, "profile"));
                    prof.closed = true;
                    if (prof.points.size() < 3) bad_param("profile needs at least 3 points");
                    sk = s.doc.add_sketch_profile(prof, pl, "Sketch");
                } else {
                    const double w = opt<double>(p, "width", 20.0), h = opt<double>(p, "height", 20.0);
                    if (w <= 0 || h <= 0) bad_param("width and height must be > 0");
                    sk = s.doc.add_sketch(SketchShape::Rectangle, pl, w, h, 0.0, "Sketch");
                }
            }
            e = s.doc.add_extrude(sk, d, false, mode, opt<std::string>(p, "name", "Extrude"));
            if (p.contains("body")) s.doc.features[e].target_body = target_body_arg(p, s.doc);
        }
        CadFeature& fe = s.doc.features[e];
        fe.flip = opt<bool>(p, "flip", false);
        fe.taper_deg = opt<double>(p, "taper", 0.0);
        if (end == "symmetric") fe.extrude_end = ExtrudeEnd::Symmetric;
        else if (end == "two_sided") { fe.extrude_end = ExtrudeEnd::TwoSided; fe.distance2 = opt<double>(p, "distance2", d); }
        else if (end == "through_all") fe.extrude_end = ExtrudeEnd::ThroughAll;
        else if (end == "up_to_face") { check_generation(s, p); fe.extrude_end = ExtrudeEnd::UpToFace; fe.up_to_face = opt<int>(p, "up_to_face", -1); }
        else if (end == "up_to_vertex") { fe.extrude_end = ExtrudeEnd::UpToVertex; fe.up_to_point = vec3_from(p.value("up_to_point", json()), "up_to_point"); }
        else fe.extrude_end = ExtrudeEnd::Blind;
    });
    return build_result(s, ok, json{{"sketch_index", ok ? sk : -1}, {"extrude_index", ok ? e : -1}, {"end", end}});
}

CAD_METHOD(revolve)
{
    DocState& s = writable(c.params);
    const json& p = c.params;
    const double angle = opt<double>(p, "angle", 360.0);
    const int axis = opt<int>(p, "axis", 0);
    const bool flip = opt<bool>(p, "flip", false);
    BooleanMode mode = bool_from(opt<std::string>(p, "boolean", "new"));
    int sk = -1, r = -1;
    const bool ok = add_and_rebuild(s, [&] {
        if (p.contains("sketch")) {
            sk = feature_index_arg(s, p, "sketch");
            if (s.doc.features[sk].type != CadFeatureType::Sketch) bad_param("'sketch' must be a Sketch feature");
        } else {
            SketchPlane pl = plane_arg(p, s.doc);
            SketchProfile prof;
            if (p.contains("profile")) {
                for (const json& q : p["profile"]) prof.points.push_back(vec2_from(q, "profile"));
                prof.closed = true;
                if (prof.points.size() < 3) bad_param("profile needs at least 3 points");
                sk = s.doc.add_sketch_profile(prof, pl, "Sketch");
            } else {
                const double w = opt<double>(p, "width", 20.0), h = opt<double>(p, "height", 10.0);
                if (w <= 0 || h <= 0) bad_param("width and height must be > 0");
                sk = s.doc.add_sketch(SketchShape::Rectangle, pl, w, h, 0.0, "Sketch");
            }
        }
        r = s.doc.add_revolve(sk, angle, axis, flip, mode, opt<std::string>(p, "name", "Revolve"));
        s.doc.features[r].revolve_axis_entity = opt<int>(p, "axis_entity", -1);
        if (p.contains("body")) s.doc.features[r].target_body = target_body_arg(p, s.doc);
    });
    return build_result(s, ok, json{{"sketch_index", ok ? sk : -1}, {"revolve_index", ok ? r : -1}});
}

std::vector<int> edge_ids_arg(const json& p, const char* verb)
{
    if (!p.contains("edge")) bad_param(std::string(verb) + " needs 'edge' (id or array of ids)");
    std::vector<int> ids;
    const json& e = p["edge"];
    if (e.is_array()) for (const json& v : e) ids.push_back(v.get<int>());
    else ids.push_back(e.get<int>());
    if (ids.empty()) bad_param(std::string(verb) + ": 'edge' is an empty array");
    return ids;
}
void check_edges_exist(const DocState& s, int bi, const std::vector<int>& ids)
{
    const int b = bi < 0 ? int(s.doc.bodies.size()) - 1 : bi;
    const int n = GeometryEngine::edge_count(s.doc.bodies[b].shape);
    for (int id : ids) if (id < 0 || id >= n) bad_param("edge id " + std::to_string(id) + " out of range (body " + std::to_string(b) + " has " + std::to_string(n) + ")");
}

json dressup(Ctx& c, bool fillet)
{
    DocState& s = writable(c.params);
    const json& p = c.params;
    const char* verb = fillet ? "fillet" : "chamfer";
    check_generation(s, p);
    const double size = opt<double>(p, fillet ? "radius" : "distance", 1.0);
    if (size <= 0) bad_param(std::string(fillet ? "radius" : "distance") + " must be > 0");
    if (s.doc.bodies.empty()) bad_param(std::string("no body to ") + verb);
    const int bi = target_body_arg(p, s.doc);
    std::vector<int> edges;
    std::optional<FaceGroup> group;
    if (p.contains("face_group")) group = FaceGroup(name_index(kFaceGroups, p["face_group"], "face_group"));
    else { edges = edge_ids_arg(p, verb); check_edges_exist(s, bi, edges); }
    int f = -1;
    const bool ok = add_and_rebuild(s, [&] {
        const std::string name = opt<std::string>(p, "name", fillet ? "Fillet" : "Chamfer");
        if (group) f = fillet ? s.doc.add_fillet(size, *group, name) : s.doc.add_chamfer(size, *group, name);
        else f = fillet ? s.doc.add_fillet(size, edges, name) : s.doc.add_chamfer(size, edges, name);
        if (bi >= 0) s.doc.features[f].target_body = bi;
    });
    return build_result(s, ok, json{{fillet ? "fillet_index" : "chamfer_index", ok ? f : -1},
                                    {"body", bi < 0 ? int(s.doc.bodies.size()) - 1 : bi}});
}
CAD_METHOD(fillet) { return dressup(c, true); }
CAD_METHOD(chamfer) { return dressup(c, false); }

json hole_common(Ctx& c, int kind)   // 0 hole, 1 hole_styled, 2 hole_standard
{
    DocState& s = writable(c.params);
    const json& p = c.params;
    if (s.doc.bodies.empty()) bad_param("no body to drill");
    const double x = opt<double>(p, "x", 0.0), y = opt<double>(p, "y", 0.0);
    const double depth = opt<double>(p, "depth", 10.0);
    const SketchPlane pl = plane_arg(p, s.doc);
    const int bi = target_body_arg(p, s.doc);
    int h = -1;
    const bool ok = add_and_rebuild(s, [&] {
        const std::string name = opt<std::string>(p, "name", "Hole");
        if (kind == 0) {
            const double dia = opt<double>(p, "diameter", 5.0);
            if (dia <= 0) bad_param("diameter must be > 0");
            h = s.doc.add_hole(dia, depth, opt<bool>(p, "through", false), x, y, pl, name);
        } else if (kind == 1) {
            const double dia = opt<double>(p, "diameter", 5.0);
            if (dia <= 0) bad_param("diameter must be > 0");
            h = s.doc.add_hole_styled(dia, depth, opt<bool>(p, "through", true), x, y, pl, opt<int>(p, "style", 0),
                                      opt<double>(p, "cbore_diameter", 0.0), opt<double>(p, "cbore_depth", 0.0),
                                      opt<double>(p, "csink_diameter", 0.0), opt<double>(p, "csink_angle", 90.0),
                                      opt<std::string>(p, "standard", ""), name);
        } else {
            const std::string desig = opt<std::string>(p, "designation", "");
            if (desig.empty()) bad_param("designation is required");
            try { h = s.doc.add_hole_standard(desig, opt<int>(p, "style", 0), opt<bool>(p, "through", true), depth, x, y, pl, name); }
            catch (const std::runtime_error& e) { bad_param(e.what()); }
        }
        if (bi >= 0) s.doc.features[h].target_body = bi;
    });
    return build_result(s, ok, json{{"hole_index", ok ? h : -1}});
}
CAD_METHOD(hole) { return hole_common(c, 0); }
CAD_METHOD(hole_styled) { return hole_common(c, 1); }
CAD_METHOD(hole_standard) { return hole_common(c, 2); }

// CubbySlicer addition: Orca's control surface has no thread verb.
CAD_METHOD(thread)
{
    DocState& s = writable(c.params);
    const json& p = c.params;
    double radius = opt<double>(p, "radius", 0.0), pitch = opt<double>(p, "pitch", 0.0), depth = opt<double>(p, "depth", 0.0);
    const bool internal = opt<bool>(p, "internal", false);
    if (p.contains("diameter")) radius = 0.5 * num(p["diameter"], "diameter");
    if (p.contains("designation")) {
        const ThreadSpec* t = find_thread_standard(opt<std::string>(p, "designation", ""));
        if (!t) bad_param("unknown thread designation (see thread_standards)");
        radius = 0.5 * t->major_diameter_mm;
        if (pitch <= 0) pitch = t->pitch_mm;
        if (depth <= 0) depth = internal ? t->internal_depth_mm() : t->thread_depth_mm();
    }
    if (radius <= 0) bad_param("thread needs 'radius', 'diameter' or 'designation'");
    if (pitch <= 0) pitch = 1.0;
    if (depth <= 0) depth = 0.6134 * pitch;
    const SketchPlane pl = plane_arg(p, s.doc);
    const int bi = target_body_arg(p, s.doc);
    int t = -1;
    const bool ok = add_and_rebuild(s, [&] {
        t = s.doc.add_thread(radius, pitch, opt<double>(p, "height", 10.0), depth, internal,
                             opt<double>(p, "x", 0.0), opt<double>(p, "y", 0.0), pl, opt<std::string>(p, "name", "Thread"));
        if (bi >= 0) s.doc.features[t].target_body = bi;
    });
    return build_result(s, ok, json{{"thread_index", ok ? t : -1}});
}

CAD_METHOD(boolean)
{
    DocState& s = writable(c.params);
    const json& p = c.params;
    BooleanMode m = bool_from(opt<std::string>(p, "op", "subtract"));
    if (m == BooleanMode::New) bad_param("op must be union | subtract | intersect");
    const int target = opt<int>(p, "target", 0), tool = opt<int>(p, "tool", 1);
    const int n = int(s.doc.bodies.size());
    if (target < 0 || target >= n || tool < 0 || tool >= n) bad_param("target/tool body index out of range (have " + std::to_string(n) + ")");
    if (target == tool) bad_param("target and tool must differ");
    const int tf = opt<int>(p, "target_face", -1), lf = opt<int>(p, "tool_face", -1);
    if (tf >= 0 || lf >= 0) check_generation(s, p);
    int b = -1;
    const bool ok = add_and_rebuild(s, [&] {
        b = s.doc.add_boolean(m, target, tool, opt<bool>(p, "keep_tool", false), opt<double>(p, "tolerance", 0.0), tf, lf,
                              opt<std::string>(p, "name", "Boolean"));
    });
    return build_result(s, ok, json{{"boolean_index", ok ? b : -1}});
}

CAD_METHOD(pattern)
{
    DocState& s = writable(c.params);
    const json& p = c.params;
    const int count = opt<int>(p, "count", 3);
    if (count < 1) bad_param("count must be >= 1");
    if (s.doc.bodies.empty()) bad_param("no body to pattern");
    const int bi = target_body_arg(p, s.doc);
    int pi = -1;
    const bool ok = add_and_rebuild(s, [&] {
        pi = s.doc.add_pattern(opt<bool>(p, "circular", false), count, opt<double>(p, "spacing", 10.0), opt<int>(p, "dir", 0),
                               opt<double>(p, "angle", 360.0), bi, opt<std::string>(p, "name", "Pattern"));
        s.doc.features[pi].plane = plane_arg(p, s.doc);
    });
    return build_result(s, ok, json{{"pattern_index", ok ? pi : -1}});
}

CAD_METHOD(pattern_on_curve)
{
    DocState& s = writable(c.params);
    const json& p = c.params;
    const int count = opt<int>(p, "count", 3);
    if (count < 1) bad_param("count must be >= 1");
    const int sketch = feature_index_arg(s, p, "sketch");
    const int entity = int_param(p, "entity");
    if (s.doc.bodies.empty()) bad_param("no body to pattern");
    const int bi = target_body_arg(p, s.doc);
    int pi = -1;
    const bool ok = add_and_rebuild(s, [&] { pi = s.doc.add_pattern_on_curve(count, sketch, entity, bi, opt<std::string>(p, "name", "PatternOnCurve")); });
    return build_result(s, ok, json{{"pattern_index", ok ? pi : -1}});
}

CAD_METHOD(shell)
{
    DocState& s = writable(c.params);
    const json& p = c.params;
    const double t = opt<double>(p, "thickness", 1.0);
    if (t <= 0) bad_param("thickness must be > 0");
    const int face = opt<int>(p, "face", -1);
    if (face >= 0) check_generation(s, p);
    if (s.doc.bodies.empty()) bad_param("no body to shell");
    const int bi = target_body_arg(p, s.doc);
    int si = -1;
    const bool ok = add_and_rebuild(s, [&] { si = s.doc.add_shell(t, face, bi, opt<std::string>(p, "name", "Shell")); },
                                    int(s.doc.features.size()));
    return build_result(s, ok, json{{"shell_index", ok ? si : -1}});
}

CAD_METHOD(rib)
{
    DocState& s = writable(c.params);
    const json& p = c.params;
    if (s.doc.bodies.empty()) bad_param("no body to rib");
    const int sketch = feature_index_arg(s, p, "sketch");
    const int entity = int_param(p, "entity");
    const int bi = target_body_arg(p, s.doc);
    int ri = -1;
    const bool ok = add_and_rebuild(s, [&] {
        ri = s.doc.add_rib(sketch, entity, opt<double>(p, "thickness", 2.0), opt<double>(p, "depth", 10.0), bi, opt<std::string>(p, "name", "Rib"));
    });
    return build_result(s, ok, json{{"rib_index", ok ? ri : -1}});
}

CAD_METHOD(draft)
{
    DocState& s = writable(c.params);
    const json& p = c.params;
    check_generation(s, p);
    const int face = int_param(p, "face");
    if (s.doc.bodies.empty()) bad_param("no body to draft");
    const int bi = target_body_arg(p, s.doc);
    int d = -1;
    const bool ok = add_and_rebuild(s, [&] { d = s.doc.add_draft(opt<double>(p, "angle", 5.0), face, bi, opt<std::string>(p, "name", "Draft")); });
    return build_result(s, ok, json{{"draft_index", ok ? d : -1}});
}

CAD_METHOD(mirror)
{
    DocState& s = writable(c.params);
    const json& p = c.params;
    if (s.doc.bodies.empty()) bad_param("no body to mirror");
    const int bi = target_body_arg(p, s.doc);
    const SketchPlane pl = plane_arg(p, s.doc, "plane", "XZ");
    const std::string mode = opt<std::string>(p, "mode", "new");
    int mi = -1;
    const bool ok = add_and_rebuild(s, [&] {
        mi = s.doc.add_mirror(pl, bi, mode == "add" ? BooleanMode::Add : BooleanMode::New, opt<std::string>(p, "name", "Mirror"));
        s.doc.features[mi].mirror_keep_original = opt<bool>(p, "keep_original", true);
    });
    return build_result(s, ok, json{{"mirror_index", ok ? mi : -1}});
}

CAD_METHOD(transform)
{
    DocState& s = writable(c.params);
    const json& p = c.params;
    if (s.doc.bodies.empty()) bad_param("no body to transform");
    const int bi = target_body_arg(p, s.doc);
    int ti = -1;
    const bool ok = add_and_rebuild(s, [&] {
        ti = s.doc.add_transform(bi, Vec3d(opt<double>(p, "dx", 0), opt<double>(p, "dy", 0), opt<double>(p, "dz", 0)),
                                 Vec3d(opt<double>(p, "axis_x", 0), opt<double>(p, "axis_y", 0), opt<double>(p, "axis_z", 1)),
                                 Vec3d(opt<double>(p, "pivot_x", 0), opt<double>(p, "pivot_y", 0), opt<double>(p, "pivot_z", 0)),
                                 opt<double>(p, "angle", 0.0), opt<bool>(p, "copy", false), opt<std::string>(p, "name", "Transform"));
    });
    return build_result(s, ok, json{{"transform_index", ok ? ti : -1}});
}

CAD_METHOD(thicken)
{
    DocState& s = writable(c.params);
    const json& p = c.params;
    check_generation(s, p);
    const int face = int_param(p, "face");
    if (s.doc.bodies.empty()) bad_param("no body to thicken");
    const int bi = target_body_arg(p, s.doc);
    int ti = -1;
    const bool ok = add_and_rebuild(s, [&] {
        ti = s.doc.add_thicken(bi, face, opt<double>(p, "thickness", 2.0), opt<bool>(p, "flip", false), opt<std::string>(p, "name", "Thicken"));
    });
    return build_result(s, ok, json{{"thicken_index", ok ? ti : -1}});
}

CAD_METHOD(split)
{
    DocState& s = writable(c.params);
    const json& p = c.params;
    check_generation(s, p);
    const int face = int_param(p, "face");
    if (s.doc.bodies.empty()) bad_param("no body to split");
    const int bi = target_body_arg(p, s.doc);
    int si = -1;
    const bool ok = add_and_rebuild(s, [&] {
        si = s.doc.add_split_by_face(bi, opt<int>(p, "face_body", -1), face, opt<bool>(p, "keep_upper", true),
                                     opt<bool>(p, "keep_lower", true), opt<std::string>(p, "name", "Split"));
    });
    return build_result(s, ok, json{{"split_index", ok ? si : -1}});
}

// CubbySlicer: plane cut (Orca's Cut card; McpControl only has split-by-face).
CAD_METHOD(cut)
{
    DocState& s = writable(c.params);
    const json& p = c.params;
    if (s.doc.bodies.empty()) bad_param("no body to cut");
    const int bi = target_body_arg(p, s.doc);
    const SketchPlane pl = plane_arg(p, s.doc);
    int ci = -1;
    const bool ok = add_and_rebuild(s, [&] {
        ci = s.doc.add_cut(pl, opt<double>(p, "offset", 0.0), opt<bool>(p, "flip", false), opt<bool>(p, "keep_upper", true),
                           opt<bool>(p, "keep_lower", false), bi, opt<std::string>(p, "name", "Cut"));
    });
    return build_result(s, ok, json{{"cut_index", ok ? ci : -1}});
}

CAD_METHOD(project)
{
    DocState& s = writable(c.params);
    const json& p = c.params;
    check_generation(s, p);
    std::vector<int> edges;
    if (p.contains("edges")) for (const json& e : p["edges"]) edges.push_back(e.get<int>());
    const SketchPlane pl = plane_arg(p, s.doc);
    int pi = -1;
    const bool ok = add_and_rebuild(s, [&] {
        pi = s.doc.add_project_edges(opt<int>(p, "source_body", -1), edges, opt<int>(p, "face", -1), pl, opt<std::string>(p, "name", "Project"));
    });
    return build_result(s, ok, json{{"project_index", ok ? pi : -1}});
}

CAD_METHOD(delete_face)
{
    DocState& s = writable(c.params);
    const json& p = c.params;
    check_generation(s, p);
    std::vector<int> faces;
    if (p.contains("faces")) for (const json& f : p["faces"]) faces.push_back(f.get<int>());
    if (faces.empty()) bad_param("delete_face needs 'faces'");
    if (s.doc.bodies.empty()) bad_param("no body");
    const int bi = target_body_arg(p, s.doc);
    int fi = -1;
    const bool ok = add_and_rebuild(s, [&] { fi = s.doc.add_delete_face(bi, faces, opt<std::string>(p, "name", "DeleteFace")); });
    return build_result(s, ok, json{{"feature_index", ok ? fi : -1}});
}

CAD_METHOD(bridge)
{
    DocState& s = writable(c.params);
    const json& p = c.params;
    const int sketch = feature_index_arg(s, p, "sketch");
    int ei = -1;
    const bool ok = add_and_rebuild(s, [&] {
        ei = s.doc.add_bridge(sketch, int_param(p, "ent_a"), opt<int>(p, "end_a", 1), int_param(p, "ent_b"), opt<int>(p, "end_b", 0),
                              opt<std::string>(p, "name", "Bridge"));
        if (ei < 0) bad_param("bridge: invalid sketch or entity references");
    });
    return build_result(s, ok, json{{"sketch_index", sketch}, {"entity_index", ok ? ei : -1}});
}

CAD_METHOD(axis)
{
    DocState& s = writable(c.params);
    const json& p = c.params;
    static const char* const types[] = {"two_points", "face_normal", "cylinder", "plane_intersection", "along_edge"};
    const AxisType t = AxisType(name_index(types, p.value("type", json("two_points")), "axis type"));
    if (p.contains("face") || p.contains("edge")) check_generation(s, p);
    int ai = -1;
    const bool ok = add_and_rebuild(s, [&] {
        ai = s.doc.add_axis(t, opt<std::string>(p, "name", "Axis"));
        CadFeature& f = s.doc.features[ai];
        if (p.contains("p1")) f.axis_p1 = vec3_from(p["p1"], "p1");
        if (p.contains("p2")) f.axis_p2 = vec3_from(p["p2"], "p2");
        f.axis_body = opt<int>(p, "body", f.axis_body); f.axis_face = opt<int>(p, "face", f.axis_face);
        f.axis_edge = opt<int>(p, "edge", f.axis_edge); f.axis_plane_a = opt<int>(p, "plane_a", f.axis_plane_a);
        f.axis_plane_b = opt<int>(p, "plane_b", f.axis_plane_b);
    });
    return build_result(s, ok, json{{"axis_index", ok ? ai : -1}});
}

CAD_METHOD(coordsys)
{
    DocState& s = writable(c.params);
    const json& p = c.params;
    static const char* const types[] = {"point_world", "face_and_direction"};
    const CoordSysType t = CoordSysType(name_index(types, p.value("type", json("point_world")), "coordsys type"));
    if (p.contains("face") || p.contains("edge")) check_generation(s, p);
    const Vec3d pt = p.contains("point") ? vec3_from(p["point"], "point") : Vec3d::Zero();
    int ci = -1;
    const bool ok = add_and_rebuild(s, [&] {
        ci = s.doc.add_coordsys(t, pt, opt<std::string>(p, "name", "CoordSys"));
        CadFeature& f = s.doc.features[ci];
        f.coordsys_body = opt<int>(p, "body", f.coordsys_body); f.coordsys_face = opt<int>(p, "face", f.coordsys_face);
        f.coordsys_edge = opt<int>(p, "edge", f.coordsys_edge);
        if (p.contains("x_hint")) f.coordsys_x_hint = vec3_from(p["x_hint"], "x_hint");
    });
    return build_result(s, ok, json{{"coordsys_index", ok ? ci : -1}});
}

CAD_METHOD(helix)
{
    DocState& s = writable(c.params);
    const json& p = c.params;
    const SketchPlane pl = plane_arg(p, s.doc);
    int hi = -1;
    const bool ok = add_and_rebuild(s, [&] {
        hi = s.doc.add_helix(pl, opt<double>(p, "radius", 10.0), opt<double>(p, "pitch", 5.0), opt<double>(p, "height", 20.0),
                             opt<bool>(p, "left_handed", false), opt<double>(p, "taper_deg", 0.0), opt<std::string>(p, "name", "Helix"));
    });
    return build_result(s, ok, json{{"helix_index", ok ? hi : -1}});
}

CAD_METHOD(mate)
{
    DocState& s = writable(c.params);
    const json& p = c.params;
    int mi = -1;
    const bool ok = add_and_rebuild(s, [&] {
        mi = s.doc.add_mate(opt<int>(p, "kind", 0), int_param(p, "cs_a"), int_param(p, "cs_b"), opt<double>(p, "offset", 0.0),
                            opt<double>(p, "angle", 0.0), opt<bool>(p, "flip", false), opt<std::string>(p, "name", "Mate"));
    });
    return build_result(s, ok, json{{"mate_index", ok ? mi : -1}});
}

CAD_METHOD(mate_options)
{
    DocState& s = doc_of(c.params);
    json a = json::array();
    for (const auto& o : s.doc.mate_options(int_param(c.params, "cs_a"), int_param(c.params, "cs_b")))
        a.push_back(json{{"kind", o.kind}, {"viable", o.viable}, {"reason", o.reason}});
    return json{{"options", a}};
}

// CubbySlicer: sweep / loft / datum plane verbs (Orca has the add_* functions, no socket verb).
CAD_METHOD(sweep)
{
    DocState& s = writable(c.params);
    const json& p = c.params;
    const int prof = feature_index_arg(s, p, "profile"), path = feature_index_arg(s, p, "path");
    int si = -1;
    const bool ok = add_and_rebuild(s, [&] {
        si = s.doc.add_sweep(prof, path, bool_from(opt<std::string>(p, "boolean", "new")), opt<std::string>(p, "name", "Sweep"));
        if (p.contains("body")) s.doc.features[si].target_body = target_body_arg(p, s.doc);
    });
    return build_result(s, ok, json{{"sweep_index", ok ? si : -1}});
}

CAD_METHOD(loft)
{
    DocState& s = writable(c.params);
    const json& p = c.params;
    if (!p.contains("profiles") || !p["profiles"].is_array()) bad_param("loft needs 'profiles' (array of Sketch feature indices)");
    std::vector<int> profiles;
    for (const json& j : p["profiles"]) profiles.push_back(j.get<int>());
    int li = -1;
    const bool ok = add_and_rebuild(s, [&] {
        li = s.doc.add_loft(profiles, opt<bool>(p, "ruled", false), bool_from(opt<std::string>(p, "boolean", "new")), opt<std::string>(p, "name", "Loft"));
        if (p.contains("body")) s.doc.features[li].target_body = target_body_arg(p, s.doc);
    });
    return build_result(s, ok, json{{"loft_index", ok ? li : -1}});
}

CAD_METHOD(plane_datum)
{
    DocState& s = writable(c.params);
    const json& p = c.params;
    int pi = -1;
    const bool ok = add_and_rebuild(s, [&] {
        pi = s.doc.add_plane(opt<int>(p, "base", 0), opt<double>(p, "offset", 20.0), opt<double>(p, "angle", 0.0), opt<int>(p, "axis", 0),
                             opt<std::string>(p, "name", "Plane"));
        if (p.contains("fields")) apply_fields(s.doc.features[pi], p["fields"], s.doc.modeling_origin, false);
    });
    return build_result(s, ok, json{{"plane_index", ok ? pi : -1}});
}

json surface_verb(Ctx& c, const std::function<int(DocState&, const json&)>& add)
{
    DocState& s = writable(c.params);
    int fi = -1;
    const bool ok = add_and_rebuild(s, [&] { fi = add(s, c.params); });
    return build_result(s, ok, json{{"feature_index", ok ? fi : -1}});
}
CAD_METHOD(surface_extrude)
{
    return surface_verb(c, [](DocState& s, const json& p) {
        return s.doc.add_surface_extrude(feature_index_arg(s, p, "sketch"), opt<double>(p, "distance", 10.0), opt<std::string>(p, "name", "SurfaceExtrude"));
    });
}
CAD_METHOD(surface_revolve)
{
    return surface_verb(c, [](DocState& s, const json& p) {
        return s.doc.add_surface_revolve(feature_index_arg(s, p, "sketch"), opt<double>(p, "angle", 360.0), opt<int>(p, "axis", 0),
                                         opt<std::string>(p, "name", "SurfaceRevolve"));
    });
}
CAD_METHOD(thicken_surface)
{
    return surface_verb(c, [](DocState& s, const json& p) {
        if (s.doc.bodies.empty()) bad_param("no sheet body to thicken");
        return s.doc.add_thicken_surface(target_body_arg(p, s.doc), opt<double>(p, "thickness", 2.0), opt<bool>(p, "flip", false),
                                         opt<std::string>(p, "name", "ThickenSurface"));
    });
}
CAD_METHOD(surface_offset)
{
    return surface_verb(c, [](DocState& s, const json& p) {
        if (s.doc.bodies.empty()) bad_param("no sheet body to offset");
        return s.doc.add_surface_offset(target_body_arg(p, s.doc), opt<double>(p, "offset", 1.0), opt<std::string>(p, "name", "SurfaceOffset"));
    });
}
CAD_METHOD(surface_loft)
{
    return surface_verb(c, [](DocState& s, const json& p) {
        if (!p.contains("profiles") || !p["profiles"].is_array()) bad_param("surface_loft needs 'profiles'");
        std::vector<int> profiles;
        for (const json& j : p["profiles"]) profiles.push_back(j.get<int>());
        return s.doc.add_surface_loft(profiles, opt<bool>(p, "ruled", false), opt<std::string>(p, "name", "SurfaceLoft"));
    });
}
CAD_METHOD(surface_fill)
{
    return surface_verb(c, [](DocState& s, const json& p) {
        return s.doc.add_surface_fill(feature_index_arg(s, p, "sketch"), opt<std::string>(p, "name", "SurfaceFill"));
    });
}

CAD_METHOD(import_step)
{
    DocState& s = writable(c.params);
    std::vector<TopoDS_Shape> solids = read_step_blob(c);
    int first = int(s.doc.features.size());
    const std::string base = opt<std::string>(c.params, "name", "STEP");
    const bool ok = add_and_rebuild(s, [&] {
        for (const TopoDS_Shape& sh : solids) {
            CadFeature f;
            f.type = CadFeatureType::Import;
            f.name = base + std::to_string(int(s.doc.features.size()) + 1);
            f.imported_solid = as_saved(sh);
            f.mode = BooleanMode::New;
            s.doc.features.push_back(f);
        }
    });
    return build_result(s, ok, json{{"imported", solids.size()}, {"first_feature", ok ? first : -1}});
}

CAD_METHOD(import_mesh)
{
    DocState& s = writable(c.params);
    const json& p = c.params;
    const json pj = p.value("positions", json()), ij = p.value("indices", json());
    if (!pj.is_object() || !ij.is_object()) bad_param("import_mesh needs positions:{offset,count} and indices:{offset,count} (blob)");
    const uint64_t po = cs::get_u64(pj, "offset"), pc = cs::get_u64(pj, "count"), io = cs::get_u64(ij, "offset"), ic = cs::get_u64(ij, "count");
    if (po % 4 || io % 4 || pc % 3 || ic % 3 || pc == 0 || ic == 0) bad_param("mesh arrays must be 4-byte aligned triples");
    if (!cs::checked_range(po, pc, 4, c.blob_len) || !cs::checked_range(io, ic, 4, c.blob_len)) bad_param("mesh arrays out of the blob");
    indexed_triangle_set its;
    const float* P = reinterpret_cast<const float*>(c.blob + po);
    const uint32_t* I = reinterpret_cast<const uint32_t*>(c.blob + io);
    for (uint64_t i = 0; i < pc; i += 3) {
        if (!std::isfinite(P[i]) || !std::isfinite(P[i + 1]) || !std::isfinite(P[i + 2])) bad_param("non-finite vertex");
        its.vertices.emplace_back(P[i], P[i + 1], P[i + 2]);
    }
    for (uint64_t i = 0; i < ic; i += 3) {
        if (I[i] >= pc / 3 || I[i + 1] >= pc / 3 || I[i + 2] >= pc / 3) bad_param("triangle index out of range");
        its.indices.emplace_back(int(I[i]), int(I[i + 1]), int(I[i + 2]));
    }
    GeometryEngine::MeshBrepStats st;
    const TopoDS_Shape shape = GeometryEngine::mesh_to_brep(its, opt<double>(p, "tolerance", 0.01), opt<double>(p, "merge_angle_deg", 5.0), st);
    if (shape.IsNull()) fail(kFailure, "mesh conversion produced no geometry");
    const int first = int(s.doc.features.size());
    const bool ok = add_and_rebuild(s, [&] {
        CadFeature f;
        f.type = CadFeatureType::Import;
        f.name = opt<std::string>(p, "name", "Mesh" + std::to_string(first + 1));
        f.imported_solid = as_saved(shape);
        f.mode = BooleanMode::New;
        s.doc.features.push_back(f);
    });
    return build_result(s, ok, json{{"first_feature", ok ? first : -1}, {"input_triangles", st.input_tris}, {"kept_triangles", st.kept_tris},
        {"degenerate_collapsed", st.degenerate_collapsed}, {"degenerate_sliver", st.degenerate_sliver}, {"faces_built", st.faces_built},
        {"faces_failed", st.faces_failed}, {"faces_final", st.faces_final}, {"unique_edges", st.unique_edges},
        {"boundary_edges", st.boundary_edges}, {"nonmanifold_edges", st.nonmanifold_edges}, {"watertight", st.watertight},
        {"is_solid", st.is_solid}, {"volume", st.volume}});
}

// ---- primitives (CubbySlicer) ------------------------------------------------------------------
// box / cylinder / sphere / cone / torus as ONE history entry: an Import feature holding the
// solid's B-rep (Orca has no primitive feature; Import is its own rigid-B-rep feature, saved as
// BRep text in orca_cad.bin and read back by desktop Orca 2.5.0, which uses the same OCCT 8.0.1).
// Not parametric: the dimensions are baked into the B-rep (edit = delete and add again).
// Frame: `plane` (default XY through modeling_origin) moved by `x`,`y` (plane coords) and `z`
// (along the normal); every shape rests on that plane, centred on the point, height along the
// plane's normal (CubbyCAD's frame: base on z = 0): box w (plane X) x d x h, cylinder / cone base
// circle on the plane, sphere centre at +radius, torus centre at +minor_radius.
json primitive_common(Ctx& c, const char* kind)
{
    DocState& s = writable(c.params);
    const json& p = c.params;
    const SketchPlane pl = plane_arg(p, s.doc);
    const Vec3d N = pl.normal.normalized(), X = pl.x_axis.normalized();
    if (!std::isfinite(N.norm()) || !std::isfinite(X.norm()) || std::abs(N.dot(X)) > 1e-6) bad_param("plane: normal and x_axis must be perpendicular unit vectors");
    const Vec3d Yp = pl.y_axis.normalized();
    const Vec3d o = pl.origin + X * opt<double>(p, "x", 0.0) + Yp * opt<double>(p, "y", 0.0) + N * opt<double>(p, "z", 0.0);
    const Vec3d Y = N.cross(X);
    auto pnt = [](const Vec3d& v) { return gp_Pnt(v.x(), v.y(), v.z()); };
    auto ax = [&](const Vec3d& at) { return gp_Ax2(pnt(at), gp_Dir(N.x(), N.y(), N.z()), gp_Dir(X.x(), X.y(), X.z())); };
    auto pos = [&](const char* k, double d) {
        const double v = opt<double>(p, k, d);
        if (!(v > 0) || !std::isfinite(v)) bad_param(std::string("'") + k + "' must be > 0");
        return v;
    };
    const std::string k = kind;
    TopoDS_Shape sh;
    std::string dflt;
    if (k == "box") {
        const double w = pos("width", 20), d = pos("depth", 20), h = pos("height", 20);
        sh = BRepPrimAPI_MakeBox(ax(o - X * (w / 2) - Y * (d / 2)), w, d, h).Shape();
        dflt = "Box";
    } else if (k == "cylinder") {
        const double r = pos("radius", 10), h = pos("height", 20);
        sh = BRepPrimAPI_MakeCylinder(ax(o), r, h).Shape();
        dflt = "Cylinder";
    } else if (k == "cone") {
        const double r1 = opt<double>(p, "radius1", opt<double>(p, "radius_bottom", 10.0));
        const double r2 = opt<double>(p, "radius2", opt<double>(p, "radius_top", 0.0));
        const double h = pos("height", 20);
        if (!(r1 >= 0) || !(r2 >= 0) || (r1 <= 0 && r2 <= 0) || std::abs(r1 - r2) < 1e-9)
            bad_param("cone: radius1 / radius2 must be >= 0, not both 0 and not equal (use cylinder)");
        sh = BRepPrimAPI_MakeCone(ax(o), r1, r2, h).Shape();
        dflt = "Cone";
    } else if (k == "sphere") {
        const double r = pos("radius", 10);
        sh = BRepPrimAPI_MakeSphere(ax(o + N * r), r).Shape();
        dflt = "Sphere";
    } else {
        const double R = pos("major_radius", opt<double>(p, "radius", 10.0)), r = pos("minor_radius", opt<double>(p, "tube", 2.5));
        if (r >= R) bad_param("torus: minor_radius must be smaller than major_radius");
        sh = BRepPrimAPI_MakeTorus(ax(o + N * r), R, r).Shape();
        dflt = "Torus";
    }
    if (sh.IsNull()) fail(kFailure, std::string(kind) + " produced no geometry");
    const int fi = int(s.doc.features.size());
    const bool ok = add_and_rebuild(s, [&] {
        CadFeature f;
        f.type = CadFeatureType::Import;
        f.name = opt<std::string>(p, "name", dflt + " " + std::to_string(fi + 1));
        f.imported_solid = as_saved(sh);
        f.mode = BooleanMode::New;
        s.doc.features.push_back(f);
    });
    return build_result(s, ok, json{{"primitive_index", ok ? fi : -1}, {"body", ok ? int(s.doc.bodies.size()) - 1 : -1}});
}
CAD_METHOD(box) { return primitive_common(c, "box"); }
CAD_METHOD(cylinder) { return primitive_common(c, "cylinder"); }
CAD_METHOD(cone) { return primitive_common(c, "cone"); }
CAD_METHOD(sphere) { return primitive_common(c, "sphere"); }
CAD_METHOD(torus) { return primitive_common(c, "torus"); }

// ---- bodies ----------------------------------------------------------------------------------
CAD_METHOD(set_body)
{
    DocState& s = writable(c.params);
    const int b = int_param(c.params, "body");
    if (b < 0 || b >= int(s.doc.bodies.size())) bad_param("body out of range");
    Mutation m(s);
    CadBody& body = s.doc.bodies[b];
    if (c.params.contains("name")) {
        if (c.params["name"].is_null() || c.params["name"] == "") { body.has_user_name = false; body.user_name.clear(); }
        else { body.has_user_name = true; body.user_name = c.params["name"].get<std::string>(); }
    }
    if (c.params.contains("color")) {
        const json& col = c.params["color"];
        if (col.is_null()) body.has_color = false;
        else {
            if (!col.is_array() || col.size() < 3) bad_param("color must be [r, g, b, a?] in 0..1");
            body.has_color = true;
            body.color = ColorRGBA(float(num(col[0], "color")), float(num(col[1], "color")), float(num(col[2], "color")),
                                   col.size() > 3 ? float(num(col[3], "color")) : 1.f);
        }
    }
    m.finish(true);
    return json{{"ok", true}, {"bodies", bodies_summary(s.doc)}};
}

std::vector<int> bodies_arg(const DocState& s, const json& p)
{
    std::vector<int> which;
    if (p.contains("bodies") && p["bodies"].is_array()) {
        for (const json& j : p["bodies"]) {
            const int i = j.get<int>();
            if (i < 0 || i >= int(s.doc.bodies.size())) bad_param("body " + std::to_string(i) + " out of range");
            which.push_back(i);
        }
    } else for (int i = 0; i < int(s.doc.bodies.size()); ++i) which.push_back(i);
    return which;
}

// Orca's Commit to Plate mesh: SketchEngine::tessellate (welded, degenerate triangles dropped)
// at the document's deflection, which is what recompute() already produced.
CAD_METHOD(export_mesh)
{
    DocState& s = doc_of(c.params);
    const double lin = opt<double>(c.params, "linear_deflection", s.doc.linear_deflection);
    const double ang = opt<double>(c.params, "angular_deflection", s.doc.angular_deflection);
    const bool same = lin == s.doc.linear_deflection && ang == s.doc.angular_deflection;
    json meshes = json::array();
    // per-body slices of display_tri_face
    std::vector<size_t> tri_start(s.doc.display_body_meshes.size() + 1, 0);
    for (size_t i = 0; i < s.doc.display_body_meshes.size(); ++i) tri_start[i + 1] = tri_start[i] + s.doc.display_body_meshes[i].its.indices.size();
    for (int i : bodies_arg(s, c.params)) {
        if (same && i < int(s.doc.display_body_meshes.size()) && tri_start[i + 1] <= s.doc.display_tri_face.size()) {
            std::vector<int> tf(s.doc.display_tri_face.begin() + tri_start[i], s.doc.display_tri_face.begin() + tri_start[i + 1]);
            meshes.push_back(plate_mesh(s.doc.display_body_meshes[i], &tf, i, c.out_blob));
        } else {
            std::vector<int> tf;
            TriangleMesh tm = SketchEngine::tessellate(s.doc.bodies[i].shape, tf, lin, ang);
            meshes.push_back(plate_mesh(tm, &tf, i, c.out_blob));
        }
    }
    return json{{"meshes", meshes}, {"generation", s.doc.topo_generation}};
}

CAD_METHOD(export_step)
{
    DocState& s = doc_of(c.params);
    const std::vector<int> which = bodies_arg(s, c.params);
    if (which.empty()) fail(kFailure, "nothing to export");
    BRep_Builder bld;
    TopoDS_Compound comp;
    bld.MakeCompound(comp);
    for (int i : which) if (!s.doc.bodies[i].shape.IsNull()) bld.Add(comp, s.doc.bodies[i].shape);
    const std::string path = "/tmp/cad_export.step";
    STEPControl_Writer writer;
    if (writer.Transfer(comp, STEPControl_AsIs) != IFSelect_RetDone) fail(kFailure, "STEP transfer failed");
    if (writer.Write(path.c_str()) != IFSelect_RetDone) fail(kFailure, "cannot write STEP file");
    std::ifstream f(path, std::ios::binary);
    std::string bytes((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    ::unlink(path.c_str());
    c.out_blob.put(bytes.data(), 1, bytes.size());
    return json{{"bytes", bytes.size()}, {"bodies", which.size()}};
}

// ---- queries ---------------------------------------------------------------------------------
CAD_METHOD(query_topology)
{
    DocState& s = doc_of(c.params);
    const TopoDS_Shape& shape = body_shape(s, c.params);
    const std::vector<TopoDS_Face> all_faces = GeometryEngine::faces_of(shape);
    const std::vector<TopoDS_Edge> all_edges = GeometryEngine::edges_of(shape);
    json faces = json::array();
    for (int i = 0; i < int(all_faces.size()); ++i) {
        const TopoDS_Face& f = all_faces[i];
        if (f.IsNull()) continue;
        json jf{{"id", i}, {"centroid", vec3(GeometryEngine::face_centroid_world(f))}, {"normal", vec3(GeometryEngine::face_normal_world(f))},
                {"kind", "planar"}, {"surface", surface_kind(BRepAdaptor_Surface(f).GetType())}};
        GeometryEngine::CylinderFace cyl = GeometryEngine::cylinder_of_face(f);
        if (cyl.ok) { jf["kind"] = "cylindrical"; jf["radius"] = cyl.radius; jf["axis"] = vec3(cyl.axis); jf["internal"] = cyl.internal; }
        else if (BRepAdaptor_Surface(f).GetType() != GeomAbs_Plane) jf["kind"] = "other";
        faces.push_back(std::move(jf));
    }
    json edges = json::array();
    for (int i = 0; i < int(all_edges.size()); ++i) {
        const TopoDS_Edge& e = all_edges[i];
        if (e.IsNull()) continue;
        std::vector<Vec3d> pts = GeometryEngine::sample_edge_world(e);
        if (pts.size() < 2) continue;
        double len = 0;
        for (size_t k = 1; k < pts.size(); ++k) len += (pts[k] - pts[k - 1]).norm();
        json je{{"id", i}, {"length", len}, {"p0", vec3(pts.front())}, {"p1", vec3(pts.back())}, {"kind", "line"}};
        GeometryEngine::CylinderFace circ = GeometryEngine::circle_of_edge(e);
        if (circ.ok) { je["kind"] = "circle"; je["radius"] = circ.radius; je["center"] = vec3(circ.base); }
        else if (BRepAdaptor_Curve(e).GetType() != GeomAbs_Line) je["kind"] = "other";
        edges.push_back(std::move(je));
    }
    return json{{"body", opt<int>(c.params, "body", 0)}, {"face_count", all_faces.size()}, {"edge_count", all_edges.size()},
                {"faces", faces}, {"edges", edges}, {"generation", s.doc.topo_generation}};
}

CAD_METHOD(face_info)
{
    DocState& s = doc_of(c.params);
    const TopoDS_Shape& body = body_shape(s, c.params);
    const TopoDS_Face f = face_arg(s, c.params, body);
    BRepAdaptor_Surface surf(f);
    GProp_GProps gp;
    BRepGProp::SurfaceProperties(f, gp);
    json j{{"id", int_param(c.params, "face")}, {"surface", surface_kind(surf.GetType())}, {"area", gp.Mass()},
           {"centroid", vec3(GeometryEngine::face_centroid_world(f))}, {"normal", vec3(GeometryEngine::face_normal_world(f))},
           {"bbox", bbox_json(f)}, {"reversed", f.Orientation() == TopAbs_REVERSED}};
    SketchPlane pl;
    if (s.doc.plane_of_face(opt<int>(c.params, "body", 0), int_param(c.params, "face"), pl)) j["plane"] = plane_json(pl);
    GeometryEngine::CylinderFace cyl = GeometryEngine::cylinder_of_face(f);
    if (cyl.ok) j["cylinder"] = json{{"base", vec3(cyl.base)}, {"axis", vec3(cyl.axis)}, {"radius", cyl.radius}, {"height", cyl.height}, {"internal", cyl.internal}};
    ShapeIndexMap emap;
    TopExp::MapShapes(body, TopAbs_EDGE, emap);
    json eids = json::array();
    ShapeIndexMap seen;
    for (TopExp_Explorer ex(f, TopAbs_EDGE); ex.More(); ex.Next()) {
        if (seen.Contains(ex.Current())) continue;
        seen.Add(ex.Current());
        const int k = emap.FindIndex(ex.Current());
        if (k > 0) eids.push_back(k - 1);
    }
    j["edges"] = eids;
    j["generation"] = s.doc.topo_generation;
    return j;
}

CAD_METHOD(edge_info)
{
    DocState& s = doc_of(c.params);
    const TopoDS_Shape& body = body_shape(s, c.params);
    const TopoDS_Edge e = edge_arg(s, c.params, body);
    BRepAdaptor_Curve curve(e);
    GProp_GProps gp;
    BRepGProp::LinearProperties(e, gp);
    std::vector<Vec3d> pts = GeometryEngine::sample_edge_world(e);
    json poly = json::array();
    for (const Vec3d& p : pts) poly.push_back(vec3(p));
    json j{{"id", int_param(c.params, "edge")}, {"curve", curve_kind(curve.GetType())}, {"length", gp.Mass()},
           {"degenerate", BRep_Tool::Degenerated(e)}, {"polyline", poly}};
    if (!pts.empty()) { j["p0"] = vec3(pts.front()); j["p1"] = vec3(pts.back()); }
    GeometryEngine::CylinderFace circ = GeometryEngine::circle_of_edge(e);
    if (circ.ok) j["circle"] = json{{"center", vec3(circ.base)}, {"axis", vec3(circ.axis)}, {"radius", circ.radius}};
    AncestorMap anc;
    TopExp::MapShapesAndAncestors(body, TopAbs_EDGE, TopAbs_FACE, anc);
    std::vector<TopoDS_Face> faces = GeometryEngine::faces_of(body);
    json fids = json::array();
    bool seam = false;
    if (const NCollection_List<TopoDS_Shape>* fl = anc.Seek(e)) {
        for (NCollection_List<TopoDS_Shape>::Iterator it(*fl); it.More(); it.Next()) {
            seam = seam || BRep_Tool::IsClosed(e, TopoDS::Face(it.Value()));
            for (int k = 0; k < int(faces.size()); ++k)
                if (faces[k].IsSame(it.Value())) { if (std::find(fids.begin(), fids.end(), json(k)) == fids.end()) fids.push_back(k); break; }
        }
    }
    j["faces"] = fids;
    j["seam"] = seam;
    j["generation"] = s.doc.topo_generation;
    return j;
}

CAD_METHOD(plane_of_face)
{
    DocState& s = doc_of(c.params);
    check_generation(s, c.params);
    SketchPlane pl;
    const bool ok = s.doc.plane_of_face(int_param(c.params, "body"), int_param(c.params, "face"), pl);
    json r{{"ok", ok}};
    if (ok) r["plane"] = plane_json(pl);
    else r["error"] = "not a planar face of that body";
    return r;
}

CAD_METHOD(datums)
{
    DocState& s = doc_of(c.params);
    json planes = json::array(), axes = json::array(), css = json::array();
    for (const auto& [name, pl] : s.doc.resolve_datum_planes()) planes.push_back(json{{"name", name}, {"plane", plane_json(pl)}});
    for (const auto& a : s.doc.resolve_datum_axes())
        axes.push_back(json{{"name", a.name}, {"origin", vec3(a.origin)}, {"direction", vec3(a.direction)}, {"error", a.error}});
    for (const auto& d : s.doc.resolve_datum_coordsys())
        css.push_back(json{{"name", d.name}, {"origin", vec3(d.origin)}, {"x", vec3(d.x)}, {"y", vec3(d.y)}, {"error", d.error}});
    // base planes, as Orca offers them (through the modeling origin)
    json base = json::array();
    for (const char* n : {"XY", "XZ", "YZ"}) base.push_back(json{{"name", n}, {"plane", plane_json(plane_from_json(json(n), s.doc.modeling_origin))}});
    return json{{"base_planes", base}, {"planes", planes}, {"axes", axes}, {"coordsys", css}};
}

CAD_METHOD(body_touching_sketch)
{
    DocState& s = doc_of(c.params);
    return json{{"body", s.doc.body_touching_sketch(feature_index_arg(s, c.params, "sketch"))}};
}

// McpControl measure / mass_properties / slice_body / check_interference / validate_against.
bool resolve_ref(const TopoDS_Shape& shape, const json& ref, Vec3d& point, Vec3d& dir, bool& has_dir)
{
    has_dir = false;
    if (ref.contains("point")) { point = vec3_from(ref["point"], "point"); return true; }
    if (ref.contains("face")) {
        TopoDS_Face f = GeometryEngine::face_by_index(shape, ref["face"].get<int>());
        if (f.IsNull()) return false;
        point = GeometryEngine::face_centroid_world(f); dir = GeometryEngine::face_normal_world(f); has_dir = true;
        return true;
    }
    if (ref.contains("edge")) {
        TopoDS_Edge e = GeometryEngine::edge_by_index(shape, ref["edge"].get<int>());
        if (e.IsNull()) return false;
        std::vector<Vec3d> pts = GeometryEngine::sample_edge_world(e);
        if (pts.empty()) return false;
        point = pts[pts.size() / 2];
        if (pts.size() >= 2) { dir = (pts.back() - pts.front()).normalized(); has_dir = true; }
        return true;
    }
    return false;
}

CAD_METHOD(measure)
{
    DocState& s = doc_of(c.params);
    check_generation(s, c.params);
    const TopoDS_Shape& shape = body_shape(s, c.params);
    if (!c.params.contains("a") || !c.params.contains("b")) bad_param("measure needs refs 'a' and 'b' ({face|edge|point})");
    Vec3d pa, pb, da, db; bool hda = false, hdb = false;
    if (!resolve_ref(shape, c.params["a"], pa, da, hda) || !resolve_ref(shape, c.params["b"], pb, db, hdb))
        bad_param("could not resolve a measurement reference");
    json r{{"distance", (pa - pb).norm()}, {"point_a", vec3(pa)}, {"point_b", vec3(pb)}};
    if (hda && hdb) r["angle_deg"] = std::acos(std::max(-1.0, std::min(1.0, da.normalized().dot(db.normalized())))) * 180.0 / M_PI;
    return r;
}

CAD_METHOD(mass_properties)
{
    DocState& s = doc_of(c.params);
    auto mp = GeometryEngine::mass_properties(body_shape(s, c.params));
    if (!mp.valid) fail(kFailure, "mass properties could not be computed (null/empty shape)");
    if (!mp.is_solid)
        return json{{"surface_area", mp.surface_area}, {"volume", 0.0}, {"is_solid", false}, {"valid", mp.valid},
                    {"note", "sheet body (open shell): it encloses no material, so volume and inertia are not defined; surface_area is exact"}};
    return json{{"volume", mp.volume}, {"surface_area", mp.surface_area}, {"center_of_mass", vec3(mp.center_of_mass)},
                {"inertia", mp.inertia}, {"is_solid", true}, {"valid", mp.valid}};
}

std::vector<std::pair<std::vector<Vec3d>, bool>> chain_segments(std::vector<std::vector<Vec3d>> segs, double tol)
{
    std::vector<std::pair<std::vector<Vec3d>, bool>> contours;
    std::vector<char> used(segs.size(), 0);
    auto meets = [&](const Vec3d& a, const Vec3d& b) { return (a - b).norm() <= tol; };
    for (size_t i = 0; i < segs.size(); ++i) {
        if (used[i] || segs[i].size() < 2) continue;
        used[i] = 1;
        std::vector<Vec3d> cc = segs[i];
        for (int side = 0; side < 2;) {
            bool grew = false;
            for (size_t j = 0; j < segs.size(); ++j) {
                if (used[j] || segs[j].size() < 2) continue;
                if (meets(cc.back(), segs[j].front())) { cc.insert(cc.end(), segs[j].begin() + 1, segs[j].end()); used[j] = 1; grew = true; }
                else if (meets(cc.back(), segs[j].back())) { for (auto it = segs[j].rbegin() + 1; it != segs[j].rend(); ++it) cc.push_back(*it); used[j] = 1; grew = true; }
            }
            if (grew) { side = 0; continue; }
            std::reverse(cc.begin(), cc.end()); ++side;
        }
        const bool closed = cc.size() > 2 && meets(cc.front(), cc.back());
        contours.emplace_back(std::move(cc), closed);
    }
    return contours;
}

CAD_METHOD(slice_body)
{
    DocState& s = doc_of(c.params);
    const TopoDS_Shape& shape = body_shape(s, c.params);
    const std::string plane_name = opt<std::string>(c.params, "plane", "XY");
    const double offset = opt<double>(c.params, "offset", 0.0);
    gp_Dir n = plane_name == "XZ" ? gp_Dir(0, 1, 0) : plane_name == "YZ" ? gp_Dir(1, 0, 0) : gp_Dir(0, 0, 1);
    gp_Pnt o(n.X() * offset, n.Y() * offset, n.Z() * offset);
    BRepAlgoAPI_Section sect(shape, gp_Pln(o, n), false);
    sect.ComputePCurveOn1(false);
    sect.Approximation(true);
    sect.Build();
    if (!sect.IsDone()) fail(kFailure, "section failed");
    std::vector<std::vector<Vec3d>> segs;
    for (TopExp_Explorer ex(sect.Shape(), TopAbs_EDGE); ex.More(); ex.Next()) {
        std::vector<Vec3d> pts = GeometryEngine::sample_edge_world(TopoDS::Edge(ex.Current()));
        if (pts.size() >= 2) segs.push_back(std::move(pts));
    }
    const int raw = int(segs.size());
    auto contours = chain_segments(std::move(segs), 1e-3);
    json jc = json::array();
    int closed_n = 0;
    for (auto& pc : contours) {
        if (pc.second) ++closed_n;
        json pts = json::array();
        for (const Vec3d& p : pc.first) pts.push_back(vec3(p));
        jc.push_back(json{{"closed", pc.second}, {"points", pts}});
    }
    return json{{"body", opt<int>(c.params, "body", 0)}, {"plane", plane_name}, {"offset", offset}, {"segment_count", raw},
                {"contour_count", contours.size()}, {"closed_count", closed_n}, {"contours", jc}};
}

CAD_METHOD(check_interference)
{
    DocState& s = doc_of(c.params);
    const auto hits = s.doc.check_interference(opt<double>(c.params, "min_volume", 1e-6));
    json a = json::array();
    for (const auto& h : hits) a.push_back(json{{"body_a", h.body_a}, {"body_b", h.body_b}, {"volume", h.volume}});
    return json{{"ok", true}, {"count", hits.size()}, {"interferences", a}};
}

CAD_METHOD(validate_against)
{
    DocState& s = doc_of(c.params);
    const TopoDS_Shape& cand = body_shape(s, c.params);
    const json r = c.params.value("reference", json::object());
    TopoDS_Shape ref;
    if (r.contains("step")) {
        std::vector<TopoDS_Shape> solids = read_step_blob(c);
        BRep_Builder b; TopoDS_Compound comp; b.MakeCompound(comp);
        for (const TopoDS_Shape& sh : solids) if (!sh.IsNull()) b.Add(comp, sh);
        ref = comp;
    } else if (r.contains("body")) {
        const int idx = r["body"].get<int>();
        if (idx < 0 || idx >= int(s.doc.bodies.size()) || s.doc.bodies[idx].shape.IsNull()) bad_param("reference body index out of range / null");
        ref = s.doc.bodies[idx].shape;
    } else bad_param("reference must be {\"step\": true} (blob) or {\"body\": id}");
    auto metrics = [](const TopoDS_Shape& sh, double& vol, Vec3d& cen, Vec3d& mn, Vec3d& mx) {
        GProp_GProps vp; BRepGProp::VolumeProperties(sh, vp);
        vol = vp.Mass(); cen = Vec3d(vp.CentreOfMass().X(), vp.CentreOfMass().Y(), vp.CentreOfMass().Z());
        Bnd_Box bb; BRepBndLib::Add(sh, bb); double x0, y0, z0, x1, y1, z1; bb.Get(x0, y0, z0, x1, y1, z1);
        mn = Vec3d(x0, y0, z0); mx = Vec3d(x1, y1, z1);
    };
    double va, vb; Vec3d ca, cb, amin, amax, bmin, bmax;
    metrics(cand, va, ca, amin, amax);
    metrics(ref, vb, cb, bmin, bmax);
    const GeometryEngine::Deviation dev = GeometryEngine::surface_deviation(cand, ref);
    return json{{"volume", va}, {"volume_reference", vb}, {"volume_delta_pct", vb > 0 ? (va - vb) / vb * 100.0 : 0.0},
                {"centroid_offset", vec3(ca - cb)}, {"centroid_offset_mm", (ca - cb).norm()},
                {"bbox", {{"min", vec3(amin)}, {"max", vec3(amax)}}}, {"bbox_reference", {{"min", vec3(bmin)}, {"max", vec3(bmax)}}},
                {"bbox_delta", {{"min", vec3(amin - bmin)}, {"max", vec3(amax - bmax)}}},
                {"surface_deviation", {{"max_mm", dev.max_mm}, {"mean_mm", dev.mean_mm}, {"rms_mm", dev.rms_mm}, {"samples", dev.sample_count}}}};
}

// ---- sketch sessions -------------------------------------------------------------------------
SketchSession& session(DocState& s)
{
    if (!s.sketch) fail(kSketchState, "no sketch is open — call sketch_begin first");
    return *s.sketch;
}
json session_state(DocState& s)
{
    SketchSession& k = *s.sketch;
    json out{{"ok", true}, {"entities", entities_json(k.ents)}, {"constraints", constraints_json(k.cons)}, {"selection", k.sel},
             {"plane", plane_json(k.plane)}, {"feature", k.feature}};
    out.update(solve_json(k));
    out.update(loop_report(k.ents));
    return out;
}
std::vector<int> sketch_targets(const json& p, const SketchSession& k, bool all_if_empty)
{
    std::vector<int> out;
    if (p.contains("entities")) {
        for (const json& v : p["entities"]) {
            const int i = v.get<int>();
            if (i < 0 || i >= int(k.ents.size())) bad_param("entity " + std::to_string(i) + " out of range");
            out.push_back(i);
        }
        return out;
    }
    out = k.sel;
    if (out.empty() && all_if_empty) for (int i = 0; i < int(k.ents.size()); ++i) out.push_back(i);
    return out;
}
int add_entities(SketchSession& k, const std::vector<SketchEntity>& ents, bool infer)
{
    const int base = int(k.ents.size());
    k.ents.insert(k.ents.end(), ents.begin(), ents.end());
    if (infer) infer_auto_constraints(k, base);
    session_solve(k);
    return base;
}

CAD_METHOD(sketch_begin)
{
    DocState& s = writable(c.params);
    const json& p = c.params;
    if (s.sketch) fail(kSketchState, "a sketch is already open");
    SketchSession k;
    if (p.contains("feature")) {
        k.feature = feature_index_arg(s, p, "feature");
        const CadFeature& f = s.doc.features[k.feature];
        if (f.type != CadFeatureType::Sketch) bad_param("feature " + std::to_string(k.feature) + " is not a Sketch");
        k.plane = f.plane; k.ents = f.entities; k.cons = f.entity_constraints;
    } else if (p.contains("face")) {
        const json& fj = p["face"];
        check_generation(s, p);
        if (!s.doc.plane_of_face(int_param(fj, "body"), int_param(fj, "face"), k.plane))
            bad_param("that face is not a planar face of that body");
    } else {
        k.plane = plane_arg(p, s.doc);
    }
    set_sketch_auto_close(s.doc.auto_close_loops);
    session_solve(k);
    s.sketch = std::move(k);
    json r = session_state(s);
    r["plane_name"] = p.value("plane", json(nullptr));
    return r;
}

CAD_METHOD(sketch_cancel)
{
    DocState& s = doc_of(c.params);
    if (!s.sketch) fail(kSketchState, "no sketch is open");
    s.sketch.reset();
    return json{{"ok", true}};
}

CAD_METHOD(sketch_commit)
{
    DocState& s = writable(c.params);
    SketchSession k = session(s);
    // v1.2 `fields`: other Sketch fields set in the same commit (one rebuild), e.g. the rigid
    // `imported_regions` of a Text / SVG sketch, whose session has no entities.
    const json fields = c.params.value("fields", json::object());
    if (!fields.is_object()) bad_param("'fields' must be an object");
    for (const char* key : {"type", "entities", "entity_constraints", "plane"})
        if (fields.contains(key)) bad_param(std::string("sketch_commit 'fields' cannot set '") + key + "' (it comes from the session)");
    const bool regions = fields.contains("imported_regions") && fields["imported_regions"].is_array() && !fields["imported_regions"].empty();
    // desktop Orca (DesignPanel's commit callback): "Sketch empty — nothing committed"
    if (k.ents.empty() && !regions) fail(kInvalidParams, "the sketch is empty — nothing to commit (add an entity, or sketch_cancel)");
    int idx = k.feature;
    Mutation m(s);
    if (idx < 0) {
        idx = s.doc.add_sketch_entities(k.ents, k.plane, opt<std::string>(c.params, "name", "Sketch " + std::to_string(s.doc.features.size() + 1)), k.cons);
    } else {
        CadFeature& f = s.doc.features[idx];
        f.entities = k.ents; f.entity_constraints = k.cons; f.plane = k.plane;
        if (c.params.contains("name")) { f.name = c.params["name"].get<std::string>(); m.keys[idx].second = f.name; }
    }
    if (!fields.empty()) {
        try { apply_fields(s.doc.features[idx], fields, s.doc.modeling_origin, false); }
        catch (...) { m.restore(); throw; }
    }
    const bool ok = m.finish(rebuild(s));
    if (ok) s.sketch.reset();   // a failed commit keeps the session open so nothing is lost
    return build_result(s, ok, json{{"sketch_index", ok ? idx : -1}, {"features", s.doc.features.size()}, {"fields_applied", !fields.empty()}});
}

CAD_METHOD(sketch_add)
{
    DocState& s = writable(c.params);
    SketchSession& k = session(s);
    const json& p = c.params;
    std::vector<SketchEntity> ents;
    if (p.contains("entities")) ents = entities_from_json(p["entities"]);
    else if (p.contains("type")) ents.push_back(entity_from_json(p));
    else if (p.contains("rect")) {
        const json& r = p["rect"];
        if (!r.is_array() || r.size() < 4) bad_param("rect must be [x0, y0, x1, y1]");
        const double x0 = num(r[0], "rect"), y0 = num(r[1], "rect"), x1 = num(r[2], "rect"), y1 = num(r[3], "rect");
        const bool cons = opt<bool>(p, "construction", false);
        auto seg = [&](Vec2d a, Vec2d b) { SketchEntity e; e.type = SketchEntity::Type::Line; e.p0 = a; e.p1 = b; e.construction = cons; return e; };
        ents = {seg({x0, y0}, {x1, y0}), seg({x1, y0}, {x1, y1}), seg({x1, y1}, {x0, y1}), seg({x0, y1}, {x0, y0})};
    } else bad_param("sketch_add needs 'entities', a single 'type', or 'rect'");
    k.checkpoint();
    const int base = add_entities(k, ents, opt<bool>(p, "infer", true));
    json r{{"ok", true}, {"first_index", base}, {"added", ents.size()}, {"entities", k.ents.size()}, {"dof", k.last.dof},
           {"constraints", k.cons.size()}};
    r.update(solve_json(k));
    return r;
}

CAD_METHOD(sketch_select)
{
    DocState& s = doc_of(c.params);
    SketchSession& k = session(s);
    k.sel.clear();
    if (c.params.contains("entities"))
        for (const json& v : c.params["entities"]) {
            const int i = v.get<int>();
            if (i >= 0 && i < int(k.ents.size()) && std::find(k.sel.begin(), k.sel.end(), i) == k.sel.end()) k.sel.push_back(i);
        }
    return json{{"ok", true}, {"selected", k.sel.size()}, {"any", !k.sel.empty()}};
}

// Remove entities and every constraint on them; re-index the rest.
void erase_entities(SketchSession& k, std::vector<int> idx)
{
    std::sort(idx.begin(), idx.end());
    idx.erase(std::unique(idx.begin(), idx.end()), idx.end());
    auto removed = [&](int e) { return e >= 0 && std::binary_search(idx.begin(), idx.end(), e); };
    auto remap = [&](int e) { if (e < 0) return e; int sh = 0; for (int r : idx) if (r < e) ++sh; return e - sh; };
    std::vector<SketchEntityConstraintDef> keep;
    for (auto con : k.cons) {
        if (removed(con.ea) || removed(con.eb) || removed(con.ec)) continue;
        con.ea = remap(con.ea); con.eb = remap(con.eb); con.ec = remap(con.ec);
        keep.push_back(con);
    }
    k.cons = std::move(keep);
    for (auto it = idx.rbegin(); it != idx.rend(); ++it) k.ents.erase(k.ents.begin() + *it);
    k.sel.clear();
}

CAD_METHOD(sketch_delete)
{
    DocState& s = writable(c.params);
    SketchSession& k = session(s);
    const std::vector<int> tgt = sketch_targets(c.params, k, false);
    if (tgt.empty()) bad_param("nothing selected and no 'entities' given");
    const int before = int(k.ents.size());
    k.checkpoint();
    erase_entities(k, tgt);
    session_solve(k);
    json r{{"ok", true}, {"removed", before - int(k.ents.size())}, {"entities", k.ents.size()}};
    r.update(solve_json(k));
    return r;
}

CAD_METHOD(sketch_construction)
{
    DocState& s = writable(c.params);
    SketchSession& k = session(s);
    const std::vector<int> tgt = sketch_targets(c.params, k, false);
    if (tgt.empty()) bad_param("nothing selected and no 'entities' given");
    k.checkpoint();
    const bool set = c.params.contains("construction") ? opt<bool>(c.params, "construction", true) : !k.ents[tgt[0]].construction;
    int n = 0;
    for (int i : tgt) if (k.ents[i].construction != set) { k.ents[i].construction = set; ++n; }
    return json{{"ok", n > 0}, {"changed", n}};
}

CAD_METHOD(sketch_offset)
{
    DocState& s = writable(c.params);
    SketchSession& k = session(s);
    if (!c.params.contains("distance")) bad_param("sketch_offset needs 'distance'");
    const double d = num(c.params["distance"], "distance");
    std::vector<SketchEntity> src;
    for (int i : sketch_targets(c.params, k, true)) src.push_back(k.ents[i]);
    if (src.empty()) bad_param("nothing to offset");
    const auto out = SketchEngine::offset_entities(src, d);
    if (out.empty()) fail(kFailure, "offset produced nothing (ellipses and splines are not offset)");
    k.checkpoint();
    const int base = add_entities(k, out, true);
    return json{{"ok", true}, {"first_index", base}, {"added", out.size()}, {"dof", k.last.dof}};
}

CAD_METHOD(sketch_mirror)
{
    DocState& s = writable(c.params);
    SketchSession& k = session(s);
    const Vec2d a = c.params.contains("axis_a") ? vec2_from(c.params["axis_a"], "axis_a") : Vec2d(0, 0);
    const Vec2d b = c.params.contains("axis_b") ? vec2_from(c.params["axis_b"], "axis_b") : Vec2d(0, 1);
    std::vector<SketchEntity> src;
    for (int i : sketch_targets(c.params, k, true)) src.push_back(k.ents[i]);
    if (src.empty()) bad_param("nothing to mirror");
    const auto out = SketchEngine::mirror_entities(src, a, b);
    k.checkpoint();
    const int base = add_entities(k, out, true);
    return json{{"ok", true}, {"first_index", base}, {"added", out.size()}, {"dof", k.last.dof}};
}

CAD_METHOD(sketch_describe)
{
    DocState& s = doc_of(c.params);
    session(s);
    return session_state(s);
}

CAD_METHOD(sketch_validate)
{
    DocState& s = doc_of(c.params);
    SketchSession& k = session(s);
    json out{{"ok", true}, {"tolerance", opt<double>(c.params, "tolerance", 1e-3)}};
    out.update(solve_json(k));
    out.update(loop_report(k.ents));
    return out;
}

CAD_METHOD(sketch_heal)
{
    DocState& s = writable(c.params);
    SketchSession& k = session(s);
    double tol = opt<double>(c.params, "tolerance", 1e-3);
    if (tol <= 0.0) tol = 1e-3;
    const bool ic = opt<bool>(c.params, "ignore_construction", true);
    k.checkpoint();
    const int n = int(k.ents.size());
    int welded = 0;
    std::vector<SketchEntityConstraintDef> cands;
    for (int i = 0; i < n; ++i) {
        if (ic && k.ents[i].construction) continue;
        SketchPointRole ir[2]; const int ni = endpoint_roles(k.ents[i], ir);
        for (int a = 0; a < ni; ++a) {
            Vec2d pa; if (!point_at(k.ents, i, ir[a], pa)) continue;
            for (int j = i + 1; j < n; ++j) {
                if (ic && k.ents[j].construction) continue;
                SketchPointRole jr[2]; const int nj = endpoint_roles(k.ents[j], jr);
                for (int b = 0; b < nj; ++b) {
                    Vec2d pb; if (!point_at(k.ents, j, jr[b], pb)) continue;
                    const double d = (pa - pb).norm();
                    if (d > tol || has_coincident(k.cons, i, ir[a], j, jr[b])) continue;
                    if (d > 0.0) { if (jr[b] == SketchPointRole::P0) k.ents[j].p0 = pa; else k.ents[j].p1 = pa; }
                    SketchEntityConstraintDef cc;
                    cc.type = SketchConstraintType::Coincident; cc.ea = i; cc.ra = ir[a]; cc.eb = j; cc.rb = jr[b];
                    cands.push_back(cc);
                    ++welded;
                }
            }
        }
    }
    if (!cands.empty()) { try_add_constraints(k, cands); session_solve(k); }
    json out{{"ok", true}, {"welded", welded}, {"tolerance", tol}};
    out.update(solve_json(k));
    out.update(loop_report(k.ents));
    return out;
}

// DesignSketchTool's Dimension tool (dimension_kind / apply_dimension / record_dimension_constraint).
enum class DimType { None, Length, Diameter, Radius, Angle, Distance, DistanceToLine };
DimType dimension_kind(const SketchSession& k)
{
    for (int i : k.sel) if (i < 0 || i >= int(k.ents.size())) return DimType::None;
    if (k.sel.size() == 1) {
        switch (k.ents[k.sel[0]].type) {
        case SketchEntity::Type::Line: return DimType::Length;
        case SketchEntity::Type::Circle: return DimType::Diameter;
        case SketchEntity::Type::Arc: return DimType::Radius;
        default: return DimType::None;
        }
    }
    if (k.sel.size() == 2) {
        const SketchEntity &a = k.ents[k.sel[0]], &b = k.ents[k.sel[1]];
        const bool al = a.type == SketchEntity::Type::Line, bl = b.type == SketchEntity::Type::Line;
        Vec2d tmp;
        if (al && bl) return DimType::Angle;
        if (al && entity_ref_point(b, tmp)) return DimType::DistanceToLine;
        if (bl && entity_ref_point(a, tmp)) return DimType::DistanceToLine;
        if (entity_ref_point(a, tmp) && entity_ref_point(b, tmp)) return DimType::Distance;
    }
    return DimType::None;
}
double dimension_current(const SketchSession& k)
{
    switch (dimension_kind(k)) {
    case DimType::Length: { const auto& e = k.ents[k.sel[0]]; return (e.p1 - e.p0).norm(); }
    case DimType::Diameter: return 2.0 * k.ents[k.sel[0]].radius;
    case DimType::Radius: return k.ents[k.sel[0]].radius;
    case DimType::Angle: {
        const Vec2d da = k.ents[k.sel[0]].p1 - k.ents[k.sel[0]].p0, db = k.ents[k.sel[1]].p1 - k.ents[k.sel[1]].p0;
        const double na = da.norm(), nb = db.norm();
        if (na < 1e-9 || nb < 1e-9) return 0.0;
        return std::acos(std::max(-1.0, std::min(1.0, da.dot(db) / (na * nb)))) * 180.0 / M_PI;
    }
    case DimType::Distance: { Vec2d ra, rb; entity_ref_point(k.ents[k.sel[0]], ra); entity_ref_point(k.ents[k.sel[1]], rb); return (rb - ra).norm(); }
    case DimType::DistanceToLine: {
        const bool al = k.ents[k.sel[0]].type == SketchEntity::Type::Line;
        const SketchEntity &L = k.ents[k.sel[al ? 0 : 1]], &P = k.ents[k.sel[al ? 1 : 0]];
        Vec2d rp; entity_ref_point(P, rp);
        const Vec2d dir = L.p1 - L.p0; const double n = dir.norm();
        if (n < 1e-9) return 0.0;
        return std::abs((rp - L.p0).dot(Vec2d(-dir.y() / n, dir.x() / n)));
    }
    default: return 0.0;
    }
}
void apply_angle_between(SketchSession& k, int ia, int ib, double deg)
{
    SketchEntity &A = k.ents[ia], &B = k.ents[ib];
    const Vec2d aE[2] = {A.p0, A.p1}, bE[2] = {B.p0, B.p1};
    int si = -1, sj = -1; double best = 1e-6;
    for (int i = 0; i < 2; ++i) for (int j = 0; j < 2; ++j) { const double d = (aE[i] - bE[j]).squaredNorm(); if (d < best) { best = d; si = i; sj = j; } }
    Vec2d pivot, refDir, bMoving; bool moveP1;
    if (si >= 0) { pivot = aE[si]; refDir = aE[1 - si] - pivot; moveP1 = (sj == 0); bMoving = moveP1 ? B.p1 : B.p0; }
    else { pivot = B.p0; refDir = A.p1 - A.p0; moveP1 = true; bMoving = B.p1; }
    const double nr = refDir.norm(); if (nr < 1e-9) return;
    refDir /= nr;
    const Vec2d db = bMoving - pivot; const double Lb = db.norm(); if (Lb < 1e-9) return;
    const double sign = (refDir.x() * db.y() - refDir.y() * db.x()) >= 0.0 ? 1.0 : -1.0;
    const double rad = sign * deg * M_PI / 180.0;
    const Vec2d nd(refDir.x() * std::cos(rad) - refDir.y() * std::sin(rad), refDir.x() * std::sin(rad) + refDir.y() * std::cos(rad));
    (moveP1 ? B.p1 : B.p0) = pivot + Lb * nd;
}
int upsert_constraint(SketchSession& k, const SketchEntityConstraintDef& c)
{
    auto same = [&](const SketchEntityConstraintDef& o) {
        return (o.ea == c.ea && o.ra == c.ra && o.eb == c.eb && o.rb == c.rb) || (o.ea == c.eb && o.ra == c.rb && o.eb == c.ea && o.rb == c.ra);
    };
    for (int i = 0; i < int(k.cons.size()); ++i) if (k.cons[i].type == c.type && same(k.cons[i])) { k.cons[i] = c; return i; }
    k.cons.push_back(c);
    return int(k.cons.size()) - 1;
}
int upsert_dimension_constraint(SketchSession& k, const SketchEntityConstraintDef& c)
{
    const bool zero = c.type == SketchConstraintType::Coincident, dist = c.type == SketchConstraintType::Distance;
    if (zero || dist)
        k.cons.erase(std::remove_if(k.cons.begin(), k.cons.end(), [&](const SketchEntityConstraintDef& d) {
            const bool same = (d.ea == c.ea && d.eb == c.eb) || (d.ea == c.eb && d.eb == c.ea);
            return same && (zero ? d.type == SketchConstraintType::Distance : d.type == SketchConstraintType::Coincident);
        }), k.cons.end());
    return upsert_constraint(k, c);
}

CAD_METHOD(sketch_set_value)
{
    DocState& s = writable(c.params);
    SketchSession& k = session(s);
    if (!c.params.contains("value")) bad_param("sketch_set_value needs 'value'");
    const double v = num(c.params["value"], "value");
    const DimType kind = dimension_kind(k);
    static const char* const names[] = {"none", "length", "diameter", "radius", "angle", "distance", "distance_to_line"};
    if (kind == DimType::None) bad_param("the selection has no value to set — pick a line, an arc, a circle, or two entities");
    const bool positive = kind == DimType::Length || kind == DimType::Radius || kind == DimType::Diameter;
    if (positive && v <= 0.0) bad_param(std::string(names[int(kind)]) + " must be positive");
    if ((kind == DimType::Distance || kind == DimType::DistanceToLine) && v < 0.0) bad_param(std::string(names[int(kind)]) + " must be >= 0");
    const double before = dimension_current(k);
    k.checkpoint();
    const auto& sel = k.sel;
    auto role = [&](int i) { return k.ents[i].type == SketchEntity::Type::Point ? SketchPointRole::P0 : SketchPointRole::Center; };
    SketchEntityConstraintDef con;
    switch (kind) {
    case DimType::Length: {
        SketchEntity& e = k.ents[sel[0]]; const Vec2d d = e.p1 - e.p0; const double r = d.norm();
        if (r > 1e-9) e.p1 = e.p0 + (v / r) * d;
        con.type = SketchConstraintType::Distance; con.ea = sel[0]; con.ra = SketchPointRole::P0; con.eb = sel[0]; con.rb = SketchPointRole::P1; con.value = v;
        break;
    }
    case DimType::Diameter: k.ents[sel[0]].radius = 0.5 * v; con.type = SketchConstraintType::Diameter; con.ea = sel[0]; con.value = v; break;
    case DimType::Radius: k.ents[sel[0]].radius = v; con.type = SketchConstraintType::Radius; con.ea = sel[0]; con.value = v; break;
    case DimType::Angle: apply_angle_between(k, sel[0], sel[1], v); con.type = SketchConstraintType::Angle; con.ea = sel[0]; con.eb = sel[1]; con.value = v * M_PI / 180.0; break;
    case DimType::Distance: {
        Vec2d ra, rb; entity_ref_point(k.ents[sel[0]], ra); SketchEntity& b = k.ents[sel[1]]; entity_ref_point(b, rb);
        const Vec2d d = rb - ra; const double r = d.norm();
        const Vec2d target = r > 1e-9 ? Vec2d(ra + (v / r) * d) : Vec2d(ra + Vec2d(v, 0.0));
        translate_entity(b, target - rb);
        if (v < 1e-9) con.type = SketchConstraintType::Coincident; else { con.type = SketchConstraintType::Distance; con.value = v; }
        con.ea = sel[0]; con.ra = role(sel[0]); con.eb = sel[1]; con.rb = role(sel[1]);
        break;
    }
    case DimType::DistanceToLine: {
        const bool al = k.ents[sel[0]].type == SketchEntity::Type::Line;
        const int il = sel[al ? 0 : 1], ip = sel[al ? 1 : 0];
        const SketchEntity L = k.ents[il]; SketchEntity& P = k.ents[ip];
        Vec2d rp; entity_ref_point(P, rp);
        const Vec2d dir = L.p1 - L.p0; const double n = dir.norm();
        if (n > 1e-9) {
            const Vec2d nrm(-dir.y() / n, dir.x() / n);
            const double d0 = (rp - L.p0).dot(nrm);
            translate_entity(P, ((d0 >= 0.0 ? 1.0 : -1.0) * v - d0) * nrm);
        }
        con.type = SketchConstraintType::PointOnLine; con.ea = ip; con.ra = role(ip); con.eb = il; con.value = v;
        break;
    }
    default: break;
    }
    if (kind == DimType::Radius || kind == DimType::Diameter) {
        SketchEntity& e = k.ents[sel[0]];
        if (e.type == SketchEntity::Type::Arc) {
            e.p0 = e.center + e.radius * Vec2d(std::cos(e.start_angle), std::sin(e.start_angle));
            e.p1 = e.center + e.radius * Vec2d(std::cos(e.end_angle), std::sin(e.end_angle));
        }
    }
    upsert_dimension_constraint(k, con);
    session_solve(k);
    k.sel.clear();
    json out{{"ok", true}, {"kind", names[int(kind)]}, {"before", before}, {"value", v}, {"dof", k.last.dof}};
    out.update(solve_json(k));
    out.update(loop_report(k.ents));
    return out;
}

// CubbySlicer additions: constraints, values, entities, drag.
json after_solve(SketchSession& k, json extra = json::object())
{
    extra.update(solve_json(k));
    extra["entities"] = entities_json(k.ents);
    extra["constraints"] = constraints_json(k.cons);
    return extra;
}

CAD_METHOD(sketch_constrain)
{
    DocState& s = writable(c.params);
    SketchSession& k = session(s);
    std::vector<SketchEntityConstraintDef> add;
    if (c.params.contains("constraints")) add = constraints_from_json(c.params["constraints"]);
    else if (c.params.contains("constraint")) add.push_back(constraint_from_json(c.params["constraint"]));
    else add.push_back(constraint_from_json(c.params));
    check_constraint_refs(add, k.ents.size());
    check_constraint_kinds(add, k.ents);
    k.checkpoint();
    const size_t first = k.cons.size();
    const bool force = opt<bool>(c.params, "force", false);
    bool ok;
    if (force) { k.cons.insert(k.cons.end(), add.begin(), add.end()); session_solve(k); ok = true; }
    else {
        ok = try_add_constraints(k, add);
        if (!ok) {
            // report WHY: solve once with them, keep nothing
            std::vector<SketchEntity> probe = k.ents;
            auto all = k.cons; all.insert(all.end(), add.begin(), add.end());
            SketchSolveResult r = safe_solve(probe, all);
            // v1.2: the conflicting set — every existing constraint whose removal lets the batch
            // solve (SolveSpace's own `bad` often names only the newest), plus the new ones.
            std::vector<int> conflicts;
            if (k.cons.size() <= 200)
                for (size_t j = 0; j < k.cons.size(); ++j) {
                    std::vector<SketchEntity> pe = k.ents;
                    auto without = all; without.erase(without.begin() + long(j));
                    if (safe_solve(pe, without).ok) conflicts.push_back(int(j));
                }
            std::vector<int> bad = conflicts;
            for (size_t j = 0; j < add.size(); ++j) bad.push_back(int(first + j));
            for (int b : r.bad) if (std::find(bad.begin(), bad.end(), b) == bad.end()) bad.push_back(b);
            std::sort(bad.begin(), bad.end());
            k.undo.pop_back();
            return json{{"ok", false}, {"error", "the constraint conflicts with the sketch (over-constrained) or cannot be satisfied"},
                        {"solve_ok", r.ok}, {"dof", r.dof}, {"bad", bad}, {"conflicts", conflicts}, {"solver_bad", r.bad},
                        {"skipped", r.skipped}, {"result", r.result}};
        }
    }
    return after_solve(k, json{{"ok", ok}, {"index", int(first)}, {"added", add.size()}});
}

CAD_METHOD(sketch_plan)
{
    DocState& s = doc_of(c.params);
    SketchSession& k = session(s);
    const SketchConstraintType t = SketchConstraintType(constraint_type_index(c.params.value("type", json())));
    std::vector<int> e;
    for (const json& v : c.params.value("entities", json::array())) e.push_back(v.get<int>());
    while (e.size() < 3) e.push_back(-1);
    const ConstraintPlan plan = plan_entity_constraint(k.ents, e[0], e[1], e[2], t);
    static const char* const kinds[] = {"reject", "apply", "ask_value"};
    static const char* const reasons[] = {"none", "need_one_entity", "need_two_entities", "need_a_line", "need_two_lines",
        "need_two_rounds", "need_tangent_pair", "need_joinable_points", "need_measurable_points", "need_point_and_line",
        "need_two_points_or_lines", "need_axis_line", "need_round", "unsupported"};
    return json{{"kind", kinds[int(plan.kind)]}, {"reason", reasons[int(plan.reason)]}, {"constraints", constraints_json(plan.defs)},
                {"prefill", plan.prefill}};
}

CAD_METHOD(sketch_set_constraint)
{
    DocState& s = writable(c.params);
    SketchSession& k = session(s);
    const int i = int_param(c.params, "index");
    if (i < 0 || i >= int(k.cons.size())) bad_param("constraint index out of range");
    double v = num(c.params.value("value", json()), "value");
    if (k.cons[i].type == SketchConstraintType::Angle) v *= M_PI / 180.0;   // wire degrees
    k.checkpoint();
    const auto saved_ents = k.ents;
    const double old = k.cons[i].value;
    k.cons[i].value = v;
    SketchSolveResult r = safe_solve(k.ents, k.cons);
    if (!r.ok) {
        k.cons[i].value = old; k.ents = saved_ents; k.undo.pop_back();
        return json{{"ok", false}, {"error", "that value cannot be satisfied"}, {"bad", r.bad}, {"dof", r.dof}, {"solve_ok", false}};
    }
    k.last = r;
    return after_solve(k, json{{"ok", true}, {"index", i}});
}

CAD_METHOD(sketch_delete_constraint)
{
    DocState& s = writable(c.params);
    SketchSession& k = session(s);
    const int i = int_param(c.params, "index");
    if (i < 0 || i >= int(k.cons.size())) bad_param("constraint index out of range");
    k.checkpoint();
    k.cons.erase(k.cons.begin() + i);
    session_solve(k);
    return after_solve(k, json{{"ok", true}});
}

CAD_METHOD(sketch_set_entity)
{
    DocState& s = writable(c.params);
    SketchSession& k = session(s);
    const int i = int_param(c.params, "index");
    if (i < 0 || i >= int(k.ents.size())) bad_param("entity index out of range");
    k.checkpoint();
    k.ents[i] = entity_from_json(c.params.value("entity", json()));
    session_solve(k);
    return after_solve(k, json{{"ok", true}});
}

CAD_METHOD(sketch_drag)
{
    DocState& s = writable(c.params);
    SketchSession& k = session(s);
    const int ei = int_param(c.params, "entity");
    if (ei < 0 || ei >= int(k.ents.size())) bad_param("entity index out of range");
    const SketchPointRole role = SketchPointRole(name_index(kRoles, c.params.value("role", json("p0")), "role"));
    const Vec2d to = vec2_from(c.params.value("to", json()), "to");
    k.checkpoint();
    SketchEntity& e = k.ents[ei];
    if (role == SketchPointRole::Center) translate_entity(e, to - e.center);
    else if (role == SketchPointRole::P0) { if (e.type == SketchEntity::Type::Point || e.type == SketchEntity::Type::Circle) translate_entity(e, to - e.p0); else e.p0 = to; }
    else e.p1 = to;
    if (e.type == SketchEntity::Type::Arc && role != SketchPointRole::Center) {
        // keep the arc on its circle through the moved end (the solver reflows it when constrained)
        const Vec2d& q = role == SketchPointRole::P0 ? e.p0 : e.p1;
        e.radius = (q - e.center).norm();
        (role == SketchPointRole::P0 ? e.start_angle : e.end_angle) = std::atan2(q.y() - e.center.y(), q.x() - e.center.x());
    }
    if (!k.cons.empty()) {
        std::vector<SketchEntityConstraintDef> cons = k.cons;
        if (e.type == SketchEntity::Type::Line && role != SketchPointRole::Center) {   // Orca: the other end stays
            SketchEntityConstraintDef fix; fix.type = SketchConstraintType::Fix; fix.ea = ei;
            fix.ra = role == SketchPointRole::P0 ? SketchPointRole::P1 : SketchPointRole::P0;
            cons.push_back(fix);
        }
        k.last = safe_solve(k.ents, cons, ei, role);
        if (!k.last.ok) session_solve(k);
    }
    return after_solve(k, json{{"ok", true}});
}

CAD_METHOD(sketch_solve)
{
    DocState& s = doc_of(c.params);
    SketchSession& k = session(s);
    session_solve(k);
    json r = after_solve(k, json{{"ok", k.last.ok}});
    r.update(loop_report(k.ents));
    return r;
}

CAD_METHOD(sketch_infer)
{
    DocState& s = doc_of(c.params);
    SketchSession& k = session(s);
    const int e = int_param(c.params, "entity");
    if (e < 0 || e >= int(k.ents.size())) bad_param("entity index out of range");
    return json{{"constraints", constraints_json(infer_relations(k.ents, e, opt<double>(c.params, "angle_tol", kSketchInferAngleTol),
                                                                 opt<double>(c.params, "length_tol", 0.01)))}};
}

CAD_METHOD(sketch_ops)
{
    DocState& s = writable(c.params);
    SketchSession& k = session(s);
    const json& p = c.params;
    const std::string op = opt<std::string>(p, "op", "");
    k.checkpoint();
    json r{{"ok", true}};
    auto ent = [&](const char* key) { const int i = int_param(p, key); if (i < 0 || i >= int(k.ents.size())) bad_param(std::string(key) + " out of range"); return i; };
    if (op == "fillet" || op == "chamfer") {
        const int a = ent("a"), b = ent("b");
        SketchEntity ao, bo, mid;
        const double v = num(p.value(op == "fillet" ? "radius" : "distance", json()), op == "fillet" ? "radius" : "distance");
        const bool ok = op == "fillet" ? SketchEngine::fillet_lines(k.ents[a], k.ents[b], v, ao, bo, mid)
                                       : SketchEngine::chamfer_lines(k.ents[a], k.ents[b], v, ao, bo, mid);
        if (!ok) { k.undo.pop_back(); return json{{"ok", false}, {"error", op + " needs two connected, non-parallel lines and a size that fits"}}; }
        // DesignSketchTool::confirm_op: trim the legs, drop the stale corner Coincident and each
        // leg's own length, bind the new entity to the trimmed ends (+ tangents for a fillet,
        // falling back as Orca does when a tangent does not solve).
        using R = SketchPointRole;
        using CT = SketchConstraintType;
        k.ents[a] = ao; k.ents[b] = bo;
        const int xi = int(k.ents.size());
        k.ents.push_back(mid);
        auto role_near = [](const SketchEntity& ln, const Vec2d& q) { return (ln.p0 - q).squaredNorm() <= (ln.p1 - q).squaredNorm() ? R::P0 : R::P1; };
        const R ra = role_near(k.ents[a], mid.p0), rb = role_near(k.ents[b], mid.p1);
        auto refs = [](const SketchEntityConstraintDef& d, int e, R rr) { return (d.ea == e && d.ra == rr) || (d.eb == e && d.rb == rr); };
        auto self_len = [](const SketchEntityConstraintDef& d, int e) { return d.type == CT::Distance && d.ea == e && d.eb == e; };
        const size_t before = k.cons.size();
        k.cons.erase(std::remove_if(k.cons.begin(), k.cons.end(), [&](const SketchEntityConstraintDef& d) {
            return (d.type == CT::Coincident && refs(d, a, ra) && refs(d, b, rb)) || self_len(d, a) || self_len(d, b); }), k.cons.end());
        auto coin = [&](R xr, int ln, R lr) { SketchEntityConstraintDef d; d.type = CT::Coincident; d.ea = xi; d.ra = xr; d.eb = ln; d.rb = lr; return d; };
        auto tang = [&](int ln) { SketchEntityConstraintDef d; d.type = CT::Tangent; d.ea = xi; d.eb = ln; return d; };
        std::vector<std::vector<SketchEntityConstraintDef>> ladder;
        if (op == "fillet") ladder = {{coin(R::P0, a, ra), coin(R::P1, b, rb), tang(a), tang(b)}, {coin(R::P0, a, ra), coin(R::P1, b, rb), tang(a)}, {coin(R::P0, a, ra), coin(R::P1, b, rb)}};
        else ladder = {{coin(R::P0, a, ra), coin(R::P1, b, rb)}};
        int bound = 0;
        for (const auto& set : ladder) if (try_add_constraints(k, set)) { bound = int(set.size()); break; }
        if (!bound) session_solve(k);
        r["first_index"] = xi;
        r["released"] = int(before - (k.cons.size() - bound));
        r["bound"] = bound;
    } else if (op == "trim" || op == "extend") {
        const int i = ent("entity");
        std::vector<SketchEntity> others;
        for (int j = 0; j < int(k.ents.size()); ++j) if (j != i) others.push_back(k.ents[j]);
        const Vec2d pick = vec2_from(p.value("pick", json()), "pick");
        SketchEntity e = k.ents[i];
        const bool ok = op == "trim" ? SketchEngine::trim_entity(e, others, pick) : SketchEngine::extend_entity(e, others, pick);
        if (!ok) { k.undo.pop_back(); return json{{"ok", false}, {"error", op + " found no intersection to " + op + " to"}}; }
        k.ents[i] = e;
        // Orca (apply_live_trim → drop_constraints_referencing): the slid endpoint invalidates
        // every constraint on the subject; without this the solve pulls it straight back.
        const size_t before = k.cons.size();
        k.cons.erase(std::remove_if(k.cons.begin(), k.cons.end(), [&](const SketchEntityConstraintDef& d) {
            return d.ea == i || d.eb == i || d.ec == i; }), k.cons.end());
        r["released"] = int(before - k.cons.size());
        session_solve(k);
    } else if (op == "array") {
        std::vector<SketchEntity> src;
        for (int i : sketch_targets(p, k, false)) src.push_back(k.ents[i]);
        if (src.empty()) bad_param("array needs 'entities' or a selection");
        const auto out = SketchEngine::array_entities(src, opt<int>(p, "count", 3), p.contains("step") ? vec2_from(p["step"], "step") : Vec2d(10, 0),
                                                      opt<double>(p, "angle_step", 0.0), p.contains("pivot") ? vec2_from(p["pivot"], "pivot") : Vec2d(0, 0));
        r["first_index"] = add_entities(k, out, true);
        r["added"] = out.size();
    } else if (op == "transform") {
        std::vector<int> tgt = sketch_targets(p, k, true);
        std::vector<SketchEntity> src;
        for (int i : tgt) src.push_back(k.ents[i]);
        const auto out = SketchEngine::transform_entities(src, p.contains("move") ? vec2_from(p["move"], "move") : Vec2d(0, 0),
                                                          opt<double>(p, "angle", 0.0), opt<double>(p, "scale", 1.0),
                                                          p.contains("pivot") ? vec2_from(p["pivot"], "pivot") : Vec2d(0, 0));
        if (out.size() != tgt.size()) { k.undo.pop_back(); return json{{"ok", false}, {"error", "transform failed"}}; }
        if (opt<bool>(p, "copy", false)) r["first_index"] = add_entities(k, out, true);
        else { for (size_t i = 0; i < tgt.size(); ++i) k.ents[tgt[i]] = out[i]; session_solve(k); }
    } else {
        k.undo.pop_back();
        bad_param("sketch_ops: unknown op '" + op + "' (fillet, chamfer, trim, extend, array, transform)");
    }
    return after_solve(k, r);
}

CAD_METHOD(sketch_undo)
{
    DocState& s = doc_of(c.params);
    SketchSession& k = session(s);
    if (k.undo.empty()) return json{{"ok", false}};
    k.redo.push_back({k.ents, k.cons});
    k.ents = std::move(k.undo.back().ents); k.cons = std::move(k.undo.back().cons); k.undo.pop_back();
    k.sel.clear();
    session_solve(k);
    return after_solve(k, json{{"ok", true}, {"can_undo", !k.undo.empty()}, {"can_redo", !k.redo.empty()}});
}

CAD_METHOD(sketch_redo)
{
    DocState& s = doc_of(c.params);
    SketchSession& k = session(s);
    if (k.redo.empty()) return json{{"ok", false}};
    k.undo.push_back({k.ents, k.cons});
    k.ents = std::move(k.redo.back().ents); k.cons = std::move(k.redo.back().cons); k.redo.pop_back();
    k.sel.clear();
    session_solve(k);
    return after_solve(k, json{{"ok", true}, {"can_undo", !k.undo.empty()}, {"can_redo", !k.redo.empty()}});
}

// ---- dispatcher ------------------------------------------------------------------------------
std::string g_version_json;

char* dup_string(const std::string& s, int* len)
{
    char* p = static_cast<char*>(std::malloc(s.size() + 1));
    if (!p) return nullptr;
    std::memcpy(p, s.data(), s.size());
    p[s.size()] = 0;
    if (len) *len = int(s.size());
    return p;
}

void cad_init()
{
    static bool done = false;
    if (done) return;
    done = true;
    // Single-threaded module: a pool with no worker threads makes every OSD_Parallel loop
    // (BRepMesh, BOP) run inline on the calling thread instead of trying to start pthreads.
    OSD_ThreadPool::DefaultPool(1);
}

} // namespace

extern "C" {

EMSCRIPTEN_KEEPALIVE const char* cad_version(void)
{
    if (g_version_json.empty())
        g_version_json = json{{"module", "orca-cad"}, {"orca", "2.5.0"}, {"occt", OCC_VERSION_COMPLETE}, {"abi", 1},
                              {"recipeVersion", CadDocument::ORCA_CAD_RECIPE_VERSION},
#ifdef NDEBUG
                              {"build", "release"}
#else
                              {"build", "debug"}
#endif
                             }.dump();
    return g_version_json.c_str();
}

EMSCRIPTEN_KEEPALIVE int cad_call(const char* method, int method_len, const char* req_json, int req_len,
                                  const uint8_t* blob, int blob_len, char** out_json, int* out_json_len,
                                  uint8_t** out_blob, int* out_blob_len)
{
    cad_init();
    g_call = CallState{};
    if (out_blob) *out_blob = nullptr;
    if (out_blob_len) *out_blob_len = 0;
    json reply;
    int rc = 0;
    BlobOut ob;
    try {
        if (!method || method_len <= 0) throw CadError(kUnknownMethod, "missing method");
        const std::string name(method, size_t(method_len));
        json params = (req_json && req_len > 0) ? json::parse(req_json, req_json + req_len) : json::object();
        if (params.is_null()) params = json::object();
        if (!params.is_object()) throw CadError(kInvalidParams, "params must be a JSON object");
        if (blob_len < 0 || (blob_len > 0 && !blob)) throw CadError(kInvalidParams, "invalid blob");
        auto it = methods().find(name);
        if (it == methods().end()) throw CadError(kUnknownMethod, "unknown method '" + name + "'");
        Ctx ctx{params, blob, size_t(blob_len), ob};
        reply = json{{"ok", true}, {"result", it->second(ctx)}};
    } catch (const CadError& e) {
        reply = json{{"ok", false}, {"error", {{"code", e.code}, {"message", e.what()}}}};
        ob.bytes.clear();
        rc = 1;
    } catch (const json::exception& e) {
        reply = json{{"ok", false}, {"error", {{"code", int(kInvalidParams)}, {"message", std::string("bad JSON: ") + e.what()}}}};
        ob.bytes.clear();
        rc = 1;
    } catch (const Standard_Failure& e) {
        reply = json{{"ok", false}, {"error", {{"code", int(kFailure)}, {"message", std::string("OCCT: ") + (e.GetMessageString() ? e.GetMessageString() : "failure")}}}};
        ob.bytes.clear();
        rc = 1;
    } catch (const std::exception& e) {
        reply = json{{"ok", false}, {"error", {{"code", int(kFailure)}, {"message", e.what()}}}};
        ob.bytes.clear();
        rc = 1;
    } catch (...) {
        reply = json{{"ok", false}, {"error", {{"code", int(kFailure)}, {"message", "unknown failure"}}}};
        ob.bytes.clear();
        rc = 1;
    }
    if (out_json) *out_json = dup_string(reply.dump(-1, ' ', false, json::error_handler_t::replace), out_json_len);
    if (!ob.bytes.empty() && out_blob) {
        *out_blob = static_cast<uint8_t*>(std::malloc(ob.bytes.size()));
        if (*out_blob) {
            std::memcpy(*out_blob, ob.bytes.data(), ob.bytes.size());
            if (out_blob_len) *out_blob_len = int(ob.bytes.size());
        }
    }
    return rc;
}

EMSCRIPTEN_KEEPALIVE void cad_free(void* p) { std::free(p); }

} // extern "C"
