// Plugin hooks → JavaScript. See cs_hooks.hpp and ENGINE-CONTRACT.md ("Plugin hooks").
//
// The query surface mirrors the Python data model desktop Orca binds for slicing plugins
// (src/slic3r/plugin/host/PluginHostSlicing.cpp + PluginHostGeometry.cpp): objects → layers →
// regions → typed surfaces (slices / fill_surfaces, editable), merged islands (lslices,
// editable), extrusion trees (read-only), support layers, the Print's model snapshot, config
// values, and libslic3r's own Clipper geometry ops. Coordinates travel as little-endian int64,
// exactly as libslic3r stores them (scaled by SCALING_FACTOR).
#include "cs_hooks.hpp"

#include <libslic3r/ClipperUtils.hpp>
#include <libslic3r/Exception.hpp>
#include <libslic3r/ExPolygon.hpp>
#include <libslic3r/ExtrusionEntity.hpp>
#include <libslic3r/ExtrusionEntityCollection.hpp>
#include <libslic3r/Layer.hpp>
#include <libslic3r/LifecycleEvents.hpp>
#include <libslic3r/Model.hpp>
#include <libslic3r/Print.hpp>
#include <libslic3r/Surface.hpp>
#include "libslic3r_version.h"

#include <atomic>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <vector>

#include <emscripten.h>
#ifdef __EMSCRIPTEN_PTHREADS__
#include <emscripten/proxying.h>
#include <emscripten/threading.h>
#endif

using namespace Slic3r;

namespace cs::hooks {
namespace {

// ---- JS call ------------------------------------------------------------------------

EM_JS(int, cs_js_has_hook, (), { return typeof Module["csHook"] === "function" ? 1 : 0; });

// Module.csHook(kind, requestJson, null) -> result object | JSON string | null/undefined.
// rc 0 = no handler or no result, 1 = result JSON in *out, -1 = the handler threw (message in *out).
EM_JS(int, cs_js_hook, (const char* kind, const char* req, int req_len, char** out, int* out_len), {
    var fn = Module["csHook"];
    if (typeof fn !== "function") return 0;
    function put(s) {
        var bytes = new TextEncoder().encode(s);
        var p = _malloc(bytes.length + 1) >>> 0;
        HEAPU8.set(bytes, p);
        HEAPU8[p + bytes.length] = 0;
        HEAPU32[(out >>> 0) >>> 2] = p;
        HEAP32[(out_len >>> 0) >>> 2] = bytes.length;
    }
    try {
        var res = fn(UTF8ToString(kind), UTF8ToString(req >>> 0, req_len), null);
        if (res === null || res === undefined) return 0;
        put(typeof res === "string" ? res : JSON.stringify(res));
        return 1;
    } catch (e) {
        put(String((e && e.message) || e));
        return -1;
    }
});

struct Call { const char* kind; const std::string* req; char* out = nullptr; int out_len = 0; int rc = 0; };

void run_call(void* p)
{
    auto* c = static_cast<Call*>(p);
    c->rc = cs_js_hook(c->kind, c->req->data(), int(c->req->size()), &c->out, &c->out_len);
}

// Synchronous from the engine's point of view. In the pthreads build Print::process() runs on
// the slicing pthread while the main runtime thread (the engine Worker that owns Module) idles in
// its event loop, so the call is proxied there and this thread parks until it returns.
int call_js(const char* kind, const json& request, std::string& out)
{
    const std::string req = request.dump(-1, ' ', false, json::error_handler_t::replace);
    Call c{kind, &req};
#ifdef __EMSCRIPTEN_PTHREADS__
    if (!emscripten_is_main_runtime_thread()) {
        if (!emscripten_proxy_sync(emscripten_proxy_get_system_queue(), emscripten_main_runtime_thread_id(), run_call, &c))
            throw SlicingError(std::string("plugin hook '") + kind + "': could not reach the main thread");
    } else
#endif
        run_call(&c);
    out.assign(c.out ? c.out : "", size_t(c.out_len));
    std::free(c.out);
    return c.rc;
}

bool has_js_hook()
{
#ifdef __EMSCRIPTEN_PTHREADS__
    if (!emscripten_is_main_runtime_thread()) {
        int r = 0;
        emscripten_proxy_sync(emscripten_proxy_get_system_queue(), emscripten_main_runtime_thread_id(),
                              [](void* p) { *static_cast<int*>(p) = cs_js_has_hook(); }, &r);
        return r != 0;
    }
#endif
    return cs_js_has_hook() != 0;
}

// ---- state ------------------------------------------------------------------------------

const char* const kStepNames[] = {
    "posSlice", "posPerimeters", "posEstimateCurledExtrusions", "posPrepareInfill", "posInfill",
    "posIroning", "posContouring", "posSupportMaterial", "posDetectOverhangsForLift",
    "posSimplifyPath", "psWipeTower", "psSkirtBrim", "psGCodePostProcess"}; // order of SlicingPipelineStepPlugin

std::string step_name(SlicingPipelineStepPlugin s)
{
    const size_t i = size_t(s);
    return i < sizeof(kStepNames) / sizeof(kStepNames[0]) ? kStepNames[i] : "unknown";
}

struct State {
    std::mutex mutex;              // events/warnings may be recorded from any thread
    json events = json::array();
    json warnings = json::array();
};
State* g_state = nullptr;

// The hook being run. Written by the slicing thread before it proxies to the main runtime
// thread (emscripten_proxy_sync orders the accesses), read by cs_hook_query there.
struct Active { Print* print = nullptr; const PrintObject* object = nullptr; SlicingPipelineStepPlugin step{}; };
std::atomic<Active*> g_active{nullptr};

std::vector<std::string> strings_of(const ConfigBase& cfg, const char* key)
{
    if (const auto* o = cfg.option<ConfigOptionStrings>(key)) return o->values;
    return {};
}

void pipeline_hook(Print& print, const PrintObject* object, SlicingPipelineStepPlugin step)
{
    if (print.canceled()) throw CanceledException();
    const std::string name = step_name(step);
    int object_index = -1;
    for (size_t i = 0; i < print.objects().size(); ++i) if (print.objects()[i] == object) object_index = int(i);
    json begin = {{"step", name}, {"stepIndex", int(step)},
                  {"objectId", object ? json(object->id().id) : json(nullptr)},
                  {"objectIndex", object_index < 0 ? json(nullptr) : json(object_index)},
                  {"scalingFactor", SCALING_FACTOR}, {"orcaVersion", SoftFever_VERSION},
                  {"capabilities", strings_of(print.config(), "slicing_pipeline_plugin")},
                  {"plugins", strings_of(print.full_print_config(), "plugins")}};
    Active active{&print, object, step};
    g_active.store(&active);
    std::string text;
    int rc;
    try {
        rc = call_js("pipeline", begin, text);
    } catch (...) { g_active.store(nullptr); throw; }
    g_active.store(nullptr);
    if (rc == 0) return;
    json r = rc < 0 ? json{{"status", "Exception"}, {"message", text}} : json::parse(text, nullptr, false);
    if (!r.is_object()) throw SlicingError("Slicing pipeline plugin: csHook must return an object");
    const std::string status = r.value("status", std::string("Success"));
    const std::string cap = r.value("capability", std::string("?"));
    const std::string msg = r.value("message", std::string());
    // PluginHooks.cpp: cancellation reaches process(); failures become (non-critical) slicing errors.
    if (status == "Cancelled") throw CanceledException();
    if (status == "Exception" || status == "FatalError")
        throw SlicingError("Slicing pipeline plugin '" + cap + "' error: " + msg);
    if (!msg.empty() && g_state) {
        std::lock_guard<std::mutex> lk(g_state->mutex);
        g_state->warnings.push_back({{"code", "plugin"}, {"message", "Slicing pipeline plugin '" + cap + "' [" + name + "]: " + msg}});
    }
}

void lifecycle_hook(LifecycleEvent event, const LifecycleEventContext& c)
{
    if (!g_state) return;
    json ctx = {{"name", c.name}, {"code", lifecycle_evt_code_to_string(c.code)}, {"msg", c.msg}, {"id", c.id},
                {"previous_name", c.previous_name}, {"device_id", c.device_id}, {"job_id", c.job_id},
                {"source", c.source}, {"index", c.index}, {"dirty", c.dirty}};
    std::lock_guard<std::mutex> lk(g_state->mutex);
    g_state->events.push_back({{"event", lifecycle_event_to_string(event)}, {"ctx", std::move(ctx)}});
}

// ---- query encoding ---------------------------------------------------------------------------
// Output: one blob per request; each op result carries "blobOffset" (bytes) where its data starts.
// Geometry is packed in order: expolygon = contour points then each hole's points, a point being
// int64 x, y (paths: x, y, z). The JSON gives the lengths (contourLen, holeLens / n).

struct Out {
    std::string blob;
    void i64(int64_t v) { blob.append(reinterpret_cast<const char*>(&v), 8); }
    void pad8() { while (blob.size() % 8) blob.push_back('\0'); }
    void points(const Points& pts) { for (const Point& p : pts) { i64(p.x()); i64(p.y()); } }
    json expolygon(const ExPolygon& e)
    {
        points(e.contour.points);
        json holes = json::array();
        for (const Polygon& h : e.holes) { points(h.points); holes.push_back(h.points.size()); }
        return {{"contourLen", e.contour.points.size()}, {"holeLens", std::move(holes)}};
    }
    json expolygons(const ExPolygons& es) { json a = json::array(); for (const ExPolygon& e : es) a.push_back(expolygon(e)); return a; }
    json surface(const Surface& s)
    {
        json j = expolygon(s.expolygon);
        j["type"] = int(s.surface_type);
        j["thickness"] = s.thickness;
        j["thickness_layers"] = s.thickness_layers;
        j["bridge_angle"] = s.bridge_angle;
        j["extra_perimeters"] = s.extra_perimeters;
        return j;
    }
    json extrusion(const ExtrusionEntity* ee, bool flat, json* flat_out)
    {
        auto path = [&](const ExtrusionPath& p) {
            for (const auto& q : p.polyline.points) { i64(q.x()); i64(q.y()); i64(q.z()); }
            json j = {{"kind", "path"}, {"role", ExtrusionEntity::role_to_string(p.role())}, {"width", p.width},
                      {"height", p.height}, {"mm3_per_mm", p.mm3_per_mm}, {"n", p.polyline.points.size()}};
            if (flat) { flat_out->push_back(std::move(j)); return json(); }
            return j;
        };
        auto paths = [&](const ExtrusionPaths& ps, const char* kind) {
            json ch = json::array();
            for (const ExtrusionPath& p : ps) { json j = path(p); if (!flat) ch.push_back(std::move(j)); }
            return flat ? json() : json{{"kind", kind}, {"role", ExtrusionEntity::role_to_string(ee->role())}, {"children", std::move(ch)}};
        };
        if (auto* c = dynamic_cast<const ExtrusionEntityCollection*>(ee)) {
            json ch = json::array();
            for (const ExtrusionEntity* e : c->entities) { json j = extrusion(e, flat, flat_out); if (!flat) ch.push_back(std::move(j)); }
            return flat ? json() : json{{"kind", "collection"}, {"children", std::move(ch)}};
        }
        if (auto* l = dynamic_cast<const ExtrusionLoop*>(ee)) return paths(l->paths, "loop");
        if (auto* m = dynamic_cast<const ExtrusionMultiPath*>(ee)) return paths(m->paths, "multipath");
        if (auto* p = dynamic_cast<const ExtrusionPath*>(ee)) return path(*p);
        return flat ? json() : json{{"kind", "entity"}, {"role", ExtrusionEntity::role_to_string(ee->role())}};
    }
};

struct In {
    const uint8_t* data; size_t len; size_t pos;
    int64_t i64()
    {
        if (pos + 8 > len) throw std::runtime_error("request blob too short");
        int64_t v; std::memcpy(&v, data + pos, 8); pos += 8; return v;
    }
    Points points(size_t n)
    {
        if (n > len / 16) throw std::runtime_error("request blob too short");
        Points pts; pts.reserve(n);
        for (size_t i = 0; i < n; ++i) { const int64_t x = i64(), y = i64(); pts.emplace_back(coord_t(x), coord_t(y)); }
        return pts;
    }
    ExPolygon expolygon(const json& j)
    {
        ExPolygon e;
        e.contour.points = points(j.at("contourLen").get<size_t>());
        if (auto h = j.find("holeLens"); h != j.end() && h->is_array())
            for (const json& n : *h) e.holes.emplace_back(points(n.get<size_t>()));
        return e;
    }
    ExPolygons expolygons(const json& a)
    {
        ExPolygons out;
        for (const json& j : a) out.push_back(expolygon(j));
        return out;
    }
    Surface surface(const json& j)
    {
        Surface s(SurfaceType(j.value("type", int(stInternal))), expolygon(j));
        if (int(s.surface_type) < 0 || s.surface_type >= stCount) throw std::runtime_error("bad surface type");
        s.thickness = j.value("thickness", s.thickness);
        s.thickness_layers = (unsigned short)j.value("thickness_layers", int(s.thickness_layers));
        s.bridge_angle = j.value("bridge_angle", s.bridge_angle);
        s.extra_perimeters = (unsigned short)j.value("extra_perimeters", int(s.extra_perimeters));
        return s;
    }
};

json opt_value(const ConfigBase& cfg, const std::string& key)
{
    const ConfigOption* o = cfg.option(key);
    return o ? json(o->serialize()) : json(nullptr);
}

void refresh_lslices_bboxes(Layer& l)
{
    l.lslices_bboxes.clear();
    l.lslices_bboxes.reserve(l.lslices.size());
    for (const ExPolygon& island : l.lslices) l.lslices_bboxes.emplace_back(get_extents(island));
}

PrintObject* find_object(Print& print, const json& op)
{
    const uint64_t id = op.at("objectId").get<uint64_t>();
    for (size_t i = 0; i < print.objects().size(); ++i)
        if (print.objects()[i]->id().id == id) return print.get_object(i);
    throw std::runtime_error("unknown objectId");
}

Layer* find_layer(PrintObject& o, const json& op)
{
    const size_t li = op.at("layer").get<size_t>();
    if (op.value("support", false)) {
        if (li >= o.support_layers().size()) throw std::runtime_error("support layer out of range");
        return o.support_layers()[li];
    }
    if (li >= o.layers().size()) throw std::runtime_error("layer out of range");
    return o.layers()[li];
}

LayerRegion* find_region(Layer& l, const json& op)
{
    const size_t ri = op.at("region").get<size_t>();
    if (ri >= l.regions().size()) throw std::runtime_error("region out of range");
    return l.regions()[ri];
}

SurfaceCollection& which_surfaces(LayerRegion& r, const json& op)
{
    const std::string w = op.value("which", std::string("slices"));
    if (w == "slices") return r.slices;
    if (w == "fill_surfaces") return r.fill_surfaces;
    throw std::runtime_error("which must be slices or fill_surfaces");
}

json run_op(Print& print, const Active& act, const json& op, In& in, Out& out)
{
    const std::string name = op.at("op").get<std::string>();
    in.pos = op.value("blobOffset", size_t(0));
    out.pad8();
    json r = {{"blobOffset", out.blob.size()}};
    if (name == "print.info") {
        json ids = json::array();
        for (const PrintObject* o : print.objects()) ids.push_back(o->id().id);
        r["objectIds"] = ids; r["canceled"] = print.canceled(); r["scalingFactor"] = SCALING_FACTOR;
        r["step"] = step_name(act.step); r["objectId"] = act.object ? json(act.object->id().id) : json(nullptr);
    } else if (name == "print.config_value") {
        r["value"] = opt_value(print.full_print_config(), op.at("key").get<std::string>());
    } else if (name == "print.config_keys") {
        r["keys"] = print.full_print_config().keys();
    } else if (name == "object.info") {
        const PrintObject* o = find_object(print, op);
        const BoundingBox bb = o->bounding_box();
        const Transform3d& t = o->trafo();
        r["id"] = o->id().id; r["bbox"] = {bb.min.x(), bb.min.y(), bb.max.x(), bb.max.y()};
        r["trafo"] = std::vector<double>(t.matrix().data(), t.matrix().data() + 16);
        r["layerCount"] = o->layers().size(); r["supportLayerCount"] = o->support_layers().size();
        r["modelObject"] = o->model_object() ? json(o->model_object()->id().id) : json(nullptr);
    } else if (name == "object.config_value") {
        r["value"] = opt_value(find_object(print, op)->config(), op.at("key").get<std::string>());
    } else if (name == "object.config_keys") {
        r["keys"] = find_object(print, op)->config().keys();
    } else if (name == "region.config_value" || name == "region.config_keys") {
        PrintObject* o = find_object(print, op);
        const PrintRegionConfig& cfg = find_region(*find_layer(*o, op), op)->region().config();
        if (name == "region.config_keys") r["keys"] = cfg.keys(); else r["value"] = opt_value(cfg, op.at("key").get<std::string>());
    } else if (name == "object.layers") {
        PrintObject* o = find_object(print, op);
        json a = json::array();
        auto add = [&](const Layer* l) { a.push_back({{"print_z", l->print_z}, {"slice_z", l->slice_z}, {"height", l->height}, {"regionCount", l->regions().size()}}); };
        if (op.value("support", false)) for (const SupportLayer* l : o->support_layers()) add(l);
        else for (const Layer* l : o->layers()) add(l);
        r["layers"] = std::move(a);
    } else if (name == "layer.surfaces") {
        PrintObject* o = find_object(print, op);
        const SurfaceCollection& sc = which_surfaces(*find_region(*find_layer(*o, op), op), op);
        json a = json::array();
        for (const Surface& s : sc.surfaces) a.push_back(out.surface(s));
        r["surfaces"] = std::move(a);
    } else if (name == "layer.lslices") {
        PrintObject* o = find_object(print, op);
        r["expolygons"] = out.expolygons(find_layer(*o, op)->lslices);
    } else if (name == "layer.support_islands") {
        PrintObject* o = find_object(print, op);
        const size_t li = op.at("layer").get<size_t>();
        if (li >= o->support_layers().size()) throw std::runtime_error("support layer out of range");
        r["expolygons"] = out.expolygons(o->support_layers()[li]->support_islands);
    } else if (name == "layer.extrusions") {
        PrintObject* o = find_object(print, op);
        const std::string w = op.value("which", std::string("perimeters"));
        const ExtrusionEntityCollection* c = nullptr;
        if (w == "support_fills") {
            const size_t li = op.at("layer").get<size_t>();
            if (li >= o->support_layers().size()) throw std::runtime_error("support layer out of range");
            c = &o->support_layers()[li]->support_fills;
        } else {
            LayerRegion* reg = find_region(*find_layer(*o, op), op);
            c = w == "perimeters" ? &reg->perimeters : w == "fills" ? &reg->fills : w == "thin_fills" ? &reg->thin_fills : nullptr;
            if (!c) throw std::runtime_error("which must be perimeters, fills, thin_fills or support_fills");
        }
        if (op.value("flat", false)) { json f = json::array(); out.extrusion(c, true, &f); r["paths"] = std::move(f); }
        else r["tree"] = out.extrusion(c, false, nullptr);
    } else if (name == "model.objects") {
        json a = json::array();
        for (const ModelObject* mo : print.model().objects) {
            json vols = json::array();
            for (const ModelVolume* v : mo->volumes) {
                const Transform3d m = v->get_matrix();
                vols.push_back({{"id", v->id().id}, {"name", v->name}, {"type", int(v->type())},
                                {"vertexCount", v->mesh().its.vertices.size()}, {"triangleCount", v->mesh().its.indices.size()},
                                {"matrix", std::vector<double>(m.matrix().data(), m.matrix().data() + 16)}});
            }
            json insts = json::array();
            for (const ModelInstance* i : mo->instances) { const Transform3d m = i->get_matrix(); insts.push_back(std::vector<double>(m.matrix().data(), m.matrix().data() + 16)); }
            a.push_back({{"id", mo->id().id}, {"name", mo->name}, {"volumes", std::move(vols)}, {"instances", std::move(insts)}});
        }
        r["objects"] = std::move(a);
    } else if (name == "model.mesh") {
        const size_t oi = op.at("object").get<size_t>(), vi = op.at("volume").get<size_t>();
        if (oi >= print.model().objects.size() || vi >= print.model().objects[oi]->volumes.size()) throw std::runtime_error("model object/volume out of range");
        const indexed_triangle_set& its = print.model().objects[oi]->volumes[vi]->mesh().its;
        const size_t vb = its.vertices.size() * 12, ib = its.indices.size() * 12, base = out.blob.size();
        out.blob.append(reinterpret_cast<const char*>(its.vertices.data()), vb);
        out.blob.append(reinterpret_cast<const char*>(its.indices.data()), ib);
        r["vertexOffset"] = base; r["vertexCount"] = its.vertices.size();
        r["indexOffset"] = base + vb; r["triangleCount"] = its.indices.size();
    } else if (name == "surfaces.set" || name == "surfaces.append") {
        PrintObject* o = find_object(print, op);
        SurfaceCollection& sc = which_surfaces(*find_region(*find_layer(*o, op), op), op);
        Surfaces s;
        if (op.contains("surfaces")) for (const json& j : op["surfaces"]) s.push_back(in.surface(j));
        else {
            const SurfaceType t = SurfaceType(op.at("type").get<int>());
            for (ExPolygon& e : in.expolygons(op.at("expolygons"))) s.emplace_back(t, std::move(e));
        }
        if (name == "surfaces.set") sc.set(std::move(s)); else sc.append(std::move(s));
    } else if (name == "surfaces.clear") {
        PrintObject* o = find_object(print, op);
        which_surfaces(*find_region(*find_layer(*o, op), op), op).clear();
    } else if (name == "surfaces.set_type") {
        PrintObject* o = find_object(print, op);
        which_surfaces(*find_region(*find_layer(*o, op), op), op).set_type(SurfaceType(op.at("type").get<int>()));
    } else if (name == "surface.patch") {
        PrintObject* o = find_object(print, op);
        SurfaceCollection& sc = which_surfaces(*find_region(*find_layer(*o, op), op), op);
        const size_t i = op.at("index").get<size_t>();
        if (i >= sc.surfaces.size()) throw std::runtime_error("surface index out of range");
        Surface& s = sc.surfaces[i];
        if (op.contains("type")) s.surface_type = SurfaceType(op["type"].get<int>());
        if (op.contains("thickness")) s.thickness = op["thickness"].get<double>();
        if (op.contains("thickness_layers")) s.thickness_layers = (unsigned short)op["thickness_layers"].get<int>();
        if (op.contains("bridge_angle")) s.bridge_angle = op["bridge_angle"].get<double>();
        if (op.contains("extra_perimeters")) s.extra_perimeters = (unsigned short)op["extra_perimeters"].get<int>();
        if (op.contains("expolygon")) s.expolygon = in.expolygon(op["expolygon"]);
    } else if (name == "layer.set_lslices") {
        PrintObject* o = find_object(print, op);
        Layer* l = find_layer(*o, op);
        l->lslices = in.expolygons(op.at("expolygons"));
        refresh_lslices_bboxes(*l);
    } else if (name == "layer.make_slices") {
        PrintObject* o = find_object(print, op);
        Layer* l = find_layer(*o, op);
        l->make_slices();
        refresh_lslices_bboxes(*l);
    } else if (name.rfind("geom.", 0) == 0) {
        // libslic3r's own Clipper (bit-identical to desktop). Deltas / tolerances in scaled units.
        const ExPolygons a = in.expolygons(op.value("a", json::array()));
        ExPolygons res;
        if (name == "geom.offset_ex") res = offset_ex(a, float(op.at("delta").get<double>()));
        else if (name == "geom.offset") { // contours only, like Polygon.offset
            Polygons p;
            for (const ExPolygon& e : a) for (Polygon& q : offset(e.contour, float(op.at("delta").get<double>()))) p.push_back(std::move(q));
            json pa = json::array();
            for (const Polygon& q : p) { out.points(q.points); pa.push_back(q.points.size()); }
            r["polygonLens"] = std::move(pa);
            return r;
        }
        else if (name == "geom.union_ex") { const ExPolygons b = in.expolygons(op.value("b", json::array())); ExPolygons all = a; all.insert(all.end(), b.begin(), b.end()); res = union_ex(all); }
        else if (name == "geom.diff_ex") res = diff_ex(a, in.expolygons(op.value("b", json::array())));
        else if (name == "geom.intersection_ex") res = intersection_ex(a, in.expolygons(op.value("b", json::array())));
        else if (name == "geom.simplify") { for (const ExPolygon& e : a) for (ExPolygon& s : e.simplify(op.at("tolerance").get<double>())) res.push_back(std::move(s)); }
        else if (name == "geom.douglas_peucker") { res = a; for (ExPolygon& e : res) e.douglas_peucker(op.at("tolerance").get<double>()); }
        else throw std::runtime_error("unknown geometry op");
        r["expolygons"] = out.expolygons(res);
    } else {
        throw std::runtime_error("unknown op '" + name + "'");
    }
    return r;
}

} // namespace

Scope::Scope(Print&)
{
    g_state = new State();
    set_lifecycle_hook_fn(lifecycle_hook);
    // Inert unless the host installed Module.csHook (libslic3r additionally requires a
    // non-empty slicing_pipeline_plugin).
    if (has_js_hook()) Print::set_slicing_pipeline_hook_fn(pipeline_hook);
}

Scope::~Scope()
{
    Print::set_slicing_pipeline_hook_fn(nullptr);
    set_lifecycle_hook_fn(nullptr); // waits for callbacks in flight
    g_active.store(nullptr);
    delete g_state;
    g_state = nullptr;
}

void Scope::finish(json& report)
{
    if (!g_state) return;
    std::lock_guard<std::mutex> lk(g_state->mutex);
    json& ev = report["events"];
    if (!ev.is_array()) ev = json::array();
    for (json& e : g_state->events) ev.push_back(std::move(e));
    g_state->events = json::array();
    for (json& w : g_state->warnings) report["warnings"].push_back(std::move(w));
    g_state->warnings = json::array();
}

} // namespace cs::hooks

using namespace cs::hooks;

extern "C" EMSCRIPTEN_KEEPALIVE
int cs_hook_query(const char* req_json, int req_len, const uint8_t* blob, int blob_len,
                  char** out_json, int* out_json_len, uint8_t** out_blob, int* out_blob_len)
{
    if (out_json) *out_json = nullptr;
    if (out_json_len) *out_json_len = 0;
    if (out_blob) *out_blob = nullptr;
    if (out_blob_len) *out_blob_len = 0;
    Active* act = g_active.load();
#ifdef __EMSCRIPTEN_PTHREADS__
    if (!emscripten_is_main_runtime_thread()) act = nullptr; // only from inside csHook (main runtime thread)
#endif
    if (!act || !out_json || !out_json_len || !req_json || req_len <= 0 || blob_len < 0 || (blob_len > 0 && !blob)) return -1;
    json rep = {{"ok", true}, {"error", nullptr}, {"results", json::array()}};
    Out out;
    try {
        const json req = json::parse(req_json, req_json + req_len);
        const json ops = req.contains("batch") ? req["batch"] : json::array({req});
        if (!ops.is_array()) throw std::runtime_error("batch must be an array");
        In in{blob, size_t(blob_len), 0};
        for (const json& op : ops) {
            try {
                rep["results"].push_back(run_op(*act->print, *act, op, in, out));
            } catch (const std::exception& e) {
                rep["ok"] = false;
                rep["error"] = e.what();
                rep["results"].push_back({{"error", e.what()}});
                break; // later ops may depend on this one
            }
        }
    } catch (const std::exception& e) {
        rep["ok"] = false;
        rep["error"] = e.what();
    }
    const std::string js = rep.dump(-1, ' ', false, json::error_handler_t::replace);
    char* p = static_cast<char*>(std::malloc(js.size() + 1));
    if (!p) return -1;
    std::memcpy(p, js.c_str(), js.size() + 1);
    *out_json = p;
    *out_json_len = int(js.size());
    if (!out.blob.empty() && out_blob && out_blob_len) {
        uint8_t* b = static_cast<uint8_t*>(std::malloc(out.blob.size()));
        if (!b) return -1;
        std::memcpy(b, out.blob.data(), out.blob.size());
        *out_blob = b;
        *out_blob_len = int(out.blob.size());
    }
    return rep["ok"].get<bool>() ? 0 : 1;
}
