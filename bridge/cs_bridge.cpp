// Cubby Slicer ↔ OrcaSlicer (libslic3r) WebAssembly bridge.
//
// Implements the C ABI in cubby-slicer/docs/ENGINE-CONTRACT.md. Replaces the
// old wasm_wrap.cpp (orc_init/orc_slice), which carried allocator overrides,
// ad-hoc model scaling and a hand-rolled config applier.
//
// Design rules:
//  * Nothing from JS is trusted: cs::parse_job() bounds-checks every buffer.
//  * No exception ever escapes an exported function (Wasm EH would turn it
//    into a trap at the JS boundary and leave the instance unusable).
//  * Config goes through Orca's own ConfigBase::load_from_json(), so legacy
//    key renames / value substitutions behave exactly like the desktop app.
#include "common/cs_common.hpp"

#include <libslic3r/libslic3r.h>
#include <libslic3r/Config.hpp>
#include <libslic3r/CustomGCode.hpp>
#include <libslic3r/CutUtils.hpp>
#include <libslic3r/Slicing.hpp>
#include <libslic3r/TriangleSelector.hpp>
#include <libslic3r/calib.hpp>
#include <libslic3r/BrimEarsPoint.hpp>
#include <libslic3r/GCode/GCodeProcessor.hpp>
#include <libslic3r/Layer.hpp>
#include <libslic3r/Model.hpp>
#include <libslic3r/Format/bbs_3mf.hpp>
#if __has_include(<libslic3r/MixedFilament.hpp>)
#include <libslic3r/MixedFilament.hpp>
#endif
#include <libslic3r/Orient.hpp>
#include <libslic3r/Arrange.hpp>
#include <libslic3r/ModelArrange.hpp>
#include <libslic3r/GCode/WipeTower.hpp>
#include <libslic3r/Geometry.hpp>
#include <libslic3r/PlaceholderParser.hpp>
#include <libslic3r/Preset.hpp>
#include <libslic3r/Print.hpp>
#include <libslic3r/PrintConfig.hpp>
#include <libslic3r/TriangleMesh.hpp>
#include <libslic3r/Utils.hpp>
#if __has_include("libslic3r_version.h")
#include "libslic3r_version.h"  // generated into orca-build/src/libslic3r
#else
#include "common_func/common_func.hpp"  // Snapmaker Orca forks (e.g. FullSpectrum) keep versions here
#endif
#ifndef CS_ENGINE_ID
#define CS_ENGINE_ID "orca"
#endif
#if defined(FULLSPECTRUM_VERSION)
#define CS_ENGINE_VERSION FULLSPECTRUM_VERSION
// Snapmaker Orca forks track an older OrcaSlicer (2.3.x) libslic3r API.
#define CS_ORCA_LEGACY_API 1
#else
#define CS_ENGINE_VERSION SoftFever_VERSION
#endif

#include <array>
#include <atomic>
#include <cfloat>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <memory>
#include <mutex>
#include <set>
#include <sys/stat.h>
#include <unistd.h>

#ifdef CS_ENGINE_THREADS
// Pthreads engine (build-wasm-mt): real oneTBB. See cs_slice_start().
#include <climits>
#include <new>
#include <pthread.h>
#include <emscripten/eventloop.h>
#include <emscripten/threading.h>
#include <tbb/global_control.h>
#include <tbb/task_arena.h>
#endif

using namespace Slic3r;
using cs::json;

namespace {

double now_ms()
{
    using namespace std::chrono;
    return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
}

void ensure_runtime_dirs()
{
    static std::once_flag once;
    std::call_once(once, [] {
        ::mkdir("/tmp", 0777);
        ::mkdir("/data", 0777);
        // resources/info + resources/flush are preloaded (tiny); profiles are
        // resolved on the JS side and never read by the engine.
        set_resources_dir("/resources");
        set_data_dir("/data");
        set_temporary_dir("/tmp");
        set_logging_level(1); // errors only; the log sinks write to stderr
    });
}

#ifdef CS_ENGINE_THREADS
// Thread budget = the pthread pool size (csThreadCount() in wasm/mt_pre.js):
// the slicing thread + (N-1) TBB workers. Capping TBB to it keeps every TBB
// worker on a pre-created pool Worker. Main runtime thread only.
EM_JS(int, cs_js_thread_count, (), { return csThreadCount(); });
int g_engine_threads = 0;
int engine_threads()
{
    if (g_engine_threads == 0) {
        // With pthreads Emscripten's keepRuntimeAlive() is a counter even with
        // EXIT_RUNTIME=0: once a proxied call / mailbox callback finishes on the
        // main runtime thread with the counter at 0, the runtime "exits" and
        // terminates every pthread (the slice just hangs). The engine lives as
        // long as its host keeps the instance; hold one keepalive forever.
        emscripten_runtime_keepalive_push();
        g_engine_threads = std::max(1, cs_js_thread_count());
        // Process-wide while alive; intentionally never destroyed.
        new tbb::global_control(tbb::global_control::max_allowed_parallelism, size_t(g_engine_threads));
    }
    return g_engine_threads;
}
#endif

// Runs `f` single-threaded when called ON the main runtime thread of a
// pthreads build. That thread (the host's engine Web Worker) is blocked for
// the whole call, and Emscripten can only start a pthread whose creation was
// requested by another pthread once the main runtime thread returns to its
// event loop — so TBB work that waits for its workers (Orca's thread-pool
// warm-up barrier in Print::process) would deadlock. Parallel slicing goes
// through cs_slice_start() instead, which runs on its own pthread.
template<class F> auto serial_on_main_thread(F&& f)
{
#ifdef CS_ENGINE_THREADS
    if (emscripten_is_main_runtime_thread()) {
        engine_threads();
        tbb::task_arena serial(1);
        return serial.execute(std::forward<F>(f));
    }
#endif
    return f();
}

// Normalize job config values to the string / string-array shape Orca's JSON
// loader expects (it rejects bare numbers/bools).
json normalize_config_json(const json& in)
{
    json out = json::object();
    for (auto it = in.begin(); it != in.end(); ++it) {
        const json& v = it.value();
        if (v.is_array()) {
            json arr = json::array();
            for (const json& e : v) arr.push_back(e.is_array() ? e : json(cs::scalar_to_string(e)));
            out[it.key()] = std::move(arr);
        } else {
            out[it.key()] = cs::scalar_to_string(v);
        }
    }
    return out;
}

// Load a {key: value} JSON object into `cfg` via Orca's loader. Returns the
// substitutions / unknown keys for the report.
void load_config_json(DynamicPrintConfig& cfg, const json& values, json& substitutions)
{
    if (values.empty()) return;
    static std::atomic<int> seq{0};
    const std::string path = "/tmp/cs_config_" + std::to_string(++seq) + ".json";
    {
        std::ofstream f(path, std::ios::binary | std::ios::trunc);
        if (!f) throw cs::JobError("cannot write config to MEMFS");
        f << normalize_config_json(values).dump();
    }
    ConfigSubstitutionContext ctx(ForwardCompatibilitySubstitutionRule::EnableSilent);
    std::map<std::string, std::string> key_values;
    std::string reason;
    int rc = cfg.load_from_json(path, ctx, true, key_values, reason);
    ::unlink(path.c_str());
    if (rc != 0) throw cs::JobError("invalid config: " + (reason.empty() ? std::string("parse error") : reason));
    for (const ConfigSubstitution& s : ctx.substitutions)
        substitutions.push_back(s.opt_def ? s.opt_def->opt_key + ": '" + s.old_value + "' -> '" +
                                                (s.new_value ? s.new_value->serialize() : std::string()) + "'"
                                          : s.old_value);
    for (const std::string& k : ctx.unrecogized_keys) substitutions.push_back("unknown key ignored: " + k);
}

size_t vector_size(const DynamicPrintConfig& cfg, const char* key)
{
    const ConfigOption* opt = cfg.option(key);
    if (auto* v = dynamic_cast<const ConfigOptionVectorBase*>(opt)) return v->size();
    return 0;
}

std::string type_name(ConfigOptionType t)
{
    switch (t) {
    case coFloat: return "float";            case coFloats: return "floats";
    case coInt: return "int";                case coInts: return "ints";
    case coString: return "string";          case coStrings: return "strings";
    case coPercent: return "percent";        case coPercents: return "percents";
    case coFloatOrPercent: return "floatOrPercent";
    case coFloatsOrPercents: return "floatsOrPercents";
    case coPoint: return "point";            case coPoints: return "points";
    case coPoint3: return "point3";
    case coBool: return "bool";              case coBools: return "bools";
    case coEnum: return "enum";              case coEnums: return "enums";
#ifndef CS_ORCA_LEGACY_API
    case coPointsGroups: return "pointsGroups";
    case coIntsGroups: return "intsGroups";
#endif
    default: return "none";
    }
}

std::string mode_name(ConfigOptionMode m)
{
    switch (m) {
    case comSimple: return "simple";
    case comAdvanced: return "advanced";
    default: return "expert";
    }
}

std::string gui_type_name(ConfigOptionDef::GUIType g)
{
    using G = ConfigOptionDef::GUIType;
    switch (g) {
    case G::i_enum_open: return "i_enum_open";
    case G::f_enum_open: return "f_enum_open";
    case G::color: return "color";
    case G::select_open: return "select_open";
    case G::slider: return "slider";
    case G::legend: return "legend";
    case G::one_string: return "one_string";
    default: return "";
    }
}

json number_or_null(double v)
{
    // Orca uses ±FLT_MAX / INT_MAX-ish sentinels for "unbounded".
    if (!std::isfinite(v) || std::fabs(v) >= 1e30 || std::fabs(v) >= 2147483647.0) return nullptr;
    return v;
}

// Model from validated job meshes. Each job object becomes one ModelObject
// with one volume and one instance; the transform is baked into the mesh
// and the object is re-centred so the instance carries the placement.
// Mesh-local vertices → bed coordinates (column-major transform). The host
// already flips the winding of mirrored transforms; an inside-out result is
// still corrected below via the signed volume.
TriangleMesh bed_mesh(const std::vector<float>& positions, const std::vector<uint32_t>& indices, const double* T,
                      const std::string& name, std::vector<int>* tri_map = nullptr)
{
    indexed_triangle_set its;
    const size_t nv = positions.size() / 3;
    its.vertices.reserve(nv);
    for (size_t i = 0; i < nv; ++i) {
        const double x = positions[i * 3], y = positions[i * 3 + 1], z = positions[i * 3 + 2];
        const double wx = T[0] * x + T[4] * y + T[8] * z + T[12];
        const double wy = T[1] * x + T[5] * y + T[9] * z + T[13];
        const double wz = T[2] * x + T[6] * y + T[10] * z + T[14];
        if (!std::isfinite(wx) || !std::isfinite(wy) || !std::isfinite(wz))
            throw cs::JobError(name + ": transform produced non-finite coordinates");
        its.vertices.emplace_back(float(wx), float(wy), float(wz));
    }
    its.indices.reserve(indices.size() / 3);
    if (tri_map) tri_map->assign(indices.size() / 3, -1);
    for (size_t i = 0; i + 2 < indices.size(); i += 3) {
        const int a = int(indices[i]), b = int(indices[i + 1]), c = int(indices[i + 2]);
        if (a == b || b == c || a == c) continue; // degenerate
        if (tri_map) (*tri_map)[i / 3] = int(its.indices.size());
        its.indices.emplace_back(a, b, c);
    }
    if (its.indices.empty()) throw cs::JobError(name + ": mesh has no valid triangles");
    TriangleMesh mesh(std::move(its));
    if (mesh.volume() < 0) mesh.flip_triangles(); // inside-out input
    return mesh;
}

// Painted facets (Orca per-triangle hex strings, as in 3MF) onto a volume.
// `tri_map` maps the host's triangle indices to the engine mesh's (degenerate
// triangles are dropped); painting of dropped triangles is ignored.
void apply_paint(ModelVolume* vol, const json& paint, const std::vector<int>& tri_map)
{
    if (!paint.is_object() || paint.empty()) return;
    const int n = int(vol->mesh().its.indices.size());
    auto load = [&](const char* key, FacetsAnnotation& fa) {
        auto it = paint.find(key);
        if (it == paint.end() || !it->is_array() || it->empty()) return;
        fa.reserve(int(it->size()));
        for (const json& e : *it) {
            if (!e.is_array() || e.size() != 2 || !e[0].is_number_integer() || !e[1].is_string()) continue;
            const long long src = e[0].get<long long>();
            if (src < 0 || size_t(src) >= tri_map.size()) continue;
            const int t = tri_map[size_t(src)];
            const std::string& hex = e[1].get_ref<const std::string&>();
            if (t < 0 || t >= n || hex.empty() || hex.size() > 1 << 20) continue;
            if (hex.find_first_not_of("0123456789ABCDEFabcdef") != std::string::npos) continue;
            fa.set_triangle_from_string(t, hex);
        }
        fa.shrink_to_fit();
    };
    load("support", vol->supported_facets);
    load("seam", vol->seam_facets);
    load("color", vol->mmu_segmentation_facets);
    load("fuzzy", vol->fuzzy_skin_facets);
}

ModelVolumeType volume_type(const std::string& t)
{
    if (t == "negative") return ModelVolumeType::NEGATIVE_VOLUME;
    if (t == "modifier") return ModelVolumeType::PARAMETER_MODIFIER;
    if (t == "support_blocker") return ModelVolumeType::SUPPORT_BLOCKER;
    if (t == "support_enforcer") return ModelVolumeType::SUPPORT_ENFORCER;
    return ModelVolumeType::MODEL_PART;
}

void build_model(Model& model, const cs::Job& job, json& substitutions)
{
    for (const cs::MeshInput& m : job.objects) {
        ModelObject* obj = model.add_object();
        obj->name = m.name;
        obj->input_file = m.name;
        std::vector<int> tri_map;
        ModelVolume* vol = obj->add_volume(bed_mesh(m.positions, m.indices, m.transform, m.name, &tri_map));
        vol->name = m.name;
        apply_paint(vol, m.paint, tri_map);
        // Extra volumes (Orca parts / negative parts / modifiers / support
        // blockers & enforcers), each with its own settings.
        for (const cs::VolumeInput& p : m.parts) {
            std::vector<int> ptri_map;
            ModelVolume* pv = obj->add_volume(bed_mesh(p.positions, p.indices, p.transform, p.name, &ptri_map), volume_type(p.type));
            pv->name = p.name;
            apply_paint(pv, p.paint, ptri_map);
            if (!p.config.empty()) {
                DynamicPrintConfig vc;
                load_config_json(vc, p.config, substitutions);
                pv->config.assign_config(vc);
            }
        }
        // The meshes are in bed coordinates. Re-centre them on the object's own
        // origin and move that offset into the instance (Orca's
        // center_around_origin() shifts the volumes only, it does not
        // compensate the instances).
        const Vec3d centre = obj->raw_mesh_bounding_box().center();
        obj->center_around_origin(false);
        ModelInstance* inst = obj->add_instance();
        inst->set_offset(centre);
        if (job.drop_to_bed) obj->ensure_on_bed();

        // Brim ears: host positions are mesh-local; the mesh was baked to bed
        // coordinates and re-centred on `centre`.
        for (const json& bp : m.brim_points) {
            auto pos = bp.find("pos");
            if (pos == bp.end() || !pos->is_array() || pos->size() != 3) continue;
            const double* T = m.transform;
            const double x = (*pos)[0].get<double>(), y = (*pos)[1].get<double>(), z = (*pos)[2].get<double>();
            const Vec3d w(T[0] * x + T[4] * y + T[8] * z + T[12], T[1] * x + T[5] * y + T[9] * z + T[13], T[2] * x + T[6] * y + T[10] * z + T[14]);
            const Vec3d local = w - centre;
            const float r = bp.value("radius", 0.f);
            if (r > 0.f && local.allFinite()) obj->brim_points.emplace_back(local.cast<float>(), r);
        }
        // Variable layer height + height range modifiers (z from the object's bottom).
        if (!m.layer_height_profile.empty()) obj->layer_height_profile.set(std::vector<coordf_t>(m.layer_height_profile.begin(), m.layer_height_profile.end()));
        // Orca's slicing reads "layer_height" from every range (the GUI always
        // stores it); default it to the print's layer height. Overlapping ranges
        // are dropped (Orca's object list doesn't allow them).
        double default_lh = 0.2;
        if (auto lh = job.config.find("layer_height"); lh != job.config.end()) {
            try { default_lh = lh->is_string() ? std::stod(lh->get<std::string>()) : lh->get<double>(); } catch (...) {}
        }
        std::vector<std::pair<double, double>> taken;
        for (const json& r : m.layer_ranges) {
            if (!r.is_object() || !r.contains("min") || !r.contains("max") || !r["min"].is_number() || !r["max"].is_number()) continue;
            const double lo = r["min"].get<double>(), hi = r["max"].get<double>();
            if (!(hi > lo) || !std::isfinite(lo) || !std::isfinite(hi) || lo < 0) continue;
            bool overlap = false;
            for (auto& t : taken) if (lo < t.second && t.first < hi) overlap = true;
            if (overlap) continue;
            taken.emplace_back(lo, hi);
            DynamicPrintConfig rc;
            if (auto c = r.find("config"); c != r.end() && c->is_object()) load_config_json(rc, *c, substitutions);
            if (!rc.has("layer_height") || rc.opt_float("layer_height") <= 0) rc.set_key_value("layer_height", new ConfigOptionFloat(default_lh));
            obj->layer_config_ranges[{lo, hi}].assign_config(rc);
        }

        if (!m.config.empty()) {
            DynamicPrintConfig oc;
            load_config_json(oc, m.config, substitutions);
            obj->config.assign_config(oc);
        }
    }
}

json collect_stats(const Print& print, const GCodeProcessorResult& result, const DynamicPrintConfig& cfg)
{
    const auto& ps = result.print_statistics;
    json st;
    st["printTimeSec"] = double(ps.modes[static_cast<size_t>(PrintEstimatedStatistics::ETimeMode::Normal)].time);

    const auto* diam  = cfg.option<ConfigOptionFloats>("filament_diameter");
    const auto* dens  = cfg.option<ConfigOptionFloats>("filament_density");
    const auto* cost  = cfg.option<ConfigOptionFloats>("filament_cost");
    size_t n = 1;
    for (const auto& kv : ps.total_volumes_per_extruder) n = std::max(n, kv.first + 1);
    json mm = json::array(), cm3 = json::array(), g = json::array(), money = json::array();
    for (size_t e = 0; e < n; ++e) {
        auto it = ps.total_volumes_per_extruder.find(e);
        const double vol_mm3 = it == ps.total_volumes_per_extruder.end() ? 0.0 : it->second;
        const double d = diam && !diam->values.empty() ? diam->get_at(e) : 1.75;
        const double area = M_PI * d * d / 4.0;
        const double rho = dens && !dens->values.empty() ? dens->get_at(e) : 1.24; // g/cm3
        const double c = cost && !cost->values.empty() ? cost->get_at(e) : 0.0;    // per kg
        const double grams = vol_mm3 / 1000.0 * rho;
        mm.push_back(area > 0 ? vol_mm3 / area : 0.0);
        cm3.push_back(vol_mm3 / 1000.0);
        g.push_back(grams);
        money.push_back(grams / 1000.0 * c);
    }
    st["filamentMm"] = mm; st["filamentCm3"] = cm3; st["filamentG"] = g; st["filamentCost"] = money;

    std::set<long long> zs;
    double max_z = 0;
    for (const PrintObject* po : print.objects())
        for (const Layer* l : po->layers()) {
            zs.insert(llround(l->print_z * 1e4));
            max_z = std::max(max_z, l->print_z);
        }
    for (const PrintObject* po : print.objects())
        for (const SupportLayer* l : po->support_layers()) zs.insert(llround(l->print_z * 1e4));
    st["layers"] = zs.size();
    st["maxZ"] = max_z;
    return st;
}

int slice_impl(const char* job_json, int job_len, const uint8_t* blob, int blob_len,
               uint8_t** out_gcode, int* out_gcode_len, json& report)
{
    ensure_runtime_dirs();
    cs::progress(0, "Preparing");
    cs::Job job = cs::parse_job(job_json, job_len, blob, blob_len);

    json& subs = report["substitutions"];
    DynamicPrintConfig config;
    config.apply(FullPrintConfig::defaults());
    load_config_json(config, job.config, subs);

    // Mirror the desktop app / CLI multi-extruder bookkeeping.
    const int filament_count = int(std::max<size_t>(1, vector_size(config, "filament_diameter")));
    const int extruder_count = int(std::max<size_t>(1, vector_size(config, "nozzle_diameter")));
    {
        auto& fmap = config.option<ConfigOptionInts>("filament_map", true)->values;
        if (int(fmap.size()) < filament_count) fmap.resize(filament_count, 1);
        if (extruder_count == 1) std::fill(fmap.begin(), fmap.end(), 1);
#ifndef CS_ORCA_LEGACY_API
        if (!config.has("nozzle_volume_type")) {
            auto* nvt = config.option<ConfigOptionEnumsGeneric>("nozzle_volume_type", true);
            nvt->values.resize(extruder_count, nvtStandard);
        }
#endif
    }
    config.normalize_fdm();

    Model model;
    build_model(model, job, subs);

    Print print;
    print.set_plate_origin(Vec3d::Zero());
    json& warnings = report["warnings"];
    std::mutex status_mutex; // status may be reported from TBB workers (MT build)
    print.set_status_callback([&warnings, &status_mutex](const PrintBase::SlicingStatus& s) {
        std::lock_guard<std::mutex> lock(status_mutex);
        if (s.percent >= 0) cs::progress(int(s.percent * 0.9), s.text);
        if (s.warning_step != -1 && !s.text.empty())
            warnings.push_back({{"code", "slicing"}, {"message", s.text}});
    });

    const std::string printer_model = config.opt_string("printer_model", true);
    print.is_BBL_printer() = printer_model.rfind("Bambu Lab", 0) == 0;
    Model::setExtruderParams(config, filament_count);
    Model::setPrintSpeedTable(config, print.config());

    // Reject objects outside the printable volume up front with a clear
    // message (the desktop plater does this before slicing; libslic3r itself
    // would happily emit moves off the bed).
    {
        BoundingBoxf bed;
        if (const auto* area = config.option<ConfigOptionPoints>("printable_area"))
            for (const Vec2d& p : area->values) bed.merge(p);
        const double max_h = config.has("printable_height") ? config.opt_float("printable_height") : 0.0;
        const double eps = 0.05;
        if (bed.defined)
            for (const ModelObject* o : model.objects) {
                const BoundingBoxf3 bb = o->instance_bounding_box(0);
                if (bb.min.x() < bed.min.x() - eps || bb.min.y() < bed.min.y() - eps ||
                    bb.max.x() > bed.max.x() + eps || bb.max.y() > bed.max.y() + eps)
                    throw cs::JobError(o->name + " is outside the printable area of the bed.");
                if (max_h > 0 && bb.max.z() > max_h + eps)
                    throw cs::JobError(o->name + " is taller than the printer's maximum print height (" +
                                       std::to_string(int(max_h)) + " mm).");
            }
    }

    // Plate custom G-code (colour changes / pauses / custom) — Orca keys these per plate.
    if (!job.custom_gcodes.empty()) {
        CustomGCode::Info info;
        info.mode = filament_count > 1 ? CustomGCode::MultiAsSingle : CustomGCode::SingleExtruder;
        for (const json& g : job.custom_gcodes) {
            if (!g.is_object() || !g.contains("z")) continue;
            CustomGCode::Item it;
            it.print_z = g["z"].get<double>();
            const std::string t = g.value("type", std::string("color_change"));
            it.type = t == "pause" ? CustomGCode::PausePrint : t == "custom" ? CustomGCode::Custom
                    : t == "tool_change" ? CustomGCode::ToolChange : t == "template" ? CustomGCode::Template : CustomGCode::ColorChange;
            it.extruder = g.value("extruder", 1);
            it.color = g.value("color", std::string());
            it.extra = g.value("extra", std::string());
            info.gcodes.push_back(it);
        }
        std::sort(info.gcodes.begin(), info.gcodes.end());
        model.plates_custom_gcodes[0] = info;
    }
    if (!job.calib.empty()) {
        Calib_Params cp;
        static const std::map<std::string, CalibMode> modes = {
            {"pa_line", CalibMode::Calib_PA_Line}, {"pa_pattern", CalibMode::Calib_PA_Pattern}, {"pa_tower", CalibMode::Calib_PA_Tower},
            {"flow_rate", CalibMode::Calib_Flow_Rate}, {"temp_tower", CalibMode::Calib_Temp_Tower}, {"vol_speed_tower", CalibMode::Calib_Vol_speed_Tower},
            {"vfa_tower", CalibMode::Calib_VFA_Tower}, {"retraction_tower", CalibMode::Calib_Retraction_tower},
            {"input_shaping_freq", CalibMode::Calib_Input_shaping_freq}, {"input_shaping_damp", CalibMode::Calib_Input_shaping_damp},
#ifndef CS_ORCA_LEGACY_API
            {"cornering", CalibMode::Calib_Cornering},
#endif
        };
        auto mi = modes.find(job.calib.value("mode", std::string()));
        if (mi == modes.end()) throw cs::JobError("unknown calibration mode");
        cp.mode = mi->second;
        cp.start = job.calib.value("start", 0.0);
        cp.end = job.calib.value("end", 0.0);
        cp.step = job.calib.value("step", 0.0);
        cp.print_numbers = job.calib.value("printNumbers", true);
        cp.extruder_id = job.calib.value("extruderId", 0);
        cp.freqStartX = job.calib.value("freqStartX", 0.0); cp.freqEndX = job.calib.value("freqEndX", 0.0);
        cp.freqStartY = job.calib.value("freqStartY", 0.0); cp.freqEndY = job.calib.value("freqEndY", 0.0);
        cp.test_model = job.calib.value("testModel", 0);
#ifndef CS_ORCA_LEGACY_API
        cp.shaper_type = job.calib.value("shaperType", std::string());
#endif
        if (auto a = job.calib.find("accelerations"); a != job.calib.end() && a->is_array()) for (const json& v : *a) cp.accelerations.push_back(v.get<double>());
        if (auto a = job.calib.find("speeds"); a != job.calib.end() && a->is_array()) for (const json& v : *a) cp.speeds.push_back(v.get<double>());
        print.set_calib_params(cp);
    }

    print.apply(model, config);
    if (print.empty())
        throw cs::JobError("Nothing to slice: no object is fully inside the print volume.");

    if (job.validate) {
        StringObjectException warning;
        StringObjectException err = print.validate(&warning);
        if (!err.string.empty()) throw cs::JobError(err.string);
        if (!warning.string.empty()) warnings.push_back({{"code", "validate"}, {"message", warning.string}});
    }

    const double t0 = now_ms();
    print.process();
    const double t1 = now_ms();
    if (std::string conflict = print.get_conflict_string(); !conflict.empty())
        warnings.push_back({{"code", "conflict"}, {"message", conflict}});

    cs::progress(90, "Generating G-code");
    GCodeProcessorResult result;
    const std::string out_path = "/tmp/cs_output.gcode";
    ::unlink(out_path.c_str());
    const std::string written = print.export_gcode(out_path, &result, nullptr);
    const double t2 = now_ms();
#ifndef CS_ORCA_LEGACY_API
    if (result.gcode_check_result.error_code)
        warnings.push_back({{"code", "unprintable_area"},
                            {"message", "G-code contains moves outside the printable area."}});
#endif

    // Read the G-code straight into the output buffer (no intermediate copy),
    // then drop the MEMFS file so peak memory stays ~2x the G-code size.
    const std::string& path = written.empty() ? out_path : written;
    FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) throw cs::JobError("G-code export produced no file");
    std::fseek(f, 0, SEEK_END);
    long sz = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    if (sz <= 0 || sz > 0x7fffffffL) { std::fclose(f); ::unlink(path.c_str()); throw cs::JobError("G-code export is empty or too large"); }
    uint8_t* buf = static_cast<uint8_t*>(std::malloc(size_t(sz)));
    if (!buf) { std::fclose(f); ::unlink(path.c_str()); throw std::bad_alloc(); }
    const size_t got = std::fread(buf, 1, size_t(sz), f);
    std::fclose(f);
    ::unlink(path.c_str());
    if (got != size_t(sz)) { std::free(buf); throw cs::JobError("short read of exported G-code"); }

    report["stats"] = collect_stats(print, result, config);
    report["timings"] = {{"processMs", t1 - t0}, {"exportMs", t2 - t1}};
    report["ok"] = true;
    *out_gcode = buf;
    *out_gcode_len = int(sz);
    cs::progress(100, "Done");
    return 0;
}

std::string scope_of(const std::string& key)
{
    static const std::set<std::string> print(Preset::print_options().begin(), Preset::print_options().end());
    static const std::set<std::string> fil(Preset::filament_options().begin(), Preset::filament_options().end());
    static const std::set<std::string> mach = [] {
        std::set<std::string> s(Preset::printer_options().begin(), Preset::printer_options().end());
        for (const auto& k : Preset::machine_limits_options()) s.insert(k);
        return s;
    }();
    if (print.count(key)) return "print";
    if (fil.count(key)) return "filament";
    if (mach.count(key)) return "machine";
    return "other";
}

// A default's text. Enum-list defaults are spelled from the definition's key
// map: some forks build them without one (nothing to look names up in).
std::string default_text(const ConfigOptionDef& def)
{
    if (!def.default_value) return std::string();
    if (def.type == coEnums && def.enum_keys_map) {
        const auto* v = dynamic_cast<const ConfigOptionInts*>(def.default_value.get());
        if (v) {
            std::string out;
            for (size_t i = 0; i < v->values.size(); ++i) {
                if (i) out += ',';
                for (const auto& kv : *def.enum_keys_map) if (kv.second == v->values[i]) { out += kv.first; break; }
            }
            return out;
        }
    }
    return def.default_value->serialize();
}

std::string describe_impl()
{
    json opts = json::object();
    for (const auto& [key, def] : print_config_def.options) {
        if (def.type == coNone) continue;
        json o;
        o["type"] = type_name(def.type);
        o["label"] = def.label;
        o["fullLabel"] = def.full_label.empty() ? def.label : def.full_label;
        o["category"] = def.category;
        o["tooltip"] = def.tooltip;
        o["sidetext"] = def.sidetext;
        o["mode"] = mode_name(def.mode);
        o["min"] = number_or_null(def.min);
        o["max"] = number_or_null(def.max);
        o["default"] = default_text(def);
        if (!def.enum_values.empty()) {
            o["enumValues"] = def.enum_values;
            o["enumLabels"] = def.enum_labels.empty() ? def.enum_values : def.enum_labels;
        }
        o["multiline"] = def.multiline;
        o["fullWidth"] = def.full_width;
        o["readonly"] = def.readonly;
        o["nullable"] = def.nullable;
        o["guiType"] = gui_type_name(def.gui_type);
        o["scope"] = scope_of(key);
        opts[key] = std::move(o);
    }
    return json{{"engine", CS_ENGINE_ID}, {"version", CS_ENGINE_VERSION}, {"options", std::move(opts)}}.dump();
}

} // namespace

extern "C" {

EMSCRIPTEN_KEEPALIVE const char* cs_version(void)
{
    static const std::string v = json{{"engine", CS_ENGINE_ID}, {"version", CS_ENGINE_VERSION},
                                      {"bridge", cs::BRIDGE_ABI},
#ifdef NDEBUG
                                      {"build", "release"}
#else
                                      {"build", "debug"}
#endif
                                     }.dump();
    return v.c_str();
}

EMSCRIPTEN_KEEPALIVE int cs_describe_config(char** out_json, int* out_len)
{
    try {
        return cs::emit(describe_impl(), out_json, out_len);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "cs_describe_config: %s\n", e.what());
    } catch (...) {}
    return -1;
}

EMSCRIPTEN_KEEPALIVE int cs_slice(const char* job_json, int job_len, const uint8_t* blob, int blob_len,
                                  uint8_t** out_gcode, int* out_gcode_len,
                                  char** out_report, int* out_report_len)
{
    if (out_gcode) *out_gcode = nullptr;
    if (out_gcode_len) *out_gcode_len = 0;
    json report = cs::make_report();
    int rc = 1;
    try {
        if (!out_gcode || !out_gcode_len) throw cs::JobError("null output pointers");
        rc = serial_on_main_thread([&] { return slice_impl(job_json, job_len, blob, blob_len, out_gcode, out_gcode_len, report); });
    } catch (const cs::JobError& e) {
        report["error"] = e.what(); rc = 2;
    } catch (const json::exception& e) {
        report["error"] = std::string("invalid job JSON: ") + e.what(); rc = 2;
    } catch (const std::bad_alloc&) {
        report["error"] = "Out of memory while slicing. Try fewer/smaller objects or a larger layer height."; rc = 3;
    } catch (const std::exception& e) {
        report["error"] = e.what(); rc = 4;
    } catch (...) {
        report["error"] = "unknown engine exception"; rc = 5;
    }
    if (rc != 0) {
        report["ok"] = false;
        if (out_gcode && *out_gcode) { std::free(*out_gcode); *out_gcode = nullptr; *out_gcode_len = 0; }
    }
    try {
        cs::emit(report.dump(-1, ' ', false, json::error_handler_t::replace), out_report, out_report_len);
    } catch (...) {}
    return rc;
}

#ifdef CS_ENGINE_THREADS
// ---- pthreads engine: asynchronous slice -------------------------------------------
// cs_slice_start(): same arguments as cs_slice plus `state` (2 x int32, 4-byte
// aligned). Starts cs_slice on a dedicated pthread (64 MB stack, like the
// single-thread engine's main stack) and returns immediately: 0 = started,
// -1 = could not start (the host may fall back to cs_slice, which then runs
// serially). When the slice is finished — out-params written — state[1] holds
// cs_slice's return code and state[0] flips 0 -> 1 with a futex wake, so the
// host can Atomics.waitAsync()/poll on it WITHOUT blocking its event loop.
// The host must not call into the engine again until state[0] == 1.
struct SliceStart {
    const char* job; int job_len; const uint8_t* blob; int blob_len;
    uint8_t** out_gcode; int* out_gcode_len; char** out_report; int* out_report_len;
    int32_t* state;
};

static void* slice_thread_main(void* p)
{
    std::unique_ptr<SliceStart> a(static_cast<SliceStart*>(p));
    const int rc = cs_slice(a->job, a->job_len, a->blob, a->blob_len,
                            a->out_gcode, a->out_gcode_len, a->out_report, a->out_report_len);
    __atomic_store_n(&a->state[1], rc, __ATOMIC_SEQ_CST);
    __atomic_store_n(&a->state[0], 1, __ATOMIC_SEQ_CST);
    emscripten_futex_wake(&a->state[0], INT_MAX);
    return nullptr;
}

EMSCRIPTEN_KEEPALIVE int cs_slice_start(const char* job_json, int job_len, const uint8_t* blob, int blob_len,
                                        uint8_t** out_gcode, int* out_gcode_len,
                                        char** out_report, int* out_report_len, int32_t* state)
{
    if (!state || (reinterpret_cast<uintptr_t>(state) & 3u)) return -1;
    state[0] = 0;
    state[1] = -1;
    engine_threads();
    auto* a = new (std::nothrow) SliceStart{job_json, job_len, blob, blob_len,
                                            out_gcode, out_gcode_len, out_report, out_report_len, state};
    if (!a) return -1;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, size_t(64) << 20);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    pthread_t t;
    const int err = pthread_create(&t, &attr, slice_thread_main, a);
    pthread_attr_destroy(&attr);
    if (err != 0) { delete a; return -1; }
    return 0;
}

// Threads a slice uses (slicing thread + TBB workers) = pthread pool size.
EMSCRIPTEN_KEEPALIVE int cs_thread_count(void) { return engine_threads(); }
#endif

EMSCRIPTEN_KEEPALIVE int cs_eval_condition(const char* expr, int expr_len, const char* config_json, int config_len)
{
    try {
        ensure_runtime_dirs();
        if (!expr || expr_len <= 0) return 1;
        json cfgj = cs::parse_config_json(config_json, config_len);
        DynamicPrintConfig cfg;
        cfg.apply(FullPrintConfig::defaults());
        json subs = json::array();
        load_config_json(cfg, cfgj, subs);
        return PlaceholderParser::evaluate_boolean_expression(std::string(expr, size_t(expr_len)), cfg) ? 1 : 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "cs_eval_condition: %s\n", e.what());
    } catch (...) {}
    return -1;
}

// Auto-orient (OrcaSlicer's AutoOrienter, same as the desktop "Auto orient").
// Input: a job JSON with exactly one object whose `transform` should be the
// object's current rotation*scale (no translation needed). Output JSON:
//   {"ok":true,"rotation":[r00,r01,r02,r10,...] (row-major 3x3, to be
//    pre-multiplied onto the object's rotation), "axis":[x,y,z], "angle":rad}
EMSCRIPTEN_KEEPALIVE int cs_orient(const char* job_json, int job_len, const uint8_t* blob, int blob_len,
                                   char** out_json, int* out_len)
{
    json out = {{"ok", false}, {"error", nullptr}};
    try {
        ensure_runtime_dirs();
        cs::Job job = cs::parse_job(job_json, job_len, blob, blob_len);
        if (job.objects.size() != 1) throw cs::JobError("cs_orient expects exactly one object");
        const cs::MeshInput& m = job.objects.front();
        indexed_triangle_set its;
        const double* T = m.transform;
        const size_t nv = m.positions.size() / 3;
        its.vertices.reserve(nv);
        for (size_t i = 0; i < nv; ++i) {
            const double x = m.positions[i * 3], y = m.positions[i * 3 + 1], z = m.positions[i * 3 + 2];
            its.vertices.emplace_back(float(T[0] * x + T[4] * y + T[8] * z), float(T[1] * x + T[5] * y + T[9] * z),
                                      float(T[2] * x + T[6] * y + T[10] * z));
        }
        for (size_t i = 0; i + 2 < m.indices.size(); i += 3)
            its.indices.emplace_back(int(m.indices[i]), int(m.indices[i + 1]), int(m.indices[i + 2]));
        orientation::OrientMeshs items(1);
        items[0].mesh = TriangleMesh(std::move(its));
        items[0].name = m.name;
        orientation::OrientParams params;
        // _orient() invokes both callbacks unconditionally.
        params.progressind = [](unsigned, std::string) {};
        params.stopcondition = [] { return false; };
        serial_on_main_thread([&] { orientation::orient(items, {}, params); return 0; });
        const Matrix3d R = items[0].rotation_matrix;
        const Vec3d axis = items[0].axis;
        const double angle = items[0].angle;
        json rot = json::array();
        for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) rot.push_back(std::isfinite(R(r, c)) ? R(r, c) : (r == c ? 1.0 : 0.0));
        out = {{"ok", true}, {"error", nullptr}, {"rotation", rot}, {"axis", {axis.x(), axis.y(), axis.z()}}, {"angle", angle}};
    } catch (const std::exception& e) {
        out["error"] = e.what();
    } catch (...) {
        out["error"] = "unknown engine exception";
    }
    try { return cs::emit(out.dump(), out_json, out_len) == 0 && out["ok"].get<bool>() ? 0 : 1; } catch (...) { return 1; }
}

EMSCRIPTEN_KEEPALIVE void cs_free(void* p) { std::free(p); }

} // extern "C"


// =============================================================================
// Engine tools (cs_tool): painting sessions (TriangleSelector), model cut with
// connectors (Cut + GLGizmoCut3D::apply_cut_connectors), variable layer height
// profiles. One entry point, dispatched by "op":
//   job  = { op, args, meshes: [{ vertexOffset, vertexCount, indexOffset, triangleCount }] }
//   blob = mesh data (float32 xyz + uint32 triangles) the mesh entries point into
//   out  = { ok, error, result, meshes: [...] } + a blob with the result meshes.
// Painting sessions live across calls (the web worker keeps one instance for
// tools); every call frees what it allocates.
// =============================================================================
namespace {

struct ToolMesh { std::vector<float> positions; std::vector<uint32_t> indices; };

std::vector<ToolMesh> parse_tool_meshes(const json& j, const uint8_t* blob, int blob_len)
{
    std::vector<ToolMesh> out;
    auto ms = j.find("meshes");
    if (ms == j.end() || !ms->is_array()) return out;
    if (ms->size() > 4096) throw cs::JobError("too many meshes");
    const uint64_t len = uint64_t(blob_len < 0 ? 0 : blob_len);
    for (const json& m : *ms) {
        if (!m.is_object()) throw cs::JobError("mesh entry must be an object");
        const uint64_t voff = cs::get_u64(m, "vertexOffset"), vcnt = cs::get_u64(m, "vertexCount");
        const uint64_t ioff = cs::get_u64(m, "indexOffset"), tcnt = cs::get_u64(m, "triangleCount");
        if (voff % 4 || ioff % 4) throw cs::JobError("mesh offsets must be 4-byte aligned");
        if (vcnt < 3 || tcnt < 1 || vcnt > 0x7fffffffu || tcnt > 0x7fffffffu) throw cs::JobError("bad mesh size");
        if (!cs::checked_range(voff, vcnt, 12, len) || !cs::checked_range(ioff, tcnt, 12, len)) throw cs::JobError("mesh range out of bounds");
        ToolMesh t;
        t.positions.resize(vcnt * 3);
        std::memcpy(t.positions.data(), blob + voff, vcnt * 12);
        for (float f : t.positions) if (!std::isfinite(f)) throw cs::JobError("non-finite vertex");
        t.indices.resize(tcnt * 3);
        std::memcpy(t.indices.data(), blob + ioff, tcnt * 12);
        for (uint32_t i : t.indices) if (i >= vcnt) throw cs::JobError("triangle index out of range");
        out.emplace_back(std::move(t));
    }
    return out;
}

// All triangles kept (painting indexes the host's triangles).
indexed_triangle_set to_its(const ToolMesh& m)
{
    indexed_triangle_set its;
    its.vertices.reserve(m.positions.size() / 3);
    for (size_t i = 0; i + 2 < m.positions.size(); i += 3) its.vertices.emplace_back(m.positions[i], m.positions[i + 1], m.positions[i + 2]);
    its.indices.reserve(m.indices.size() / 3);
    for (size_t i = 0; i + 2 < m.indices.size(); i += 3) its.indices.emplace_back(int(m.indices[i]), int(m.indices[i + 1]), int(m.indices[i + 2]));
    return its;
}

Transform3d json_matrix(const json& a, const char* what)
{
    if (!a.is_array() || a.size() != 16) throw cs::JobError(std::string(what) + " must be 16 numbers");
    Transform3d t;
    for (int i = 0; i < 16; ++i) {
        const double v = a[i].get<double>();
        if (!std::isfinite(v)) throw cs::JobError(std::string(what) + " is not finite");
        t.matrix().data()[i] = v; // column-major, like Eigen
    }
    return t;
}

Vec3f json_vec3f(const json& a, const char* what)
{
    if (!a.is_array() || a.size() != 3) throw cs::JobError(std::string(what) + " must be 3 numbers");
    return Vec3f(a[0].get<float>(), a[1].get<float>(), a[2].get<float>());
}

// FacetsAnnotation can only live inside a ModelVolume: a throwaway one to
// (de)serialise paint strings through.
struct ScratchAnnotation {
    Model model;
    ModelVolume* vol = nullptr;
    ScratchAnnotation() { vol = model.add_object()->add_volume(TriangleMesh(its_make_cube(1., 1., 1.))); }
    FacetsAnnotation& fa() { return vol->supported_facets; }
};

struct ToolOut {
    json result = json::object();
    std::vector<indexed_triangle_set> meshes;
};

// Paint strings of one FacetsAnnotation as [[triangle, hex], ...].
json paint_map(const FacetsAnnotation& fa)
{
    json arr = json::array();
    for (const auto& m : fa.get_data().triangles_to_split)
        arr.push_back({m.triangle_idx, fa.get_triangle_as_string(m.triangle_idx)});
    return arr;
}

void load_paint_map(FacetsAnnotation& fa, const json& arr, int n_tris)
{
    if (!arr.is_array()) return;
    fa.reserve(int(arr.size()));
    for (const json& e : arr) {
        if (!e.is_array() || e.size() != 2 || !e[0].is_number_integer() || !e[1].is_string()) continue;
        const long long t = e[0].get<long long>();
        const std::string& hex = e[1].get_ref<const std::string&>();
        if (t < 0 || t >= n_tris || hex.empty() || hex.find_first_not_of("0123456789ABCDEFabcdef") != std::string::npos) continue;
        fa.set_triangle_from_string(int(t), hex);
    }
    fa.shrink_to_fit();
}

// ---- painting sessions ---------------------------------------------------------
struct PaintSession {
    TriangleMesh mesh;
    std::unique_ptr<TriangleSelector> sel;
    int max_state = 2;
};
std::map<int, std::unique_ptr<PaintSession>> g_paint;
int g_paint_next = 1;

PaintSession& session(const json& args)
{
    const int h = args.value("handle", 0);
    auto it = g_paint.find(h);
    if (it == g_paint.end()) throw cs::JobError("unknown painting session");
    return *it->second;
}

void emit_facets(PaintSession& ps, ToolOut& out)
{
    json states = json::array();
    for (int st = 1; st <= ps.max_state; ++st) {
        indexed_triangle_set its = ps.sel->get_facets(EnforcerBlockerType(st));
        if (its.indices.empty()) continue;
        states.push_back(st);
        out.meshes.emplace_back(std::move(its));
    }
    out.result["states"] = states;
}

void op_paint_open(const json& args, std::vector<ToolMesh>& meshes, ToolOut& out)
{
    if (meshes.size() != 1) throw cs::JobError("paint_open needs one mesh");
    if (g_paint.size() > 32) throw cs::JobError("too many painting sessions");
    auto ps = std::make_unique<PaintSession>();
    ps->mesh = TriangleMesh(to_its(meshes[0]));
    ps->sel = std::make_unique<TriangleSelector>(ps->mesh, float(args.value("edgeLimit", 0.6)));
    ps->max_state = std::clamp(args.value("maxState", 2), 1, int(EnforcerBlockerType::ExtruderMax));
    if (auto p = args.find("paint"); p != args.end() && p->is_array() && !p->empty()) {
        ScratchAnnotation sa;
        load_paint_map(sa.fa(), *p, int(ps->mesh.its.indices.size()));
        ps->sel->deserialize(sa.fa().get_data(), true);
    }
    const int h = g_paint_next++;
    emit_facets(*ps, out);
    g_paint[h] = std::move(ps);
    out.result["handle"] = h;
}

void op_paint_apply(const json& args, ToolOut& out)
{
    PaintSession& ps = session(args);
    const std::string kind = args.value("kind", std::string("circle"));
    const int state = std::clamp(args.value("state", 1), 0, ps.max_state);
    const EnforcerBlockerType st = EnforcerBlockerType(state);
    const int facet = args.value("facet", -1);
    const Transform3d trafo = args.contains("trafo") ? json_matrix(args["trafo"], "trafo") : Transform3d::Identity();
    Transform3d trafo_nt = trafo;
    trafo_nt.translation() = Vec3d::Zero();
    TriangleSelector::ClippingPlane clp;
    if (auto c = args.find("clip"); c != args.end() && c->is_array() && c->size() == 4)
        clp = TriangleSelector::ClippingPlane({(*c)[0].get<float>(), (*c)[1].get<float>(), (*c)[2].get<float>(), (*c)[3].get<float>()});
    const float overhang = args.value("overhang", 0.f);
    const int n = int(ps.mesh.its.indices.size());
    if (kind == "clear") {
        ps.sel->reset();
    } else if (kind != "none") {
        if (facet < 0 || facet >= n) throw cs::JobError("facet out of range");
        const Vec3f pt = json_vec3f(args["point"], "point");
        const Vec3f cam = json_vec3f(args["camera"], "camera");
        const float radius = std::max(0.05f, args.value("radius", 2.f));
        if (kind == "circle" || kind == "sphere" || kind == "pointer") {
            const auto type = kind == "circle" ? TriangleSelector::CIRCLE : kind == "sphere" ? TriangleSelector::SPHERE : TriangleSelector::POINTER;
            std::unique_ptr<TriangleSelector::Cursor> cursor;
            if (auto pv = args.find("prev"); pv != args.end() && pv->is_array() && kind != "pointer")
                cursor = TriangleSelector::DoublePointCursor::cursor_factory(json_vec3f(*pv, "prev"), pt, cam, radius, type, trafo, clp);
            else
                cursor = TriangleSelector::SinglePointCursor::cursor_factory(pt, cam, radius, type, trafo, clp);
            ps.sel->select_patch(facet, std::move(cursor), st, trafo_nt, kind != "pointer", overhang);
        } else if (kind == "height") {
            const float z_world = args.value("zWorld", 0.f);
            auto cursor = TriangleSelector::SinglePointCursor::cursor_factory(z_world, cam, radius, trafo, clp);
            ps.sel->select_patch(facet, std::move(cursor), st, trafo_nt, true, 0.f);
        } else if (kind == "fill") {
            ps.sel->seed_fill_select_triangles(pt, facet, trafo_nt, clp, args.value("angle", 30.f), overhang, true);
            ps.sel->seed_fill_apply_on_triangles(st);
        } else if (kind == "bucket") {
            ps.sel->bucket_fill_select_triangles(pt, facet, clp, args.value("angle", 30.f), true, true);
            ps.sel->seed_fill_apply_on_triangles(st);
        } else {
            throw cs::JobError("unknown paint tool '" + kind + "'");
        }
    }
    emit_facets(ps, out);
}

void op_paint_get(const json& args, ToolOut& out)
{
    PaintSession& ps = session(args);
    ScratchAnnotation sa;
    sa.fa().set(*ps.sel);
    out.result["paint"] = paint_map(sa.fa());
}

void op_paint_close(const json& args, ToolOut&)
{
    g_paint.erase(args.value("handle", 0));
}

void op_paint_from_states(const json& args, std::vector<ToolMesh>& meshes, ToolOut& out)
{
    if (meshes.size() != 1) throw cs::JobError("paint_from_states needs one mesh");
    TriangleMesh mesh(to_its(meshes[0]));
    TriangleSelector sel(mesh);
    const int n = int(mesh.its.indices.size());
    if (auto sts = args.find("states"); sts != args.end() && sts->is_array())
        for (const json& e : *sts) {
            if (!e.is_array() || e.size() != 2) continue;
            const int t = e[0].get<int>(), s = e[1].get<int>();
            if (t < 0 || t >= n || s < 0 || s > int(EnforcerBlockerType::ExtruderMax)) continue;
            sel.set_facet(t, EnforcerBlockerType(s));
        }
    ScratchAnnotation sa;
    sa.fa().set(sel);
    out.result["paint"] = paint_map(sa.fa());
}

// ---- variable layer height -----------------------------------------------------
std::unique_ptr<Model> single_object_model(const ToolMesh& m, const json& args)
{
    auto model = std::make_unique<Model>();
    ModelObject* obj = model->add_object();
    double T[16];
    const Transform3d tm = args.contains("transform") ? json_matrix(args["transform"], "transform") : Transform3d::Identity();
    std::memcpy(T, tm.matrix().data(), sizeof T);
    obj->add_volume(bed_mesh(m.positions, m.indices, T, "object"));
    const Vec3d centre = obj->raw_mesh_bounding_box().center();
    obj->center_around_origin(false);
    obj->add_instance()->set_offset(centre);
    obj->ensure_on_bed();
    return model;
}

SlicingParameters tool_slicing_params(const ModelObject& obj, const json& args)
{
    DynamicPrintConfig cfg = DynamicPrintConfig::full_print_config();
    json subs = json::array();
    if (auto c = args.find("config"); c != args.end() && c->is_object()) load_config_json(cfg, *c, subs);
    const float max_z = float(obj.instance_bounding_box(0).max.z());
    return PrintObject::slicing_parameters(cfg, obj, max_z, Vec3d::Ones());
}

void op_layer_profile(const json& args, std::vector<ToolMesh>& meshes, ToolOut& out, bool smooth)
{
    if (meshes.size() != 1) throw cs::JobError("layer profile needs one mesh");
    auto model = single_object_model(meshes[0], args);
    const ModelObject& obj = *model->objects.front();
    const SlicingParameters sp = tool_slicing_params(obj, args);
    std::vector<double> profile;
    if (!smooth) {
        profile = layer_height_profile_adaptive(sp, obj, std::clamp(args.value("quality", 0.5f), 0.f, 1.f));
    } else {
        std::vector<double> in;
        if (auto p = args.find("profile"); p != args.end() && p->is_array()) for (const json& v : *p) in.push_back(v.get<double>());
        if (in.size() < 4 || in.size() % 2) throw cs::JobError("profile must be [z, h] pairs");
        profile = smooth_height_profile(in, sp, HeightProfileSmoothingParams(unsigned(std::clamp(args.value("radius", 5), 1, 10)), args.value("keepMin", false)));
    }
    out.result["profile"] = profile;
}

// ---- cut --------------------------------------------------------------------------
indexed_triangle_set connector_mesh(const CutConnectorAttributes& a, float snap_space, float snap_bulge)
{
    int sectors = 1;
    switch (a.shape) {
    case CutConnectorShape::Triangle: sectors = 3; break;
    case CutConnectorShape::Square: sectors = 4; break;
    case CutConnectorShape::Circle: sectors = 360; break;
    case CutConnectorShape::Hexagon: sectors = 6; break;
    default: break;
    }
    if (a.type == CutConnectorType::Snap) return its_make_snap(1.0, 1.0, snap_space, snap_bulge);
    if (a.style == CutConnectorStyle::Prism) return its_make_cylinder(1.0, 1.0, 2 * PI / sectors);
    if (a.type == CutConnectorType::Plug) return its_make_frustum(1.0, 1.0, 2 * PI / sectors);
    return its_make_frustum_dowel(1.0, 1.0, sectors);
}

// GLGizmoCut3D's update_object_cut_id.
void update_cut_id(CutObjectBase& cut_id, ModelObjectCutAttributes attributes, int dowels_count)
{
    if (!attributes.has(ModelObjectCutAttribute::KeepUpper) || !attributes.has(ModelObjectCutAttribute::KeepLower) ||
        attributes.has(ModelObjectCutAttribute::InvalidateCutInfo))
        return;
    if (cut_id.id().invalid()) cut_id.init();
    int n = -1;
    if (attributes.has(ModelObjectCutAttribute::KeepUpper)) n++;
    if (attributes.has(ModelObjectCutAttribute::KeepLower)) n++;
    if (attributes.has(ModelObjectCutAttribute::CreateDowels)) n += dowels_count;
    if (n > 0) cut_id.increase_check_sum(size_t(n));
}

#if __has_include(<libslic3r/MixedFilament.hpp>)
#define CS_HAS_MIXED_FILAMENTS 1
// FullSpectrum mixed (virtual) filaments: the same regeneration Print::apply
// runs (auto pairs from the physical colours + the user's custom rows), plus
// edits, through the manager's typed definitions.
//   args: { colors:[hex], definitions:"<mixed_filament_definitions>", edits:[...] }
//   edits: {op:"add", a, b, percent} | {op:"add_definition", definition}
//        | {op:"set", stableId, definition} | {op:"remove", stableId} | {op:"percent", stableId, percent}
//   result: { definitions, filaments:[{id, stableId, custom, a, b, percent, color, kind,
//             components:[{id, percent}], distribution, cadence:{a,b}, localZMax,
//             gradient:{enabled, start, end, stops[], solidWidths[]}, offsets[], perimeterModulation,
//             pattern:[[id...]...]}] }
json mixed_definition_json(const MixedFilamentDefinition& d)
{
    json comps = json::array();
    for (const auto& c : d.recipe.blend.components) comps.push_back({{"id", c.filament.id}, {"percent", c.percent}});
    json pattern = json::array();
    if (d.recipe.manual_pattern)
        for (const auto& g : d.recipe.manual_pattern->groups) {
            json grp = json::array();
            for (const auto& r : g) grp.push_back(r.id);
            pattern.push_back(grp);
        }
    const auto& gb = d.behavior.gradient;
    return {
        {"stableId", d.identity.stable_id},
        {"custom", d.source.kind == MixedFilamentSourceKind::Custom},
        {"kind", d.recipe.kind == MixedFilamentRecipeKind::ManualPattern ? "pattern" : "blend"},
        {"components", comps},
        {"distribution", d.behavior.distribution == MixedFilamentDistributionMode::LayerCycle ? "layer_cycle" : "simple"},
        {"cadence", {{"a", d.behavior.layer_cadence.component_a_layers}, {"b", d.behavior.layer_cadence.component_b_layers}}},
        {"localZMax", d.behavior.local_z.max_sublayers},
        {"gradient", {{"enabled", gb.enabled}, {"start", gb.component_a_start}, {"end", gb.component_a_end},
                      {"stops", gb.stop_positions}, {"solidWidths", gb.solid_widths}}},
        {"offsets", d.behavior.surface_bias.component_offsets_mm},
        {"offsetA", d.behavior.surface_bias.component_a_offset_mm},
        {"offsetB", d.behavior.surface_bias.component_b_offset_mm},
        {"perimeterModulation", d.behavior.surface_bias.perimeter_modulation},
        {"pattern", pattern},
        {"color", d.presentation.display_color},
    };
}

// Apply the (partial) JSON onto a definition. Physical ids are clamped to 1..n.
void apply_mixed_definition(MixedFilamentDefinition& d, const json& j, size_t n)
{
    if (!j.is_object()) throw cs::JobError("mixed filament: definition must be an object");
    auto clampi = [](int v, int lo, int hi) { return std::max(lo, std::min(hi, v)); };
    auto clampf = [](float v, float lo, float hi) { return std::isfinite(v) ? std::max(lo, std::min(hi, v)) : lo; };
    if (auto c = j.find("components"); c != j.end() && c->is_array()) {
        std::vector<MixedFilamentWeightedComponent> comps;
        for (const json& e : *c) {
            if (!e.is_object()) continue;
            MixedFilamentWeightedComponent w;
            w.filament.id = unsigned(clampi(e.value("id", 1), 1, int(n)));
            w.percent = clampi(e.value("percent", 0), 0, 100);
            comps.push_back(w);
        }
        if (comps.size() < 2 || comps.size() > n) throw cs::JobError("mixed filament: needs 2.." + std::to_string(n) + " components");
        d.recipe.blend.components = std::move(comps);
    }
    if (auto k = j.find("kind"); k != j.end() && k->is_string())
        d.recipe.kind = *k == "pattern" ? MixedFilamentRecipeKind::ManualPattern : MixedFilamentRecipeKind::WeightedBlend;
    if (auto p = j.find("pattern"); p != j.end() && p->is_array()) {
        MixedFilamentManualPattern mp;
        for (const json& g : *p) {
            if (!g.is_array()) continue;
            std::vector<MixedFilamentPhysicalRef> grp;
            for (const json& id : g) if (id.is_number_integer()) grp.push_back({unsigned(clampi(id.get<int>(), 1, int(n)))});
            if (!grp.empty()) mp.groups.push_back(std::move(grp));
        }
        if (mp.groups.empty()) d.recipe.manual_pattern.reset(); else d.recipe.manual_pattern = std::move(mp);
    }
    if (d.recipe.kind == MixedFilamentRecipeKind::ManualPattern && !d.recipe.manual_pattern)
        throw cs::JobError("mixed filament: a pattern needs at least one filament");
    if (auto v = j.find("distribution"); v != j.end() && v->is_string())
        d.behavior.distribution = *v == "layer_cycle" ? MixedFilamentDistributionMode::LayerCycle : MixedFilamentDistributionMode::Simple;
    if (auto c = j.find("cadence"); c != j.end() && c->is_object()) {
        d.behavior.layer_cadence.component_a_layers = clampi(c->value("a", 1), 1, 100);
        d.behavior.layer_cadence.component_b_layers = clampi(c->value("b", 1), 1, 100);
    }
    if (auto v = j.find("localZMax"); v != j.end() && v->is_number()) d.behavior.local_z.max_sublayers = clampi(v->get<int>(), 0, 64);
    if (auto g = j.find("gradient"); g != j.end() && g->is_object()) {
        auto& gb = d.behavior.gradient;
        gb.enabled = g->value("enabled", gb.enabled);
        gb.component_a_start = clampf(g->value("start", gb.component_a_start), 0.f, 1.f);
        gb.component_a_end = clampf(g->value("end", gb.component_a_end), 0.f, 1.f);
        auto floats = [&](const char* key, std::vector<float>& out, float hi) {
            if (auto a = g->find(key); a != g->end() && a->is_array()) {
                out.clear();
                for (const json& x : *a) if (x.is_number()) out.push_back(clampf(x.get<float>(), 0.f, hi));
            }
        };
        floats("stops", gb.stop_positions, 1.f);
        floats("solidWidths", gb.solid_widths, 1.f);
        std::sort(gb.stop_positions.begin(), gb.stop_positions.end());
    }
    auto& sb = d.behavior.surface_bias;
    if (auto o = j.find("offsets"); o != j.end() && o->is_array()) {
        sb.component_offsets_mm.clear();
        for (const json& x : *o) if (x.is_number()) sb.component_offsets_mm.push_back(clampf(x.get<float>(), -2.f, 2.f));
    }
    if (auto v = j.find("offsetA"); v != j.end() && v->is_number()) sb.component_a_offset_mm = clampf(v->get<float>(), -2.f, 2.f);
    if (auto v = j.find("offsetB"); v != j.end() && v->is_number()) sb.component_b_offset_mm = clampf(v->get<float>(), -2.f, 2.f);
    if (auto v = j.find("perimeterModulation"); v != j.end() && v->is_boolean()) sb.perimeter_modulation = v->get<bool>();
}

void op_mixed_filaments(const json& args, ToolOut& out)
{
    std::vector<std::string> colors;
    for (const json& c : args.value("colors", json::array())) colors.push_back(c.is_string() ? c.get<std::string>() : std::string("#26A69A"));
    if (colors.size() > MixedFilamentManager::kMaxPhysicalFilaments) colors.resize(MixedFilamentManager::kMaxPhysicalFilaments);
    const size_t n = colors.size();
    MixedFilamentManager mgr;
    mgr.clear_custom_entries();
    mgr.auto_generate(colors);
    mgr.load_custom_entries(args.value("definitions", std::string()), colors);
    auto clampi = [](int v, int lo, int hi) { return std::max(lo, std::min(hi, v)); };
    auto index_of = [&](uint64_t id) -> int {
        const auto defs = mgr.mixed_filament_definitions(n);
        for (size_t i = 0; i < defs.size(); ++i) if (defs[i].identity.stable_id == id) return int(i);
        return -1;
    };
    for (const json& e : args.value("edits", json::array())) {
        const std::string op = e.value("op", std::string());
        if (op == "add") {
            const unsigned a = unsigned(e.value("a", 1)), b = unsigned(e.value("b", 2));
            if (a < 1 || b < 1 || a > n || b > n || a == b) throw cs::JobError("mixed filament: components must be two different physical filaments");
            mgr.add_custom_filament(a, b, clampi(e.value("percent", 50), 0, 100), colors);
        } else if (op == "add_definition") {
            MixedFilamentDefinition d;
            d.recipe.blend.components = {{{1}, 50}, {{2}, 50}};
            apply_mixed_definition(d, e.value("definition", json::object()), n);
            mgr.add_custom_filament_definition(std::move(d), colors);
        } else if (op == "set") {
            const int i = index_of(e.value("stableId", uint64_t(0)));
            if (i < 0) throw cs::JobError("mixed filament: unknown id");
            MixedFilamentDefinition d = mgr.mixed_filament_definitions(n)[size_t(i)];
            apply_mixed_definition(d, e.value("definition", json::object()), n);
            mgr.set_mixed_filament_definition(size_t(i), d, colors);
        } else if (op == "remove" || op == "percent") {
            const uint64_t id = e.value("stableId", uint64_t(0));
            auto& rows = mgr.mixed_filaments();
            for (size_t i = 0; i < rows.size(); ++i) {
                if (rows[i].stable_id != id) continue;
                MixedFilamentLegacyRow r = rows[i];
                if (op == "remove") r.deleted = true; else r.mix_b_percent = clampi(e.value("percent", 50), 0, 100);
                mgr.set_mixed_filament_legacy_row(i, r, n, colors);
                break;
            }
        }
    }
    mgr.refresh_display_colors(colors);
    const std::string serialized = mgr.serialize_custom_entries();
    const auto defs = mgr.mixed_filament_definitions(n);
    const auto& rows = mgr.mixed_filament_legacy_rows();
    json list = json::array();
    for (unsigned id = unsigned(n) + 1; id <= unsigned(n + defs.size()); ++id) {
        const int idx = mgr.mixed_index_from_filament_id(id, n);
        if (idx < 0 || size_t(idx) >= defs.size() || defs[size_t(idx)].visibility.tombstoned) continue;
        json f = mixed_definition_json(defs[size_t(idx)]);
        f["id"] = id;
        if (size_t(idx) < rows.size()) {
            const MixedFilamentLegacyRow& r = rows[size_t(idx)];
            f["a"] = r.component_a; f["b"] = r.component_b; f["percent"] = r.mix_b_percent;
            if (!r.display_color.empty()) f["color"] = r.display_color;
        }
        list.push_back(f);
    }
    out.result["definitions"] = serialized;
    out.result["filaments"] = list;
}
#endif

void op_cut(const json& args, std::vector<ToolMesh>& meshes, ToolOut& out)
{
    const json vols = args.value("volumes", json::array());
    if (!vols.is_array() || vols.empty() || vols.size() != meshes.size()) throw cs::JobError("cut: one volume entry per mesh");
    Model model;
    ModelObject* mo = model.add_object();
    mo->name = args.value("name", std::string("Object"));
    for (size_t i = 0; i < vols.size(); ++i) {
        const json& v = vols[i];
        TriangleMesh mesh(to_its(meshes[i]));
        if (mesh.volume() < 0) mesh.flip_triangles();
        ModelVolume* mv = mo->add_volume(std::move(mesh), volume_type(v.value("type", std::string("part"))), false);
        mv->name = v.value("name", std::string("Part"));
        mv->set_transformation(json_matrix(v.at("transform"), "volume transform"));
        std::vector<int> ident(mv->mesh().its.indices.size());
        for (size_t k = 0; k < ident.size(); ++k) ident[k] = int(k);
        if (auto p = v.find("paint"); p != v.end()) apply_paint(mv, *p, ident);
        if (auto c = v.find("config"); c != v.end() && c->is_object() && !c->empty()) {
            DynamicPrintConfig vc; json subs = json::array();
            load_config_json(vc, *c, subs);
            mv->config.assign_config(vc);
        }
    }
    ModelInstance* inst = mo->add_instance();
    if (auto off = args.find("offset"); off != args.end()) {
        const Vec3f o = json_vec3f(*off, "offset");
        inst->set_offset(o.cast<double>());
    }
    const json cut = args.value("cut", json::object());
    const Transform3d cut_matrix = json_matrix(cut.at("matrix"), "cut matrix"); // object space (world minus instance offset)
    Transform3d rotation_m = Transform3d::Identity();
    rotation_m.linear() = cut_matrix.linear();
    const Vec3d normal = (rotation_m * Vec3d::UnitZ()).normalized();

    // Connectors (GLGizmoCut3D::apply_connectors_in_model + apply_cut_connectors).
    int dowels = 0;
    const json cons = cut.value("connectors", json::array());
    const bool groove = cut.value("mode", std::string("plane")) == "groove";
    if (!groove && cons.is_array() && !cons.empty()) {
        mo->cut_id.init();
        size_t id = mo->cut_id.connectors_cnt();
        for (const json& c : cons) {
            CutConnector cc;
            cc.pos = json_vec3f(c.at("pos"), "connector pos").cast<double>();
            cc.rotation_m = rotation_m;
            cc.radius = c.value("radius", 5.f);
            cc.height = c.value("height", 10.f);
            cc.radius_tolerance = c.value("radiusTolerance", 0.f);
            cc.height_tolerance = c.value("heightTolerance", 0.1f);
            cc.z_angle = c.value("zAngle", 0.f);
            const std::string t = c.value("type", std::string("plug")), st = c.value("style", std::string("prism")), sh = c.value("shape", std::string("circle"));
            cc.attribs.type = t == "dowel" ? CutConnectorType::Dowel : t == "snap" ? CutConnectorType::Snap : CutConnectorType::Plug;
            cc.attribs.style = st == "frustum" ? CutConnectorStyle::Frustum : CutConnectorStyle::Prism;
            cc.attribs.shape = sh == "triangle" ? CutConnectorShape::Triangle : sh == "square" ? CutConnectorShape::Square
                             : sh == "hexagon" ? CutConnectorShape::Hexagon : CutConnectorShape::Circle;
            if (cc.attribs.type == CutConnectorType::Dowel) {
                if (cc.attribs.style == CutConnectorStyle::Prism) cc.height *= 2;
                dowels++;
            } else {
                cc.pos += normal * 0.5 * double(cc.height);
            }
            TriangleMesh cm(connector_mesh(cc.attribs, c.value("snapSpace", 0.3f), c.value("snapBulge", 0.15f)));
            ModelVolume* nv = mo->add_volume(std::move(cm), ModelVolumeType::NEGATIVE_VOLUME);
            nv->set_transformation(Geometry::translation_transform(cc.pos) * cc.rotation_m *
                                   Geometry::rotation_transform(-cc.z_angle * Vec3d::UnitZ()) *
                                   Geometry::scale_transform(Vec3d(cc.radius, cc.radius, cc.height)));
            nv->cut_info = { cc.attribs.type, cc.radius_tolerance, cc.height_tolerance };
            nv->name = "Connector-" + std::to_string(++id);
        }
        mo->cut_id.increase_connectors_cnt(cons.size());
    }
    const bool has_connectors = !groove && cons.is_array() && !cons.empty();
    auto flag = [&](const char* k, bool d = false) { return cut.value(k, d); };
    ModelObjectCutAttributes attrs =
        only_if(has_connectors || flag("keepUpper", true), ModelObjectCutAttribute::KeepUpper) |
        only_if(has_connectors || flag("keepLower", true), ModelObjectCutAttribute::KeepLower) |
        only_if(!has_connectors && flag("keepAsParts"), ModelObjectCutAttribute::KeepAsParts) |
        only_if(flag("placeOnCutUpper"), ModelObjectCutAttribute::PlaceOnCutUpper) |
        only_if(flag("placeOnCutLower"), ModelObjectCutAttribute::PlaceOnCutLower) |
        only_if(flag("flipUpper"), ModelObjectCutAttribute::FlipUpper) |
        only_if(flag("flipLower"), ModelObjectCutAttribute::FlipLower) |
        only_if(dowels > 0, ModelObjectCutAttribute::CreateDowels) |
        only_if(!has_connectors && !groove, ModelObjectCutAttribute::InvalidateCutInfo)
#ifndef CS_ORCA_LEGACY_API
        | only_if(flag("keepPaint", true), ModelObjectCutAttribute::KeepPaint)
#endif
        ;
    update_cut_id(mo->cut_id, attrs, dowels);

    Cut cutter(mo, 0, cut_matrix, attrs);
    const ModelObjectPtrs* result = nullptr;
    if (groove) {
        const json g = cut.value("groove", json::object());
        Cut::Groove gr;
        gr.depth = g.value("depth", 5.f); gr.width = g.value("width", 5.f);
        gr.flaps_angle = g.value("flapsAngle", float(PI / 3)); gr.angle = g.value("angle", 0.f);
        gr.depth_tolerance = g.value("depthTolerance", 0.1f); gr.width_tolerance = g.value("widthTolerance", 0.1f);
#ifdef CS_ORCA_LEGACY_API
        result = &cutter.perform_with_groove(gr, rotation_m, flag("keepAsParts"));
#else
        result = &cutter.perform_with_groove(gr, rotation_m, std::max(1, g.value("count", 1)), g.value("gap", 0.f),
                                             g.value("radius", 50.f), flag("keepAsParts"));
#endif
    } else {
        result = &cutter.perform_with_plane();
    }
    json objs = json::array();
    for (const ModelObject* no : *result) {
        json jo;
        jo["name"] = no->name;
        const Transform3d it = no->instances.empty() ? Transform3d::Identity() : no->instances.front()->get_transformation().get_matrix();
        jo["instance"] = std::vector<double>(it.matrix().data(), it.matrix().data() + 16);
        json jv = json::array();
        for (const ModelVolume* v : no->volumes) {
            json e;
            e["name"] = v->name;
            const ModelVolumeType t = v->type();
            e["type"] = t == ModelVolumeType::NEGATIVE_VOLUME ? "negative" : t == ModelVolumeType::PARAMETER_MODIFIER ? "modifier"
                      : t == ModelVolumeType::SUPPORT_BLOCKER ? "support_blocker" : t == ModelVolumeType::SUPPORT_ENFORCER ? "support_enforcer" : "part";
            const Transform3d vt = v->get_matrix();
            e["transform"] = std::vector<double>(vt.matrix().data(), vt.matrix().data() + 16);
            e["connector"] = v->cut_info.is_connector;
            json paint = json::object();
            if (!v->supported_facets.empty()) paint["support"] = paint_map(v->supported_facets);
            if (!v->seam_facets.empty()) paint["seam"] = paint_map(v->seam_facets);
            if (!v->mmu_segmentation_facets.empty()) paint["color"] = paint_map(v->mmu_segmentation_facets);
            if (!v->fuzzy_skin_facets.empty()) paint["fuzzy"] = paint_map(v->fuzzy_skin_facets);
            if (!paint.empty()) e["paint"] = paint;
            e["mesh"] = int(out.meshes.size());
            out.meshes.push_back(v->mesh().its);
            jv.push_back(e);
        }
        jo["volumes"] = jv;
        objs.push_back(jo);
    }
    out.result["objects"] = objs;
}

#ifndef CS_ORCA_LEGACY_API
// Load a 3MF with OrcaSlicer's own project loader (Format/bbs_3mf.cpp) and
// describe what it read: lets hosts check that files they write open in
// desktop Orca the same way. The raw 3MF bytes are the request blob.
void op_inspect_3mf(const uint8_t* blob, int blob_len, ToolOut& out)
{
    if (!blob || blob_len <= 0) throw cs::JobError("inspect_3mf: empty file");
    const std::string path = "/tmp/cs_inspect.3mf";
    {
        FILE* f = std::fopen(path.c_str(), "wb");
        if (!f) throw cs::JobError("inspect_3mf: cannot write temp file");
        std::fwrite(blob, 1, size_t(blob_len), f);
        std::fclose(f);
    }
    DynamicPrintConfig config;
    ConfigSubstitutionContext ctxt(ForwardCompatibilitySubstitutionRule::Enable);
    Model model;
    // The loader gives the model a backup dir (Model::get_backup_path). With
    // one set, ~Model calls remove_backup(), whose _BBS_Backup_Manager singleton
    // starts its worker with boost::thread; the single-threaded shim runs that
    // loop inline and it never returns (timed_wait/wait are no-ops, and nothing
    // can ever push Exit). Drop the dir and clear the path before ~Model, on
    // every exit path, so the manager is never touched.
    struct BackupDirGuard {
        Model& m;
        ~BackupDirGuard() { try { m.remove_backup_path_if_exist(); } catch (...) {} }
    } backup_guard{model};
    PlateDataPtrs plates;
    std::vector<Preset*> presets;
    bool is_bbl = false, is_orca = false;
    Semver version;
    const bool ok = load_bbs_3mf(path.c_str(), &config, &ctxt, &model, &plates, &presets, &is_bbl, &is_orca, &version, nullptr,
                                 LoadStrategy::LoadModel | LoadStrategy::LoadConfig | LoadStrategy::Silence);
    ::unlink(path.c_str());
    if (!ok) throw cs::JobError("OrcaSlicer's 3MF loader rejected the file");
    auto volume_type = [](ModelVolumeType t) {
        return t == ModelVolumeType::NEGATIVE_VOLUME ? "negative" : t == ModelVolumeType::PARAMETER_MODIFIER ? "modifier"
             : t == ModelVolumeType::SUPPORT_BLOCKER ? "support_blocker" : t == ModelVolumeType::SUPPORT_ENFORCER ? "support_enforcer" : "part";
    };
    auto trafo_of = [](const Geometry::Transformation& t) {
        const Vec3d off = t.get_offset(), rot = t.get_rotation(), sc = t.get_scaling_factor();
        return json{{"offset", {off.x(), off.y(), off.z()}}, {"rotation", {rot.x(), rot.y(), rot.z()}}, {"scale", {sc.x(), sc.y(), sc.z()}},
                    {"mirror", t.is_left_handed()}};
    };
    json objs = json::array();
    for (const ModelObject* o : model.objects) {
        json vols = json::array();
        for (const ModelVolume* v : o->volumes) {
            json vc = json::object();
            for (const std::string& k : v->config.keys()) vc[k] = v->config.opt_serialize(k);
            vols.push_back({{"name", v->name}, {"type", volume_type(v->type())}, {"triangles", v->mesh().its.indices.size()},
                            {"paint", {{"support", !v->supported_facets.empty()}, {"seam", !v->seam_facets.empty()},
                                       {"color", !v->mmu_segmentation_facets.empty()}, {"fuzzy", !v->fuzzy_skin_facets.empty()}}},
                            {"transform", trafo_of(v->get_transformation())}, {"config", vc}});
        }
        json oc = json::object();
        for (const std::string& k : o->config.keys()) oc[k] = o->config.opt_serialize(k);
        json inst = json::array();
        for (const ModelInstance* i : o->instances) {
            json ji = trafo_of(i->get_transformation());
            ji["printable"] = i->printable;
            inst.push_back(ji);
        }
        objs.push_back({{"name", o->name}, {"volumes", vols}, {"config", oc}, {"instances", inst},
                        {"layerHeightProfile", o->layer_height_profile.get().size()}, {"layerRanges", o->layer_config_ranges.size()}});
    }
    json pl = json::array();
    for (const PlateData* p : plates) {
        json members = json::array();
        for (const auto& oi : p->objects_and_instances) members.push_back({oi.first, oi.second});
        // obj_inst_map: 3MF object id -> (instance index, identify_id), as model_settings.config lists them.
        json map = json::array();
        for (const auto& kv : p->obj_inst_map) map.push_back({kv.first, kv.second.first, kv.second.second});
        json pcfg = json::object();
        for (const std::string& k : p->config.keys()) pcfg[k] = p->config.opt_serialize(k);
        pl.push_back({{"index", p->plate_index}, {"name", p->plate_name}, {"locked", p->locked}, {"instances", members},
                      {"objInstMap", map}, {"config", pcfg}, {"thumbnail", p->thumbnail_file}, {"gcode", p->gcode_file}});
    }
    for (PlateData* p : plates) delete p;
    for (Preset* p : presets) delete p;
    out.result["isOrca"] = is_bbl || is_orca;
    out.result["isBbl"] = is_bbl;
    out.result["hasOrcaTag"] = is_orca; // desktop Orca: From_Orca only when set, else From_BBS
    json cg = json::object();
    for (const auto& [plate, info] : model.plates_custom_gcodes) {
        json items = json::array();
        for (const CustomGCode::Item& it : info.gcodes)
            items.push_back({{"z", it.print_z}, {"type", int(it.type)}, {"extruder", it.extruder}, {"color", it.color}, {"extra", it.extra}});
        cg[std::to_string(plate)] = {{"mode", int(info.mode)}, {"items", items}};
    }
    out.result["customGcodes"] = cg;
    out.result["objects"] = objs;
    out.result["plates"] = pl;
    auto get = [&](const char* k) { return config.has(k) ? config.opt_serialize(k) : std::string(); };
    out.result["settings"] = {{"printer", get("printer_settings_id")}, {"process", get("print_settings_id")}, {"filaments", get("filament_settings_id")},
                              {"filament_colour", get("filament_colour")}, {"sparse_infill_density", get("sparse_infill_density")},
                              {"layer_height", get("layer_height")}, {"keys", config.keys().size()}};
    json pc = json::object();
    for (const std::string& k : config.keys()) pc[k] = config.opt_serialize(k);
    out.result["config"] = pc;
    out.result["version"] = version.to_string();
    out.result["substitutions"] = ctxt.substitutions.size();
}
#endif

#ifndef CS_ORCA_LEGACY_API
// -----------------------------------------------------------------------------
// Auto arrange: OrcaSlicer's ArrangeJob (desktop "Arrange all objects" A /
// "Arrange objects on current plate" Shift+A) with the GUI parts ported:
// prepare_all / prepare_partplate (+ prepare_selected, which Orca has but
// doesn't wire to a button), prepare_wipe_tower, check_unprintable, process()
// and the PartPlateList bed-index bookkeeping of finalize().
//
// Request (cs_tool op "arrange"): the slice-job fields `config` (the global
// config) and `objects` (mesh-local meshes + transform, parts, per-object
// config, paint) at the top level, plus
//   args: { mode: "all" | "plate" | "selection", currentPlate,
//           plates: [{ locked, seq: "" | "by layer" | "by object", toolChanges: [extruder, …] }],
//           items: [{ plate: -1 | index, printable, selected }]   (parallel to objects),
//           settings: { distance, enableRotation, alignToYAxis, allowMultiMaterials, avoidCaliRegion },
//           bbl }                                                  (BBL vendor printer)
// Object transforms are plate-local (plate grid offset removed) for objects
// on a plate. Result:
//   { items: [{ plate, x, y, dRot, moved }]  (x, y: the mesh-local origin, plate-local; dRot: about world Z),
//     plates: plate count after arranging, unplaced: [names], warnings: [text] }
// -----------------------------------------------------------------------------
namespace arr = Slic3r::arrangement;
constexpr int ARRANGE_MAX_PLATES = 36; // PartPlateList::MAX_PLATES_COUNT / MAX_NUM_PLATES

int cfg_int(const DynamicPrintConfig& c, const char* k, int dflt = 0)
{
    const ConfigOption* o = c.option(k);
    return o ? o->getInt() : dflt;
}

// PartPlate::get_extruders(true) over the objects of one plate.
std::vector<int> plate_extruders(const std::vector<const ModelObject*>& mos, const DynamicPrintConfig& glb, const std::vector<int>& tool_changes)
{
    std::vector<int> out;
    const int glb_support_intf_extr = cfg_int(glb, "support_interface_filament");
    const int glb_support_extr = cfg_int(glb, "support_filament");
    int glb_outer_wall_extr = cfg_int(glb, "outer_wall_filament_id");
    int glb_inner_wall_extr = cfg_int(glb, "inner_wall_filament_id");
    if (glb_outer_wall_extr == 0) glb_outer_wall_extr = glb_inner_wall_extr;
    if (glb_inner_wall_extr == 0) glb_inner_wall_extr = glb_outer_wall_extr;
    const int glb_sparse_infill_extr = cfg_int(glb, "sparse_infill_filament_id");
    const int glb_internal_solid_extr = cfg_int(glb, "internal_solid_filament_id");
    int glb_top_surface_extr = cfg_int(glb, "top_surface_filament_id");
    int glb_bottom_surface_extr = cfg_int(glb, "bottom_surface_filament_id");
    if (glb_top_surface_extr == 0) glb_top_surface_extr = glb_internal_solid_extr;
    if (glb_bottom_surface_extr == 0) glb_bottom_surface_extr = glb_internal_solid_extr;
    bool glb_support = glb.has("enable_support") && glb.opt_bool("enable_support");
    glb_support |= cfg_int(glb, "raft_layers") > 0;
    auto obj_int = [](const ModelObject* mo, const char* k) { const ConfigOption* o = mo->config.option(k); return o ? o->getInt() : 0; };

    for (const ModelObject* mo : mos) {
        for (const ModelVolume* mv : mo->volumes) {
            std::vector<int> ve = mv->get_extruders();
            out.insert(out.end(), ve.begin(), ve.end());
        }
        for (const auto& range : mo->layer_config_ranges)
            if (range.second.has("extruder"))
                if (int id = range.second.option("extruder")->getInt(); id > 0) out.push_back(id);

        bool obj_support = false;
        const ConfigOption* so = mo->config.option("enable_support");
        const ConfigOption* ro = mo->config.option("raft_layers");
        if (so || ro) {
            if (so) obj_support = so->getBool();
            if (ro) obj_support |= ro->getInt() > 0;
        } else obj_support = glb_support;
        if (obj_support) {
            if (int e = obj_int(mo, "support_interface_filament"); e != 0) out.push_back(e);
            else if (glb_support_intf_extr != 0) out.push_back(glb_support_intf_extr);
            if (int e = obj_int(mo, "support_filament"); e != 0) out.push_back(e);
            else if (glb_support_extr != 0) out.push_back(glb_support_extr);
        }
        int outer = obj_int(mo, "outer_wall_filament_id");
        if (outer == 0) outer = obj_int(mo, "inner_wall_filament_id");
        if (outer != 0) out.push_back(outer); else if (glb_outer_wall_extr != 0) out.push_back(glb_outer_wall_extr);
        int inner = obj_int(mo, "inner_wall_filament_id");
        if (inner == 0) inner = obj_int(mo, "outer_wall_filament_id");
        if (inner != 0) out.push_back(inner); else if (glb_inner_wall_extr != 0) out.push_back(glb_inner_wall_extr);
        if (int e = obj_int(mo, "sparse_infill_filament_id"); e != 0) out.push_back(e);
        else if (glb_sparse_infill_extr != 0) out.push_back(glb_sparse_infill_extr);
        const int solid = obj_int(mo, "internal_solid_filament_id");
        if (solid != 0) out.push_back(solid); else if (glb_internal_solid_extr != 0) out.push_back(glb_internal_solid_extr);
        int top = obj_int(mo, "top_surface_filament_id");
        if (top == 0) top = solid;
        if (top != 0) out.push_back(top); else if (glb_top_surface_extr != 0) out.push_back(glb_top_surface_extr);
        int bottom = obj_int(mo, "bottom_surface_filament_id");
        if (bottom == 0) bottom = solid;
        if (bottom != 0) out.push_back(bottom); else if (glb_bottom_surface_extr != 0) out.push_back(glb_bottom_surface_extr);
    }
    const int nums_extruders = glb.has("filament_colour") ? int(glb.option<ConfigOptionStrings>("filament_colour")->values.size()) : 0;
    for (int e : tool_changes) if (e <= nums_extruders) out.push_back(e);
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

// PartPlate::estimate_wipe_tower_size.
Vec3d estimate_wipe_tower_size(const DynamicPrintConfig& config, double w, double wipe_volume, int extruder_count, int plate_extruder_size,
                               double max_height, bool enable_wrapping_detection)
{
    Vec3d size = Vec3d::Zero();
    double layer_height = 0.08;
    if (const ConfigOption* o = config.option("layer_height")) layer_height = o->getFloat();
    if (plate_extruder_size == 0) return size;
    size(2) = max_height;
    auto timelapse_type = config.option<ConfigOptionEnum<TimelapseType>>("timelapse_type");
    const bool need_wipe_tower = (timelapse_type ? timelapse_type->value == TimelapseType::tlSmooth : false) | enable_wrapping_detection;
    const double extra_spacing = config.option("prime_tower_infill_gap")->getFloat() / 100.;
    auto rib_opt = config.option<ConfigOptionEnum<WipeTowerWallType>>("wipe_tower_wall_type");
    const bool use_rib_wall = rib_opt ? rib_opt->value == WipeTowerWallType::wtwRib : false;
    double rib_width = config.option("wipe_tower_rib_width")->getFloat();
    double filament_change_volume = 0.;
    {
        const auto* lengths = config.option<ConfigOptionFloats>("filament_change_length");
        const double length = lengths && !lengths->values.empty() ? *std::max_element(lengths->values.begin(), lengths->values.end()) : 0;
        const auto* diams = config.option<ConfigOptionFloats>("filament_diameter");
        const double d = diams && !diams->values.empty() ? *std::max_element(diams->values.begin(), diams->values.end()) : 1.75;
        filament_change_volume = length * PI * d * d / 4.;
    }
    double volume = wipe_volume * (extruder_count == 2 ? plate_extruder_size : (plate_extruder_size - 1));
    if (extruder_count == 2) volume += filament_change_volume * (int) (plate_extruder_size / 2);
    double depth;
    if (use_rib_wall) {
        depth = std::sqrt(volume / layer_height * extra_spacing);
        if (need_wipe_tower || plate_extruder_size > 1) {
            const float min_depth = WipeTower::get_limit_depth_by_height(max_height);
            const double volume_depth = depth;
            depth = std::max((double) min_depth, depth);
            rib_width = std::min(rib_width, depth / 2);
            depth = rib_width / std::sqrt(2) + std::max(depth + config.opt_float("wipe_tower_extra_rib_length"), volume_depth);
            size(0) = size(1) = depth;
        }
    } else {
        depth = volume / (layer_height * w) * extra_spacing;
        if (need_wipe_tower || depth > EPSILON) {
            const float min_depth = WipeTower::get_limit_depth_by_height(max_height);
            depth = std::max((double) min_depth, depth);
        }
        size(0) = w;
        size(1) = depth;
    }
    return size;
}

// The vertices of a mesh that decide everything arrange reads from it: the
// ones on the convex hull of its XY projection under `W` (mesh → world: the
// arrange outline; a rotation about Z keeps that set), the extremes along the
// object's own axes under `L` (mesh → object: raw bounding box, the object's
// centre) and the extremes in world Z (its height). Orca
// builds the same outline from this subset as from the whole mesh (the 3D
// hull's projection is the projected vertices' hull) without Orca's
// connectivity stats and qhull over every vertex. Returns a fan "mesh" over
// the subset, or the input when it is already small.
void arrange_support_mesh(const std::vector<float>& pos, const std::vector<uint32_t>& idx, const Transform3d& L, const Transform3d& W,
                          std::vector<float>& out_pos, std::vector<uint32_t>& out_idx)
{
    const size_t n = pos.size() / 3;
    if (n <= 64) { out_pos = pos; out_idx = idx; return; }
    std::vector<char> keep(n, 0);
    std::array<size_t, 8> ext{};
    std::array<double, 8> val{ DBL_MAX, -DBL_MAX, DBL_MAX, -DBL_MAX, DBL_MAX, -DBL_MAX, DBL_MAX, -DBL_MAX };
    std::vector<std::pair<Vec2d, size_t>> pts;
    pts.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        const Vec3d p(pos[i * 3], pos[i * 3 + 1], pos[i * 3 + 2]);
        const Vec3d v = L * p, w = W * p;
        pts.emplace_back(Vec2d(w.x(), w.y()), i);
        const double c[8] = { v.x(), v.x(), v.y(), v.y(), v.z(), v.z(), w.z(), w.z() };
        for (int k = 0; k < 8; ++k)
            if ((k % 2 == 0) ? c[k] < val[k] : c[k] > val[k]) { val[k] = c[k]; ext[k] = i; }
    }
    for (size_t e : ext) keep[e] = 1;
    // Andrew's monotone chain over the distinct projected points. Points within
    // 10 nm of the boundary are kept too: Orca rounds to nanometres and drops
    // collinear points itself, so keeping a superset can't change its outline.
    std::sort(pts.begin(), pts.end(), [](const auto& a, const auto& b) { return a.first.x() < b.first.x() || (a.first.x() == b.first.x() && a.first.y() < b.first.y()); });
    pts.erase(std::unique(pts.begin(), pts.end(), [](const auto& a, const auto& b) { return a.first == b.first; }), pts.end());
    constexpr double TOL = 1e-5; // mm
    // Signed distance of b from the line o→a (left positive).
    auto side = [](const Vec2d& o, const Vec2d& a, const Vec2d& b) {
        const Vec2d d = a - o;
        const double len = d.norm();
        return len > 0 ? (d.x() * (b.y() - o.y()) - d.y() * (b.x() - o.x())) / len : 0.;
    };
    std::vector<size_t> hull(2 * pts.size());
    size_t k = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        while (k >= 2 && side(pts[hull[k - 2]].first, pts[hull[k - 1]].first, pts[i].first) < -TOL) --k;
        hull[k++] = i;
    }
    for (size_t i = pts.size() - 1, t = k + 1; i-- > 0;) {
        while (k >= t && side(pts[hull[k - 2]].first, pts[hull[k - 1]].first, pts[i].first) < -TOL) --k;
        hull[k++] = i;
    }
    for (size_t i = 0; i < k; ++i) keep[pts[hull[i]].second] = 1;
    out_pos.clear();
    for (size_t i = 0; i < n; ++i)
        if (keep[i]) { out_pos.push_back(pos[i * 3]); out_pos.push_back(pos[i * 3 + 1]); out_pos.push_back(pos[i * 3 + 2]); }
    const uint32_t m = uint32_t(out_pos.size() / 3);
    if (m < 3) { out_pos = pos; out_idx = idx; return; }
    out_idx.clear();
    for (uint32_t i = 1; i + 1 < m; ++i) { out_idx.push_back(0); out_idx.push_back(i); out_idx.push_back(i + 1); }
}

bool has_color_paint(const json& paint)
{
    auto it = paint.find("color");
    return paint.is_object() && it != paint.end() && it->is_array() && !it->empty();
}

struct ArrangeItem {
    ModelObject* mo = nullptr;
    int plate = -1;
    bool printable = true;
    bool selected = false;
    Vec3d centre = Vec3d::Zero(); // mesh-local bbox centre (Orca's object origin)
    Transform3d start;            // instance matrix before arranging
};

void op_arrange(const char* job_json, int job_len, const uint8_t* blob, int blob_len, const json& args, ToolOut& out)
{
    auto t_last = std::chrono::steady_clock::now();
    json timing = json::object();
    auto lap = [&](const char* what) {
        const auto now = std::chrono::steady_clock::now();
        timing[what] = std::chrono::duration<double, std::milli>(now - t_last).count();
        t_last = now;
    };
    cs::Job job = cs::parse_job(job_json, job_len, blob, blob_len);
    json subs = json::array();
    DynamicPrintConfig config;
    config.apply(FullPrintConfig::defaults());
    load_config_json(config, job.config, subs);
    config.normalize_fdm();
    lap("config");

    const std::string mode = args.value("mode", std::string("all"));
    if (mode != "all" && mode != "plate" && mode != "selection") throw cs::JobError("arrange: unknown mode '" + mode + "'");
    const json plates_j = args.value("plates", json::array());
    const json items_j = args.value("items", json::array());
    const json settings = args.value("settings", json::object());
    if (!plates_j.is_array() || plates_j.empty() || int(plates_j.size()) > ARRANGE_MAX_PLATES) throw cs::JobError("arrange: bad plate list");
    if (!items_j.is_array() || items_j.size() != job.objects.size()) throw cs::JobError("arrange: items must match objects");
    const int n_plates = int(plates_j.size());
    const int current_plate = std::clamp(args.value("currentPlate", 0), 0, n_plates - 1);
    std::vector<bool> plate_locked(n_plates);
    std::vector<std::vector<int>> plate_tool_changes(n_plates);
    std::vector<std::string> plate_seq(n_plates);
    for (int i = 0; i < n_plates; ++i) {
        const json& p = plates_j[i];
        plate_locked[i] = p.value("locked", false);
        plate_seq[i] = p.value("seq", std::string());
        if (auto tc = p.find("toolChanges"); tc != p.end() && tc->is_array())
            for (const json& e : *tc) if (e.is_number_integer()) plate_tool_changes[i].push_back(e.get<int>());
    }
    const bool global_by_object = config.has("print_sequence") && config.opt_enum<PrintSequence>("print_sequence") == PrintSequence::ByObject;
    // PartPlate::get_real_print_seq: the plate's own sequence, else the global one.
    auto plate_by_object = [&](int i, bool* same_as_global) {
        const std::string& s = plate_seq[i];
        const bool by_obj = s.empty() ? global_by_object : s == "by object";
        if (same_as_global) *same_as_global = s.empty() || by_obj == global_by_object;
        return by_obj;
    };

    // Model: one object + instance per host object, volumes centred on the
    // mesh bounding box (as Orca loads them), the instance keeping the host's
    // rotation / scale / mirror so arrange sees the same Euler Z.
    Model model;
    std::vector<ArrangeItem> items(job.objects.size());
    for (size_t k = 0; k < job.objects.size(); ++k) {
        const cs::MeshInput& m = job.objects[k];
        const json& ij = items_j[k];
        ArrangeItem& it = items[k];
        it.plate = ij.value("plate", -1);
        if (it.plate >= n_plates) it.plate = -1;
        it.printable = ij.value("printable", true);
        it.selected = ij.value("selected", false);
        Transform3d T = Transform3d::Identity();
        T.matrix() = Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(m.transform);
        const Transform3d Tinv = T.inverse();
        static const double I[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
        ModelObject* obj = model.add_object();
        obj->name = m.name;
        std::vector<int> tri_map;
        std::vector<float> sp; std::vector<uint32_t> si;
        // Colour paint decides the volume's filaments: those keep the whole mesh (paint is per triangle).
        const bool main_painted = has_color_paint(m.paint);
        if (!main_painted) arrange_support_mesh(m.positions, m.indices, Transform3d::Identity(), T, sp, si);
        ModelVolume* vol = obj->add_volume(main_painted ? bed_mesh(m.positions, m.indices, I, m.name, &tri_map) : bed_mesh(sp, si, I, m.name));
        vol->name = m.name;
        if (main_painted) apply_paint(vol, m.paint, tri_map);
        for (const cs::VolumeInput& p : m.parts) {
            Transform3d Tp = Transform3d::Identity();
            Tp.matrix() = Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(p.transform);
            const Eigen::Matrix<double, 4, 4, Eigen::ColMajor> local = (Tinv * Tp).matrix();
            std::vector<int> ptri_map;
            const bool painted = has_color_paint(p.paint);
            ModelVolume* pv;
            if (painted) pv = obj->add_volume(bed_mesh(p.positions, p.indices, local.data(), p.name, &ptri_map), volume_type(p.type));
            else {
                arrange_support_mesh(p.positions, p.indices, Tinv * Tp, Tp, sp, si);
                pv = obj->add_volume(bed_mesh(sp, si, local.data(), p.name), volume_type(p.type));
            }
            pv->name = p.name;
            if (painted) apply_paint(pv, p.paint, ptri_map);
            if (!p.config.empty()) {
                DynamicPrintConfig vc;
                load_config_json(vc, p.config, subs);
                pv->config.assign_config(vc);
            }
        }
        if (!m.config.empty()) {
            DynamicPrintConfig oc;
            load_config_json(oc, m.config, subs);
            obj->config.assign_config(oc);
        }
        it.centre = obj->raw_mesh_bounding_box().center();
        obj->center_around_origin(false);
        ModelInstance* inst = obj->add_instance();
        inst->printable = it.printable;
        it.start = T * Eigen::Translation3d(it.centre);
        inst->set_transformation(Geometry::Transformation(it.start));
        it.mo = obj;
    }
    lap("model");

    // ---- init_arrange_params ----
    const int filament_count = int(std::max<size_t>(1, vector_size(config, "filament_diameter")));
    Model::setExtruderParams(config, filament_count);
    {
        PrintConfig pc;
        pc.apply(config, true);
        Model::setPrintSpeedTable(config, pc);
    }
    // GLCanvas3D::get_arrange_settings: the "seq print" set when printing by object.
    const bool settings_seq = global_by_object;
    arr::ArrangeParams params;
    {
        Print print;
        Model plate_model;
        for (const ArrangeItem& it : items)
            if (it.plate == current_plate && it.printable) plate_model.add_object(*it.mo);
        float skirt_offset = 0;
        try {
            print.apply(plate_model, config);
            skirt_offset = std::get<0>(print.object_skirt_offset());
        } catch (const std::exception&) {}
        const PrintConfig& pc = print.config();
        params.clearance_height_to_rod = pc.extruder_clearance_height_to_rod.value;
        params.clearance_height_to_lid = pc.extruder_clearance_height_to_lid.value;
        params.clearance_radius = pc.extruder_clearance_radius.value + skirt_offset * 2;
        params.object_skirt_offset = skirt_offset;
        params.printable_height = pc.printable_height.value;
        params.nozzle_height = pc.nozzle_height.value;
        params.align_center = pc.best_object_pos.value;
    }
    params.allow_rotations = settings.value("enableRotation", false);
    params.allow_multi_materials_on_same_plate = settings.value("allowMultiMaterials", true);
    // GLCanvas3D::_render_arrange_menu: only offered (else off) for BBL printers that scan the first layer.
    const bool bbl = args.value("bbl", false);
    const bool scan_first_layer = config.has("scan_first_layer") && config.opt_bool("scan_first_layer");
    params.avoid_extrusion_cali_region = bbl && scan_first_layer && settings.value("avoidCaliRegion", true);
    params.is_seq_print = settings_seq;
    params.min_obj_distance = scaled(std::max(0.0, settings.value("distance", 0.0)));
    // Align to Y axis is disabled (and cleared) while auto rotation is on.
    params.align_to_y_axis = !params.allow_rotations && settings.value("alignToYAxis", false);
    if (mode == "plate") {
        bool same = true;
        params.is_seq_print = plate_by_object(current_plate, &same);
        if (!same) params.min_obj_distance = 0;
    }
    if (params.is_seq_print) {
        params.bed_shrink_x = BED_SHRINK_SEQ_PRINT;
        params.bed_shrink_y = BED_SHRINK_SEQ_PRINT;
    }
    lap("params");

    // ---- prepare ----
    const Points bed_shape = get_bed_shape(config);
    const BoundingBox bed_bb(bed_shape);
    const double plate_width = unscaled(bed_bb.size().x()), plate_depth = unscaled(bed_bb.size().y());
    arr::ArrangePolygons selected, unselected, unprintable, locked;
    std::vector<int> sel_idx, unprintable_idx, locked_idx; // item index per polygon
    auto instance_poly = [&](int k) {
        arr::ArrangePolygon ap = get_instance_arrange_poly(items[k].mo->instances.front(), config);
        ap.setter = nullptr;
        return ap;
    };
    auto push = [](arr::ArrangePolygons& cont, std::vector<int>* idx, arr::ArrangePolygon&& ap, int k) {
        ap.itemid = int(cont.size());
        cont.emplace_back(std::move(ap));
        if (idx) idx->push_back(k);
    };
    std::vector<int> unselected_idx;
    // PartPlateList::preprocess_arrange_polygon: plate-local coordinates are
    // already what the host sent, so only the bed indices are set here.
    auto preprocess = [&](int k, arr::ArrangePolygon& ap, bool sel) {
        const int p = items[k].plate;
        if (p >= 0) {
            int locked_before = 0;
            for (int i = 0; i < p; ++i) locked_before += plate_locked[i] ? 1 : 0;
            if (plate_locked[p]) { ap.bed_idx = p; return true; }
            if (!sel) ap.bed_idx = p - locked_before;
            return false;
        }
        if (!sel) ap.bed_idx = ARRANGE_MAX_PLATES;
        return false;
    };
    bool selected_is_locked = false;
    if (mode == "all") {
        for (int i = 0; i < n_plates; ++i) {
            bool same = true;
            plate_by_object(i, &same);
            if (!plate_locked[i] && !same) plate_locked[i] = true; // ArrangeJob locks them for the run
        }
        for (int k = 0; k < int(items.size()); ++k) {
            arr::ArrangePolygon ap = instance_poly(k);
            if (preprocess(k, ap, true)) { push(locked, &locked_idx, std::move(ap), k); selected_is_locked = true; }
            else if (items[k].printable) push(selected, &sel_idx, std::move(ap), k);
            else push(unprintable, &unprintable_idx, std::move(ap), k);
        }
        if (selected.empty())
            out.result["warnings"].push_back(selected_is_locked ? "All the selected objects are on a locked plate.\nCannot auto-arrange these objects."
                                                                : "No arrangeable objects are selected.");
    } else if (mode == "selection") {
        for (int k = 0; k < int(items.size()); ++k) {
            arr::ArrangePolygon ap = instance_poly(k);
            if (preprocess(k, ap, items[k].selected)) { push(locked, &locked_idx, std::move(ap), k); if (items[k].selected) selected_is_locked = true; }
            else if (!items[k].printable) push(unprintable, &unprintable_idx, std::move(ap), k);
            else if (items[k].selected) push(selected, &sel_idx, std::move(ap), k);
            else push(unselected, &unselected_idx, std::move(ap), k);
        }
        if (selected.empty()) {
            if (!selected_is_locked) { selected.swap(unselected); sel_idx.swap(unselected_idx); }
            else out.result["warnings"].push_back("All the selected objects are on a locked plate.\nCannot auto-arrange these objects.");
        }
    } else {
        if (plate_locked[current_plate]) {
            out.result["warnings"].push_back("This plate is locked.\nCannot auto-arrange on this plate.");
        } else {
            for (int k = 0; k < int(items.size()); ++k) {
                arr::ArrangePolygon ap = instance_poly(k);
                const bool in_plate = items[k].plate == current_plate;
                // preprocess_arrange_polygon_other_locked: everything off this plate stays put.
                if (!in_plate) { ap.bed_idx = items[k].plate >= 0 ? items[k].plate : ARRANGE_MAX_PLATES; push(locked, &locked_idx, std::move(ap), k); }
                else if (items[k].printable) push(selected, &sel_idx, std::move(ap), k);
                else push(unprintable, &unprintable_idx, std::move(ap), k);
            }
        }
    }

    lap("polygons");
    // ---- prepare_wipe_tower ----
    const bool enable_wrapping = config.has("enable_wrapping_detection") && config.opt_bool("enable_wrapping_detection");
    const bool only_on_partplate = mode == "plate";
    auto plate_objects = [&](int p, bool printable_only) {
        std::vector<const ModelObject*> mos;
        for (const ArrangeItem& it : items) if (it.plate == p && (!printable_only || it.printable)) mos.push_back(it.mo);
        return mos;
    };
    // Per-plate facts, computed once (the wipe tower loop visits up to 36 beds).
    std::map<int, double> height_cache;
    auto plate_max_height = [&](int p) {
        if (auto it = height_cache.find(p); it != height_cache.end()) return it->second;
        double h = 0;
        for (const ModelObject* mo : plate_objects(p, false)) h = std::max(h, mo->bounding_box_exact().size().z());
        return height_cache[p] = h;
    };
    std::map<int, std::vector<int>> extruder_cache;
    auto plate_extruders_of = [&](int p) -> const std::vector<int>& {
        if (auto it = extruder_cache.find(p); it != extruder_cache.end()) return it->second;
        return extruder_cache[p] = plate_extruders(plate_objects(p, false), config, plate_tool_changes[p]);
    };
    const int nozzle_nums = int(std::max<size_t>(1, vector_size(config, "nozzle_diameter")));
    const float tower_w = float(config.opt_float("prime_tower_width"));
    const float prime_volume = float(config.opt_float("prime_volume"));
    const float tower_brim = float(config.opt_float("prime_tower_brim_width"));
    auto tower_xy = [&](int p) {
        const auto* xs = config.option<ConfigOptionFloats>("wipe_tower_x");
        const auto* ys = config.option<ConfigOptionFloats>("wipe_tower_y");
        return Vec2d(xs && !xs->values.empty() ? xs->get_at(p) : 15., ys && !ys->values.empty() ? ys->get_at(p) : 220.);
    };
    // PartPlate::estimate_wipe_tower_polygon (the plate itself, `p_valid`, supplies the objects).
    auto estimate_tower = [&](int bedid, int p_valid, int plate_extruder_size) {
        if (plate_extruder_size == 0) plate_extruder_size = int(plate_extruders_of(p_valid).size());
        const Vec3d sz = estimate_wipe_tower_size(config, tower_w, prime_volume, nozzle_nums, plate_extruder_size, plate_max_height(p_valid), enable_wrapping);
        const float depth = float(sz(1));
        const float margin = float(WIPE_TOWER_MARGIN) + tower_brim;
        float brim = tower_brim;
        if (brim < 0) brim = WipeTower::get_auto_brim_by_height(float(sz.z()));
        Vec2d xy = tower_xy(bedid);
        float x = std::clamp(float(xy.x()), margin, std::max(margin, float(plate_width) - tower_w - margin - brim));
        float y = std::clamp(float(xy.y()), margin, std::max(margin, float(plate_depth) - depth - margin - brim));
        arr::ArrangePolygon ap;
        ap.poly.contour = Polygon({{scaled(x - brim), scaled(y - brim)}, {scaled(x + tower_w + brim), scaled(y - brim)},
                                   {scaled(x + tower_w + brim), scaled(y + depth + brim)}, {scaled(x - brim), scaled(y + depth + brim)}});
        ap.bed_idx = bedid;
        ap.name = "WipeTower";
        ap.is_virt_object = true;
        ap.is_wipe_tower = true;
        return ap;
    };
    // GLCanvas3D::reload_scene: the prepare-view wipe tower a plate shows (Orca's get_wipe_tower()).
    const bool smooth_timelapse = config.has("timelapse_type") && config.opt_enum<TimelapseType>("timelapse_type") == TimelapseType::tlSmooth;
    const bool enable_prime_tower = config.has("enable_prime_tower") && config.opt_bool("enable_prime_tower");
    const int filaments_count = int(vector_size(config, "filament_colour"));
    auto shown_tower = [&](int p, arr::ArrangePolygon& ap) {
        const bool need = smooth_timelapse || enable_wrapping;
        if (!enable_prime_tower || !(need || filaments_count > 1)) return false;
        if (plate_by_object(p, nullptr) && plate_objects(p, true).size() != 1) return false;
        const std::vector<int>& pe = plate_extruders_of(p);
        if (!need && pe.size() < 2) return false;
        if (plate_objects(p, false).empty()) return false;
        const Vec3d sz = estimate_wipe_tower_size(config, tower_w, prime_volume, nozzle_nums, int(pe.size()), plate_max_height(p), enable_wrapping);
        const Vec2d xy = tower_xy(p);
        float brim = tower_brim;
        if (brim < 0) brim = WipeTower::get_auto_brim_by_height(float(sz.z()));
        BoundingBoxf bb(Vec2d(xy.x(), xy.y()), Vec2d(xy.x() + sz.x(), xy.y() + sz.y()));
        bb.offset(brim);
        bb.offset(brim); // get_wipe_tower_info offsets by the brim twice
        ap = arr::ArrangePolygon{};
        ap.poly.contour = Polygon({{scaled(bb.min.x()), scaled(bb.min.y())}, {scaled(bb.max.x()), scaled(bb.min.y())},
                                   {scaled(bb.max.x()), scaled(bb.max.y())}, {scaled(bb.min.x()), scaled(bb.max.y())}});
        ap.name = "WipeTower";
        ap.is_virt_object = true;
        ap.is_wipe_tower = true;
        ++ap.priority;
        ap.bed_idx = 0;
        return true;
    };
    auto prepare_wipe_tower = [&]() {
        if (!enable_prime_tower || params.is_seq_print) return;
        bool need_wipe_tower = smooth_timelapse;
        for (const auto& item : selected) {
            std::set<int> e(item.extrude_ids.begin(), item.extrude_ids.end());
            if (e.size() > 1) { need_wipe_tower = true; break; }
        }
        if (params.allow_multi_materials_on_same_plate) {
            std::map<int, std::set<int>> bed_temp_to_extruders;
            for (const auto& item : selected) for (int id : item.extrude_ids) bed_temp_to_extruders[item.bed_temp].insert(id);
            for (const auto& be : bed_temp_to_extruders) if (be.second.size() > 1) { need_wipe_tower = true; break; }
        }
        std::set<int> extruder_ids;
        if (!only_on_partplate) {
            for (int p = 0; p < n_plates; ++p) {
                const std::vector<int>& pe = plate_extruders_of(p);
                extruder_ids.insert(pe.begin(), pe.end());
            }
        }
        int bedid_unlocked = 0;
        for (int bedid = 0; bedid < ARRANGE_MAX_PLATES; ++bedid) {
            const int p_valid = std::min(bedid, n_plates - 1);
            if (bedid < n_plates && plate_locked[p_valid]) continue;
            arr::ArrangePolygon ap;
            if (bedid < n_plates && shown_tower(bedid, ap)) {
                ap.bed_idx = bedid_unlocked;
                unselected.emplace_back(ap);
            } else if (need_wipe_tower) {
                if (only_on_partplate) {
                    const std::vector<int>& pe = plate_extruders_of(p_valid);
                    extruder_ids = std::set<int>(pe.begin(), pe.end());
                }
                ap = estimate_tower(bedid, p_valid, int(extruder_ids.size()));
                ap.bed_idx = bedid_unlocked;
                unselected.emplace_back(ap);
            }
            ++bedid_unlocked;
        }
    };
    if (mode != "plate") prepare_wipe_tower();
    else if (!plate_locked[current_plate]) {
        arr::ArrangePolygon ap;
        if (shown_tower(current_plate, ap)) unselected.emplace_back(ap);
    }

    // PartPlateList::preprocess_exclude_areas: wrapping detection + bed exclude areas, one per plate.
    auto preprocess_exclude_areas = [&](arr::ArrangePolygons& cont, int num_plates, float inflation) {
        auto add = [&](const Polygon& poly, const std::string& name) {
            for (int j = 0; j < num_plates; ++j) {
                arr::ArrangePolygon ret;
                ret.poly.contour = poly;
                ret.is_virt_object = true;
                ret.bed_idx = j;
                ret.height = 1;
                ret.name = name;
                ret.inflation = coord_t(inflation);
                cont.emplace_back(ret);
            }
        };
        if (enable_wrapping) {
            if (const auto* wa = config.option<ConfigOptionPoints>("wrapping_exclude_area"); wa && !wa->values.empty()) {
                Polygon ap;
                for (const Vec2d& p : wa->values) ap.append({scale_(p(0)), scale_(p(1))});
                add(ap, "WrappingRegion");
            }
        }
        if (const auto* ea = config.option<ConfigOptionPoints>("bed_exclude_area"); ea && ea->values.size() >= 3) {
            // PartPlate::m_exclude_bounding_box: the exclude area's bounding box.
            BoundingBoxf bb;
            for (const Vec2d& p : ea->values) bb.merge(p);
            add(Polygon({{scaled(bb.min.x()), scaled(bb.min.y())}, {scaled(bb.max.x()), scaled(bb.min.y())},
                         {scaled(bb.max.x()), scaled(bb.max.y())}, {scaled(bb.min.x()), scaled(bb.max.y())}}), "ExcludedRegion0");
        }
    };
    preprocess_exclude_areas(unselected, mode == "plate" ? current_plate + 1 : ARRANGE_MAX_PLATES, 0.f);
    lap("wipeTower");

    // ---- check_unprintable ----
    for (size_t i = 0; i < selected.size();) {
        if (selected[i].poly.area() < 0.001 || selected[i].height > params.printable_height) {
            if (selected[i].poly.area() < 0.001) out.result["warnings"].push_back("Object " + selected[i].name + " has zero size and can't be arranged.");
            unprintable.push_back(selected[i]);
            unprintable_idx.push_back(sel_idx[i]);
            selected.erase(selected.begin() + i);
            sel_idx.erase(sel_idx.begin() + i);
        } else ++i;
    }

    // ---- process ----
    if (bbl && params.avoid_extrusion_cali_region && scan_first_layer) {
        Polygon ap = scaled(BoundingBoxf(Vec2d{18, 0}, Vec2d{240, 15})).polygon();
        for (int j = 0; j < ARRANGE_MAX_PLATES; ++j) {
            arr::ArrangePolygon ret;
            ret.poly.contour = ap;
            ret.is_virt_object = true;
            ret.is_extrusion_cali_object = true;
            ret.bed_idx = j;
            ret.height = 1;
            ret.name = "NonpreferedRegion0";
            unselected.emplace_back(ret);
        }
    }
    arr::update_arrange_params(params, &config, selected);
    arr::update_selected_items_inflation(selected, &config, params);
    arr::update_unselected_items_inflation(unselected, &config, params);
    arr::update_selected_items_axis_align(selected, &config, params);
    const Points bedpts = arr::get_shrink_bedpts(&config, params);
    preprocess_exclude_areas(params.excluded_regions, 1, float(scale_(1)));
    params.stopcondition = [] { return false; };
    params.progressind = [](unsigned, std::string) {};
    lap("arrangeParams");
    if (!selected.empty()) arr::arrange(selected, unselected, bedpts, params);
    lap("nest");

    // ---- finalize ----
    // Bed indices → plates (PartPlateList::postprocess_bed_index_for_selected /
    // _for_current_plate); new plates are appended as needed.
    int plate_count = n_plates;
    json unplaced = json::array();
    int beds = 0;
    for (size_t i = 0; i < selected.size(); ++i) {
        arr::ArrangePolygon& ap = selected[i];
        if (only_on_partplate) {
            if (ap.bed_idx == -1) {}
            else if (ap.bed_idx == 0) ap.bed_idx += current_plate;
            else ap.bed_idx = plate_count;
        } else if (ap.bed_idx != -1) {
            bool found = false;
            for (int p = 0; p < plate_count; ++p) {
                if (plate_locked[p]) ap.bed_idx += 1;
                else if (ap.bed_idx <= p) { found = true; break; }
            }
            if (!found) {
                while (plate_count < ARRANGE_MAX_PLATES) {
                    const int idx = plate_count++;
                    plate_locked.push_back(false);
                    if (ap.bed_idx <= idx) break;
                }
            }
        }
        beds = std::max(ap.bed_idx, beds);
    }
    // (Orca also counts the "not on any plate" marker here, which would send
    // unprintable objects 36 plates away; it is left out.)
    for (const arr::ArrangePolygon& ap : locked) if (ap.bed_idx < ARRANGE_MAX_PLATES) beds = std::max(ap.bed_idx, beds);
    for (int k : unselected_idx) if (items[k].plate >= 0) beds = std::max(items[k].plate, beds);

    json res_items = json::array();
    for (size_t k = 0; k < items.size(); ++k) res_items.push_back({{"moved", false}});
    // postprocess_arrange_polygon + apply(): the new instance matrix → the host's mesh-origin position.
    auto emit = [&](int k, const arr::ArrangePolygon& ap, Vec2d tr, int bed) {
        ArrangeItem& it = items[k];
        Geometry::Transformation t(it.start);
        const double rot0 = t.get_rotation().z();
        ModelInstance* inst = it.mo->instances.front();
        inst->apply_arrange_result(tr, ap.rotation);
        const Transform3d M = inst->get_matrix() * Eigen::Translation3d(-it.centre);
        const Vec3d origin = M.translation();
        res_items[k] = {{"moved", true}, {"plate", bed}, {"x", origin.x()}, {"y", origin.y()}, {"dRot", ap.rotation - rot0}};
    };
    for (size_t i = 0; i < selected.size(); ++i) {
        arr::ArrangePolygon ap = selected[i];
        const int k = sel_idx[i];
        Vec2d tr = ap.translation.cast<double>();
        int bed = ap.bed_idx;
        if (bed == -1) {
            // Doesn't fit any plate: Orca parks it in the top-left corner of the plate after the last.
            bed = plate_count;
            const BoundingBox apbox = get_extents(ap.transformed_poly());
            const Vec2crd s = apbox.size();
            tr = Vec2d(0.5 * s.x(), scaled<double>(plate_depth) - 0.5 * s.y());
            unplaced.push_back(items[k].mo->name);
        }
        emit(k, ap, tr, bed);
    }
    // Unprintable items go to the bed after the last one used (Orca's last virtual bed).
    for (size_t i = 0; i < unprintable.size(); ++i) {
        const arr::ArrangePolygon& ap = unprintable[i];
        emit(unprintable_idx[i], ap, ap.translation.cast<double>(), beds + 1);
    }
    out.result["items"] = res_items;
    out.result["plates"] = plate_count;
    out.result["unplaced"] = unplaced;
    out.result["seqPrint"] = params.is_seq_print;
    // Arrange order (Orca sorts objects by it after arranging; it is the by-object print order).
    std::vector<std::pair<int, int>> by_item;
    for (size_t i = 0; i < selected.size(); ++i) by_item.emplace_back(selected[i].itemid, sel_idx[i]);
    std::stable_sort(by_item.begin(), by_item.end(), [](auto& a, auto& b) { return a.first < b.first; });
    json order = json::array();
    for (auto& [_, k] : by_item) order.push_back(k);
    out.result["order"] = order;
    if (!out.result.contains("warnings")) out.result["warnings"] = json::array();
    lap("finalize");
    out.result["timing"] = timing;
}
#endif

int tool_impl(const char* job_json, int job_len, const uint8_t* blob, int blob_len, std::string& out_json, std::string& out_blob)
{
    ToolOut out;
    json rep = {{"ok", false}, {"error", nullptr}};
    try {
        if (!job_json || job_len <= 0) throw cs::JobError("empty request");
        const json j = json::parse(job_json, job_json + job_len);
        const std::string op = j.value("op", std::string());
        const json args = j.value("args", json::object());
        std::vector<ToolMesh> meshes = op == "inspect_3mf" ? std::vector<ToolMesh>{} : parse_tool_meshes(j, blob, blob_len);
        if (op == "paint_open") op_paint_open(args, meshes, out);
        else if (op == "paint_apply") op_paint_apply(args, out);
        else if (op == "paint_get") op_paint_get(args, out);
        else if (op == "paint_close") op_paint_close(args, out);
        else if (op == "paint_from_states") op_paint_from_states(args, meshes, out);
        else if (op == "layer_profile_adaptive") op_layer_profile(args, meshes, out, false);
        else if (op == "layer_profile_smooth") op_layer_profile(args, meshes, out, true);
        else if (op == "cut") op_cut(args, meshes, out);
#ifndef CS_ORCA_LEGACY_API
        else if (op == "inspect_3mf") op_inspect_3mf(blob, blob_len, out);
        else if (op == "arrange") op_arrange(job_json, job_len, blob, blob_len, args, out);
#endif
#ifdef CS_HAS_MIXED_FILAMENTS
        else if (op == "mixed_filaments") op_mixed_filaments(args, out);
#endif
        else throw cs::JobError("unknown tool '" + op + "'");
        rep["ok"] = true;
    } catch (const std::bad_alloc&) {
        rep["error"] = "out of memory";
    } catch (const std::exception& e) {
        rep["error"] = e.what();
    }
    // Pack result meshes: float32 positions then uint32 indices, 4-aligned.
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
    rep["result"] = out.result;
    rep["meshes"] = mj;
    out_json = rep.dump();
    return rep["ok"].get<bool>() ? 0 : 1;
}

} // namespace

extern "C" {

EMSCRIPTEN_KEEPALIVE
int cs_tool(const char* job_json, int job_len, const uint8_t* blob, int blob_len,
            char** out_json, int* out_json_len, uint8_t** out_blob, int* out_blob_len)
{
    if (out_json) *out_json = nullptr;
    if (out_json_len) *out_json_len = 0;
    if (out_blob) *out_blob = nullptr;
    if (out_blob_len) *out_blob_len = 0;
    std::string js, bl;
    int rc = 1;
    try {
        rc = serial_on_main_thread([&] { return tool_impl(job_json, job_len, blob, blob_len, js, bl); });
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

} // extern "C"
