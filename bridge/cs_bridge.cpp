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
#include "../../wasm-bridge/cs_common.hpp"

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
#if __has_include(<libslic3r/MixedFilament.hpp>)
#include <libslic3r/MixedFilament.hpp>
#endif
#include <libslic3r/Orient.hpp>
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

#include <atomic>
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
        o["default"] = def.default_value ? def.default_value->serialize() : std::string();
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
// edits. args: { colors:[hex], definitions:"<mixed_filament_definitions>",
// edits:[{op:"add",a,b,percent} | {op:"remove",stableId} | {op:"percent",stableId,percent}] }.
// result: { definitions, filaments:[{id, stableId, custom, a, b, percent, components, color}] }.
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
    for (const json& e : args.value("edits", json::array())) {
        const std::string op = e.value("op", std::string());
        if (op == "add") {
            const unsigned a = unsigned(e.value("a", 1)), b = unsigned(e.value("b", 2));
            if (a < 1 || b < 1 || a > n || b > n || a == b) throw cs::JobError("mixed filament: components must be two different physical filaments");
            mgr.add_custom_filament(a, b, clampi(e.value("percent", 50), 0, 100), colors);
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
    json list = json::array();
    const auto& rows = mgr.mixed_filament_legacy_rows();
    for (unsigned id = unsigned(n) + 1; id <= unsigned(n + rows.size()); ++id) {
        const int idx = mgr.mixed_index_from_filament_id(id, n);
        if (idx < 0 || size_t(idx) >= rows.size()) continue;
        const MixedFilamentLegacyRow& r = rows[size_t(idx)];
        if (r.deleted) continue;
        json comps = json::array();
        for (unsigned c : MixedFilamentManager::decode_gradient_component_ids(r.gradient_component_ids, n)) comps.push_back(c);
        if (comps.empty()) comps = json::array({r.component_a, r.component_b});
        list.push_back({{"id", id}, {"stableId", r.stable_id}, {"custom", r.custom}, {"a", r.component_a}, {"b", r.component_b},
                        {"percent", r.mix_b_percent}, {"components", comps}, {"color", r.display_color}});
    }
    out.result["definitions"] = mgr.serialize_custom_entries();
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

int tool_impl(const char* job_json, int job_len, const uint8_t* blob, int blob_len, std::string& out_json, std::string& out_blob)
{
    ToolOut out;
    json rep = {{"ok", false}, {"error", nullptr}};
    try {
        if (!job_json || job_len <= 0) throw cs::JobError("empty request");
        const json j = json::parse(job_json, job_json + job_len);
        const std::string op = j.value("op", std::string());
        const json args = j.value("args", json::object());
        std::vector<ToolMesh> meshes = parse_tool_meshes(j, blob, blob_len);
        if (op == "paint_open") op_paint_open(args, meshes, out);
        else if (op == "paint_apply") op_paint_apply(args, out);
        else if (op == "paint_get") op_paint_get(args, out);
        else if (op == "paint_close") op_paint_close(args, out);
        else if (op == "paint_from_states") op_paint_from_states(args, meshes, out);
        else if (op == "layer_profile_adaptive") op_layer_profile(args, meshes, out, false);
        else if (op == "layer_profile_smooth") op_layer_profile(args, meshes, out, true);
        else if (op == "cut") op_cut(args, meshes, out);
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
