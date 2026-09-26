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
#include <libslic3r/GCode/GCodeProcessor.hpp>
#include <libslic3r/Layer.hpp>
#include <libslic3r/Model.hpp>
#include <libslic3r/Orient.hpp>
#include <libslic3r/Geometry.hpp>
#include <libslic3r/PlaceholderParser.hpp>
#include <libslic3r/Preset.hpp>
#include <libslic3r/Print.hpp>
#include <libslic3r/PrintConfig.hpp>
#include <libslic3r/TriangleMesh.hpp>
#include <libslic3r/Utils.hpp>
#include "libslic3r_version.h"  // generated into orca-build/src/libslic3r

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
    case coPointsGroups: return "pointsGroups";
    case coIntsGroups: return "intsGroups";
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
                      const std::string& name)
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
    for (size_t i = 0; i + 2 < indices.size(); i += 3) {
        const int a = int(indices[i]), b = int(indices[i + 1]), c = int(indices[i + 2]);
        if (a == b || b == c || a == c) continue; // degenerate
        its.indices.emplace_back(a, b, c);
    }
    if (its.indices.empty()) throw cs::JobError(name + ": mesh has no valid triangles");
    TriangleMesh mesh(std::move(its));
    if (mesh.volume() < 0) mesh.flip_triangles(); // inside-out input
    return mesh;
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
        ModelVolume* vol = obj->add_volume(bed_mesh(m.positions, m.indices, m.transform, m.name));
        vol->name = m.name;
        // Extra volumes (Orca parts / negative parts / modifiers / support
        // blockers & enforcers), each with its own settings.
        for (const cs::VolumeInput& p : m.parts) {
            ModelVolume* pv = obj->add_volume(bed_mesh(p.positions, p.indices, p.transform, p.name), volume_type(p.type));
            pv->name = p.name;
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
        if (!config.has("nozzle_volume_type")) {
            auto* nvt = config.option<ConfigOptionEnumsGeneric>("nozzle_volume_type", true);
            nvt->values.resize(extruder_count, nvtStandard);
        }
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
    if (result.gcode_check_result.error_code)
        warnings.push_back({{"code", "unprintable_area"},
                            {"message", "G-code contains moves outside the printable area."}});

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
    return json{{"engine", "orca"}, {"version", SoftFever_VERSION}, {"options", std::move(opts)}}.dump();
}

} // namespace

extern "C" {

EMSCRIPTEN_KEEPALIVE const char* cs_version(void)
{
    static const std::string v = json{{"engine", "orca"}, {"version", SoftFever_VERSION},
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
