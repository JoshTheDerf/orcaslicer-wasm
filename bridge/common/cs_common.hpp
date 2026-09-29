// Engine-neutral part of the Cubby Slicer WASM bridge (see
// cubby-slicer/docs/ENGINE-CONTRACT.md). Header-only; included by exactly one
// TU per engine (orca: bridge/cs_bridge.cpp, preflight: bridge/cs_bridge.cpp).
//
// Everything here is about *not trusting input*: offsets, counts and indices
// that arrive from JS are validated before any memory is touched, so a bad job
// produces an error report instead of an out-of-bounds read inside the engine.
#pragma once

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#ifdef __EMSCRIPTEN_PTHREADS__
#include <emscripten/proxying.h>
#include <emscripten/threading.h>
#endif
#endif

namespace cs {

using json = nlohmann::json;

constexpr int BRIDGE_ABI = 1;

struct JobError : std::runtime_error { using std::runtime_error::runtime_error; };

// An extra volume of an object (OrcaSlicer/PrusaSlicer ModelVolume): a model
// part, negative part, modifier, support blocker or support enforcer. Its
// transform maps its own mesh-local coordinates straight to bed coordinates.
struct VolumeInput {
    std::string name;
    std::string type = "part"; // part | negative | modifier | support_blocker | support_enforcer
    std::vector<float>    positions;
    std::vector<uint32_t> indices;
    double transform[16];
    json config = json::object(); // per-volume overrides
    json paint = json::object();  // { support|seam|color|fuzzy: [[triangle, hex], ...] }
};

struct MeshInput {
    std::string name;
    std::vector<float>    positions;   // xyz, copied out of the blob
    std::vector<uint32_t> indices;     // triangle triples, validated < vertexCount
    double transform[16];              // column-major
    json config = json::object();      // per-object overrides
    std::vector<VolumeInput> parts;    // extra volumes (optional)
    json paint = json::object();       // painted facets of the object's own mesh
    std::vector<double> layer_height_profile; // [z0, h0, z1, h1, ...] (print coords, from the object's bottom)
    json layer_ranges = json::array(); // [{ min, max, config }]
    json brim_points = json::array();  // [{ pos: [x,y,z], radius }] mesh-local
};

struct Job {
    json config = json::object();
    std::vector<MeshInput> objects;
    bool validate = true;
    bool drop_to_bed = true;
    // Extra G-code placeholder variables { name: { type, value } } (engines that
    // run another slicer's profiles use them for variables they don't define).
    json placeholder_vars = json::object();
    json custom_gcodes = json::array(); // plate CustomGCode items [{ z, type, extruder, color, extra }]
    json calib = json::object();        // Calib_Params (Calibration menu), empty = none
};

// Serialize one JSON config value to the engine's native string form. Vector
// options arrive as arrays; `join` is how the engine's deserializer expects
// the elements separated ("," for numeric vectors, ";" for string vectors).
inline std::string scalar_to_string(const json& v)
{
    if (v.is_string()) return v.get<std::string>();
    if (v.is_boolean()) return v.get<bool>() ? "1" : "0";
    if (v.is_number_integer()) return std::to_string(v.get<long long>());
    if (v.is_number()) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%.17g", v.get<double>());
        return buf;
    }
    if (v.is_null()) return "";
    throw JobError("unsupported config value type: " + v.dump());
}

inline bool checked_range(uint64_t off, uint64_t count, uint64_t elem, uint64_t len)
{
    if (elem == 0) return false;
    if (count > (std::numeric_limits<uint64_t>::max() / elem)) return false;
    uint64_t bytes = count * elem;
    return off <= len && bytes <= len - off;
}

inline uint64_t get_u64(const json& o, const char* key)
{
    auto it = o.find(key);
    if (it == o.end() || !it->is_number_integer() || it->get<long long>() < 0)
        throw JobError(std::string("object field '") + key + "' must be a non-negative integer");
    return it->get<uint64_t>();
}

// Mesh range + transform of one object / volume entry (bounds-checked copy out of the blob).
inline void parse_mesh_entry(const json& o, const std::string& name, const uint8_t* blob, uint64_t len,
                             std::vector<float>& positions, std::vector<uint32_t>& indices, double (&transform)[16])
{
    uint64_t voff = get_u64(o, "vertexOffset"), vcnt = get_u64(o, "vertexCount");
    uint64_t ioff = get_u64(o, "indexOffset"),  tcnt = get_u64(o, "triangleCount");
    if (voff % 4 || ioff % 4) throw JobError(name + ": mesh offsets must be 4-byte aligned");
    if (vcnt < 3 || tcnt < 1) throw JobError(name + ": mesh is empty");
    if (vcnt > 0x7fffffffu || tcnt > 0x7fffffffu) throw JobError(name + ": mesh too large");
    if (!checked_range(voff, vcnt, 12, len)) throw JobError(name + ": vertex range out of bounds");
    if (!checked_range(ioff, tcnt, 12, len)) throw JobError(name + ": index range out of bounds");
    positions.resize(vcnt * 3);
    std::memcpy(positions.data(), blob + voff, vcnt * 12);
    for (float f : positions)
        if (!std::isfinite(f)) throw JobError(name + ": non-finite vertex coordinate");
    indices.resize(tcnt * 3);
    std::memcpy(indices.data(), blob + ioff, tcnt * 12);
    for (uint32_t idx : indices)
        if (idx >= vcnt) throw JobError(name + ": triangle index out of range");
    auto t = o.find("transform");
    if (t == o.end() || !t->is_array() || t->size() != 16) throw JobError(name + ": transform must be 16 numbers");
    for (int i = 0; i < 16; ++i) {
        if (!(*t)[i].is_number()) throw JobError(name + ": transform must be numeric");
        transform[i] = (*t)[i].get<double>();
        if (!std::isfinite(transform[i])) throw JobError(name + ": non-finite transform");
    }
}

// Parse + validate a job. Copies mesh data out of the blob so the engine never
// holds a pointer into JS-owned memory.
inline Job parse_job(const char* job_json, int job_len, const uint8_t* blob, int blob_len)
{
    if (!job_json || job_len <= 0) throw JobError("empty job");
    if (blob_len < 0 || (blob_len > 0 && !blob)) throw JobError("invalid blob");
    json j = json::parse(job_json, job_json + job_len); // throws json::exception on bad input
    if (!j.is_object()) throw JobError("job must be a JSON object");

    Job job;
    if (auto it = j.find("config"); it != j.end()) {
        if (!it->is_object()) throw JobError("job.config must be an object");
        job.config = *it;
    }
    if (auto it = j.find("options"); it != j.end() && it->is_object()) {
        job.validate    = it->value("validate", true);
        job.drop_to_bed = it->value("dropToBed", true);
        if (auto pv = it->find("placeholderVars"); pv != it->end() && pv->is_object())
            job.placeholder_vars = *pv;
        if (auto cg = it->find("customGcodes"); cg != it->end() && cg->is_array())
            job.custom_gcodes = *cg;
        if (auto cb = it->find("calib"); cb != it->end() && cb->is_object())
            job.calib = *cb;
    }
    auto objs = j.find("objects");
    if (objs == j.end() || !objs->is_array() || objs->empty()) throw JobError("job has no objects");
    if (objs->size() > 4096) throw JobError("too many objects");

    const uint64_t len = static_cast<uint64_t>(blob_len);
    for (const json& o : *objs) {
        if (!o.is_object()) throw JobError("object entry must be an object");
        MeshInput m;
        m.name = o.value("name", std::string("object"));
        parse_mesh_entry(o, m.name, blob, len, m.positions, m.indices, m.transform);
        if (auto ps = o.find("parts"); ps != o.end() && !ps->is_null()) {
            if (!ps->is_array()) throw JobError(m.name + ": parts must be an array");
            if (ps->size() > 1024) throw JobError(m.name + ": too many parts");
            for (const json& pj : *ps) {
                if (!pj.is_object()) throw JobError(m.name + ": part entry must be an object");
                VolumeInput v;
                v.name = pj.value("name", m.name + " part");
                v.type = pj.value("type", std::string("part"));
                if (v.type != "part" && v.type != "negative" && v.type != "modifier" && v.type != "support_blocker" && v.type != "support_enforcer")
                    throw JobError(v.name + ": unknown part type '" + v.type + "'");
                parse_mesh_entry(pj, v.name, blob, len, v.positions, v.indices, v.transform);
                if (auto c = pj.find("config"); c != pj.end()) {
                    if (!c->is_object()) throw JobError(v.name + ": config must be an object");
                    v.config = *c;
                }
                if (auto pt = pj.find("paint"); pt != pj.end() && pt->is_object()) v.paint = *pt;
                m.parts.emplace_back(std::move(v));
            }
        }
        if (auto c = o.find("config"); c != o.end()) {
            if (!c->is_object()) throw JobError(m.name + ": config must be an object");
            m.config = *c;
        }
        if (auto pt = o.find("paint"); pt != o.end() && pt->is_object()) m.paint = *pt;
        if (auto lh = o.find("layerHeightProfile"); lh != o.end() && lh->is_array()) {
            if (lh->size() % 2 || lh->size() > 200000) throw JobError(m.name + ": layerHeightProfile must be [z, h] pairs");
            for (const json& v : *lh) {
                if (!v.is_number() || !std::isfinite(v.get<double>())) throw JobError(m.name + ": layerHeightProfile must be numeric");
                m.layer_height_profile.push_back(v.get<double>());
            }
        }
        if (auto lr = o.find("layerRanges"); lr != o.end() && lr->is_array()) m.layer_ranges = *lr;
        if (auto bp = o.find("brimPoints"); bp != o.end() && bp->is_array()) m.brim_points = *bp;
        job.objects.emplace_back(std::move(m));
    }
    return job;
}

// ---- output buffers ------------------------------------------------------

// Copy `s` into a fresh malloc'd buffer owned by JS (released with cs_free).
inline int emit(const std::string& s, char** out, int* out_len)
{
    if (!out || !out_len) return 0;
    *out = nullptr; *out_len = 0;
    if (s.size() > static_cast<size_t>(std::numeric_limits<int>::max())) return -1;
    char* p = static_cast<char*>(std::malloc(s.size() + 1));
    if (!p) return -1;
    std::memcpy(p, s.data(), s.size());
    p[s.size()] = '\0';
    *out = p;
    *out_len = static_cast<int>(s.size());
    return 0;
}

inline json make_report() {
    return json{{"ok", false}, {"error", nullptr}, {"warnings", json::array()},
                {"stats", nullptr}, {"substitutions", json::array()}};
}

// ---- progress --------------------------------------------------------------

#ifdef __EMSCRIPTEN__
// Calls Module.csProgress(percent, message) if the host installed it. Never
// throws back into C++ (a JS exception crossing Wasm EH frames is fatal).
EM_JS(void, cs_js_progress, (int percent, const char* msg), {
    try {
        if (typeof Module["csProgress"] === "function")
            Module["csProgress"](percent, msg ? UTF8ToString(msg) : "");
    } catch (e) {}
});
#else
inline void cs_js_progress(int, const char*) {}
#endif

#if defined(__EMSCRIPTEN__) && defined(__EMSCRIPTEN_PTHREADS__)
// Pthreads engines: Module.csProgress only exists on the main runtime thread
// (a pthread worker has its own, empty Module). Progress reported from any
// other thread is queued to the main runtime thread (FIFO, fire-and-forget).
struct ProgressMsg { int percent; char text[1]; };
inline void progress_on_main(void* p)
{
    auto* m = static_cast<ProgressMsg*>(p);
    cs_js_progress(m->percent, m->text);
    std::free(m);
}
#endif

inline void progress(int percent, const std::string& msg)
{
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
#if defined(__EMSCRIPTEN__) && defined(__EMSCRIPTEN_PTHREADS__)
    if (!emscripten_is_main_runtime_thread()) {
        auto* m = static_cast<ProgressMsg*>(std::malloc(sizeof(ProgressMsg) + msg.size()));
        if (!m) return;
        m->percent = percent;
        std::memcpy(m->text, msg.c_str(), msg.size() + 1);
        if (!emscripten_proxy_async(emscripten_proxy_get_system_queue(), emscripten_main_runtime_thread_id(),
                                    progress_on_main, m))
            std::free(m);
        return;
    }
#endif
    cs_js_progress(percent, msg.c_str());
}

// Engine config JSON for cs_eval_condition: {"key": value|[values]}.
inline json parse_config_json(const char* cfg, int len)
{
    if (!cfg || len <= 0) return json::object();
    json j = json::parse(cfg, cfg + len);
    if (!j.is_object()) throw JobError("config must be an object");
    return j;
}

} // namespace cs
