// Fast strtod/strtof for the WASM engines (linked with -Wl,--wrap=strtod,--wrap=strtof).
//
// musl's strtod parses through `long double`, which on wasm32 is a 128-bit
// software float (__addtf3/__multf3/fmodl): ~20x slower than native. libslic3r
// parses floats on every G-code line (CoolingBuffer's atof, std::stof in the
// G-code processor, config deserialisation), which made G-code export the
// slowest phase of a slice. fast_float is correctly rounded (bit-identical to
// a conforming strtod) for decimal input; anything it doesn't cover — hex
// floats, over-long spans, out-of-range values — falls through to musl so
// errno/endptr semantics stay exactly those of the C library.
#include <cctype>
#include <cstring>
#include <system_error>
#include <fast_float/fast_float.h>

extern "C" double __real_strtod(const char*, char**);
extern "C" float __real_strtof(const char*, char**);

namespace {
constexpr size_t kMaxSpan = 256;

// Length of the candidate number at p (digits, sign, '.', exponent, inf/nan letters),
// bounded so a pointer into a huge buffer is never scanned to its end.
inline size_t span(const char* p) {
    size_t n = 0;
    while (n < kMaxSpan && p[n] && std::strchr("0123456789+-.eEinfatyINFATY", p[n])) ++n;
    return n;
}

template <typename T, typename Real>
inline T fast(const char* s, char** end, Real real) {
    const char* p = s;
    while (std::isspace(static_cast<unsigned char>(*p))) ++p;
    const char* q = p;
    if (*q == '+') ++q;                              // fast_float rejects a leading '+'
    const char* digits = (*q == '-') ? q + 1 : q;
    if (digits[0] == '0' && (digits[1] == 'x' || digits[1] == 'X')) return real(s, end);
    const size_t n = span(q);
    if (n == 0 || n == kMaxSpan) return real(s, end);
    T v{};
    const auto r = fast_float::from_chars(q, q + n, v);
    if (r.ec != std::errc() || r.ptr == q || (q != p && *q == '-')) return real(s, end); // errors, "+-1"
    if (end) *end = const_cast<char*>(r.ptr);
    return v;
}
} // namespace

extern "C" double __wrap_strtod(const char* s, char** end) { return fast<double>(s, end, __real_strtod); }
extern "C" float __wrap_strtof(const char* s, char** end) { return fast<float>(s, end, __real_strtof); }
