#pragma once
// OpenSSL is not linked in the WASM build. libslic3r only needs the legacy
// MD5_* API (file checksums in utils.cpp / bbs_3mf.cpp). This is a compact,
// standalone RFC 1321 implementation (verified against md5sum by
// wasm/tests/md5_selftest.cpp); the previous shim was a non-MD5 placeholder.
#include <cstddef>
#include <cstdint>
#include <cstring>

#define MD5_DIGEST_LENGTH 16

typedef struct MD5state_st {
    uint32_t a, b, c, d;
    uint64_t len;          // bytes processed
    unsigned char buf[64]; // pending block
    size_t used;
} MD5_CTX;

namespace orca_wasm_md5 {
inline uint32_t rotl(uint32_t x, int c) { return (x << c) | (x >> (32 - c)); }
inline void block(MD5_CTX* s, const unsigned char* p)
{
    static const uint32_t K[64] = {
        0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee, 0xf57c0faf, 0x4787c62a, 0xa8304613, 0xfd469501,
        0x698098d8, 0x8b44f7af, 0xffff5bb1, 0x895cd7be, 0x6b901122, 0xfd987193, 0xa679438e, 0x49b40821,
        0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa, 0xd62f105d, 0x02441453, 0xd8a1e681, 0xe7d3fbc8,
        0x21e1cde6, 0xc33707d6, 0xf4d50d87, 0x455a14ed, 0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a,
        0xfffa3942, 0x8771f681, 0x6d9d6122, 0xfde5380c, 0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70,
        0x289b7ec6, 0xeaa127fa, 0xd4ef3085, 0x04881d05, 0xd9d4d039, 0xe6db99e5, 0x1fa27cf8, 0xc4ac5665,
        0xf4292244, 0x432aff97, 0xab9423a7, 0xfc93a039, 0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1,
        0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1, 0xf7537e82, 0xbd3af235, 0x2ad7d2bb, 0xeb86d391};
    static const int R[64] = {7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
                              5, 9,  14, 20, 5, 9,  14, 20, 5, 9,  14, 20, 5, 9,  14, 20,
                              4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23,
                              6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21};
    uint32_t M[16];
    for (int i = 0; i < 16; ++i)
        M[i] = uint32_t(p[i * 4]) | uint32_t(p[i * 4 + 1]) << 8 | uint32_t(p[i * 4 + 2]) << 16 | uint32_t(p[i * 4 + 3]) << 24;
    uint32_t A = s->a, B = s->b, C = s->c, D = s->d;
    for (int i = 0; i < 64; ++i) {
        uint32_t F; int g;
        if (i < 16)      { F = (B & C) | (~B & D); g = i; }
        else if (i < 32) { F = (D & B) | (~D & C); g = (5 * i + 1) & 15; }
        else if (i < 48) { F = B ^ C ^ D;          g = (3 * i + 5) & 15; }
        else             { F = C ^ (B | ~D);       g = (7 * i) & 15; }
        const uint32_t t = D;
        D = C; C = B;
        B = B + rotl(A + F + K[i] + M[g], R[i]);
        A = t;
    }
    s->a += A; s->b += B; s->c += C; s->d += D;
}
} // namespace orca_wasm_md5

inline int MD5_Init(MD5_CTX* c)
{
    if (!c) return 0;
    c->a = 0x67452301; c->b = 0xefcdab89; c->c = 0x98badcfe; c->d = 0x10325476;
    c->len = 0; c->used = 0;
    return 1;
}

inline int MD5_Update(MD5_CTX* c, const void* data, size_t n)
{
    if (!c || (!data && n)) return 0;
    const unsigned char* p = static_cast<const unsigned char*>(data);
    c->len += n;
    while (n > 0) {
        const size_t take = (64 - c->used) < n ? (64 - c->used) : n;
        std::memcpy(c->buf + c->used, p, take);
        c->used += take; p += take; n -= take;
        if (c->used == 64) { orca_wasm_md5::block(c, c->buf); c->used = 0; }
    }
    return 1;
}

inline int MD5_Final(unsigned char* md, MD5_CTX* c)
{
    if (!c || !md) return 0;
    const uint64_t bits = c->len * 8;
    const unsigned char pad = 0x80, zero = 0;
    MD5_Update(c, &pad, 1);
    while (c->used != 56) MD5_Update(c, &zero, 1);
    unsigned char lenb[8];
    for (int i = 0; i < 8; ++i) lenb[i] = static_cast<unsigned char>(bits >> (8 * i));
    MD5_Update(c, lenb, 8);
    const uint32_t v[4] = {c->a, c->b, c->c, c->d};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) md[i * 4 + j] = static_cast<unsigned char>(v[i] >> (8 * j));
    return 1;
}
