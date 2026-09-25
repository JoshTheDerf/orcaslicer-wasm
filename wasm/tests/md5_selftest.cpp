#include "../wasm_shims/openssl/md5.h"
#include <cstdio>
#include <string>
int main(int argc, char** argv) {
    std::string s = argc > 1 ? argv[1] : "";
    MD5_CTX c; MD5_Init(&c);
    // feed in odd-sized pieces to exercise buffering
    for (size_t i = 0; i < s.size(); i += 7) MD5_Update(&c, s.data() + i, std::min<size_t>(7, s.size() - i));
    unsigned char d[16]; MD5_Final(d, &c);
    for (int i = 0; i < 16; ++i) printf("%02x", d[i]);
    printf("\n");
}
