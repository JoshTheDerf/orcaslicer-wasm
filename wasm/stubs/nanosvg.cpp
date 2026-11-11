// nanosvg's implementation. Upstream instantiates it in the GUI (BitmapCache.cpp)
// and the CLI tools, neither of which is built; libslic3r's NSVGUtils (SVG emboss,
// Design-tab SVG import) needs it. Not a stub: the real parser, compiled once,
// through the same <nanosvg/...> include path NSVGUtils.cpp uses.
#define NANOSVG_IMPLEMENTATION
#include <nanosvg/nanosvg.h>
