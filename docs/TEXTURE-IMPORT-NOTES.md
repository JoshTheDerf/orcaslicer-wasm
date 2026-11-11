# Texture import + texture-displacement colour — paused work (orcaslicer-wasm, Orca 2.5.0)

Status 2026-11-10 ~20:50: STOPPED on the owner's request before the port commit. The tracked tree is back to its
pre-work state (only off-by-default toolchain additions remain); the finished-but-unshipped work is kept as untracked
files + a patch, listed below. Nothing was staged into cubby-slicer.

## What Orca 2.5.0 actually uses
- **Assimp**: only `libslic3r/Format/AssimpImport.cpp` (`load_assimp_textured_model`, path-based `Importer::ReadFile`,
  flags Triangulate|GenNormals|PreTransformVertices|SortByPType, +FlipUVs for .fbx/.glb; FBX read-all-layers /
  materials / textures; material colour = BASE_COLOR else DIFFUSE; texture = DIFFUSE else BASE_COLOR, embedded or an
  external file resolved by `ResourcePathUtils`). It does NOT read glTF vertex colours. Orca's deps: Assimp 5.4.3,
  importers glTF(+GLB), OBJ, FBX only, ASSIMP_NO_EXPORT. `Model::read_from_file` routes .glb/.gltf/.fbx there; .obj
  stays on Orca's own OBJ loader (`obj_to_textured_mesh` for map_Kd textures; `precomputed_face_colors` /
  `precomputed_vertex_colors` for vertex colours and MTL Kd). No PLY path in Orca.
- **OpenCV**: `TexturePainting.cpp` — `cv::imdecode` (PNG/JPEG), `cvtColor` RGB(A)->BGR, `Mat` ROI `copyTo` (multi-texture
  atlas); `TextureToColor/TextureToColor.cpp` — `cv::Mat` only as the texture container (its clustering/smoothing is
  Orca's own code + CGAL PMP repair). `ObjColorUtils.hpp` (kmeans, HSV/Lab cvtColor) is used only by the CLI
  (`OrcaSlicer.cpp`), not by the import dialog. So: core + imgproc + imgcodecs (PNG, JPEG). Orca pins OpenCV 4.14.0.
  An embedded TGA (OpenCV has no TGA reader) fails analysis on desktop too -> geometry-only import.
- **Texture displacement colour** needs NO OpenCV: libslic3r `build_texture_displacement(..., TextureColorRequest*)`
  + the GUI's palette code in `GLGizmoTextureDisplacement.cpp` (cached_palette, mix_targets, rank_mixes, pick_mixes,
  make_palette, palette_filaments, make_palette_quantizer, srgb_to_lab) + `DeltaE00` from slic3r/Utils/ColorSpaceConvert.cpp
  (that file needs wx, so it was copied) + `TextureDisplacementBakeJob::finalize` (transfer_facet_paint of the volume's
  paint when the topology changed, colour merged over the mmu paint, mixed slots via `Sidebar::ensure_mixed_filament` =
  `find_fixed_mixed_filament` or append, clear baked layer masks, transfer_uv_channel).
- **Import Model dialog** (`slic3r/GUI/TextureImportDialog.cpp`): defaults colour count 4 (first run automatic = 0, then
  the control is set to the result 2..32), smooth 5 (/10), repair decision Ask; `do_auto_match` (auto-merge on: nearest
  filament by CIEDE2000, new "Bambu PLA Basic" filament when ΔE > 5; off: one new filament per colour),
  `compact_used_virtual_filaments`, sort rows by filament, `apply_auto_standard_mix` (CMYW/RYBW via
  `lookup_standard_recipe`, bases = official PLA Basic colours), `add_decomposed_mixed_filament`; on OK compact again.
  `Plater::apply_textured_mesh_import_result`: remap existing physical -> slot, existing mixed -> slot + new physical
  count, new physical appended after physical, new mixed appended at the end; extruder = lowest filament painted;
  `apply_painted_mesh_to_volume` replaces the mesh (painted mesh is in the raw file frame) and paints per triangle.

## Decisions taken
- Displacement colour belongs in the slicer's existing `cs_tool texture_displacement` (no new library).
- Assimp + OpenCV go in a SEPARATE lazily loaded single-threaded module `texture.{mjs,wasm,data}` (like the CAD module),
  so the slicer download/start-up is unchanged. Measured: texture.wasm 7,406,449 B (gzip -9 2.21 MB, brotli 1.56 MB),
  texture.data 253,854 B (resources/filament_mixing). It shares the slicer's libslic3r archive (built in build-wasm/).
  The slicer grew only by the palette port: slicer.wasm ST 15,381,247 -> 15,453,495 B (+72 KB), MT 16,282,982 -> 16,367,465 B.
- MT variants of Assimp/OpenCV not built (nothing links them; `WASM_THREADS=1 bash build-deps.sh texture` would).

## What is where now
- Toolchain (built, kept): `../wasm-deps/assimp-5.4.3/` (libassimp.a, glTF/OBJ/FBX, port zlib) and
  `../wasm-deps/opencv-4.14.0/` (libopencv_{core,imgproc,imgcodecs}.a, port libpng/libjpeg/zlib, no SIMD/parallel
  framework, no -fexceptions — checked). Recipe: `toolchain/build-deps.sh` functions `build_assimp`, `build_opencv`,
  `em_ports_ready`, targets `assimp|opencv|texture` (NOT in `all`; tracked file, kept: off by default; copy synced to
  ../wasm-deps/build-deps.sh). Command: `cd ../wasm-deps && NPROC=3 bash build-deps.sh texture` (~25 min).
- Untracked, complete, unreferenced by the default build:
  - `bridge/cs_texture_palette.{hpp,cpp}` (gizmo palette port)
  - `bridge/texture_bridge.cpp` (module: `tex_version`, `tex_call`, `tex_free`; ops load / analyze / match / add_mixed /
    apply / close; JSON+blob like cs_tool with named `blobs`)
  - `wasm/texture.cmake`, `scripts/build-texture.sh` (target `texture` in build-wasm/, writes texture-version.json)
  - `tests/texture-test.mjs` (59 checks, ALL PASSED on the build made at 20:27) and fixtures
    `tests/fixtures/texture/{vertex_color_cube.obj (written by me), maxPbrMaterial_metalRough.fbx,
    box_orphant_embedded_texture.fbx, phong_cube.fbx (Assimp 5.4.3 test/models/FBX, BSD-3)}`; Orca's
    `orca/tests/data/texture_import/` (two_color_png/jpeg.glb, two_color_sphere OBJ+MTL(+JPEG)) used in place.
  - `docs/texture-import-wip/`: `tracked-changes.patch` (my edits to bridge/cs_bridge.cpp, wasm/CMakeLists.txt,
    wasm/stubs/formats.cpp, tests/cs-tool-test.mjs, README.md, wasm/README.md — `git apply` checked),
    `assimp_import.cpp` (-> wasm/stubs/), `cubby-slicer-ENGINE-CONTRACT.patch` (contract text for both features),
    `cubby-slicer-fetch-texture-wasm.sh` (-> cubby-slicer/scripts/).
- Build dirs: build-wasm/ and build-wasm-mt/ currently hold slicers built WITH the patch (+ build-wasm/texture.*);
  they must be rebuilt from the reverted tree before any re-stage. cubby-slicer public/wasm/orca is untouched (10:45 build).

## Tested before the stop (with the patch applied)
- ST: cs-slice-test 112, cs-tool-test 55 (46 + 9 new displacement-colour checks), cs-arrange-test 31: ALL PASSED.
- MT: built; suites were not run (stopped).
- texture-test 59/59 on ST.

## To resume
1. `git apply docs/texture-import-wip/tracked-changes.patch`; `cp docs/texture-import-wip/assimp_import.cpp wasm/stubs/`
   (strip its first two comment lines); in cubby-slicer apply the contract patch and copy the fetch script.
   Consider putting the texture.cmake include and the palette source behind options if they must ship separately.
2. `bash scripts/build-wasm.sh`, `WASM_THREADS=1 bash scripts/build-wasm.sh`, `bash scripts/build-texture.sh`.
3. Suites on both variants + `node tests/texture-test.mjs build-wasm`; then stage (fetch-orca-wasm.sh, fetch-texture-wasm.sh).
4. Open items: the app side (Import Model dialog UI, Texture Displacement colour panel); MT run of the suites;
   per-library size attribution inside texture.wasm (not measured; whole module 7.4 MB).
