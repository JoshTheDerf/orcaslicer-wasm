# Not linked in the WASM build. "Found" without an OpenVDB::openvdb target, so
# libslic3r leaves OpenVDBUtils.cpp out; its only user, SLA hollowing, is
# replaced by wasm/stubs/sla_hollowing.cpp.
set(OpenVDB_FOUND TRUE)
set(OPENVDB_FOUND TRUE)
set(OpenVDB_VERSION "10.0.0")
