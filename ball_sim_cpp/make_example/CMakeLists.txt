cmake_minimum_required(VERSION 3.16)

project(ball_sim_cpp_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

get_filename_component(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(CARGO_TARGET_DIR "${REPO_ROOT}/target")
set(RUST_MANIFEST "${REPO_ROOT}/ball_sim_cpp/Cargo.toml")

set(RUST_PROFILE "debug")
set(RUST_PROFILE_FLAG "")
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(RUST_PROFILE "release")
  set(RUST_PROFILE_FLAG "--release")
endif()

set(RUST_LIB "${CARGO_TARGET_DIR}/${RUST_PROFILE}/libball_sim_cpp.a")
set(CXXBRIDGE_DIR "${CARGO_TARGET_DIR}/cxxbridge")
set(CXXBRIDGE_HDR "ball_sim.h")

add_custom_command(
    OUTPUT "${RUST_LIB}" "${CXXBRIDGE_HDR}"
    COMMAND cargo build ${RUST_PROFILE_FLAG} --manifest-path "${RUST_MANIFEST}"
    WORKING_DIRECTORY "${REPO_ROOT}"
    COMMENT "Building Rust static library with cargo"
    VERBATIM
)

add_custom_target(ball_sim_cpp_build DEPENDS "${RUST_LIB}" "${CXXBRIDGE_HDR}")

add_executable(ball_sim_example main.cpp)
add_dependencies(ball_sim_example ball_sim_cpp_build)

target_include_directories(ball_sim_example PRIVATE
    "${CXXBRIDGE_DIR}"
    "${CXXBRIDGE_DIR}/include"
)

target_link_libraries(ball_sim_example PRIVATE
    "${RUST_LIB}"
)

if(UNIX AND NOT APPLE)
  target_link_libraries(ball_sim_example PRIVATE dl pthread)
endif()
