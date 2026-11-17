cmake_minimum_required(VERSION 3.22)

project(social_force_model VERSION 0.1.0 LANGUAGES CXX)

option(SFM_BUILD_TESTS "Build unit tests" ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(sfm_core
    src/agent.cpp
    src/simulation.cpp
    src/run.cpp
    src/coarse_graining.cpp
    src/admissibility.cpp
)

target_include_directories(sfm_core
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(sfm_core PUBLIC cxx_std_20)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(sfm_core PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

add_executable(sfm_sim src/main.cpp)
target_link_libraries(sfm_sim PRIVATE sfm_core)

if(SFM_BUILD_TESTS)
    enable_testing()
    add_executable(sfm_smoke_test tests/smoke_test.cpp)
    target_link_libraries(sfm_smoke_test PRIVATE sfm_core)
    add_test(NAME smoke COMMAND sfm_smoke_test
        ${CMAKE_CURRENT_SOURCE_DIR}/config/example.toml
        ${CMAKE_CURRENT_SOURCE_DIR}/config/bidirectional_fixed_vector_smoke.toml)
endif()
