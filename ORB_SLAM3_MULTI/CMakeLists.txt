cmake_minimum_required(VERSION 3.10)
project(ORB_SLAM3_MULTI)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Release)

message(STATUS "Building ORB_SLAM3_MULTI")

find_package(Eigen3 REQUIRED)

# ============================================================
# g2o interno de ORB_SLAM3_MULTI
# ============================================================

set(G2O_ROOT ${PROJECT_SOURCE_DIR}/Thirdparty/g2o)
set(G2O_LIB_DIR ${G2O_ROOT}/lib)
set(G2O_LIB ${G2O_LIB_DIR}/libg2o_multi.so)

if(NOT EXISTS ${G2O_LIB})
    message(FATAL_ERROR "g2o library not found: ${G2O_LIB}. Build Thirdparty/g2o first.")
endif()

# ============================================================
# Librería ORB_SLAM3_MULTI
# ============================================================

add_library(ORB_SLAM3_MULTI SHARED
    src/MultiEssentialGraphOptimizer.cc
)

target_include_directories(ORB_SLAM3_MULTI
    PUBLIC
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${G2O_ROOT}
        ${G2O_ROOT}/g2o
)

target_link_libraries(ORB_SLAM3_MULTI
    ${G2O_LIB}
)

# ============================================================
# Forzar runtime path para que use este g2o, no el de ORB_SLAM3.
# ============================================================

set_target_properties(ORB_SLAM3_MULTI PROPERTIES
    BUILD_RPATH "${G2O_LIB_DIR}"
    INSTALL_RPATH "${G2O_LIB_DIR}"
    BUILD_WITH_INSTALL_RPATH TRUE
)