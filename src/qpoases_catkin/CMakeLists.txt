cmake_minimum_required(VERSION 3.14)
project(qpoases_catkin)

find_package(catkin REQUIRED)

include(FetchContent)

# Define directories
set(QPOASES_DEVEL_PREFIX ${CATKIN_DEVEL_PREFIX} CACHE STRING "QPOASES install path")
set(QPOASES_INCLUDE_DIR ${QPOASES_DEVEL_PREFIX}/include)
set(QPOASES_LIB_DIR ${QPOASES_DEVEL_PREFIX}/lib)
set(QPOASES_DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/download)
set(QPOASES_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/build)

# Create directories if they do not exist
file(MAKE_DIRECTORY ${QPOASES_INCLUDE_DIR})
file(MAKE_DIRECTORY ${QPOASES_LIB_DIR})
file(MAKE_DIRECTORY ${QPOASES_DOWNLOAD_DIR})
file(MAKE_DIRECTORY ${QPOASES_BUILD_DIR})

# HPIPM Settings
set(BUILD_SHARED_LIBS ON CACHE STRING "Build shared libraries" FORCE)
set(QPOASES_BUILD_EXAMPLES OFF CACHE BOOL "Examples disable")

# Download & build source
#
# Use a source archive instead of git clone to reduce transient network failures
# during FetchContent (works the same on x86_64 and aarch64).
FetchContent_Declare(qpoasesDownload
        URL https://github.com/coin-or/qpOASES/archive/268b2f2659604df27c82aa6e32aeddb8c1d5cc7f.tar.gz
        SOURCE_DIR ${QPOASES_DOWNLOAD_DIR}
        )
FetchContent_MakeAvailable(qpoasesDownload)

# Copy header to where catkin expects them
file(COPY ${QPOASES_DOWNLOAD_DIR}/include/qpOASES.hpp DESTINATION ${QPOASES_INCLUDE_DIR})

file(GLOB_RECURSE HEADERS "${QPOASES_DOWNLOAD_DIR}/include/qpOASES/*")
foreach (HEADER_FILE ${HEADERS})
    message(STATUS "FOUND HEADER: " ${HEADER_FILE})
    file(COPY ${HEADER_FILE} DESTINATION ${QPOASES_INCLUDE_DIR}/qpOASES)
endforeach ()

file(GLOB_RECURSE HEADERS "${QPOASES_DOWNLOAD_DIR}/include/qpOASES/extras/*")
foreach (HEADER_FILE ${HEADERS})
    message(STATUS "FOUND HEADER: " ${HEADER_FILE})
    file(COPY ${HEADER_FILE} DESTINATION ${QPOASES_INCLUDE_DIR}/qpOASES/extras)
endforeach ()

set_target_properties(qpOASES PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${QPOASES_LIB_DIR})

catkin_package(
        INCLUDE_DIRS ${QPOASES_INCLUDE_DIR}
        LIBRARIES qpOASES
)
