cmake_minimum_required(VERSION 3.10)
project(playback)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Base warnings
add_compile_options(-Wall)

# Performance optimizations for Eigen
add_compile_options(
    -O3
    -march=native
    -ffast-math
    -funroll-loops
)

if(APPLE)
    message(STATUS "Configuring for macOS...")
    include_directories(
        /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk/usr/include
    )
endif()

# PCL Integration
find_package(PCL 1.10 REQUIRED)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

# Executable + Libraries
add_executable(ukf_highway
    src/main.cpp
    src/ukf.cpp
    src/tools.cpp
    src/render/render.cpp
)
target_link_libraries(ukf_highway ${PCL_LIBRARIES})

message(STATUS "Final compile flags: ${CMAKE_CXX_FLAGS}")
