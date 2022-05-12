cmake_minimum_required(VERSION 3.16)
project(smooth-contact-aware-facial-blendshapes-transfer)

cmake_policy(SET CMP0074 NEW)

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmakes)



add_compile_definitions("_CRT_SECURE_NO_WARNINGS")
add_compile_definitions("WIN32")
add_compile_definitions("_WINDOWS")
add_compile_definitions("IMGUI_IMPL_OPENGL_LOADER_GLAD")

set(CMAKE_CXX_STANDARD 17)


# Libigl
option(LIBIGL_WITH_COMISO            "Use CoMiso"                   ON)
option(LIBIGL_WITH_EMBREE            "Use Embree"                   ON)
option(LIBIGL_WITH_OPENGL            "Use OpenGL"                   ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"                     ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui"                    ON)
option(LIBIGL_WITH_PNG               "Use PNG"                      ON)
option(LIBIGL_WITH_TETGEN            "Use Tetgen"                   ON)
option(LIBIGL_WITH_TRIANGLE          "Use Triangle"                 ON)
option(LIBIGL_WITH_PREDICATES        "Use exact predicates"         ON)
option(LIBIGL_WITH_XML               "Use XML"                      ON)

# library
include(libigl)
include(spdlog)
include(yaml-cpp)
include(pcl)

#include(zlib)
#set(ZLIB_ROOT ${zlib_SOURCE_DIR})
#include(cnpy)

#use multi byte compile
add_definitions(-D_MBCS)

add_subdirectory(src)
# excutable
add_subdirectory(tests)