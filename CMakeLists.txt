cmake_minimum_required(VERSION 3.16)
project(smooth-contact-aware-facial-blendshapes-transfer)

cmake_policy(SET CMP0074 NEW)


#if(CMAKE_CONFIGURATION_TYPES)
#	if(NOT "DEBUG_WITH_LOG" IN_LIST CMAKE_CONFIGURATION_TYPES)
#		list(APPEND CMAKE_CONFIGURATION_TYPES "DEBUG_WITH_LOG")
#	endif()
#endif()



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
option(PRINT_PRETTY_LOG_ALL               "Use matrix log"                      OFF)
option(PRINT_PRETTY_LOG_DENSE               "Use matrix log"                      OFF)

if (PRINT_PRETTY_LOG_ALL)
  add_definitions(-DPRINT_PRETTY_LOG_ALL)
endif()
if (PRINT_PRETTY_LOG_DENSE)
  add_definitions(-DPRINT_PRETTY_LOG_DENSE)
endif()


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

