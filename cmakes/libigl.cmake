if(TARGET igl::core)
    return()
endif()

include(FetchContent)
FetchContent_Declare(
    libigl
    GIT_REPOSITORY https://github.com/RohJiHyun/libigl.git
    GIT_TAG 65584ab76e8c0640a8c76de974fa2afba2ab8213
)

# Note: In libigl v3.0.0, the following will become a one-liner:
# FetchContent_MakeAvailable(libigl)

FetchContent_GetProperties(libigl)
if(NOT libigl_POPULATED)
    FetchContent_Populate(libigl)
endif()
list(PREPEND CMAKE_MODULE_PATH "${libigl_SOURCE_DIR}/cmake")
include(${libigl_SOURCE_DIR}/cmake/libigl.cmake)
#set(EIGEN_INCLUDE_DIR "${libigl_SOURCE_DIR}/external/eigen") # for PCL library