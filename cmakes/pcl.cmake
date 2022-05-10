if(TARGET pcl)
    return()
endif()

include(FetchContent)
FetchContent_Declare(
    pcl
    GIT_REPOSITORY https://github.com/PointCloudLibrary/pcl.git
    GIT_TAG e8ed4be802f7d0b1acff2f8b01d7c5f381190e05
)


FetchContent_GetProperties(pcl)
if (NOT pcl_POPULATED)
    FetchContent_Populate(pcl)
    add_subdirectory(${pcl_SOURCE_DIR} ${pcl_BINARY_DIR})
endif ()