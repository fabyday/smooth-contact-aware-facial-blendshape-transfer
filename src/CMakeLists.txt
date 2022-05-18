
add_library(meshlib INTERFACE)
message(STATUS "CMake inatall directory: " ${CMAKE_INSTALL_INCLUDEDIR})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
message("${PCL_LIBRARY_DIRS} wow fafafa")
add_definitions(${PCL_DEFINITIONS})
target_include_directories(meshlib INTERFACE .)
target_link_libraries(meshlib INTERFACE igl::core igl::opengl_glfw igl::opengl_glfw_imgui ${PCL_LIBRARIES})