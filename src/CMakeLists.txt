
add_library(meshlib INTERFACE)
message(STATUS "CMake inatall directory: " ${CMAKE_INSTALL_INCLUDEDIR})
target_include_directories(meshlib INTERFACE .)
target_link_libraries(meshlib INTERFACE igl::core igl::opengl_glfw igl::opengl_glfw_imgui)