set(src dg_solver.cpp)

add_library(meshlib ${src})
target_link_libraries(meshlib PUBLIC igl::core igl::opengl_glfw igl::opengl_glfw_imgui)