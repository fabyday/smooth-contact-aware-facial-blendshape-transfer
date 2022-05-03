add_executable(test1 sample_main.cpp)



target_link_libraries(meshlib PUBLIC igl::core igl::opengl_glfw igl::opengl_glfw_imgui lib)