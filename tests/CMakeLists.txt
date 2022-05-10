set(resource_path ${PROJECT_SOURCE_DIR}/examples)


add_executable(test1 sample_main.cpp)
#
target_link_libraries(test1 PUBLIC meshlib igl::core igl::opengl_glfw igl::opengl_glfw_imgui yaml-cpp)
target_compile_definitions(test1 PUBLIC "-DRES=\"${resource_path}\"")