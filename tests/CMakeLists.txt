set(resource_path ${PROJECT_SOURCE_DIR}/examples)

link_libraries(meshlib igl::core igl::opengl_glfw igl::opengl_glfw_imgui yaml-cpp)
#add_compile_definitions("-DRES=\"${resource_path}\"")

add_executable(test1 sample_main.cpp ../eigen.natvis)
#target_link_libraries(test1 PUBLIC meshlib igl::core igl::opengl_glfw igl::opengl_glfw_imgui yaml-cpp)
target_compile_definitions(test1 PUBLIC "-DRES=\"${resource_path}\"")

add_executable(test2 small_example.cpp ../eigen.natvis)
#target_link_libraries(test2 PUBLIC meshlib igl::core igl::opengl_glfw igl::opengl_glfw_imgui yaml-cpp)
#target_compile_definitions(test2 PUBLIC "-DRES=\"${resource_path}\"")

add_executable(closest_test1 closest_test.cpp ../eigen.natvis)
