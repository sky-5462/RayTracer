cmake_minimum_required (VERSION 3.17)

set(CMAKE_CXX_STANDARD 17)

project ("RayTracer")

add_executable(RayTracer "src/main.cpp" "src/RayTracer.cpp" "src/BVH.cpp" "src/Triangle.cpp" "src/Camera.cpp")
target_include_directories(RayTracer PUBLIC ./include)
target_compile_options(RayTracer PUBLIC "/arch:AVX2")
target_compile_options(RayTracer PUBLIC "$<$<CONFIG:RELEASE>:/GL>")
target_link_options(RayTracer PUBLIC "$<$<CONFIG:RELEASE>:/LTCG>")

set(INSTALL_PATH "${CMAKE_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE}")
install(TARGETS RayTracer RUNTIME DESTINATION ${INSTALL_PATH})