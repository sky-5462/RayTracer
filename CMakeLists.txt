cmake_minimum_required (VERSION 3.17)

set(CMAKE_CXX_STANDARD 17)

project ("RayTracer")

add_executable(RayTracer "src/main.cpp" "src/RayTracer.cpp" "src/BVH.cpp" "src/Triangle.cpp" "src/Camera.cpp" "src/Texture.cpp" "src/ImageIO.cpp")
target_include_directories(RayTracer PUBLIC "include")
target_link_directories(RayTracer PUBLIC "lib")
target_link_libraries(RayTracer PUBLIC assimp-vc142-mt PUBLIC tbb)
