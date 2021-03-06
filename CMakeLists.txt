cmake_minimum_required(VERSION 3.0.0)

project(robust_fusion)
# 设置编译选项
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11 -fopenmp")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
# 设置项目路径
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
# 寻找package
find_package(OpenCV REQUIRED)
find_package(PythonLibs REQUIRED)

message("OpenCV Version: ${OpenCV_VERSION}")

# 设置头文件包含路径
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    "/usr/include/eigen3"
    ${OpenCV_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
    )
# 设置第三方共享库
set(THIRD_PARTY_LIBS
    ${OpenCV_LIBS}
    ${PYTHON_LIBRARIES}
    )

add_subdirectory(${PROJECT_SOURCE_DIR}/src)
add_subdirectory(${PROJECT_SOURCE_DIR}/examples)
