project(yolov5ncnn)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} g -Wextra -fexceptions")
cmake_minimum_required(VERSION 3.4.1)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fexceptions")
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99 -fvisibility=hidden -fomit-frame-pointer -fstrict-aliasing -ffunction-sections -fdata-sections -ffast-math -flax-vector-conversions")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fvisibility=hidden -fvisibility-inlines-hidden -fomit-frame-pointer -fstrict-aliasing -ffunction-sections -fdata-sections -ffast-math -fno-rtti -fno-exceptions -flax-vector-conversions")
set (CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -Wl,--gc-sections")


set(ncnn_DIR /home/zkl/code/android/ncnn-20210525-android-vulkan/${ANDROID_ABI}/lib/cmake/ncnn)
find_package(ncnn REQUIRED)


set(OpenCV_DIR /home/zkl/Downloads/opencv-mobile-3.4.15-android/sdk/native/jni)
find_package(OpenCV REQUIRED)

add_library(yolov5ncnn SHARED yolov5ncnn_jni.cpp reader.cpp)

target_link_libraries(yolov5ncnn
        ncnn
        ${OpenCV_LIBS}
        jnigraphics
        -static-openmp -fopenmp)
