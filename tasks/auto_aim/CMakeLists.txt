cmake_minimum_required(VERSION 3.16)

set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

option(USE_TENSORRT "Enable TensorRT inference backend (Jetson only)" OFF)

add_subdirectory(planner/tinympc)

set(AUTO_AIM_SOURCES
    armor.cpp
    classifier.cpp
    detector.cpp
    solver.cpp
    aimer.cpp
    target.cpp
    tracker.cpp
    voter.cpp
    shooter.cpp
    yolo.cpp
    yolos/yolov5.cpp
    yolos/yolov8.cpp
    yolos/yolo11.cpp
    multithread/commandgener.cpp
    multithread/mt_detector.cpp
    planner/planner.cpp
)

if(USE_TENSORRT)
    find_package(CUDA REQUIRED)
    list(APPEND AUTO_AIM_SOURCES yolos/yolov5_trt.cpp)
endif()

add_library(auto_aim OBJECT ${AUTO_AIM_SOURCES})

if(USE_TENSORRT)
    target_compile_definitions(auto_aim PUBLIC USE_TENSORRT)
    target_include_directories(auto_aim PRIVATE
        /usr/include/aarch64-linux-gnu
        ${CUDA_INCLUDE_DIRS}
    )
    target_link_libraries(auto_aim io openvino::runtime tinympcstatic nvinfer ${CUDA_LIBRARIES})
else()
    target_link_libraries(auto_aim io openvino::runtime tinympcstatic)
endif()