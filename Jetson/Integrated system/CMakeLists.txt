cmake_minimum_required(VERSION 3.29)
project(YOLO_CUDA CUDA CXX)

# Set CUDA and C++ standards
set(CMAKE_CUDA_STANDARD 20)
set(CMAKE_CXX_STANDARD 11)

# Header files (if any)
set(Headers ModelLoadingHelper.h)

# Add the executable, combining CUDA and C++ files
add_executable(YOLO_CUDA 
    ${PROJECT_SOURCE_DIR}/main.cu
    ModelLoadingHelper.cu
    Conv2D.cu
    MaxPool2D.cu
    FullyConnected.cu
    yolo.cu
    robotController.cu
    serialHelper.cpp        # C++ source files
    positionController.cpp
    aiHelperUtils.cpp
    pid.cpp
    visualServoing.cpp
    navigationSystem.cpp
)

# Enable separable compilation for CUDA
set_target_properties(YOLO_CUDA PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
)

# Include directories
include_directories("${CMAKE_SOURCE_DIR}")
include_directories("/usr/include/opencv4")  # OpenCV include path

# Find and link OpenCV package
find_package(OpenCV REQUIRED)
target_link_libraries(YOLO_CUDA ${OpenCV_LIBS})

# Find and link Boost
find_package(Boost REQUIRED COMPONENTS system)  # Add more components if needed
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(YOLO_CUDA Boost::boost ${Boost_LIBRARIES})

# Link pthread library (for multithreading)
target_link_libraries(YOLO_CUDA pthread)

# Print the OpenCV version for confirmation (optional)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

# Additional compiler flags for C++
target_compile_options(YOLO_CUDA PRIVATE -Wall -Wextra)  # Equivalent to -Wall -Wextra from the Makefile
