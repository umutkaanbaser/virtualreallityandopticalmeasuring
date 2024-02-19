cmake_minimum_required(VERSION 3.16)
project(image_reconstruct)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} )

add_subdirectory(include)

add_executable(image_reconstruct 
    main.cpp
)
target_link_libraries(image_reconstruct ImageReconstruct cameraSpecs ${OpenCV_LIBS})

