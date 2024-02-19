add_library(ImageReconstruct STATIC 
ImageReconstruct.cpp
)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} )

target_link_libraries(ImageReconstruct cameraSpecs Eigen3::Eigen  ${OpenCV_LIBS})

target_include_directories(ImageReconstruct PUBLIC "./")

