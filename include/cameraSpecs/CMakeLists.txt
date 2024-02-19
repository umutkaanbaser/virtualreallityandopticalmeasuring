add_library(cameraSpecs STATIC 
  cameraSpecs.cpp
)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} )

target_link_libraries(cameraSpecs ${OpenCV_LIBS})

target_include_directories(cameraSpecs PUBLIC "./")

