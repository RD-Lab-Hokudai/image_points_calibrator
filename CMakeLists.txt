cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 14)

project(Image_Points_Calibrator)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

if(PCL_FOUND)
    # link_directories must be before add_executable
    include_directories(${PCL_INCLUDE_DIRS})
    #add_definitions(${PCL_DEFINITIONS})
    link_directories(${PCL_LIBRARY_DIRS})
    link_libraries(${PCL_LIBRARIES} ${OpenCV_LIBS})

    add_executable(Calibrator Calibrator.cpp)
endif()