cmake_minimum_required(VERSION 3.1)
project(pc)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BUILD_TYPE Debug)  # Release

SET(ENV "surui")

if (ENV STREQUAL "mac")
    message("-- Environment: MacOS")
    # opencv
    set(OPENCV_INCLUDE_DIR /Users/surui/CLionProjects/LIBRARY/opencv@3/3.3.1/include)
    set(OPENCV_LIBRARY_DIR /Users/surui/CLionProjects/LIBRARY/opencv-3.3.1/release/lib)
    set(OPENCV_LIBS -lopencv_core -lopencv_features2d -lopencv_highgui -lopencv_imgproc -lopencv_ml -lopencv_imgcodecs)
else()
    message("-- Environment: arch-linux")
    # OpenMP
    FIND_PACKAGE(OpenMP)
    if(OPENMP_FOUND)
        message("OPENMP FOUND")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    endif()

    # opencv
    set(OPENCV_INCLUDE_DIR /Users/surui/CLionProjects/LIBRARY/opencv@3/3.3.1/include)
    set(OPENCV_LIBRARY_DIR /Users/surui/CLionProjects/LIBRARY/opencv-3.3.1/release/lib)
    set(OPENCV_LIBS -lopencv_core -lopencv_features2d -lopencv_highgui -lopencv_imgproc -lopencv_ml -lopencv_imgcodecs)

endif ()



include_directories(${OPENCV_INCLUDE_DIR})
link_directories(${OPENCV_LIBRARY_DIR})

# boost
find_package(Boost REQUIRED COMPONENTS)

# pcl
find_package( PCL REQUIRED)
add_definitions( ${PCL_DEFINITIONS} )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})


set(SOURCE_FILES main.cpp camera_parameter.h pixel_tool.cpp pixel_tool.h)
add_executable(pcd_restore ${SOURCE_FILES})
#target_link_libraries(pc_restore ${OPENCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ncnn)
message( ${PCL_LIBRARIES} )
target_link_libraries(pcd_restore ${OPENCV_LIBS} ${PCL_LIBRARIES})

