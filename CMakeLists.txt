cmake_minimum_required(VERSION 3.5)
# Project
set(PROJECT Boris_System_Brain_v2)
project(${PROJECT})
#SET(CMAKE_BUILD_TYPE RelWithDebInfo) # Release, RelWithDebInfo
SET(CMAKE_VERBOSE_MAKEFILE ON)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

#flags
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
#set(CMAKE_CXX_FLAGS "-Wall"
set(CMAKE_CXX_FLAGS " -march=native -Wno-duplicate-decl-specifier -Wno-ignored-qualifiers -Wno-reorder -pthread")

# Versions
set(VERSION_MAJOR 0)
set(VERSION_MINOR 1)
set(VERSION_PATCH 1)


# Add plain cmake packages
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Protobuf REQUIRED)
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread system)
find_package( PCL 1.8 REQUIRED )
#find_package(Glog REQUIRED)

message(STATUS "All default dependencies ready to go !")


# Include dirs
INCLUDE_DIRECTORIES(
        ${Eigen_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${Glog_INCLUDE_DIRS}
        ${YAML_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${Protobuf_INCLUDE_DIRS}

        ${PROJECT_SOURCE_DIR}/include
)
include_directories( ${CSPARSE_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR})

# Set link libraries
LIST(APPEND LINK_LIBS_EXTERNAL
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        ${GLOG_LIBRARY}
        ${Protobuf_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        ${PCL_LIBRARY_DIRS}
        Eigen
        )


# setup params for the build
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/Bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# add all coponments and ceate all libraries
add_subdirectory(src/aslam)

LIST(APPEND LINK_LIBS_ASLAM
        ${aslam_lib_common}
        ${aslam_lib_pipeline}
        ${aslam_lib_cameras}
        ${aslam_lib_frames}
        )

########################
# LIBRARIES Utils + IO #
########################

add_library(BorisBrain_RTMapping_utils
        src/utils/utils-common.cc)
target_link_libraries(BorisBrain_RTMapping_utils
                        ${LINK_LIBS_EXTERNAL}
                        ${LINK_LIBS_ASLAM})

add_library(BorisBrain_RTMapping_io
        src/io/aerial-mapper-io.cc)

target_link_libraries(BorisBrain_RTMapping_io
        ${LINK_LIBS_EXTERNAL}
        ${LINK_LIBS_ASLAM}
        )

#######################
# LIBRARIES Dense pcl #
#######################

add_library(BorisBrain_TRMapping_DensePcl
            src/dense_pcl/stereo.cpp
            src/dense_pcl/densifier.cpp
            src/dense_pcl/block-matching-sgbm.cpp
            src/dense_pcl/block-matching-bm.cpp
            src/dense_pcl/rectifier.cpp)

target_link_libraries(BorisBrain_TRMapping_DensePcl
        ${LINK_LIBS_EXTERNAL}
        ${LINK_LIBS_ASLAM}
        ${BorisBrain_RTMapping_io})


#Create Boris library
#target_link_libraries(BorisBrain_RTMapping ${LINK_LIBS_EXTERNAL})



##########################################################################################
# PROGRAMS
##########################################################################################
#ADD_EXECUTABLE(BorisBrain src/Boris_Brain.cpp src/EnvProcessing/VisualOdometry/VO_Pipeline_Live.cpp)
#TARGET_LINK_LIBRARIES(BorisBrain BorisBrain_VisualOdometry ${LINK_LIBS_EXTERNAL})