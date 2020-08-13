cmake_minimum_required(VERSION 3.5)
# Project
set(PROJECT Boris_System_Brain_v2)
project(${PROJECT})
#SET(CMAKE_BUILD_TYPE RelWithDebInfo) # Release, RelWithDebInfo
SET(CMAKE_VERBOSE_MAKEFILE ON)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

#flags
set(CMAKE_CXX_STANDARD 14)
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
find_package(gflags REQUIRED)
find_package(Glog REQUIRED)
find_package(GDAL REQUIRED)

message(STATUS "All default dependencies ready to go !")


# Include dirs
INCLUDE_DIRECTORIES(
        ${Eigen_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${GLOG_INCLUDE_DIRS}
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
        ${GLOG_LIBRARIES}
        ${Protobuf_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        ${PCL_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        ${GDAL_LIBRARY}
        )

# setup params for the build
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/Bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# add all coponments and ceate all libraries
add_subdirectory(src/aslam)
add_subdirectory(src/utils/EXIF_Parsing)

LIST(APPEND LINK_LIBS_ASLAM
        aslam_lib_common
        aslam_lib_pipeline
        aslam_lib_cameras
        aslam_lib_frames
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
        ${BorisBrain_RTMapping_utils}
        )

#######################
# LIBRARIES Dense pcl #
#######################

add_library(BorisBrain_RTMapping_DensePcl
            src/dense_pcl/stereo.cpp
            src/dense_pcl/densifier.cpp
            src/dense_pcl/block-matching-sgbm.cpp
            src/dense_pcl/block-matching-bm.cpp
            src/dense_pcl/rectifier.cpp)

target_link_libraries(BorisBrain_RTMapping_DensePcl
        ${LINK_LIBS_EXTERNAL}
        ${LINK_LIBS_ASLAM}
        BorisBrain_RTMapping_io
        )

######################
# LIBRARIES Grid Map #
######################

add_library(BorisBrain_RTMapping_GridMap
        src/grid_map/aerial-mapper-grid-map.cc)

target_link_libraries(BorisBrain_RTMapping_GridMap
        ${LINK_LIBS_EXTERNAL}
        ${LINK_LIBS_ASLAM}
        BorisBrain_RTMapping_utils
        )


###############
# LIBRARY DSM #
###############

add_library(BorisBrain_RTMapping_DSM
        src/dsm/dsm.cc)

target_link_libraries(BorisBrain_RTMapping_DSM
        ${LINK_LIBS_EXTERNAL}
        ${LINK_LIBS_ASLAM}
        BorisBrain_RTMapping_utils
        BorisBrain_RTMapping_GridMap
        )

#################################################


LIST(APPEND LINK_LIBS_BORIS_RT
        BorisBrain_RTMapping_DensePcl
        BorisBrain_RTMapping_io
        )

LIST(APPEND test_src
        src/programs/BorisBrainV2_RT.cpp)

##########################################################################################
# PROGRAMS
##########################################################################################
ADD_EXECUTABLE(BorisBrainV2 ${test_src})
TARGET_LINK_LIBRARIES(BorisBrainV2 ${LINK_LIBS_EXTERNAL} ${LINK_LIBS_BORIS_RT})