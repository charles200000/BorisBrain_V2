############
# PROTOBUF #
############
set(PROTO_DEFNS ${PROJECT_SOURCE_DIR}/src/aslam/common/proto/aslam/common/id.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})
#############
# LIBRARIES #
#############
add_library(aslam_lib_common
        common/channel.cc
        common/channel-serialization.cc
        common/covariance-helpers.cc
        common/hash-id.cc
        common/reader-first-reader-writer-lock.cc
        common/reader-writer-lock.cc
        common/sensor.cc
        common/statistics.cc
        common/thread-pool.cc
        common/timer.cc
        common/internal/unique-id.cc
        ${PROTO_SRCS}
        ${PROTO_HDRS}
        )

add_library(aslam_lib_cameras
        cameras/camera-3d-lidar.cc
        cameras/camera-factory.cc
        cameras/camera-pinhole.cc
        cameras/camera-unified-projection.cc
        cameras/camera.cc
        cameras/distortion-equidistant.cc
        cameras/distortion-fisheye.cc
        cameras/distortion-radtan.cc
        cameras/distortion.cc
        cameras/ncamera.cc
        cameras/random-camera-generator.cc
        cameras/ncamera-yaml-serialization.cpp
        cameras/camera-yaml-serialization.cpp
        )

add_library(aslam_lib_frames
        frames/visual-frame.cc
        frames/visual-nframe.cc)


add_library(aslam_lib_pipeline
        pipeline/test/convert-maps-legacy.cc
        pipeline/undistorter.cc
        pipeline/undistorter-mapped.cc
        pipeline/visual-npipeline.cc
        pipeline/visual-pipeline-brisk.cc
        pipeline/visual-pipeline-freak.cc
        pipeline/visual-pipeline-null.cc
        pipeline/visual-pipeline.cc
        )

add_library(brisk
        brisk/brisk-descriptor-extractor.cc
        brisk/brisk-feature-detector.cc
        brisk/brisk-layer.cc
        brisk/brisk-opencv.cc
        brisk/brisk-scale-space.cc
        brisk/brute-force-matcher.cc
        brisk/harris-feature-detector.cc
        brisk/harris-score-calculator.cc
        brisk/harris-score-calculator-float.cc
        brisk/harris-scores.cc
        brisk/image-down-sampling.cc
        brisk/pattern-provider.cc
        brisk/vectorized-filters.cc
        brisk/test/image-io.cc
        brisk/timer.cc)

add_library(agast
        agast/agast5-8.cc
        agast/agast5-8-nms.cc
        agast/agast7-12d.cc
        agast/agast7-12d-nms.cc
        agast/agast7-12s.cc
        agast/agast7-12s-nms.cc
        agast/ast-detector.cc
        agast/non-maximum-suppression.cc
        agast/oast9-16.cc
        agast/oast9-16-nms.cc)


#####################
# LIBRARIES LINKING #
#####################

target_link_libraries(agast
        ${OpenCV_LIBS}
        ${GLOG_LIBRARY})

target_link_libraries(aslam_lib_cameras
        aslam_lib_common
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        ${GLOG_LIBRARY}
        ${Protobuf_LIBRARIES}
        ${YAML_CPP_LIBRARIES})

target_link_libraries(brisk
        ${Boost_LIBRARIES}
        ${agast}
        ${GLOG_LIBRARY})

target_link_libraries(aslam_lib_common
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        ${GLOG_LIBRARY}
        ${Protobuf_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        )

target_link_libraries(aslam_lib_frames
        ${OpenCV_LIBS}
        aslam_lib_common
        aslam_lib_cameras
        ${Boost_LIBRARIES}
        ${GLOG_LIBRARY}
        ${Protobuf_LIBRARIES}
        ${YAML_CPP_LIBRARIES})

target_link_libraries(aslam_lib_pipeline
        ${OpenCV_LIBS}
        aslam_lib_common
        aslam_lib_frames
        aslam_lib_cameras
        ${Boost_LIBRARIES}
        ${GLOG_LIBRARY}
        ${Protobuf_LIBRARIES}
        ${YAML_CPP_LIBRARIES})