/*
 *    Filename: densifier.h
 *  Created on: Nov 01, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef DENSIFIER_H_
#define DENSIFIER_H_

// NON-SYSTEM
#include <eigen3/Eigen/Core>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


// Not using ROS
//#include <ros/node_handle.h>

#include "block-matching-bm.h"
#include "block-matching-sgbm.h"
#include "common.h"

namespace stereo {

class Densifier {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Densifier(const BlockMatchingParameters& block_matching_params,
            const cv::Size& image_dimension);

  void computePointCloud(const StereoRigParameters& stereo_pair,
                         const RectifiedStereoPair& rectified_stereo_pair,
                         DensifiedStereoPair* densified_stereo_pair,
                         pcl::PCLPointCloud2& point_cloud) const;

  inline void computeDisparityMap(
      const RectifiedStereoPair& rectified_stereo_pair,
      DensifiedStereoPair* densified_stereo_pair) const {
    block_matcher_->computeDisparityMap(rectified_stereo_pair,
                                        densified_stereo_pair);
  }

 private:
  static constexpr int kPositionX = 0;
  static constexpr int kPositionY = 4;
  static constexpr int kPositionZ = 8;
  static constexpr int kPositionIntensity = 12;
  static constexpr float kInvalidPoint =
      std::numeric_limits<float>::quiet_NaN();
  static constexpr int kMaxInvalidDisparity = 1;
  static constexpr size_t kSizeOfFloat = sizeof(float);
  static constexpr size_t kSizeOfUint32T = sizeof(uint32_t);

  std::unique_ptr<BlockMatchingBase> block_matcher_;
  const cv::Size image_resolution_;
};

}  // namespace stereo

#endif  // DENSIFIER_H_
