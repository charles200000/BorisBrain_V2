/*
 *    Filename: stereo.cpp
 *  Created on: Nov 01, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "stereo.h"

namespace stereo {

Stereo::Stereo(const std::shared_ptr<aslam::NCamera> ncameras,
               const Settings& settings,
               const BlockMatchingParameters& block_matching_params)
    : ncameras_(ncameras),
      settings_(settings),
      first_frame_(true) {
  CHECK(ncameras_);

  cv::Size image_resolution;
  image_resolution.width = (ncameras_->getCamera(kFrameIdx).imageWidth());
  image_resolution.height = (ncameras_->getCamera(kFrameIdx).imageHeight());

  // Undistorter.
  static constexpr float undistortion_alpha = 1.0;
  static constexpr float undistortion_scale = 1.0;
  undistorter_ = aslam::createMappedUndistorter(
      ncameras_->getCamera(kFrameIdx), undistortion_alpha, undistortion_scale,
      aslam::InterpolationMethod::Linear);

  rectifier_.reset(new Rectifier(image_resolution));
  densifier_.reset(new Densifier(block_matching_params, image_resolution));

  // Set the calibration matrix K (assumed to be constant for all frames).
  aslam::PinholeCamera::ConstPtr pinhole_camera_ptr =
      std::dynamic_pointer_cast<const aslam::PinholeCamera>(
          ncameras_->getCameraShared(kFrameIdx));
  stereo_rig_params_.K = pinhole_camera_ptr->getCameraMatrix();

  // Set the camera-IMU transformation (assumed to be constant for all frames).
  T_B_C_ = ncameras_->get_T_C_B(kFrameIdx).inverse();

  // Define the point cloud message.
    std::uint8_t FLOAT32 = 7;
    std::uint8_t UINT32  = 6;

    pointCloud.header.frame_id = "world";
    pointCloud.height = image_resolution.height;
    pointCloud.width = image_resolution.width;
    pointCloud.fields.resize(4);

    pointCloud.fields[0].name = "x";
    pointCloud.fields[0].offset = 0;
    pointCloud.fields[0].count = 1;
    pointCloud.fields[0].datatype = FLOAT32;

    pointCloud.fields[1].name = "y";
    pointCloud.fields[1].offset = 4;
    pointCloud.fields[1].count = 1;
    pointCloud.fields[1].datatype = FLOAT32;

    pointCloud.fields[2].name = "z";
    pointCloud.fields[2].offset = 8;
    pointCloud.fields[2].count = 1;
    pointCloud.fields[2].datatype = FLOAT32;

    pointCloud.fields[3].name = "rgb";
    pointCloud.fields[3].offset = 12;
    pointCloud.fields[3].count = 1;
    pointCloud.fields[3].datatype = UINT32;

  pointCloud.point_step = 16;
  pointCloud.row_step =
      pointCloud.point_step * pointCloud.width;
  pointCloud.data.resize(pointCloud.row_step *
                                   pointCloud.height);
  pointCloud.is_dense = false;

  /*
  pub_point_cloud_ = node_handle_.advertise<sensor_msgs::PointCloud2>(
      "/planar_rectification/point_cloud", 100); */

}

void Stereo::addFrames(const Poses& T_G_Bs, const Images& images,
                       AlignedType<std::vector, Eigen::Vector3d>::type* point_cloud,
                       std::vector<int>* point_cloud_intensities) {
  CHECK(point_cloud);
  point_cloud->clear();
  if (point_cloud_intensities) {
    point_cloud_intensities->clear();
  }

  size_t skip = 0u;
  for (size_t i = 0u; i < images.size(); ++i) {
    if (++skip % settings_.use_every_nth_image == 0) {
      LOG(INFO) << "Processing image " << i << "/" << images.size();
      AlignedType<std::vector, Eigen::Vector3d>::type point_cloud_tmp;
      std::vector<int> point_cloud_intensities_tmp;
      addFrame(T_G_Bs[i], images[i], &point_cloud_tmp,
               &point_cloud_intensities_tmp);

      // Append 3D points and (optional) corresponding pixel intensities.
      CHECK(point_cloud_tmp.size() == point_cloud_intensities_tmp.size());
      point_cloud->insert(point_cloud->end(), point_cloud_tmp.begin(),
                          point_cloud_tmp.end());
      if (point_cloud_intensities) {
        point_cloud_intensities->insert(point_cloud_intensities->end(),
                                        point_cloud_intensities_tmp.begin(),
                                        point_cloud_intensities_tmp.end());
      }
    }
  }
}

void Stereo::addFrame(const Pose& T_G_B, const Image& image_raw,
                      AlignedType<std::vector, Eigen::Vector3d>::type* point_cloud,
                      std::vector<int>* point_cloud_intensities) {
  CHECK(point_cloud);
  // SGBM/BM blockmatching requires images of type CV_8UC1.
  cv::Mat image;
  if (image_raw.type() == CV_8UC1) {
  } else if (image_raw.type() == CV_8UC3) {
    cv::cvtColor(image_raw, image, CV_RGB2GRAY);
  } else {
    LOG(FATAL) << "Image type not supported";
  }
  CHECK(image.type() == CV_8UC1);

  if (first_frame_) {
    // Prepare the first/left frame of the stereo pair.
    stereo_rig_params_.t_G_C1 = (T_G_B * T_B_C_).getPosition();
    stereo_rig_params_.R_G_C1 = (T_G_B * T_B_C_).getRotationMatrix();
    image_distorted_1_ = image_raw;
    first_frame_ = false;
    return;
  }
  // Prepare the second/right frame of the stereo pair.
  stereo_rig_params_.t_G_C2 = (T_G_B * T_B_C_).getPosition();
  stereo_rig_params_.R_G_C2 = (T_G_B * T_B_C_).getRotationMatrix();
  image_distorted_2_ = image_raw;

  processStereoFrame(point_cloud, point_cloud_intensities);

  // Prepare next iteration: The previously second/right frame is
  // now the first/left frame.
  stereo_rig_params_.t_G_C1 = stereo_rig_params_.t_G_C2;
  stereo_rig_params_.R_G_C1 = stereo_rig_params_.R_G_C2;
  image_distorted_1_ = image_distorted_2_;
}


int pcl_i = 0;

void Stereo::processStereoFrame(
    AlignedType<std::vector, Eigen::Vector3d>::type* point_cloud,
    std::vector<int>* point_cloud_intensities) {
  // 1. Undistort raw images.
  cv::Mat image_undistorted_1 = image_distorted_1_;
  cv::Mat image_undistorted_2 = image_distorted_2_;
  if (settings_.images_need_undistortion) {
    undistortRawImages(image_distorted_1_, image_distorted_2_,
                       &image_undistorted_1, &image_undistorted_2);
  }

  // 2. Rectify undistorted images.
  RectifiedStereoPair rectified_stereo_pair;
  rectifier_->rectifyStereoPair(stereo_rig_params_, image_undistorted_1,
                                image_undistorted_2, &rectified_stereo_pair);

  // 3. Compute disparity map based on rectified images.
  DensifiedStereoPair densified_stereo_pair;
  densifier_->computeDisparityMap(rectified_stereo_pair,
                                  &densified_stereo_pair);

  // 4. Compute point cloud.
  pointCloud.data.clear();
  pointCloud.data.resize(pointCloud.row_step *
                                   pointCloud.height);
  //ros::Time timestamp = ros::Time::now();
  uint64_t p1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
  pointCloud.header.stamp = p1;
  densifier_->computePointCloud(stereo_rig_params_, rectified_stereo_pair,
                                &densified_stereo_pair, pointCloud);
  *point_cloud = densified_stereo_pair.point_cloud_eigen;
  if (point_cloud_intensities) {
    *point_cloud_intensities = densified_stereo_pair.point_cloud_intensities;
  }
  
  // 5. Publish the point cloud.
  pcl::PointCloud<pcl::PointXYZRGB> pointCloud_PCL;
  pcl::fromPCLPointCloud2(pointCloud, pointCloud_PCL);
  std::string fileName = "pcd_" + std::to_string(pcl_i) + ".pcd";
  pcl_i++;
  pcl::io::savePCDFileASCII (fileName, pointCloud_PCL);
  //pub_point_cloud_.publish(pointCloud);
  //ros::spinOnce();

  // [Optional] Visualize rectification.
  if (settings_.show_rectification) {
    visualizeRectification(image_undistorted_1, image_undistorted_2,
                           rectified_stereo_pair.image_left,
                           rectified_stereo_pair.image_right);
  }
}

void Stereo::undistortRawImages(const cv::Mat& image_distorted_1,
                                const cv::Mat& image_distorted_2,
                                cv::Mat* image_undistorted_1,
                                cv::Mat* image_undistorted_2) const {
  CHECK_NOTNULL(image_undistorted_1);
  CHECK_NOTNULL(image_undistorted_2);
  // Undistort the raw images.
  undistorter_->processImage(image_distorted_1, image_undistorted_1);
  undistorter_->processImage(image_distorted_2, image_undistorted_2);
}

void Stereo::visualizeRectification(
    const cv::Mat& image_undistorted_1, const cv::Mat& image_undistorted_2,
    const cv::Mat& image_undistorted_rectified_1,
    const cv::Mat& image_undistorted_rectified_2) const {
  cv::Mat images_undistorted, images_undistorted_rectified, all_images;
  cv::hconcat(image_undistorted_1, image_undistorted_2, images_undistorted);
  cv::hconcat(image_undistorted_rectified_1, image_undistorted_rectified_2,
              images_undistorted_rectified);
  for (int i = 50; i < image_undistorted_1.rows; i = i + 50) {
    cv::line(images_undistorted_rectified, cv::Point(0, i),
             cv::Point(image_undistorted_1.cols * 2, i),
             cv::Scalar(255, 255, 255));
  }
  cv::vconcat(images_undistorted, images_undistorted_rectified, all_images);
  cv::imshow("top: undistorted, bottom: undistorted + rectified",
             all_images);
  cv::waitKey(1);
}

}  // namespace stereo
