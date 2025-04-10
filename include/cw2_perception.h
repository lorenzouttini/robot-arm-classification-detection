#ifndef cw2_PERCEPTION_H
#define cw2_PERCEPTION_H

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <sstream>
#include <iomanip>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>

// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Dense>

// opencv
#include <opencv2/opencv.hpp>

namespace cw2_perception
{

    // Convert cluster (projected onto XY plane) to a binary image
    cv::Mat clusterToBinaryImage(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cluster,
        double resolution,
        bool save_debug = false);

    // Fetch all clusters from the point cloud
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> getAllClusters(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_ptr,
        tf2_ros::Buffer &tfBuffer,
        double height_min = 0.05,
        double height_max = 1.5);

    // Get the shape orientation from the binary image
    double getShapeOrientation(
        const cv::Mat &binaryImage,
        bool save_debug = false);

    // Check if the cluster is an obstacle
    bool isObstacle(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cluster);

    // Check if the cluster is a basket
    bool isBasket(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cluster,
        double similarity_threshold = 0.95);

    // Check if the cluster is a nought
    bool isNought(
        const cv::Mat &binaryImage,
        double hole_ratio_threshold = 0.1);

    // Transform the point cloud to Panda Base frame
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformCloudToFrame(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_in,
        const std::string &target_frame,
        tf2_ros::Buffer &tfBuffer);

    void clearDebugImages();

    double detectTableHeight(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
        tf2_ros::Buffer &tfBuffer);

}

#endif // end of include guard for PERCEPTION_H