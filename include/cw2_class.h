// include guards, prevent .h file being defined multiple times (linker error)
#ifndef cw2_CLASS_H_
#define cw2_CLASS_H_

// system includes
#include <ros/ros.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <sstream>
#include <iomanip>

// Service from the spawner package (Task 1)
#include "cw2_world_spawner/Task1Service.h"

// Service from the spawner package (Task 2)
#include "cw2_world_spawner/Task2Service.h"

// Service from the spawner package (Task 3)
#include "cw2_world_spawner/Task3Service.h"

// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
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

// custom header files
#include "cw2_perception.h"

class cw2
{
public:
    /* ----- class member functions ----- */

    // constructor
    cw2(ros::NodeHandle nh);

    // service callbacks for tasks 1, 2, and 3
    bool t1_callback(cw2_world_spawner::Task1Service::Request &request,
                     cw2_world_spawner::Task1Service::Response &response);

    bool t2_callback(cw2_world_spawner::Task2Service::Request &request,
                     cw2_world_spawner::Task2Service::Response &response);

    bool t3_callback(cw2_world_spawner::Task3Service::Request &request,
                     cw2_world_spawner::Task3Service::Response &response);

    /* ----- class member variables ----- */

    // Node handle
    ros::NodeHandle nh_;

    // Service servers
    ros::ServiceServer t1_service_;
    ros::ServiceServer t2_service_;
    ros::ServiceServer t3_service_;

    // MoveIt interfaces
    moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
    moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    // Point cloud subscriber
    ros::Subscriber pointcloud_sub_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr_;

    // PCL marker visualization publisher
    ros::Publisher marker_pub_;

    // TF components
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;

    // Robot frames & constants
    std::string base_frame_ = "panda_link0";
    double gripper_open_ = 0.08;   // 80 mm open width.
    double gripper_closed_ = 0.01; // 10 mm closed width.

    // Default pose (home pose) for resets
    geometry_msgs::PoseStamped default_pose_;

    // PCL related functions
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

    // Publishes cluster markers for visualization
    void publishClusterMarkers(
        const std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &clusters);

    // ObjectInfo structure to hold information about the object
    struct ObjectInfo
    {
        geometry_msgs::PoseStamped pose;                 // Object position in world frame
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster; // Point cloud
        std::string shapeType;                           // "nought", "cross", "obstacle", "basket"
        double orientation;                              // Orientation in degrees
        cv::Mat binaryImage;                             // 2D projection for classification
        bool isAccessible;                               // Whether robot can reach it
        int viewCount;                                   // Number of times observed
        std::string uniqueId;                            // Generated unique ID
    };

    // Global object tracking collections
    std::vector<ObjectInfo> all_objects_;
    std::vector<visualization_msgs::Marker> all_markers_;
    int next_marker_id_ = 0;

    // Helper functions
    bool isNewObject(const Eigen::Vector4f &centroid,
                     const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cluster,
                     int &existing_index,
                     double distance_threshold = 0.15);

    // Add or update object in the collection
    void addOrUpdateObject(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cluster,
        const std::string &shapeType,
        double orientation = 0.0);

    // Publish all object markers for visualization
    void publishAllObjectMarkers();

    // Add objects to planning scene
    void addObstacleToPlanningScene(
        const ObjectInfo &obj);

    // Clear planning scene
    void clearPlanningScene();

    // Select the best object to pick based on multiple criteria
    const ObjectInfo *selectBestObject(
        const std::string &shape_type);

    std::vector<std::pair<const ObjectInfo *, double>> selectBestObjects(
        const std::string &shape_type,
        int max_candidates = 4);

    // Calculate minimum distance from an object to any obstacle
    double calculateObjectClearance(
        const ObjectInfo &obj);

    // Calculate how well aligned the object is with the gripper
    double calculateOrientationScore(
        const ObjectInfo &obj);

    // Calculate a score based on object size
    double calculateSizeScore(
        const ObjectInfo &obj);

    // Publish a grasp pose marker
    void publishGraspPoseMarker(
        const geometry_msgs::PoseStamped &grasp_pose);

    // Publish debug visualization of clusters during processing
    void publishDebugClusters(
        const std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &clusters);
};

#endif // end of include guard for cw2_CLASS_H_
