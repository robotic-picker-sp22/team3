#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/segmentation/extract_clusters.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "visualization_msgs/Marker.h"
#include "perception/object.h"
#include "perception/object_recognizer.h"

namespace perception {

// Add function definitions here later
// void Cloud2Indices(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices *indices);
// void SegmentBinObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
//                            std::vector<pcl::PointIndices>* indices);
// void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
//                                geometry_msgs::Pose* pose,
//                                geometry_msgs::Vector3* dimensions);

// Does a complete bin segmentation pipeline.
//
// Args:
//  cloud: The point cloud with the bin and the objects in it.
//  objects: The output objects.
void SegmentObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                          std::vector<Object>* objects);
void SegmentObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                          std::vector<Object>* objects, int algo);

class Segmenter {
 public:
  Segmenter(const ros::Publisher& marker_pub);

  Segmenter(const ros::Publisher& marker_pub,
            const ros::Publisher& object_pub,
            const ObjectRecognizer& recognizer);

  void Callback(const sensor_msgs::PointCloud2& msg);
  

 private:
  ros::Publisher marker_pub_;
  ros::Publisher object_pub_;
  ObjectRecognizer recognizer_;
};
}  // namespace perception