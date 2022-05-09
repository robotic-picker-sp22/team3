#include "perception/crop.h"
#include "perception/downsample.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "perception/segmentation.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_demo");
  ros::NodeHandle nh;

  // Cropper
  ros::Publisher crop_pub =
      nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
  perception::Cropper cropper(crop_pub);
  ros::Subscriber crop_sub =
      nh.subscribe("cloud_in", 1, &perception::Cropper::Callback, &cropper);
    
  // Downsampler
  ros::Publisher downsample_pub =
      nh.advertise<sensor_msgs::PointCloud2>("downsampled_cloud", 1, true);
  perception::DownSampler downsampler(downsample_pub);
  ros::Subscriber downsample_sub =
      nh.subscribe("cropped_cloud", 1, &perception::DownSampler::Callback, &downsampler);
  
  // Segmenter
  ros::Publisher segment_pub =
      nh.advertise<visualization_msgs::Marker>("segment_cloud", 1, true);
  perception::Segmenter segmenter(segment_pub);
  ros::Subscriber segment_sub =
      nh.subscribe("downsampled_cloud", 1, &perception::Segmenter::Callback, &segmenter);
  
  
  ros::spin();
  return 0;
}