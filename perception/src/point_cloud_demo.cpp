#include "perception/crop.h"
#include "perception/downsample.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "perception/segmentation.h"

int main(int argc, char** argv) {

  if (argc < 2) {
    ROS_INFO("Usage: rosrun perception point_cloud_demo DATA_DIR");
    ros::spinOnce();
  }
  std::string data_dir(argv[1]);

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
  
  // Create the object recognizer.
  std::vector<perception_msgs::ObjectFeatures> dataset;
  perception::LoadData(data_dir, &dataset);
  perception::ObjectRecognizer recognizer(dataset);


  // Segmenter
  // TODO: figure out the publishers for the segmenter
  ros::Publisher points_pub =
      nh.advertise<visualization_msgs::Marker>("points_pub", 1, true);
  ros::Publisher segment_pub =
      nh.advertise<visualization_msgs::Marker>("segment_cloud", 1, true);
  perception::Segmenter segmenter(points_pub, segment_pub,
                                  recognizer);
  ros::Subscriber segment_sub =
      nh.subscribe("downsampled_cloud", 1, &perception::Segmenter::Callback, &segmenter);
  
  ros::spin();
  return 0;
}