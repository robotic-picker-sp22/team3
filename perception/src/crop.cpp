#include "perception/crop.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/filters/crop_box.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/common.h"
#include "pcl_ros/transforms.h"
#include "tf/transform_listener.h"
typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {
    Cropper::Cropper(const ros::Publisher& pub) : pub_(pub) {
        // tf::TransformListener listener;
        // if (!getenv('ROBOT')) {
        //     ROS_INFO('Transforming crop...');
        //     listener.waitForTransform('base_link', 'head_camera_depth_frame');
        //     listener.lookupTransform('base_link', 'head_camera_depth_frame', ros::Time(0), transform_);
        // } else {
        //     listener.waitForTransform('base_link', 'base_link');
        //     listener.lookupTransform('base_link', 'base_link', ros::Time(0), transform_);
        // }
    }

    void transform_cloud(const sensor_msgs::PointCloud2& cloud_in, sensor_msgs::PointCloud2& cloud_out) {
        tf::TransformListener tf_listener;                                                 
        // tf_listener.waitForTransform("base_link", cloud_in.header.frame_id,                     
        //                             ros::Time(0), ros::Duration(5.0));
        tf_listener.waitForTransform("ar_marker_15", cloud_in.header.frame_id,                     
                                    ros::Time(0), ros::Duration(5.0));                       
        tf::StampedTransform transform;                                                       
        try {                                                                                 
            // tf_listener.lookupTransform("base_link", cloud_in.header.frame_id,                    
            //                             ros::Time(0), transform);                       
            tf_listener.lookupTransform("ar_marker_15", cloud_in.header.frame_id,                    
                                        ros::Time(0), transform);                           
        } catch (tf::LookupException& e) {                                                    
            std::cerr << e.what() << std::endl;                                                 
            return;                                                                           
        } catch (tf::ExtrapolationException& e) {                                             
            std::cerr << e.what() << std::endl;                                                 
            return;                                                         
        }                                                                                                                                                                     
        // pcl_ros::transformPointCloud("base_link", transform, cloud_in, cloud_out);
        pcl_ros::transformPointCloud("ar_marker_15", transform, cloud_in, cloud_out);
    }

    void Cropper::Callback(const sensor_msgs::PointCloud2& msg) {
        PointCloudC::Ptr cropped_cloud(new PointCloudC());
        sensor_msgs::PointCloud2 transformed_cloud;
        transform_cloud(msg, transformed_cloud);  

        PointCloudC::Ptr cloud(new PointCloudC()); 
        pcl::fromROSMsg(transformed_cloud, *cloud);     
        ROS_INFO("Got point cloud with %ld points", cloud->size());                                           
    
        double min_x, min_y, min_z, max_x, max_y, max_z;
        ros::param::param("crop_min_x", min_x, 0.3);
        ros::param::param("crop_min_y", min_y, -1.0);
        ros::param::param("crop_min_z", min_z, 0.74);
        ros::param::param("crop_max_x", max_x, 0.9);
        ros::param::param("crop_max_y", max_y, 1.0);
        ros::param::param("crop_max_z", max_z, 1.5);
        Eigen::Vector4f min_pt(min_x, min_y, min_z, 1);
        Eigen::Vector4f max_pt(max_x, max_y, max_z, 1);
        
        pcl::CropBox<PointC> crop;
        crop.setInputCloud(cloud);
        crop.setMin(min_pt);
        crop.setMax(max_pt);
        crop.filter(*cropped_cloud);
        ROS_INFO("Cropped to %ld points", cropped_cloud->size());
        sensor_msgs::PointCloud2 msg_out;
        pcl::toROSMsg(*cropped_cloud, msg_out);
        pub_.publish(msg_out);



        PointC min_pcl;
        PointC max_pcl;
        pcl::getMinMax3D<PointC>(*cropped_cloud, min_pcl, max_pcl);
        ROS_INFO("min x: %f, max x: %f", min_pcl.x, max_pcl.x);
        ROS_INFO("min y: %f, max y: %f", min_pcl.y, max_pcl.y);
        ROS_INFO("x: %f, y: %f", (max_pcl.x - min_pcl.x) / 2 + min_pcl.x, (max_pcl.y - min_pcl.y) / 2 + min_pcl.y);
    }
}