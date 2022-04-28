#include "perception/downsample.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {
    DownSampler::DownSampler(const ros::Publisher& pub) : pub_(pub) {}

    void DownSampler::Callback(const sensor_msgs::PointCloud2& msg) {
        PointCloudC::Ptr cloud(new PointCloudC());
        pcl::fromROSMsg(msg, *cloud);
        pcl::VoxelGrid<PointC> vox;
        vox.setInputCloud(cloud);

        PointCloudC::Ptr downsampled_cloud(new PointCloudC());

        double voxel_size;
        ros::param::param("voxel_size", voxel_size, 0.01);
        vox.setLeafSize(voxel_size, voxel_size, voxel_size);
        vox.filter(*downsampled_cloud);
        ROS_INFO("Filtered to %ld points", downsampled_cloud->size());
        sensor_msgs::PointCloud2 msg_out;
        pcl::toROSMsg(*downsampled_cloud, msg_out);
        pub_.publish(msg_out);
    }
}