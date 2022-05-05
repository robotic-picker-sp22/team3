#include "perception/segmentation.h"
#include "pcl_conversions/pcl_conversions.h"
#include "visualization_msgs/Marker.h"
#include <pcl/common/common.h>
#include "pcl/filters/extract_indices.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {
void SegmentBinObjects(PointCloudC::Ptr cloud, std::vector<pcl::PointIndices>* indices) {
        double cluster_tolerance;
    int min_cluster_size, max_cluster_size;
    ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.01);
    ros::param::param("ec_min_cluster_size", min_cluster_size, 10);
    ros::param::param("ec_max_cluster_size", max_cluster_size, 10000);

    // pcl::PointIndices inside_bin_indices;
    // Cloud2Indices(cloud, &inside_bin_indices);

    pcl::EuclideanClusterExtraction<PointC> euclid;
    euclid.setInputCloud(cloud);
    // euclid.setIndices(&inside_bin_indices); // << TODO: is cloud already cropped or do we get indices for the crop?
    euclid.setClusterTolerance(cluster_tolerance);
    euclid.setMinClusterSize(min_cluster_size);
    euclid.setMaxClusterSize(max_cluster_size);
    euclid.extract(*indices);

    // Find the size of the smallest and the largest object,
    // where size = number of points in the cluster
    size_t min_size = std::numeric_limits<size_t>::max();
    size_t max_size = std::numeric_limits<size_t>::min();
    for (size_t i = 0; i < indices->size(); ++i) {
        // TODO: implement this
        size_t cluster_size = (*indices)[i].indices.size();
        max_size = std::max(max_size, cluster_size);
        min_size = std::min(min_size, cluster_size);
    }

    ROS_INFO("Found %ld objects, min size: %ld, max size: %ld",
            indices->size(), min_size, max_size);
}

void Cloud2Indices(const PointCloudC::Ptr cloud, pcl::PointIndices* indices) {
    for (size_t i=0; i<indices->indices.size(); ++i) {
        int index = indices->indices[i];
        const PointC& pt = cloud->points[index];
    }
}

void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                               geometry_msgs::Pose* pose,
                               geometry_msgs::Vector3* dimensions) {
    float max_x, max_y, max_z;
    float min_x, min_y, min_z;
    max_x = max_y = max_z = std::numeric_limits<float>::min();
    min_x = min_y = min_z = std::numeric_limits<float>::max();

    for (auto& point : cloud->points) {
        max_x = std::max(max_x, point.x);
        max_y = std::max(max_x, point.y);
        max_z = std::max(max_x, point.z);

        min_x = std::min(min_x, point.x);
        min_y = std::min(min_x, point.y);
        min_z = std::min(min_x, point.z);
    }

    pose->position.x = min_x;
    pose->position.x = min_y;
    pose->position.x = min_z;
    dimensions->x = max_x - min_x;
    dimensions->y = max_y - min_y;
    dimensions->z = max_z - min_z;
}

Segmenter::Segmenter(const ros::Publisher& marker_pub)
    : marker_pub_(marker_pub) {}

void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
    PointCloudC::Ptr cloud(new PointCloudC());
    pcl::fromROSMsg(msg, *cloud);
    std::vector<pcl::PointIndices> object_indices;
    SegmentBinObjects(cloud, &object_indices);

    for (size_t i = 0; i < object_indices.size(); ++i) {
        // Reify indices into a point cloud of the object.
        pcl::PointIndices::Ptr indices(new pcl::PointIndices);
        *indices = object_indices[i];
        PointCloudC::Ptr object_cloud(new PointCloudC());
        // TODO: fill in object_cloud using indices
        pcl::ExtractIndices<PointC> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(indices);
        extract.filter(*object_cloud);

        // Publish a bounding box around it.
        visualization_msgs::Marker object_marker;
        object_marker.ns = "objects";
        object_marker.id = i;
        object_marker.header.frame_id = "base_link";
        object_marker.type = visualization_msgs::Marker::CUBE;
        GetAxisAlignedBoundingBox(object_cloud, &object_marker.pose,
                                    &object_marker.scale);
        object_marker.color.g = 1;
        object_marker.color.a = 0.3;
        marker_pub_.publish(object_marker);
    }
}
}  // namespace perception
