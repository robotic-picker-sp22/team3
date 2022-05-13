#include "perception/segmentation.h"
#include "pcl_conversions/pcl_conversions.h"
#include "visualization_msgs/Marker.h"
#include <pcl/common/common.h>
#include "pcl/filters/extract_indices.h"
#include "perception/object.h"
#include <math.h>
#include <sstream>
#include "perception/object_recognizer.h"
#include <pcl/segmentation/region_growing.h>
#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {

void SegmentObjects(PointCloudC::Ptr cloud,
                          std::vector<Object>* objects) {
    // Same as callback, but with visualization code removed.
    std::vector<pcl::PointIndices> object_indices;
    SegmentBinObjects(cloud, &object_indices);

    for (int i = 0; i < object_indices.size(); i++) {
        struct Object object;
        pcl::ExtractIndices<PointC> extract;
        pcl::PointIndices::Ptr indices(new pcl::PointIndices);
        *indices = object_indices[i];
        extract.setInputCloud(cloud);
        extract.setIndices(indices);
        PointCloudC::Ptr object_cloud(new PointCloudC());
        extract.filter(*object_cloud);
        object.cloud = object_cloud;
        GetAxisAlignedBoundingBox(object.cloud, &(object.pose), &(object.dimensions));
        objects->push_back(object);
    }
    // struct Object object;
    // GetAxisAlignedBoundingBox(cloud, &(object.pose), &(object.dimensions));
    // objects->push_back(object);
    ROS_INFO("Finished with segmentobjects");
}

void SegmentBinObjects(PointCloudC::Ptr cloud, std::vector<pcl::PointIndices>* indices) {
        double cluster_tolerance;
    int min_cluster_size, max_cluster_size;
    ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.02);
    ros::param::param("ec_min_cluster_size", min_cluster_size, 10);
    ros::param::param("ec_max_cluster_size", max_cluster_size, 20000);
    // ROS_INFO("Tolerance: %f", cluster_tolerance);
    // ROS_INFO("Min cluster: %d", min_cluster_size);
    // ROS_INFO("Max cluster: %d", max_cluster_size);
    // ROS_INFO("Cloud size %ld", cloud->size());
    // pcl::PointIndices inside_bin_indices;
    // Cloud2Indices(cloud, &inside_bin_indices);
    // ROS_INFO("Got params");
    pcl::EuclideanClusterExtraction<PointC> euclid;
    euclid.setInputCloud(cloud);
    // euclid.setIndices(&inside_bin_indices); // << TODO: is cloud already cropped or do we get indices for the crop?
    euclid.setClusterTolerance(cluster_tolerance);
    euclid.setMinClusterSize(min_cluster_size);
    euclid.setMaxClusterSize(max_cluster_size);
    euclid.extract(*indices);

    // pcl::search::Search<PointC>::Ptr tree(new pcl::search::KdTree<PointC>);
    // pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // pcl::NormalEstimation<PointC, pcl::Normal> normal_estimator;
    // normal_estimator.setSearchMethod(tree);
    // normal_estimator.setInputCloud(cloud);
    // normal_estimator.setKSearch(50);
    // normal_estimator.compute(*normals);
    // pcl::RegionGrowing<PointC, pcl::Normal> clustering;
    // clustering.setSearchMethod(tree);
    // clustering.setNumberOfNeighbours(30);
    // clustering.setInputCloud(cloud);
    // // clustering.setIndices (indices);
    // clustering.setInputNormals(normals);
    // clustering.setSmoothnessThreshold(2.5 / 180.0 * M_PI);
    // clustering.setCurvatureThreshold(1.0);

    // pcl::RegionGrowingRGB<PointC> clustering;
    // clustering.setInputCloud(cloud);
    // clustering.setSearchMethod(tree);
    // clustering.setDistanceThreshold(0.02);
    // clustering.setPointColorThreshold(6);
    // clustering.setRegionColorThreshold(5);

    // ROS_INFO("Finished extracting");

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
    PointC min_pcl;
    PointC max_pcl;
    pcl::getMinMax3D<PointC>(*cloud, min_pcl, max_pcl);

    pose->position.x = (min_pcl.x + max_pcl.x) / 2;
    pose->position.y = (min_pcl.y + max_pcl.y) / 2;
    pose->position.z = (min_pcl.z + max_pcl.z) / 2;
    dimensions->x = max_pcl.x - min_pcl.x;
    dimensions->y = max_pcl.y - min_pcl.y;
    dimensions->z = max_pcl.z - min_pcl.z;
    // ROS_INFO("Min: x=%lf, y=%lf, z=%lf", min_pcl.x, min_pcl.y, min_pcl.z);
    // ROS_INFO("Max: x=%lf, y=%lf, z=%lf", max_pcl.x, max_pcl.y, max_pcl.z);
    // ROS_INFO("position->x %lf", pose->position.x);
    // ROS_INFO("position->y %lf", pose->position.y);
    // ROS_INFO("position->z %lf", pose->position.y);
    // ROS_INFO("dimensions->x %lf", dimensions->x);
    // ROS_INFO("dimensions->y %lf", dimensions->y);
    // ROS_INFO("dimensions->z %lf", dimensions->z);
}

Segmenter::Segmenter(const ros::Publisher& points_pub,
                    const ros::Publisher& marker_pub,
                    const ObjectRecognizer& recognizer)
    : points_pub_(points_pub),
        marker_pub_(marker_pub),
        recognizer_(recognizer) {}

void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
    // ROS_INFO("In Segementer Callback");
    PointCloudC::Ptr cloud_unfiltered(new PointCloudC());
    pcl::fromROSMsg(msg, *cloud_unfiltered);
    PointCloudC::Ptr cloud(new PointCloudC());
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*cloud_unfiltered, *cloud, index);


    std::vector<Object> objects;
    SegmentObjects(cloud, &objects);
    ROS_INFO("Back in callback");
    
    for (size_t i = 0; i < objects.size(); ++i) {
        ROS_INFO("making mark %ld", i);
        // // Reify indices into a point cloud of the object.
        // pcl::PointIndices::Ptr indices(new pcl::PointIndices);
        // *indices = object_indices[i];
        // PointCloudC::Ptr object_cloud(new PointCloudC());
        // // TODO: fill in object_cloud using indices
        // pcl::ExtractIndices<PointC> extract;
        // extract.setInputCloud(cloud);
        // extract.setIndices(indices);
        // extract.filter(*object_cloud);

        const Object& object = objects[i];

        // Publish a bounding box around it.
        visualization_msgs::Marker object_marker;
        object_marker.ns = "objects";
        object_marker.id = i;
        object_marker.header.frame_id = "ar_marker_15";
        object_marker.type = visualization_msgs::Marker::CUBE;
        object_marker.pose = object.pose;
        object_marker.scale = object.dimensions;
        object_marker.color.g = 1;
        object_marker.color.a = 0.3;
        // ros::Publisher marker_pub_;
        marker_pub_.publish(object_marker);

        // Recognize the object.
        std::string name;
        double confidence;
        // TODO: recognize the object with the recognizer_. /////////////////////////////
        recognizer_.Recognize(object, &name, &confidence);
        confidence = round(1000 * confidence) / 1000;

        std::stringstream ss;
        ss << name << " (" << confidence << ")";

        // Publish the recognition result.
        visualization_msgs::Marker name_marker;
        name_marker.ns = "recognition";
        name_marker.id = i;
        name_marker.header.frame_id = "ar_marker_15";
        name_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        name_marker.pose.position = object.pose.position;
        name_marker.pose.position.z += 0.1;
        name_marker.pose.orientation.w = 1;
        name_marker.scale.x = 0.025;
        name_marker.scale.y = 0.025;
        name_marker.scale.z = 0.025;
        name_marker.color.r = 0;
        name_marker.color.g = 0;
        name_marker.color.b = 1.0;
        name_marker.color.a = 1.0;
        name_marker.text = ss.str();
        marker_pub_.publish(name_marker);
    }
}
}  // namespace perception
