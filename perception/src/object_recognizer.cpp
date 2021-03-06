#include "perception/object_recognizer.h"

#include <limits.h>
#include <math.h>
#include <string>
#include <vector>

#include "boost/filesystem.hpp"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "perception/feature_extraction.h"
#include "perception_msgs/ObjectFeatures.h"
#include "ros/ros.h"

using boost::filesystem::directory_iterator;
using perception_msgs::ObjectFeatures;

namespace perception {
namespace {
double EuclideanDistance(const std::vector<double>& v1,
                         const std::vector<double>& v2) {
  double total = 0;
  for (int i = 0; i < v1.size(); i++) {
      total += std::pow(v1[i]-v2[i], 2);
  }
  return std::sqrt(total);
}
}

void LoadData(const std::string& data_dir,
              std::vector<perception_msgs::ObjectFeatures>* dataset) {
  directory_iterator end;
  for (directory_iterator file_it(data_dir); file_it != end; ++file_it) {
    if (boost::filesystem::is_regular_file(file_it->path())) {  
      rosbag::Bag bag;
      bag.open(file_it->path().string(), rosbag::bagmode::Read);
      std::vector<std::string> topics;
      topics.push_back("object_features");
      rosbag::View view(bag, rosbag::TopicQuery(topics));

      for (rosbag::View::iterator it = view.begin(); it != view.end(); ++it) {
        ObjectFeatures::ConstPtr fp = it->instantiate<ObjectFeatures>();
        if (fp != NULL) {
          dataset->push_back(*fp);
        }
      }
    }
  }
}

ObjectRecognizer::ObjectRecognizer(const std::vector<ObjectFeatures>& dataset)
    : dataset_(dataset) {}

void ObjectRecognizer::Recognize(const Object& object, std::string* name,
                                 double* confidence) {
  // TODO: extract features from the object
  perception_msgs::ObjectFeatures features; 
  perception::ExtractFeatures(object, &features);

  double min_distance = std::numeric_limits<double>::max();
  double second_min_distance = std::numeric_limits<double>::max();
  for (size_t i = 0; i < dataset_.size(); ++i) {
    // TODO: compare the features of the input object to the features of the current dataset object.
    // For each feature in object, find corresponding feature in the dataset
    // std::vector<double> compare;
    // for (std::string name : features.names) {
    //     std::vector<std::string> other_names = dataset_[i].names;
    //     auto it = find(other_names.begin(), other_names.end(), name);
    //     if (it != other_names.end()) {
    //         int index = it - other_names.begin();
    //         compare.push_back(dataset_[i].values[index]);
    //     }
    // }
    std::vector<double> compare = dataset_[i].values;
    double distance = EuclideanDistance(features.values, compare);
    if (distance < min_distance) {
      second_min_distance = min_distance;
      min_distance = distance;
      *name = dataset_[i].object_name;
    } else if (distance < second_min_distance) {
      second_min_distance = distance;
    }
  }

  // Confidence is based on the distance to the two nearest results.
  *confidence = 1 - min_distance / (min_distance + second_min_distance);
}
}  // namespace perception