#include "perception/feature_extraction.h"

#include <algorithm> // std::min and std::max

#include "perception/object.h"
#include "perception_msgs/ObjectFeatures.h"
#include "ros/ros.h"

namespace perception {
void ExtractSizeFeatures(const perception::Object& object,
                         perception_msgs::ObjectFeatures* features) {
  // "x" dimension is always the smallest of x and y to account for rotations.
  // z always points up.
  double x = object.dimensions.x;
  double y = object.dimensions.y;
  if (x > y) {
    x = object.dimensions.y;
    y = object.dimensions.x;
  }
  features->names.push_back("box_dim_x");
  features->values.push_back(x);  
  features->names.push_back("box_dim_y");
  features->values.push_back(y);
  features->names.push_back("box_dim_z");
  features->values.push_back(object.dimensions.z);
}
}  // namespace perception