#include "rosbag/bag.h"
std::string filename(name + ".bag");
rosbag::Bag bag;
bag.open(filename, rosbag::bagmode::Write);
bag.write("head_camera/depth_registered/points", ros::Time::now(), cloud_out);
bag.close();

return 0;
