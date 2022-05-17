. /opt/ros/noetic/setup.bash
. ~/catkin_ws/devel/setup.bash

rm -rf combined_labels
mkdir combined_labels
rosrun perception extract_features pill_bottle.bag  pill_bottle
rosrun perception extract_features pill_box.bag     pill_box
rosrun perception extract_features sink_basket.bag  sink_basket
rosrun perception extract_features box.bag          box
rosrun perception extract_features tape_measure.bag tape_measure

mv *_label.bag combined_labels