. /opt/ros/noetic/setup.bash
. ~/catkin_ws/devel/setup.bash

rm -rf combined_labels
mkdir combined_labels
# rosrun perception extract_features pillow.bag pillow
# rosrun perception extract_features gloves.bag gloves
# rosrun perception extract_features pills.bag pills
rosrun perception extract_features table_box.bag box

mv *_label.bag combined_labels