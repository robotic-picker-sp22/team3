. /opt/ros/noetic/setup.bash
. ~/catkin_ws/devel/setup.bash

rm -rf final_labels
mkdir final_labels
shopt -s nullglob

for file in raw_item_bags/*
do
    rosrun perception extract_features $file ${file%.*}
done
# rosrun perception extract_features pill_bottle.bag  pill_bottle
# rosrun perception extract_features pill_box.bag     pill_box
# rosrun perception extract_features sink_basket.bag  sink_basket
# rosrun perception extract_features box.bag          box
# rosrun perception extract_features tape_measure.bag tape_measure
shopt -u nullglob
mv *_label.bag final_labels