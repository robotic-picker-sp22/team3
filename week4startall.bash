# End previous server
tmux kill-server
sleep 2

# Start roscore
printf "starting roscore... "
tmux new -d -s roscore
tmux send-keys -t roscore "roscore" Enter
sleep 2
printf "started.\n"

# Start gazebo
printf "starting gazebo... "
tmux new -d -s gazebo
tmux send-keys -t gazebo "roslaunch fetch_gazebo playground.launch" Enter
sleep 8
printf "started.\n"

# Start RViz
printf "starting rviz... "
tmux new -d -s rviz
tmux send-keys -t rviz "roslaunch applications nav_rviz.launch" Enter
printf "started.\n"

# Start Moveit
printf "starting MoveIt... "
tmux new -d -s moveit
tmux send-keys -t moveit "roslaunch robot_api move_group.launch" Enter
printf "started.\n"

# Center head
rosrun applications head_demo.py pan_tilt 0 .4
printf "head centered.\n"
sleep 2

# Raise torso
rosrun applications torso_demo.py .4
printf "torso raised.\n"
sleep 2


printf "now run:\n"
printf "roslaunch robot_api move_group.launch\n"