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

# Launch server
# Backend
# tmux new -d -s backend
# tmux send-keys -t backend "roslaunch web_teleop backend.launch" Enter

# Launch frontend
# tmux new -d -s frontend
# tmux send-keys -t frontend "cd ~/catkin_ws/src/fetch-picker/web_teleop/frontend" Enter
# tmux send-keys -t frontend "npm run dev" Enter
# sleep 2
# printf "started. \n"

# Open webserver
# printf "opening web app... \n"
# firefox http://localhost:3000/