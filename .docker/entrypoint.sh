set -e

# setup ros environment
source "/opt/ros/noetic/setup.bash"
source "/dev_ws/devel/setup.bash"
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:/dev_ws/src/gazebo_models
# export ROS_MASTER_URI=http://10.41.1.1:11311
# export ROS_IP=10.41.1.10

exec "$@"
