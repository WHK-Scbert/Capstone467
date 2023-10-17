BUILD
sudo DOCKER_BUILDKIT=1 docker build . --build-arg ROS_DISTRO=humble --build-arg LOCAL_WS_DIR=./sturdy-canary/ros2_ws --build-arg APP_NAME=catkin-canary -t my_agent

