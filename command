BUILD
sudo DOCKER_BUILDKIT=1 docker build . --build-arg ROS_DISTRO=humble --build-arg LOCAL_WS_DIR=./sturdy-canary/ros2_ws --build-arg APP_NAME=catkin-canary -t my_agent

RUN
docker run -it -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
-u robomaker -e ROBOMAKER_GAZEBO_MASTER_URI=http://localhost:5555 \
-e ROBOMAKER_ROS_MASTER_URI=http://localhost:11311 \
robomaker-helloworld-robot-app:latest roslaunch hello_world_robot rotate.launch


Mavproxy for Docker

mavproxy.py --master=tcp:127.0.0.1:5760 --out=udp:127.0.0.1:14550 --out=udp:172.17.0.2:14551

