FROM ubuntu:jammy


# ROS Distribution (ex: melodic, foxy, etc.)
ARG ROS_DISTRO=humble
# Application Name (ex: helloworld)
ARG APP_NAME=robomaker_app
# Path to workspace directory on the host (ex: ./robot_ws)
ARG LOCAL_WS_DIR=workspace
# User to create and use (default: robomaker)
ARG USERNAME=robomaker
# The gazebo version to use if applicable (ex: gazebo-9, gazebo-11)
ARG GAZEBO_VERSION=gazebo-9
# Where to store the built application in the runtime image.
ARG IMAGE_WS_DIR=/home/$USERNAME/workspace


#################################################################################
# Set the locale
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

# Set the timezone
ENV ROS_VERSION=2
ENV ROS_DISTRO=humble
ENV ROS_PYTHON_VERSION=3
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Setup the sources
RUN apt-get update && apt-get install -y software-properties-common curl && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

    # Install ROS 2 packages
    RUN apt-get update && apt-get upgrade -y && \
        apt-get install -y ros-humble-desktop 

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    nano \
    iputils-ping \
    wget \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro humble

#######################################################################

RUN groupadd $USERNAME && \
   useradd -ms /bin/bash -g $USERNAME $USERNAME && \
   sh -c 'echo "$USERNAME ALL=(root) NOPASSWD:ALL" >> /etc/sudoers'
   
USER $USERNAME
WORKDIR /home/$USERNAME

RUN mkdir -p $IMAGE_WS_DIR

WORKDIR $IMAGE_WS_DIR
COPY --chown=$USERNAME:$USERNAME $LOCAL_WS_DIR/src $IMAGE_WS_DIR/src

RUN sudo apt update && \ 
   rosdep update && \
   rosdep fix-permissions

# Note: This will install all dependencies. 
# You could further optimize this by only defining the exec dependencies. 
# Then, install the build dependencies in the build image.
RUN rosdep install --from-paths src --ignore-src -r -y





#######################################################################


# Environment setup
WORKDIR /
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN sudo chmod +x /ros_entrypoint.sh && \
   sudo chown -R $USERNAME /ros_entrypoint.sh

RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
RUN echo '#!/usr/bin/env bash' > /ros_entrypoint.sh
RUN echo 'source /opt/ros/humble/setup.bash' >> /ros_entrypoint.sh
RUN echo 'exec "$@"' >> /ros_entrypoint.sh
#RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
# Run bash
CMD ["bash"]
