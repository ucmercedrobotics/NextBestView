FROM python:3.10.12 AS builder
COPY requirements.txt /requirements.txt

RUN pip install -r requirements.txt

FROM osrf/ros:humble-desktop-full AS base
# TODO: downgrade this image in production

RUN mkdir -p ~/ros2-control/src
RUN cd ~/ros2-control/ && \
    vcs import --input https://raw.githubusercontent.com/ros-controls/ros2_control_ci/master/ros_controls.$ROS_DISTRO.repos src
RUN cd ~/ros2-control/ && \
    sudo apt-get update && \
    /bin/bash -c "rosdep update --rosdistro=$ROS_DISTRO && \
    rosdep install --from-paths src --ignore-src -r -y && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install"

# copy over all python files from builder stage
COPY --from=builder /usr/local /usr/local

# any utilities you want
RUN apt-get update && apt-get install -y git wget python3-pip vim net-tools netcat \
    python3-colcon-common-extensions python3-vcstool

# set root directory
WORKDIR /nbv

ENV COLCON_WS=/root/workspace/ros2_kortex_ws
RUN mkdir -p ${COLCON_WS}/src

# TODO: add CUDA capabilities for when we wish to add GPU queries

# configure DISPLAY env variable for novnc connection
ENV DISPLAY=novnc:0.0

RUN cd $COLCON_WS && \
    git clone https://github.com/Kinovarobotics/ros2_kortex.git src/ros2_kortex && \
    vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex.$ROS_DISTRO.repos && \
    vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex-not-released.$ROS_DISTRO.repos && \
    vcs import src --skip-existing --input src/ros2_kortex/simulation.humble.repos

# build artifacts to run by default
RUN /bin/bash -c "cd $COLCON_WS && \
    source /root/ros2-control/install/setup.bash && \
    rosdep install --ignore-src --from-paths src -y -r && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 3"

RUN echo "source /root/ros2-control/install/setup.bash" >> /root/.bashrc
RUN echo "source ${COLCON_WS}/install/setup.bash" >> /root/.bashrc
