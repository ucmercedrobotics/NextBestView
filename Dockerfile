FROM python:3.10.12 AS builder
COPY requirements.txt /requirements.txt

RUN pip install -r requirements.txt

FROM osrf/ros:humble-desktop-full AS base
# TODO: downgrade this image in production

# copy over all python files from builder stage
COPY --from=builder /usr/local /usr/local

# any utilities you want
RUN apt-get update && apt-get install -y git wget python3-pip vim net-tools netcat \
    python3-colcon-common-extensions python3-vcstool

# config for cyclone -> needed for moveit
RUN apt update && apt install ros-humble-rmw-cyclonedds-cpp -y
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

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
    vcs import src --skip-existing --input src/ros2_kortex/simulation.humble.repos && \
    vcs import src --skip-existing --input src/ros2_control/ros2_control.$ROS_DISTRO.repos && \
    # TODO: this is a hack to remove the duplicate package that we don't need. we should manage this better
    rm -rf src/ros-controls/control_msgs
    # END

# build artifacts to run by default
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd $COLCON_WS && \
    rosdep install --ignore-src --from-paths src -y -r && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 3

# build local project
COPY . /nbv
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build

# source packages
RUN echo "source ${COLCON_WS}/install/setup.bash" >> /root/.bashrc
RUN echo "source /nbv/install/setup.bash" >> /root/.bashrc
