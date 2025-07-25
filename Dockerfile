FROM python:3.10.12 AS builder
COPY requirements.txt /requirements.txt

RUN pip install -r requirements.txt

FROM osrf/ros:humble-desktop-full AS base
# TODO: downgrade this image in production

# copy over all python files from builder stage
COPY --from=builder /usr/local /usr/local

# any utilities you want
RUN apt-get update && apt-get install -y git wget python3-pip vim net-tools netcat \
    python3-colcon-common-extensions python3-vcstool \
    gstreamer1.0-tools gstreamer1.0-libav libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev gstreamer1.0-plugins-good gstreamer1.0-plugins-base \
    ros-humble-moveit-visual-tools ros-humble-pcl-conversions ros-humble-pcl-ros

# config for cyclone -> needed for moveit
RUN apt update && apt install ros-humble-rmw-cyclonedds-cpp -y
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# TODO: add CUDA capabilities for when we wish to add GPU queries

# set root directory
WORKDIR /nbv

# BEGIN kortex ros2 module compilation
ENV KORTEX_WS=/root/workspace/ros2_kortex_ws
RUN mkdir -p ${KORTEX_WS}/src
# since they don't tag their own versions, we'll tag it here for them
ENV KORTEX_WORKING_SHA=97a0e7c9a2b7970f8de5830919e2fe0d7eea3bf6

# checkout and build the Dec 9th commit of this package.
RUN cd $KORTEX_WS && \
    git clone https://github.com/Kinovarobotics/ros2_kortex.git src/ros2_kortex && \
    cd src/ros2_kortex && git reset --hard $KORTEX_WORKING_SHA && cd ../.. && \
    vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex.$ROS_DISTRO.repos && \
    vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex-not-released.$ROS_DISTRO.repos && \
    vcs import src --skip-existing --input src/ros2_kortex/simulation.humble.repos && \
    vcs import src --skip-existing --input src/ros2_control/ros2_control.$ROS_DISTRO.repos && \
    # TODO: this is a hack to remove the duplicate package that we don't need. we should manage this better
    rm -rf src/ros-controls/control_msgs
    # END

# build artifacts to run by default
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd $KORTEX_WS && \
    apt update && \
    rosdep install --ignore-src --from-paths src -y -r && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 3
# END kortex ros2 module compilation

# BEGIN vision module compilation
ENV VISION_WS=/root/workspace/vision_ws
RUN mkdir -p ${VISION_WS}/src

RUN cd $VISION_WS && git clone https://github.com/Kinovarobotics/ros2_kortex_vision.git && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --from-paths . --ignore-src -r -y && \
    colcon build
# END vision module end

# build local project
COPY . /nbv
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# noVNC setup
ENV DISPLAY=:2 \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
  __GLX_VENDOR_LIBRARY_NAME=nvidia \
  __NV_PRIME_RENDER_OFFLOAD=1

# source packages
RUN echo "source ${KORTEX_WS}/install/setup.bash" >> /root/.bashrc
RUN echo "source ${VISION_WS}/install/setup.bash" >> /root/.bashrc
RUN echo "source /nbv/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
