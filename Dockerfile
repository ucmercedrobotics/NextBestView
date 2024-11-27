FROM osrf/ros:humble-desktop-full AS base 
# TODO: downgrade this image in production

ARG UID
ARG GID

# any utilities you want
RUN apt-get update && apt-get install -y git wget python3-pip vim net-tools netcat

# TODO: add CUDA capabilities for when we wish to add GPU queries

WORKDIR /nbv

COPY requirements.txt /nbv/requirements.txt

RUN pip install -r requirements.txt

# configure DISPLAY env variable for novnc connection
ENV DISPLAY=novnc:0.0

# # # build artifacts to run by default
# RUN /bin/bash -c "cd /nbv/nbv && \
#                     colcon build"

RUN adduser -u ${UID} --disabled-password --gecos "" appuser && chown -R appuser /nbv
USER appuser

RUN echo "source /nbv/install/setup.bash" >> /home/appuser/.bashrc
