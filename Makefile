KINOVA_NIC:= en7

repo-init:
	python3 -m pip install pre-commit && \
	pre-commit install

network:
	docker network create nbv

build-image:
	docker build . -t nbv --target base

vnc:
	docker run -d --rm \
	--net=nbv \
	--env="DISPLAY_WIDTH=2000" \
	--env="DISPLAY_HEIGHT=1800" \
	--env="RUN_XTERM=no" \
	--name=novnc \
	-p=8080:8080 \
	theasp/novnc:latest

sim:
	docker run -it --rm \
	--net=nbv \
	--privileged \
	-v ./Makefile:/nbv/Makefile:Z \
	nbv bash

# TODO: replace with moveit to include bracelet?
sim-run:
	ros2 launch kortex_bringup gen3.launch.py \
	robot_ip:=yyy.yyy.yyy.yyy \
	use_fake_hardware:=true \
	vision:=true \
	dof:=6

config-target-network:
	sudo ifconfig ${KINOVA_NIC} 192.168.1.11 netmask 255.255.255.0

target:
	docker run -it --rm \
	--net=host \
	--privileged \
	-v ./Makefile:/nbv/Makefile:Z \
	nbv bash

target-run:
	ros2 launch kortex_bringup gen3.launch.py \
	robot_ip:=192.168.1.10 \
	dof:=6 \
	vision:=true \
	launch_rviz:=false

clean:
	rm -rf build/ install/ log/
