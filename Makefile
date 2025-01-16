WORKSPACE:= /nbv
KINOVA_NIC:= en7

repo-init:
	pre-commit install

network:
	docker network create nbv

config-target-network:
	sudo ifconfig ${KINOVA_NIC} 192.168.1.11 netmask 255.255.255.0

build-image:
	docker build . -t nbv --target base

vnc:
	docker run -d --rm \
	--net=nbv \
	--env="DISPLAY_WIDTH=1920" \
	--env="DISPLAY_HEIGHT=1080" \
	--env="RUN_XTERM=no" \
	--name=novnc \
	-p=8080:8080 \
	theasp/novnc:latest

bash:
	docker run -it --rm \
	--net=nbv \
	--privileged \
	-p=12345:12345 \
	-v ${CURDIR}:${WORKSPACE}/ \
	nbv

nbv:
	colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

clean:
	rm -rf build/* install/* log/* build/.* install/.* log/.*

gazebo:
	ros2 launch kortex_bringup kortex_sim_control.launch.py \
	use_sim_time:=true \
	launch_rviz:=false \
	robot_ip:=yyy.yyy.yyy.yyy \
	use_fake_hardware:=true \
	dof:=6 \
	gripper:=robotiq_2f_85 \
	robot_name:=gen3 \
	robot_controller:=joint_trajectory_controller \
	vision:=true

moveit:
	ros2 launch next_best_view moveit.launch.py \
	use_sim_time:=true \
	robot_ip:=yyy.yyy.yyy.yyy \
	use_fake_hardware:=true \
	vision:=true

moveit-target:
	ros2 launch next_best_view moveit.launch.py \
	robot_ip:=192.168.1.10 \
	use_fake_hardware:=false \
	launch_rviz:=true \
	vision:=true

vision:
	ros2 launch kinova_vision kinova_vision.launch.py depth_registration:=true

moveit-example:
	ros2 run next_best_view hello_moveit

mission-interface:
	ros2 run mission_interface mission_interface

detect-object:
	ros2 run next_best_view object_detection
