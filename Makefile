network:
	docker network create nbv

build-image:
	docker build --build-arg UID=$(shell id -u) --build-arg GID=$(shell id -g) . -t nbv --target base

vnc:
	docker run -d --rm \
	--net=nbv \
	--env="DISPLAY_WIDTH=3000" \
	--env="DISPLAY_HEIGHT=1800" \
	--env="RUN_XTERM=no" \
	--name=novnc \
	-p=8080:8080 \
	theasp/novnc:latest

bash:
	docker run -it --rm \
	--net=nbv \
	--user $(shell id -u):$(shell id -g) \
	-v ./Makefile:/nbv/Makefile:Z \
	-v ~/.ssh:/home/appuser/.ssh:ro \
	nbv bash

clean: 
	rm -rf build/ install/ log/