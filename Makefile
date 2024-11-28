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
	--env="DISPLAY_WIDTH=3000" \
	--env="DISPLAY_HEIGHT=1800" \
	--env="RUN_XTERM=no" \
	--name=novnc \
	-p=8080:8080 \
	theasp/novnc:latest

bash:
	docker run -it --rm \
	--net=nbv \
	--privileged \
	-v ./Makefile:/nbv/Makefile:Z \
	nbv bash

clean:
	rm -rf build/ install/ log/
