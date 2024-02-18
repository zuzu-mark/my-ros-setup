help:

b:
	docker build -t myros2 .

b2:
	./build_add.sh

IMG:=myros2
ENTRY:=--entrypoint bash 

ENTRY:=
IMG:=myros2:custom2
r: run
run:
	docker run --rm -it --privileged  --ipc=host \
        --device=/dev/dri:/dev/dri \
		-p 9090:9090 \
		-p 8765:8765 \
        -v $$(pwd)/data:/root/data \
        -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$${DISPLAY} \
        -v $${HOME}/.Xauthority:/home/$$(id -un)/.Xauthority -e XAUTHORITY=/home/$$(id -un)/.Xauthority \
        -e ROS_IP=127.0.0.1 \
		$(ENTRY) \
        $(IMG) -c terminator
