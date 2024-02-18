help:

b:
	docker build -t myros2 .

b2:
	./build_add.sh

IMG:=myros2
ENTRY:=--entrypoint bash 

ENTRY:=
IMG:=myros2:custom2

DIR:=/home/ubuntu
#DIR:=/root
r: run
run:
	docker run --rm -it --privileged  --ipc=host \
        --device=/dev/dri:/dev/dri \
		-p 9090:9090 \
		-p 8765:8765 \
		-p 2244:22 \
        -v $$(pwd)/data:$(DIR)/data \
        -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$${DISPLAY} \
        -v $${HOME}/.Xauthority:/home/$$(id -un)/.Xauthority -e XAUTHORITY=/home/$$(id -un)/.Xauthority \
        -e ROS_IP=127.0.0.1 \
		$(ENTRY) \
        $(IMG) terminator
	#-c terminator


ssh:
	ssh -p 2244 localhost
