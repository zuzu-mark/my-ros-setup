all:
	docker run --name viz --rm -p "8080:8080" ghcr.io/foxglove/studio:latest

dep:
	apt install ros-noetic-foxglove-bridge


start:
	roslaunch --screen foxglove_bridge foxglove_bridge.launch port:=8765
