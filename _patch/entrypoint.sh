#!/bin/bash

if [[ ! -f ~/.tmux.conf ]]; then
	echo CP
	cp .tmux.conf ~
fi

apt update

apt install -y ros-noetic-rosbridge-server


#pip3 install pyyaml rospkg twisted cryptography six pyopenssl autobahn tornado bson pymongo 

pip3 install pyOpenSSL --upgrade



### https://tricknotes.hateblo.jp/entry/20120227/p1
### https://b.ueda.tech/?post=20220502_rosbridge
### exec
 #npm install -g ws

 #wscat -c ws://172.17.0.2:9090
 # subscribe
 #{"op": "subscribe", "topic":"/rosout","type":"rosgraph_msgs/Log"}

 #{"op": "publish", "topic":"/cmd_vel","msg":{"linear":{"x":0.1,"y":0,"z":0},
    #"angular":{"x":0,"y":0,"z":0}},"type":"geometry_msgs/Twist"}

#	{ "op": "publish", "topic": "/my_msg", "msg": {"data":"hell"} }


###########################
sudo npm install -g roslib
