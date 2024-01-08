#!/usr/bin/env node

// Connecting to ROS 
var ROSLIB = require('roslib');

var ros = new ROSLIB.Ros({
    url : 'ws://172.17.0.2:9090'
});

ros.on('connection', function() {
console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
console.log('Connection to websocket server closed.');
});

const cmdVel = new ROSLIB.Topic({
    ros : ros,
    name : '/my_msg',
    messageType : 'std_msgs/String'
});
const twist = new ROSLIB.Message({
    data : "abc"
});

cmdVel.publish(twist);
