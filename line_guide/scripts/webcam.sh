#!/bin/sh
# Using v4l2-ctl from ivtv-utils to setup the camera feed
# Image messages are published via gscam
v4l2-ctl -d /dev/video1 -p 10
v4l2-ctl -d /dev/video1 -v width=160,width=120,pixelformat=1
