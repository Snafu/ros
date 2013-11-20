#!/bin/sh
# Using v4l2-ctl from ivtv-utils to setup the camera feed
# Image messages are published via gscam
v4l2-ctl -s ntsc
v4l2-ctl -v width=320,height=240,pixelformat=BGR3
