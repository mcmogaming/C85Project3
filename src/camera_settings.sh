#!/bin/sh
v4l2-ctl -d /dev/video1 --set-ctrl saturation=100
v4l2-ctl -d /dev/video1 --set-ctrl exposure_auto=1 
