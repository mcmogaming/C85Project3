#!/bin/bash
#Microsoft camera
# This script allows you to set the camera params from the commandline
# You should always set these two
device='/dev/video1'

v4l2-ctl -d $device --set-ctrl saturation=200
# You need to first set the exposure into manual mode, and then change the exposure level using exposure_absolute
v4l2-ctl -d $device --set-ctrl exposure_auto=1 # sets exposure to manual mode

for i in 5, 9, 10, 19, 39, 78, 156, 312, {313..1000}
do
  echo $i   
  v4l2-ctl -d $device --set-ctrl exposure_absolute=$i #min=5 max=20000 step=1
  sleep 2s
done
# To list all the available options for the camera settings
#v4l2-ctl -d $device --list-ctrls-menus

# These are optional, but you might need to tweak them depending on the lighting conditions in the room (and even time of the day in the same room!)
# You might want to create different scripts with optimal settings for different working conditions
#v4l2-ctl -d $device --set-ctrl brightness=1 #min=30 max=255 step=1
#v4l2-ctl -d $device --set-ctrl contrast=1 #min=0 max=10 step=1

# If you want to adjust white white_balance_temperature, you need to first set it to manual mode
# You need both of the commands below:
#v4l2-ctl -d $device --set-ctrl white_balance_temperature_auto=0 #need to disable this flag first
#v4l2-ctl -d $device --set-ctrl white_balance_temperature=4500 #min=2800 max=10000 step=1
