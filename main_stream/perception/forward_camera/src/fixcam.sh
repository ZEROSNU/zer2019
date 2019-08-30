#!/bin/bash
v4l2-ctl -d /dev/video1 --set-ctrl=white_balance_temperature_auto=0
v4l2-ctl -d /dev/video1 --set-ctrl=white_balance_temperature=4000
v4l2-ctl -d /dev/video1 --set-ctrl=exposure_auto_priority=0
v4l2-ctl -d /dev/video1 --set-ctrl=exposure_auto=1
v4l2-ctl -d /dev/video1 --set-ctrl=exposure_absolute=100

v4l2-ctl -d /dev/video2 --set-ctrl=white_balance_temperature_auto=0
v4l2-ctl -d /dev/video2 --set-ctrl=white_balance_temperature=4000
v4l2-ctl -d /dev/video2 --set-ctrl=exposure_auto_priority=0
v4l2-ctl -d /dev/video2 --set-ctrl=exposure_auto=1
v4l2-ctl -d /dev/video2 --set-ctrl=exposure_absolute=100

v4l2-ctl -d /dev/video3 --set-ctrl=white_balance_temperature_auto=0
v4l2-ctl -d /dev/video3 --set-ctrl=white_balance_temperature=4000
v4l2-ctl -d /dev/video3 --set-ctrl=exposure_auto_priority=0
v4l2-ctl -d /dev/video3 --set-ctrl=exposure_auto=1
v4l2-ctl -d /dev/video3 --set-ctrl=exposure_absolute=100

v4l2-ctl -d /dev/video4 --set-ctrl=white_balance_temperature_auto=0
v4l2-ctl -d /dev/video4 --set-ctrl=white_balance_temperature=4000
v4l2-ctl -d /dev/video4 --set-ctrl=exposure_auto_priority=0
v4l2-ctl -d /dev/video4 --set-ctrl=exposure_auto=1
v4l2-ctl -d /dev/video4 --set-ctrl=exposure_absolute=100
