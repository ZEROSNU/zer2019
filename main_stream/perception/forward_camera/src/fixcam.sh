#!/bin/bash
wbt=4500
exp=8
wbt_front=4000
exp_front=7
v4l2-ctl -d /dev/video1 --set-ctrl=white_balance_temperature_auto=0
v4l2-ctl -d /dev/video1 --set-ctrl=white_balance_temperature=${wbt}
v4l2-ctl -d /dev/video1 --set-ctrl=exposure_auto_priority=0
v4l2-ctl -d /dev/video1 --set-ctrl=exposure_auto=1
v4l2-ctl -d /dev/video1 --set-ctrl=exposure_absolute=${exp}

v4l2-ctl -d /dev/video3 --set-ctrl=white_balance_temperature_auto=0
v4l2-ctl -d /dev/video3 --set-ctrl=white_balance_temperature=${wbt}
v4l2-ctl -d /dev/video3 --set-ctrl=exposure_auto_priority=0
v4l2-ctl -d /dev/video3 --set-ctrl=exposure_auto=1
v4l2-ctl -d /dev/video3 --set-ctrl=exposure_absolute=${exp}

v4l2-ctl -d /dev/video2 --set-ctrl=white_balance_temperature_auto=0
v4l2-ctl -d /dev/video2 --set-ctrl=white_balance_temperature=${wbt_front}
v4l2-ctl -d /dev/video2 --set-ctrl=exposure_auto_priority=0
v4l2-ctl -d /dev/video2 --set-ctrl=exposure_auto=1
v4l2-ctl -d /dev/video2 --set-ctrl=exposure_absolute=${exp_front}

v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_temperature_auto=0
v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_temperature=${wbt}
v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto_priority=0
v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto=1
v4l2-ctl -d /dev/video0 --set-ctrl=exposure_absolute=${exp}
