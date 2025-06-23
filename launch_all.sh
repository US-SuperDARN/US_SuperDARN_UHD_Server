#!/bin/bash

sudo ./tools/optimize_network.sh &


errlog -name mcm.a -lp 41000 &

rawacfwrite -r mcm.a -lp 41102 -ep 41000 &

fitacfwrite -r mcm.a -lp 41103 -ep 41000 &


rtserver -rp 41104 -ep 41000 -tp 1401 & # ch 4

#Start USRP drivers and CUDA driver on second radar:
RADAR_2=192.168.100.2
ssh radar@$RADAR_2 'python3 /home/radar/repos/SuperDARN_UHD_Server/tools/srr_watchdog.py &' &

python3 /home/radar/repos/SuperDARN_UHD_Server/tools/srr_watchdog.py server &

sleep 25
#ssh radar@$RADAR_2 '/home/radar/repos/SuperDARN_UHD_Server/launch_second_radar.sh &' &
sleep 5
#schedule -l /data/ros/scdlog/mcm.a.scdlog -f /data/ros/scd/mcm.a.scd & 
#/home/radar/ros.3.6/bin/schedule -l /data/ros/scdlog/mcm.b.scdlog -f /data/ros/scd/mcm.b.scd & 
