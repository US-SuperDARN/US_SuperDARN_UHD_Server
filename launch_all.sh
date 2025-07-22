#!/bin/bash

radar='mcm'

sudo ./tools/optimize_network.sh &

errlog -name ${radar}.a -lp 41000 &
#errlog -name ${radar}.b -lp 42000 &

rawacfwrite -lp 41102 -ep 41000 -c a &
#rawacfwrite -lp 42102 -ep 42000 -c b &

fitacfwrite -lp 41103 -ep 41000 -c a &
#fitacfwrite -lp 42103 -ep 42000 -c b &


rtserver -rp 41104 -ep 41000 -tp 1024 & # ch 1
#rtserver -rp 42104 -ep 42000 -tp 1025 & # ch 2


#Start USRP drivers and CUDA driver on second radar:
RADAR_2=192.168.100.2
ssh radar@$RADAR_2 'python3 /home/radar/repos/SuperDARN_UHD_Server/tools/srr_watchdog.py &' &

python3 /home/radar/repos/SuperDARN_UHD_Server/tools/srr_watchdog.py server &

sleep 25
#ssh radar@$RADAR_2 '/home/radar/repos/SuperDARN_UHD_Server/launch_second_radar.sh &' &
sleep 5
schedule -name ${radar}.a /data/ros/scd/${radar}.a.scd &
#schedule -name ${radar}.b /data/ros/scd/${radar}.b.scd &

