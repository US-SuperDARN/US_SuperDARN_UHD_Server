#!/bin/bash

radar='kod'

sudo ./tools/optimize_network.sh &

errlog -name ${radar}.d -lp 41000 &
#errlog -name ${radar}.c -lp 42000 &

rawacfwrite -lp 41102 -ep 41000 -c d &
#rawacfwrite -lp 42102 -ep 42000 -c d &

fitacfwrite -lp 41103 -ep 41000 -c d &
#fitacfwrite -lp 42103 -ep 42000 -c c &


rtserver -rp 41104 -ep 41000 -tp 1401 & # ch 1
#rtserver -rp 42104 -ep 42000 -tp 1402 & # ch 2


python3 /home/radar/repos/SuperDARN_UHD_Server/tools/srr_watchdog.py server &

sleep 25
schedule -l /data/ros/scdlog/kod.d.scdlog -f /data/ros/scd/kod.d.scd &
# schedule -name ${radar}.d /data/ros/scd/${radar}.d.scd &
#schedule -name ${radar}.c /data/ros/scd/${radar}.c.scd &

