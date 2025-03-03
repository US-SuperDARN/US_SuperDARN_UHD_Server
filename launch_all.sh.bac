#!/bin/bash

#sudo ./init_network.sh &

/home/radar/ros.3.6/bin/errlog -name mcm.a -lp 41000 &
#/home/radar/ros.3.6/bin/errlog -name mcm.b -lp 42000 &

/home/radar/ros.3.6/bin/rawacfwrite -r mcm.a -lp 41102 -ep 41000 &
#/home/radar/ros.3.6/bin/rawacfwrite -r mcm.b -lp 42102 -ep 42000 &

/home/radar/ros.3.6/bin/fitacfwrite -r mcm.a -lp 41103 -ep 41000 &
#/home/radar/ros.3.6/bin/fitacfwrite -r mcm.b -lp 42103 -ep 42000 &


/home/radar/ros.3.6/bin/rtserver -rp 41104 -ep 41000 -tp 1401 & # ch 4
#/home/radar/ros.3.6/bin/rtserver -rp 42104 -ep 42000 -tp 1402 & # ch 3


python3 /home/radar/repos/SuperDARN_UHD_Server/tools/srr_watchdog.py server &

sleep 5
/home/radar/ros.3.6/bin/schedule -l /data/ros/scdlog/mcm.a.scdlog -f /data/ros/scd/mcm.a.scd & 
#/home/radar/ros.3.6/bin/schedule -l /data/ros/scdlog/mcm.b.scdlog -f /data/ros/scd/mcm.b.scd & 

