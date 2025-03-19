#!/bin/bash

pkill -KILL -f srr_watchdog.py

/home/radar/repos/SuperDARN_UHD_Server/srr.py stop
# from old stop.radar
killall -s 2 schedule

killall schedule
killall rtserver
killall fitacfwrite
killall rawacfwrite
killall iqwrite

killall shellserver
killall errlog



# pkill -f usrp_driver
# pkill -f cuda_driver.py
# pkill -f run_cuda_driver
# pkill -f usrp_server
# pkill -f uafscan
