# SuperDARN USRP Imaging Overview
This repository houses code for the US SuperDARN group's UHD Server code and adapts the use of Ettus software defined radios for our radar sites.
The branches are currently being renamed with tags to distigush which are stable, being developed, used for quick fixes, etc.

This code is intended to integrate with the existing SuperDARN RST-ROS repository and for use on Ubuntu 24.04 computers.
It replaces all software from arby\_server on down, including the timing, dio, and receiver cards.

## usrp\_server.py
usrp\_server interfaces with existing SuperDARN control programs and is a replacement for the old QNX6 arby\_server code.
In an imaging configuration with distributed USRPs across multiple computers, this server may talk to CUDA and USRP drivers across multiple computers.

## cuda\_driver.py
cuda\_driver does downconversion of RF samples from the usrp\_driver(s)

## usrp\_driver.cpp
usrp\_driver uses the UHD API to tune a USRP and grab samples.
In an imaging configuration with multiple USRPs, one of these would run for each USRP.

## Dependencies
python 3 with numpy, pycuda

## Data Directories

- /data/image\_samples/bb\_data
- /data/image\_samples/if\_data
- /data/log
- /data/log/cfs
- /data/log/clear\_freq
- /data/log/fft\_spectrum
- /data/log/usrp\_driver
- /data/log/watchdog

## Testing:
- run `python3 srr.py start cuda`
- run `python3 srr.py start usrps`
- run `python3 srr.py start cfs`
- run `python3 srr.py start server`
- run `python3 srr.py stop all`

