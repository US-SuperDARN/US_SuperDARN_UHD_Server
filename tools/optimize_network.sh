#!/bin/bash

sudo sysctl -w net.core.rmem_max=33554432
sudo sysctl -w net.core.wmem_max=33554432
sudo sysctl -w net.core.rmem_default=33554432
sudo sysctl -w net.core.wmem_default=33554432
sudo sysctl -w net.core.optmem_max=5242870
sudo sysctl -w net.core.netdev_max_backlog=300000
sudo sysctl -w net.core.netdev_budget=600

# edit this line for your computer. It should return just the 10 G-bit NICs
eth_nics=`sudo lshw -class network | grep logical | grep -v eno | grep -v enx | cut -d ":" -f 2`

for nic in $eth_nics;
do
    echo $nic
    sudo ethtool -C $nic adaptive-tx off
    sudo ethtool -C $nic adaptive-rx off
    sudo ethtool -A $nic tx on
    sudo ethtool -A $nic rx on
    sudo ethtool -G $nic tx 4096 rx 4096
done

# set CPU governor to performance

for ((i=0;i<$(nproc --all);i++)); do sudo cpufreq-set -c $i -r -g performance; done
