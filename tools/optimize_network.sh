sudo sysctl -w net.core.rmem_max=33554432
sudo sysctl -w net.core.wmem_max=33554432


# One of the follwing commands seemed to solve this error:
#    Error: EnvironmentError: IOError: Radio ctrl (A) packet parse error - AssertionError: packet_info.packet_count == (seq_to_ack & 0xfff)
#  (happening while uhd_usrp_probe with MTU 9000)
sudo sysctl -w net.core.rmem_default=5242870
sudo sysctl -w net.core.wmem_default=5242870
sudo sysctl -w net.core.optmem_max=5242870
sudo sysctl -w net.core.netdev_max_backlog=300000
sudo sysctl -w net.core.netdev_budget=600


eth_nics=`sudo lshw -class network | grep logical | cut -d ":" -f 2`

for nic in $eth_nics;
do
    echo $nic
    sudo ethtool -C $nic adaptive-tx off
    sudo ethtool -C $nic adaptive-rx off
done

# disable interrupt coallesing:
#sudo /home/radar/repos/SuperDARN_UHD_Server/tools/change_priority_by_name.sh ksoftirqd 60



