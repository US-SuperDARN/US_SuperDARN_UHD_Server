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


# disable cpu throttling
sudo cpufreq-set -g PERFORMANCE

# disable interrupt coallesing:
sudo ethtool -C ens102f0np0 adaptive-tx off
sudo ethtool -C ens102f1np1 adaptive-tx off
sudo ethtool -C ens102f2np2 adaptive-tx off
sudo ethtool -C ens102f3np3 adaptive-tx off

sudo ethtool -C ens102f0np0 adaptive-rx off
sudo ethtool -C ens102f1np1 adaptive-rx off
sudo ethtool -C ens102f2np2 adaptive-rx off
sudo ethtool -C ens102f3np3 adaptive-rx off

sudo ethtool -C ens106f0np0 adaptive-tx off
sudo ethtool -C ens106f1np1 adaptive-tx off
sudo ethtool -C ens106f2np2 adaptive-tx off
sudo ethtool -C ens106f3np3 adaptive-tx off

sudo ethtool -C ens106f0np0 adaptive-rx off
sudo ethtool -C ens106f1np1 adaptive-rx off
sudo ethtool -C ens106f2np2 adaptive-rx off
sudo ethtool -C ens106f3np3 adaptive-rx off

sudo ethtool -C ens81f0np0 adaptive-tx off
sudo ethtool -C ens81f1np1 adaptive-tx off

sudo ethtool -C ens81f0np0 adaptive-rx off
sudo ethtool -C ens81f1np1 adaptive-rx off
