#!/bin/bash

echo " "
echo "Start deleting remap the device serial port(ttyUSBX) to cyglidar and rplidar"
echo " "
echo "sudo rm   /etc/udev/rules.d/cyglidar.rules"
sudo rm   /etc/udev/rules.d/cyglidar.rules
echo " "
echo "sudo rm   /etc/udev/rules.d/rplidar.rules"
sudo rm   /etc/udev/rules.d/rplidar.rules
echo " "
echo "Restart udev"
echo ""
sudo udevadm control --reload-rules
sudo service udev restart
sudo udevadm trigger
echo "Deleted"