#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo " "
echo "remap the device serial ports (ttyUSBX) to  cyglidar and rplidar"
echo "Check if USB is identified as /dev/cyglidar or /dev/rplidar using the command: ls -l /dev|grep ttyUSB"
echo "Start copying cyglidar. rules to  /etc/udev/rules.d/"
sudo cp "${SCRIPT_DIR}/cyglidar.rules"  /etc/udev/rules.d
echo " "
echo "Start copying rplidar.rules to  /etc/udev/rules.d/"
sudo cp "${SCRIPT_DIR}/rplidar.rules"  /etc/udev/rules.d
echo " "
echo "Restart udev"
echo ""
sudo udevadm control --reload-rules
sudo service udev restart
sudo udevadm trigger
echo "Created"