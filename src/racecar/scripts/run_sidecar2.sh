#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/racecar/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/davinci-mini/fastdds_udp.xml
cd ~/racecar/src/racecar/scripts
timeout 20 python3 -u vision_sidecar.py > /tmp/sidecar2.log 2>&1
echo "bytes: $(wc -c < /tmp/sidecar2.log)"
grep -vE 'UserWarning|warnings.warn|Axes3D|may be due' /tmp/sidecar2.log | head -12
