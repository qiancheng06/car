#!/bin/bash
source /opt/ros/humble/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/davinci-mini/fastdds_udp.xml
timeout 25 python3 -u /home/davinci-mini/racecar/src/racecar/scripts/dbg7.py > /tmp/dbg7.log 2>&1
echo "bytes: $(wc -c < /tmp/dbg7.log)"
grep -vE 'UserWarning|warnings.warn|Axes3D|may be due' /tmp/dbg7.log | head -20
