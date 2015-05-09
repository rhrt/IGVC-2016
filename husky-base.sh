# Script for running the base ROS launch for the Husky
if [ $# -ne 0 ]
  then
    roslaunch husky_base base.launch port:=$1
  else
    echo "  please specify USB port (/dev/ttyUSB<number>)"
fi