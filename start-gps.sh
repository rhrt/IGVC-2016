# Script for getting basic GPS data
# Ensure the GPS is not plugged into a USB3.0 port
if [ $# -ne 0 ]
  then
    sudo chmod a+rw $1
    ls -l $1

    gpsd -S 4000 $1
    xgps &
  else
    echo "  please specify USB port (/dev/ttyUSB<number>)"
fi