# Script for setting up the Husky and joystick
if [ $# -ne 0 ]
  then
    sudo chmod a+rw $1
    ls -l $1
    # works for Logitech extreme 3D Pro joystick
    sudo modprobe usbserial vendor=0x067b product=0x2303
  else
  	echo "  please specify USB port (/dev/ttyUSB<number>)"
fi