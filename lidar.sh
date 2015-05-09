# starting up the Hokuyo LIDAR
sudo modprobe cdc_acm vendor=0x15d1 product=0x0000
rosparam set hokuyo_node/calibrate_time false
rosparam set hokuyo_node/port /dev/ttyACM0
rosparam set hokuyo_node/frame_id /base_laser


