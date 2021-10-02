# test-rig
scripts and other tools for test rig setup

# Setup
cd /home/misc_payload/teensy/

sudo sh synchronize.sh
sudo /etc/init.d/chrony stop
sudo /etc/init.d/chrony start
chronyc sources -v
chronyc tracking
sudo sh -c 'echo 2000 > /sys/module/usbcore/parameters/usbfs_memory_mb'
roscore
roslaunch pointgrey_camera_driver right_mapping.launch 
roslaunch pointgrey_camera_driver left_mapping.launch
cd sensors_ws/
source devel/setup.bash
roslaunch dji_sdk sdk.launch
roslaunch launchers payload_sensors.launch
roslaunch launchers record_mapping_velodyne_imu.launch

data is saved in /home/frc-uav/data/


to calibrate the cameras:

rosrun camera_calibration cameracalibrator.py --size 11x9 --square 0.043 right:=/mapping/right/image_raw left:=/mapping/left/image_raw left_camera:=/mapping/left right_camera:=/mapping/right --approximate=0.01

the files generated are saved in the folder /tmp/calibrationdata.tar.gz

