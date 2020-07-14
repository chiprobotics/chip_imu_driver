# chip_imu_driver
ROS driver for Chip Robotics IMU Sensor  https://chiprobotics.com/chip-robotics-imu-sensor/

For imu visualization make sure you have the rviz imu plugin installed 

e.g. for melodic

sudo apt-get install ros-melodic-rviz-imu-plugin


create a directory to install the driver and cd into the directory

mkdir src

cd src

catkin_init_workspace

git clone https://github.com/chiprobotics/chip_imu_driver.git

cd ..

catkin_make

source ./devel/setup.bash

roslaunch chip_imu_driver example_rviz.launch

or

roslaunch chip_imu_driver example.launch


make sure you have the right permission set on /dev/ttyUSB0 (for a quick hack just use sudo chmod 777 /dev/ttyUSB0)

instruction on using udev will be coming soon.

It is recommended you first run the IMU with the windows or linux software before running ROS.