
#git install
############
sudo apt-get install git -y

#python 2.7 and 3 install
sudo apt-get install python python-pip python3 python3-pip -y

#ros install and configure
##########################
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-desktop-full -y
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y
sudo rosdep init
rosdep update
echo "source ~/qb/ros_ws/devel/setup.bash" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://192.168.0.178:11311" >> ~/.bashrc
echo "export ROS_HOSTNAME=192.168.0.178" >> ~/.bashrc
echo "export ROS_IP=192.168.0.178" >> ~/.bashrc

#install qB ROS dependecies
################################
sudo apt-get install -y ros-melodic-map-server ros-melodic-gmapping ros-melodic-rosbridge-server ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-web-video-server ros-melodic-amcl ros-melodic-move-base ros-melodic-dwa-local-planner ros-melodic-rosapi ros-melodic-pointcloud-to-laserscan ros-melodic-rtabmap-ros

#clone qB and setup
########################
cd ~/
git clone https://github.com/whoobee/qb.git
cd ~/qB/ros_ws
git submodule update --init --recursive
catkin_make clean

#install Odrive
###############
mkdir ~/qb/tools
cd ~/qb/tools
git clone https://github.com/madcowswe/ODrive
cd ODrive/tools
sudo pip install monotonic # required for py < 3
python setup.py sdist
sudo pip install dist/odrive-*.tar.gz

#seup ydlidar
#############
sudo sh ~/qb/ros_ws/src/ydlidar_ros/startup/initenv.sh

#setup realsense cameras
########################
sudo apt-get install ros-melodic-realsense2-camera -y
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
sudo apt-get install librealsense2-dkms -y
sudo apt-get install librealsense2-utils -y

#make qB
#############
catkin_make

#enable power button listener script
####################################
#enable shutdown for all users
sudo chmod u+s /sbin/shutdown
#copy custome power button handler
sudo cp /home/whoobee/qb/script/powerbtn/power /etc/acpi/events/power
#install powerbutton listener
sudo service acpid restart
sudo cp /home/whoobee/qb/scripts/powerbtn/power_listener.service /lib/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable power_listener.service
sudo systemctl start power_listener.service