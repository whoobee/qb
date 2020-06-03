
#git install
############
sudo apt-get install git -y

#python 2.7 and 3 install
sudo apt-get install python python-pip python3 python3-pip -y

#clone MiroBot and setup
########################
cd ~/
git clone https://github.com/whoobee/mirobot.git
cd ~/mirobot/ros_ws
git submodule update --init --recursive
catkin_make clean

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
echo "source ~/mirobot/ros_ws/devel/setup.bash" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://192.168.0.26:11311" >> ~/.bashrc
echo "export ROS_HOSTNAME=192.168.0.26" >> ~/.bashrc
echo "export ROS_IP=192.168.0.26" >> ~/.bashrc

#install MiroBot ROS dependecies
################################
sudo apt-get install ros-melodic-map-server ros-melodic-gmapping ros-melodic-rosbridge-server ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-web-video-server ros-melodic-amcl ros-melodic-move-base ros-melodic-rosapi -y


#install Odrive
###############
mkdir ~/mirobot/tools
cd ~/mirobot/tools
git clone https://github.com/madcowswe/ODrive
cd ODrive/tools
sudo pip install monotonic # required for py < 3
python setup.py sdist
sudo pip install dist/odrive-*.tar.gz

#seup ydlidar
#############
sudo sh ~/mirobot/ros_ws/src/ydlidar_ros/startup/initenv.sh

#setup realsense cameras
########################
sudo apt-get install ros-melodic-realsense2-camera -y

#make MiroBot
#############
catkin_make


