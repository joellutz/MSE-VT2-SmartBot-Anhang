# Installation of the whole setup from the NTB project SmartBot (Joël Lutz, August 2018)
# Installed software: ROS, Gazebo, OpenAI Gym, gym-gazebo, MoveIt, OpenAI Gym baselines,
# tensorflow, keras, freenect, arbotix drivers, pincher arm packages etc.

# refresh apt
sudo apt-get update
sudo apt-get upgrade # takes forever

# ++++++++++++ install ROS ++++++++++++
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop # takes forever
sudo rosdep init
rosdep update
echo "# For ROS" >> ~/.bashrc
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
# checking ROS installation
printenv | grep ROS
# should yield:
# ROS_ROOT=/opt/ros/kinetic/share/ros
# ROS_PACKAGE_PATH=/opt/ros/kinetic/share
# ROS_MASTER_URI=http://localhost:11311
# ROS_VERSION=1
# ROSLISP_PACKAGE_DIRECTORIES=
# ROS_DISTRO=kinetic
# ROS_ETC_DIR=/opt/ros/kinetic/etc/ros

# ++++++++++++ install Gazebo 7 ++++++++++++
sudo apt remove '.*gazebo.*' '.*sdformat.*' '.*ignition-.*'
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
# alternative mirror site: sudo sh -c 'echo "deb http://apt-mirror.jderobot.org/osrf-gazebo/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install gazebo7
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
# check gazebo
gazebo --version
# should yield:
# Gazebo multi-robot simulator, version 7.14.0
# Copyright (C) 2012 Open Source Robotics Foundation.
# Released under the Apache 2 License.
# http://gazebosim.org

# no matter what, reboot
sudo reboot

# ++++++++++++ install Visual Studio Code ++++++++++++
sudo add-apt-repository -y "deb https://packages.microsoft.com/repos/vscode stable main"
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys EB3E94ADBE1229CF
sudo apt update
sudo apt -y install code
# install python extension in Visual Studio Code
code --install-extension ms-python.python
# install pylint (for Visual Studio Code)
sudo pip install -U "pylint<2.0.0"

# ++++++++++++ installing OpenAI Gym (and pip) ++++++++++++
sudo apt-get install -y python-dev python-pip python3-dev python3-pip python3-numpy python3-wheel python-numpy python-dev cmake zlib1g-dev libjpeg-dev xvfb libav-tools xorg-dev python-opengl libboost-all-dev libsdl2-dev swig

sudo apt-get remove python-pip
curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
sudo python get-pip.py
sudo python3 get-pip.py

cd ~/Documents
git clone https://github.com/openai/gym.git
cd gym
sudo pip install -e .

cd

# ++++++++++++ installing gym-gazebo (with the pincher environment) ++++++++++++
sudo pip3 install rospkg catkin_pkg
sudo apt-get install python3-pyqt4

sudo apt-get install \
cmake gcc g++ qt4-qmake libqt4-dev \
libusb-dev libftdi-dev \
python3-defusedxml python3-vcstool \
ros-kinetic-octomap-msgs        \
ros-kinetic-joy                 \
ros-kinetic-geodesy             \
ros-kinetic-octomap-ros         \
ros-kinetic-control-toolbox     \
ros-kinetic-pluginlib	       \
ros-kinetic-trajectory-msgs     \
ros-kinetic-control-msgs	       \
ros-kinetic-std-srvs 	       \
ros-kinetic-nodelet	       \
ros-kinetic-urdf		       \
ros-kinetic-rviz		       \
ros-kinetic-kdl-conversions     \
ros-kinetic-eigen-conversions   \
ros-kinetic-tf2-sensor-msgs     \
ros-kinetic-pcl-ros \
ros-kinetic-navigation


cd ~/Documents
git clone https://github.com/joellutz/gym-gazebo
cd gym-gazebo
sudo pip install -e .

echo "alias killgazebogym='killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient'" >> ~/.bashrc
source ~/.bashrc

cd

# ++++++++++++ getting the gazebo files for the kinect, table plane and the object to pick update ++++++++++++
cd ~/Documents
git clone https://github.com/joellutz/gazebo-models.git
cd

# ++++++++++++ install MoveIt ++++++++++++
sudo apt-get install -y ros-kinetic-moveit
sudo apt-get install ros-kinetic-joint-state-controller
sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers
# maybe needed
# rosdep update
# sudo apt-get update
# sudo apt-get dist-upgrade
# sudo apt-get install ros-kinetic-catkin python-catkin-tools

# ++++++++++++ install OpenAI Gym baselines (with the code for some pincher environment algorithms) ++++++++++++
sudo apt-get update && sudo apt-get install cmake libopenmpi-dev python3-dev zlib1g-dev
cd ~/Documents
git clone https://github.com/joellutz/baselines.git
cd baselines/
sudo pip install -e .

cd

# ++++++++++++ install tensorflow ++++++++++++

# sudo apt-get install python-pip python-dev
# sudo pip install -U pip
# sudo pip install -U tensorflow # ging nicht (Fehlermeldung: Cannot uninstall 'enum34'. It is a distutils installed project and thus we cannot accurately determine which files belong to it which would lead to only a partial uninstall.)
# ging auch nicht.
# sudo pip install --upgrade https://storage.googleapis.com/tensorflow/linux/cpu/tensorflow-1.10.0-cp27-none-linux_x86_64.whl

# sudo python get-pip.py
# sudo pip install --ignore-installed tensorflow
# sudo pip install testresources
# sudo pip uninstall tensorflow

# needed?
# sudo apt-get remove python-pip python-dev
# sudo apt-get install python-pip python-dev

# maybe after all this is all we need (from https://github.com/pypa/pip/issues/5373#issuecomment-392742141)
sudo easy_install pip
sudo pip uninstall tensorflow
sudo pip install tensorflow==1.5
# checking tensorflow installation
python -c "import tensorflow as tf; print(tf.__version__)"
# should yield
# 1.5.0

# ++++++++++++ install keras ++++++++++++
sudo pip install keras

# ++++++++++++ install freenect (drivers for real kinect camera) ++++++++++++
sudo apt-get install ros-kinetic-freenect-launch

# ++++++++++++ install catkin (not really necessary, should already be shipped with ROS) ++++++++++++
sudo apt-get install ros-kinetic-catkin
sudo apt-get install python-catkin-tools # this may me necessary


# ++++++++++++ getting the pincher arm packages ++++++++++++
cd ~/Documents
mkdir -p pincherArm/src
cd pincherArm/src
git clone https://github.com/joellutz/pincher_arm.git
cd ..
catkin build
echo "source ~/Documents/pincherArm/devel/setup.bash" >> ~/.bashrc
echo "export TURTLEBOT_ARM1=pincher" >> ~/.bashrc
source ~/.bashrc

# ++++++++++++ getting the correct arbotix drivers (for the real pincher arm) ++++++++++++
cd ~/Documents
mkdir -p arbotix/src
cd arbotix/src
git clone https://github.com/corb555/arbotix_ros.git
cd ..
catkin build
echo "source ~/Documents/arbotix/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc