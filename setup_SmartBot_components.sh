# Installation of the whole setup from the NTB project SmartBot: Robot with Reinforcement Learning (Joël Lutz, August 2018)
# Installed software: ROS, Gazebo, OpenAI Gym, gym-gazebo, MoveIt, OpenAI Gym baselines,
# tensorflow, keras, freenect, arbotix drivers, pincher arm packages etc.

# No guarantee for completeness, that it will work or that only necessary things will be installed!!
# Better take this script as a guideline (just like the pirate codex).


# POSSIBLE ERRORS / CONFUSIONS WHEN RUNNING AN RL ALGORITHM WITH THE PINCHER ENVIRONMENT

# if you can't see a kinect camera in Gazebo, don't worry. Check the ~/Pictures/ folder where the simulated Kinect camrea
# saves all of its depth images. If you see some pictures in there after (or during) running a RL algorithm, everything should be fine.

# if an error like "RuntimeError: Unable to connect to move_group action server 'move_group' within allotted time (5s)" occurs
# after starting an RL algorithm (e.g. ~/Documents/gym-gazebo/examples/pincher_arm/smartbot_pincher_kinect_ddpg.py),
# try starting it again (after killgazebogym) and/or run the installing MoveIt part again.

# ++++++++++++ refresh apt ++++++++++++
sudo apt-get update
sudo apt-get upgrade -y # takes forever

# ++++++++++++ install ROS Kinetic ++++++++++++
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install -y ros-kinetic-desktop # takes forever
sudo rosdep init
rosdep update
echo "# For ROS" >> ~/.bashrc
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential
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

# ++++++++++++ install Gazebo 8 ++++++++++++
sudo apt remove -y '.*gazebo.*' '.*sdformat.*' '.*ignition-.*'
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
# alternative mirror site: sudo sh -c 'echo "deb http://apt-mirror.jderobot.org/osrf-gazebo/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install -y gazebo8																# for gazebo 7: sudo apt-get install -y gazebo7
sudo apt-get install -y ros-kinetic-gazebo8-ros-pkgs ros-kinetic-gazebo8-ros-control		# for gazebo 7: sudo apt-get install -y ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
# check gazebo
gazebo --version
# should yield:
# Gazebo multi-robot simulator, version 8.6.0
# Copyright (C) 2012 Open Source Robotics Foundation.
# Released under the Apache 2 License.
# http://gazebosim.org

# ++++++++++++ installing pip ++++++++++++
# a way to get a newer version of pip (not really needed maybe):
# curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
# sudo python get-pip.py
sudo apt-get install python-pip
sudo pip install numpy==1.14.5
sudo pip install --upgrade cryptography
sudo python -m easy_install --upgrade pyOpenSSL

# checking pip installation
pip -V
# should yield:
# pip 8.1.1 from /usr/local/lib/python2.7/dist-packages/pip (python 2.7)

# ++++++++++++ install Visual Studio Code ++++++++++++
sudo add-apt-repository -y "deb https://packages.microsoft.com/repos/vscode stable main"
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys EB3E94ADBE1229CF
sudo apt update
sudo apt -y install code
# install python extension in Visual Studio Code
code --install-extension ms-python.python
sudo pip install -U "pylint<2.0.0"									# with newer pip: sudo pip install --ignore-installed -U "pylint<2.0.0"

# ++++++++++++ installing OpenAI Gym ++++++++++++
sudo apt-get install -y python-dev python3-dev python3-numpy python3-wheel python-numpy python-dev cmake zlib1g-dev libjpeg-dev xvfb libav-tools xorg-dev python-opengl libboost-all-dev libsdl2-dev swig

cd ~/Documents
git clone https://github.com/openai/gym.git
cd gym
sudo pip install -e .

cd

# ++++++++++++ installing gym-gazebo (with the pincher environment) ++++++++++++
sudo pip install rospkg catkin_pkg
sudo apt-get install -y python3-pyqt4

sudo apt-get install -y \
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
sudo apt-get install -y ros-kinetic-ros-control ros-kinetic-ros-controllers

# ++++++++++++ install OpenAI Gym baselines (with the code for some pincher environment algorithms) ++++++++++++
sudo apt-get update && sudo apt-get install cmake libopenmpi-dev python3-dev zlib1g-dev
cd ~/Documents
git clone https://github.com/joellutz/baselines.git
cd baselines/
sudo pip install -e .

cd

# ++++++++++++ checking tensorflow ++++++++++++
# sudo pip uninstall tensorflow
# sudo pip install tensorflow==1.5
# checking tensorflow installation
python -c "import tensorflow as tf; print(tf.__version__)"
# should yield:
# 1.10.0

# ++++++++++++ install keras ++++++++++++
sudo pip install keras

# ++++++++++++ install freenect (drivers for real kinect camera) ++++++++++++
sudo apt-get install -y ros-kinetic-freenect-launch

# ++++++++++++ install catkin ++++++++++++
sudo apt-get install -y python-catkin-tools


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

# ++++++++++++ getting the keras-rl repository (only needed for one ddpg implementation: ~/Documents/gym-gazebo/examples/pincher_arm/smartbot_pincher_kinect_ddpg3_keras.py) ++++++++++++
cd ~/Documents
git clone https://github.com/keras-rl/keras-rl.git
cd keras-rl
python setup.py install

# ++++++++++++ reboot ++++++++++++
sudo reboot