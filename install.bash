#!/usr/bin/env bash

sudo apt-get install libboost-all-dev

echo "install libboost finished"

cd && mkdir Repo && cd Repo
git clone https://github.com/opencv/opencv.git
cd opencv
git branch -a
git checkout origin/2.4
git branch -m origin/2.4 opencv-2.4
git branch
mkdir build && cd build
cmake .. && make -j4
sudo make install
echo "opencv2.4 has been installed"

sudo apt-get install libeigen3-dev

cd && mkdir -p slam_ws/src && cd slam_ws/src
catkin_init_workspace
cd .. && catkin_make
echo "source ~/slam_ws/devel/setup.bash" >> ~/.bashrc
git clone https://github.com/johnchars/ORB_SLAM.git
echo "ORB_SLAM has been clone"

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/xbot/slam_ws/src

cd ~/slam_ws/src/ORB_SLAM/Thirdparty/g2o/
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ~/slam_ws/src/ORB_SLAM/Thirdparty/DBoW2
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ~/slam_ws/src/ORB_SLAM/
mkdir build && cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j4

cd ~/slam_ws/src/ORB_SLAM/Data/
tar xzvf ORBvoc.txt.rar.gz

echo "install finished"

