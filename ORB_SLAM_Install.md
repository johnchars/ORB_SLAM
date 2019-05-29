### ORB—SLAM安装

---

##### 机器型号

硬件：XbotU-bj008

系统：Ubuntu 16.04.1

ROS版本：Kinect 1.12.14

#### 提供脚本安装方式
```shell
cd
mkdir -p slam_ws/src && cd slam_ws/src
catkin_init_workspace
cd .. && catkin_make
git clone https://github.com/johnchars/ORB_SLAM.git
cd ORB_SLAM
chmod +x install.bash
./install.bash
```
#### raulmur的readme.md 安装
##### 依赖项安装

* Boost

  ```velocity
  sudo apt-get install libboost-all-dev
  ```

  Boost 库用于同时启动不同的线程，ORB-SLAM分为三个线程Tracking, Local Mapping and Loop Closing

  

- ROS

  参考官方[wiki](http://wiki.ros.org/cn/kinetic/Installation)

- OpenCV

  ```shell
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
  ```

  > 这里也可以使用[官网](https://opencv.org/releases/)下载，注意选择2.4版本; make -j*中数字可以自己决定

- g2o

  ```shell
  	sudo apt-get install libeigen3-dev
  ```

  安装g2o图优化库需要安装eigen库，使用了一个修改过的g2o库，在Thirdparty/下有这个库，不需要下载

- DBow2

  同样的第三库，在Thirdparty/下有这个库，用于回环检测。

##### 编译安装

1. 创建一个安装位置

   ```shell 
   cd && mkdir -p slam_ws/src && cd slam_ws/src
   catkin_init_workspace
   cd .. && catkin_make
   echo "source ~/slam_ws/devel/setup.bash" >> ~/.bashrc
   git clone https://github.com/raulmur/ORB_slam_ws.git ORB_slam_ws
   ```

   

2. [TODO] 修改环境变量 ROS_PAKCAGE_PATH，在~/.bashrc末尾增加

   > export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/xbot/slam_ws/src

3. 编译g2o

   ```shell
   cd ~/slam_ws/src/ORB_SLAM/Thirdparty/g2o/
   mkdir build && cd build
   cmake .. -DCMAKE_BUILD_TYPE=Release
   make -j4
   ```

4. 编译DBoW2

   ```shell
   cd ../../DBoW2
   mkdir build && cd build
   cmake .. -DCMAKE_BUILD_TYPE=Release
   make -j4
   ```

5. 编译ORB_SLAM

   删除manifest.xml 中的 <depend package="opencv2"/>

   在ORBextractor.cc中添加头文件，路径是/home/xbot/slam_ws/src/ORB_SLAM/src

   > #include <opencv2/features2d/features2d.hpp>
   >
   > #include <opencv2/imgproc/imgproc.hpp>

   在/home/xbot/slam_ws/Thirdparty/g2o/g2o/solvers中的linear_solver_eigen.h中修改

   ```vim
   56: typedef Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, SparseMatrix::Index> PermutationMatrix;
   修改为
   56: typedef Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> PermutationMatrix;
   ```

   在/home/xbot/slam_ws/src/ORB_slam_ws下修改CMakeLists.txt文件，增加

   ```txt
   find_package(Boost COMPONENTS system)
   
   include_directories(
   ${Boost_INCLUDE_DIRS} // adding this line
   )
   target_link_libraries(${PROJECT_NAME}
   ${Boost_LIBRARIES}
   )
   ```

   开始编译...

   ```shell
   cd ~/slam_ws/src/ORB_SLAM/
   mkdir build && cd build
   cmake .. -DROS_BUILD_TYPE=Release
   make -j4
   编译成功显示
   Build type: RelWithDebInfo
   -- Boost version: 1.58.0
   -- Found the following Boost libraries:
   --   system
   -- Configuring done
   -- Generating done
   -- Build files have been written to: /home/xbot/slam_ws/src/ORB_SLAM/build
   [  0%] Built target rospack_genmsg_libexe
   [  0%] Built target rosbuild_precompile
   [  5%] Linking CXX executable ../bin/ORB_SLAM
   [100%] Built target ORB_SLAM
   ```

6. 可能的编译错误解决

- [x] 'FAST' was not declared in this scope FAST(cellImage, cellKeyPoints [i] [j], fastTh, true);

> 解决方法GitHub [#44](https://github.com/raulmur/ORB_SLAM/issues/44)
>
> 在src/ORBextractor.cc中增加
>
> > #include <opencv2/opencv.hpp>
>
> 增加在#include <opencv2/core/core.hpp>前

- [x] DSO missing from command line

> 解决方Github[#552](https://github.com/raulmur/ORB_SLAM2/issues/552)
>
> 在CMakeLists.txt中增加Boost头文件
>
> include_directories(
>
> ${Boost_INCLUDE_DIRS} #增加这一行
>
> )

- [x] cmake Configuring incomplete, errors occurred!

  解决方法，检查ROS_PACKAGE_PATH路径，在.bashrc文件下

- [ ] 其他问题可以在[ORB_SLAM](https://github.com/raulmur/ORB_SLAM/issues)下检索关键词

##### 运行

a. 解压Data下的词库和设置文件

```shell
cd /home/xbot/slam_ws/src/ORB_SLAM/Data
xbot@nuc:~/slam_ws/src/ORB_SLAM/Data$ tar xzvf ORBvoc.txt.tar.gz 
ls 
增加 ORBvoc.txt文件
```

b.逐个文件启动

```shell
Ctrl+Alt+t
roscore
Ctrl+Alt+t
cd /home/xbot/slam_ws/src/ORB_SLAM
rosrun ORB_SLAM ORB_SLAM Data/ORBvoc.txt Data/Setting.yaml
```

[todo] adding picture orb_node_success

```shell 
Ctrl+Alt+t
rosrun image_view image_view image:=/ORB_SLAM/Frame _autosize:=true
```

[todo] adding picture

```shell
cd /home/xbot/slam_ws/src/ORB_SLAM
rosrun rviz rviz -d Data/rviz.rviz
```

[todo] adding picture orb_rviz_d

c.使用launch文件启动

```shell
cd /home/xbot/slam_ws/src/ORB_SLAM
roslaunch ExampleGroovyOrNewer.launch
```

d. 使用Example.bag 检测安装是否成功

- 下载[bag](webdiis.unizar.es/~raulmur/orbslam/downloads/Example.bag.tar.gz)文件

- 切换到bag文件路径，这里在/home/xbot/下建立了Dataset/文件夹，并将文件放到这里

  ```shell
  cd /home/xbot/Dataset/
  tar xzvf Example.bag.tar.gz
  rosbag play --pause Example.bag #按空格开始
  ```

[todo] 效果图

特别感谢这篇[教程](https://blog.csdn.net/lixujie666/article/details/80475451)，解决了一周的难受

##### 使用单目摄像头

这里需要注意的是ORB_slam_ws只接收来自名为/camera/image_raw的topic信息，如果使用单目摄像头如logitech c270i或者Realsense D415的color camera，需要修改topic名称。

首先标定摄像头的内参矩阵，可以通过OpenCV，MATLAB，ROS等方式得到

内容一般包含

```txt
Camera intrinsic matrix: 
[708.0230464853036, 0, 313.299267971209;
 0, 714.8509096655857, 189.0366118434282;
 0, 0, 1]

Distorted arguments: 
[-0.09135352304365538, 0.9392941836066392, -0.00487793090233253, -0.005752201943911559, -2.133389933308491]
```

其中 3*3矩阵对应着 fx=708.023,fy=714.850,cx=313.2992,cy=189.0366

畸变系数k1=-0.0913, k2=0.9393, p1=-0.0049, p2=-0058。

如果使用的是webcam，也就是USB插入的摄像头，需要引入一个节点发布image消息，参考[publisher_image](https://yt.droid.ac.cn/xbot-u/vslam_ws-evaluation/tree/master/ORB/Publisher%20of%20image)

如果使用的是D415，可以在rs_camera.launch文件中对应的nodelet.launch.xml文件中增加remap

```txt
<node pkg="nodelet" type="nodelet" name="realsense2_camera" args="load realsense2_camera/RealSenseNodeFactory $(arg manager)">
    <remap from="/camera/color/image_raw" to="/camera/image_raw" /> 
```

再启动rs_camera.launch即可通过摄像头来使用ORB
