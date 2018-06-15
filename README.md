# 简介

* 程序实现了室内自主导航机器人。基本功能有激光SLAM建图、AMCL定位、机器人运动规划、车辆监控和任务规划。
* 底盘采用[Autolabor](http://www.autolabor.com.cn)公司的autolabor pro1，激光雷达选用[rplidar A1](http://www.slamtec.com/cn/Lidar/A1)，Nvidia TX2 作为主控，操作系统Ubuntu 16.04,基于ROS kinetic实现。

---

[TOC]

# 配置步骤

# 1. 安装依赖项
```shell
$ sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard
$ sudo apt-get install ros-kinetic-gmapping ros-kinetic-move-base ros-kinetic-amcl \
  ros-kinetic-map-server ros-kinetic-dwa-local-planner ros-kinetic-global-planner

```

# 2. 建立ros工作空间,clone 代码
```shell
$ mkdir -p ~/auto_ws/src
$ cd ~/auto_ws/src
$ git clone https://github.com/kinglintianxia/autolabor_pro1.git
$ cd ~/auto_ws
$ catkin_make
```
## 代码树形图：
```shell
.
├── autolabor_pro1
├── autolabor_pro1_description
├── autolabor_pro1_driver
├── autolabor_pro1_nav
└── README.md
```

# 3. autolabor pro1 底盘配置
## autolabor pro1底盘驱动下载与解压，代码中底盘驱动位于autolabor_pro1_driver目录下
```
$ wget http://autolabor.cn/file/AutolaborPro1_ROS_Driver_Package20180115.zip
$ unzip -d ./ AutolaborPro1_ROS_Driver_Package20180115.zip
```
## autolabor_pro1_driver节点
autolabor_pro1_driver节点通过串口和底盘进行通讯，实现控制。具体通讯协议见[AutolaborPro1产品使用手册.pdf](http://www.autolabor.cn/file/AutolaborPro1%E4%BA%A7%E5%93%81%E4%BD%BF%E7%94%A8%E6%89%8B%E5%86%8C20180424.pdf)

* 发布的topic:
```
轮式⾥程计：/wheel_odom (nav_msgs/Odometry),
剩余电池电量：/remaining_battery (std_msgs::Int32)
```
* 发布的tf:
```
odom->base_link

```
* 订阅的topic:
```
/cmd_vel (geometry_msgs/Twist)
```
## 添加udev rules
机器人中可能会使用多个串口，比如本例中底盘和rplidar都是串口通讯，所以在ubuntu `/dev`目录下会出现 /dev/ttyUSB0 和 /dev/ttyUSB1，为了使程序识别正确对应的器件，使用udev rules对串口进行重映射并赋权限。格式如下：
```shell
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE:="0666", SYMLINK+="autolabor_pro1"
```
获取Vendor id和Product id，有两种方法:

* 在能分辩串口器件的情况下，使用
```
$ lsusb
> ID `0403:6001` => (idVendor:idProduct)
```
* 不能分辩串口器件的情况下，使用ttyUSB序号获取
```
$ udevadm info -q all -n /dev/ttyUSB0 | grep ID_VENDOR_ID
> ID_VENDOR_ID=0403 => (idVendor)

$ udevadm info -q all -n /dev/ttyUSB0 | grep ID_MODEL_ID
> ID_MODEL_ID=6001 => (idProduct)
```
编写好的`autolabor.rules`文件位于`autolabor_pro1_driver/rules`目录下，使用命令将其拷贝到`/etc/udev/rules.d/`目录下，并重启服务
```shell
$ sudo cp autolabor.rules /etc/udev/rules.d/
$ sudo service udev restart
```
重新插拔串口后，可以在`/dev`目录下看到
```shell
autolabor_pro1->/dev/ttyUSB*
```
最后在autolabor_pro1_driver/launch/driver.launch中将`/dev/ttyUSB0`改为`/dev/autolabor_pro1`，这样就不用担心器件插入顺序和权限问题了。

## 上位机控制底盘
将底盘串口线插入笔记本USB中，并打开上位机控制按钮，启动launch文件
```shell
$ roslaunch autolabor_pro1_nav auto_driver.launch
```
即可实现使用游戏手柄或键盘控制底盘移动。

* 游戏手柄控制
>一直按着`1`键使能遥控;
> 左手摇杆控制行进方向;
> 右手**上**按键使能加速;

* 键盘控制
```
   u    i    o
   j    k    l
   m    ,    .
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
```
## 标定里程计
标定详情参考`ros_by_example_indigo_volume_1`-7.4节。在地板上测量标记出1m的长度，用于标定Linear。

* Linear Calibration
```shell
$ roslaunch autolabor_pro1_nav auto_driver.launch
$ rosrun autolabor_pro1_nav calibrate_linear.py
$ rosrun rqt_reconfigure rqt_reconfigure
```
![Linear Calibration](https://github.com/kinglintianxia/autolabor_pro1/blob/master/autolabor_pro1/img/rqt_linear.png)
```
1.将小车移动到1m标识的起点，选择rqt_reconfigure窗口的`start\_test`，按钮，小车启动往前移动，测量实际前进距离记为`real_dis`
2.ratio = real_dis/1m
3.修改rqt_reconfigure GUI 窗口`odom_linear_scale_correction`为`ratio`
4.重复以上步骤，直到精度达到满意。
```
将autolabor_pro1_driver/launch/driver.launch文件中`reduction_ratio`改为1/ratio

* Angular Calibration
```shell
$ roslaunch autolabor_pro1_nav auto_driver.launch
$ rosrun autolabor_pro1_nav calibrate_angular.py
$ rosrun rqt_reconfigure rqt_reconfigure
```
![Angular Calibration](https://github.com/kinglintianxia/autolabor_pro1/blob/master/autolabor_pro1/img/rqt_angular.png)
```
1.将小车移动到相对空旷的地方，在地面上贴好0°和360°标识，选择rqt_reconfigure窗口的`start\_test`，按钮，小车旋转360°，测量实际旋转角度`real_ang`。
2.ratio = real_ang/360°
3.修改rqt_reconfigure GUI 窗口`odom\_angular\_scale\_correction`为`ratio`
4. 重复以上步骤，直到精度达到满意。
```
将autolabor_pro1_driver/launch/driver.launch文件中`model_param`改为`model_param/ratio`

# 4. rplidar A1 配置
## rplidar ROS驱动下载，代码中rplidar驱动位于auto_ws/src目录下
```
$ cd ~/auto_ws/src
$ git clone https://github.com/robopeak/rplidar_ros.git
$ cd ~/auto_ws && catkin_make
```
最近几天rplidar_ros代码更新加入rplidar A3的支持，但是发现对rplidar A1驱动失败，不能读取雷达数据，可以从我的仓库[下载](https://github.com/kinglintianxia/rplidar_ros.git)
## 添加udev rules
机器人中可能会使用多个串口，比如本例中底盘和rplidar都是串口通讯，所以在ubuntu `/dev`目录下会出现 /dev/ttyUSB0 和 /dev/ttyUSB1，为了使程序识别正确对应的器件，使用udev rules对串口进行重映射并赋权限。
```shell
$ cd rplidar_ros/scripts/
$ sudo cp rplidar.rules /etc/udev/rules.d/
$ sudo service udev restart
```
重新插拔rplidar后，可以在`/dev`目录下看到
```shell
rplidar->/dev/ttyUSB*
```
最后在rplidar_ros/launch/rplidar.launch中将`/dev/ttyUSB0`改为`/dev/rplidar`，这样就不用担心器件插入顺序和权限问题了。
## rplidar测试
```shell
$ roslaunch rplidar_ros view_rplidar.launch
```
在rviz中可以看到雷达点云数据。
![view_rplidar](https://github.com/kinglintianxia/autolabor_pro1/blob/master/autolabor_pro1/img/rplidar.png)

# 5. 机器人模型
为了在激光SLAM建图、move_base运动规划和amcl定位中更直观的观测机器人的footprint，建立URDF模型,模型按照真实尺寸建立。
URDF模型package位于autolabor_pro1/autolabor_pro1_description下，详细教程见`ros_by_example_indigo_volume_2`-4章。模型链接关系如下:
```shell
root Link: base_footprint has 1 child(ren)
    child(1):  base_link
        child(1):  base_l1_wheel_link
        child(2):  base_l2_wheel_link
        child(3):  base_laser_bottom
            child(1):  base_laser_middle
                child(1):  base_laser
        child(4):  base_r1_wheel_link
        child(5):  base_r2_wheel_link
```
启动rviz 观察机器人URDF模型:
```
$ roslaunch autolabor_pro1_description auto_pro1_laser_view.launch
```
![URDF](https://github.com/kinglintianxia/autolabor_pro1/blob/master/autolabor_pro1/img/urdf.png)
* 最后，要将rplidar_ros/launch/rplidar.launch中的`frame_id`改为`base_laser`以和URDF模型保持一致。

# 6. 机器人运动规划
运动规划库使用ROS move_base, 关于move_base的详细教程见[move_base ros wiki](http://wiki.ros.org/move_base)和`ros_by_example_indigo_volume_2`-7、8章和[ROSBot Tutorials](https://husarion.com/tutorials/ros-tutorials/7-path-planning/)
* move_base 配置文件位于autolabor_pro1_nav/config下。config下几个配置文件选用Dynamic Window Approach（dwa）算法，修改自autolabor2.5的配置文件。config/0文件下为turtlebot和ROSBot的配置文件，最终经过调试发现autolabor2.5的配置文件性能较好。
## move_base测试
* 测试move_base基本功能。注意避免机器人碰撞。
```shell
$ roslaunch autolabor_pro1_nav auto_move_base_blank_map.launch
```
使用Rviz工具栏中的**2D Nav Goal**按钮选择Goal，测试move_base是否正常工作。

* 测试move_base避障功能。注意避免机器人碰撞。
```shell
$ roslaunch autolabor_pro1_nav auto_move_base_blank_map_with_obstacle.launch
```
使用Rviz工具栏中的**2D Nav Goal**按钮选择Goal，测试move_base避障功能是否正常工作。

# 7. 激光SLAM建图
激光SLAM建图部分使用gmapping和Google cartographer开源方案。
## gmapping建图
* gmapping开源方案已经整合到ROS库中,详细参数说明见[gmapping ros wiki](http://wiki.ros.org/gmapping)。我们比较关心的参数有：
> "base_frame"，这里为"base_footprint"
> "odom_frame"，这里为 "odom"
> 建图分辨率"delta"，这里选为"0.05",单位为m。
* 建图
```shell
$ roslaunch autolabor_pro1_nav auto_gmapping.launch
```
使用键盘、手柄或者在使用Rviz工具栏中的**2D Nav Goal**按钮选择Goal，进行环境建图。
建图完成后保存地图,并将地图文件`map_name.pgm`和`map_name.yaml`放入autolabor_pro1_nav/maps文件夹下备用。
```shell
$ rosrun map_server map_saver -f map_name
```
## cartographer建图
cartographer方案为Google公司开源,详细教程见[Cartographer](https://google-cartographer.readthedocs.io/en/latest/)。

* 首先下载源码编译安装。由于cartographer要求环境比较特殊，需要新建ROS工作空间。
```shell
$ mkdir -p ~/carto_ws/src && cd ~/carto_ws/src
$ cd ~/carto_ws
$ wstool init src

# Install wstool and rosdep.
$ sudo apt-get update
$ sudo apt-get install -y python-wstool python-rosdep ninja-build

# Merge the cartographer_ros.rosinstall file and fetch code for dependencies.
$ wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall
$ wstool update -t src

# Install proto3.
$ ./src/cartographer/scripts/install_proto3.sh

# Install deb dependencies.
# The command 'sudo rosdep init' will print an error if you have already
# executed it since installing ROS. This error can be ignored.
$ sudo rosdep init
$ rosdep update
$ rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

# Build and install.
$ catkin_make_isolated --install --use-ninja
$ source install_isolated/setup.bash
```
* 测试cartographer，如果测试OK，说明cartographer已经编译完成。
```shell
# Download the 2D backpack example bag.
wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/cartographer_paper_deutsches_museum.bag

# Launch the 2D backpack demo.
roslaunch cartographer_ros demo_backpack_2d.launch bag_filename:=${HOME}/Downloads/cartographer_paper_deutsches_museum.bag
```
* 配置cartographer+rplidar雷达
为了适配rplidar，需要更改.lua配置文件。下载[rplidar_2d.lua](https://github.com/kinglintianxia/cartographer_ros/blob/master/cartographer_ros/launch/rplidar_2d.launch)放到`cartographer_ros/cartographer_ros/configuration_files`目录下。下载[rplidar_2d.launch](https://github.com/kinglintianxia/cartographer_ros/blob/master/cartographer_ros/launch/rplidar_2d.launch)放到`cartographer_ros/cartographer_ros/launch`目录下。重新编译程序:
```shell
$ cd ~/carto_ws
$ catkin_make_isolated --install --use-ninja
$ source install_isolated/setup.bash
```
* cartographer+rplidar雷达建图
```shell
$ roslaunch autolabor_pro1_nav auto_cartographer.launch
```
注意将launch文件中`<include file = "/home/king/auto_ws/src/cartographer_ros/cartographer_ros/launch/rplidar_2d.launch" />`改为自己的路径。
使用键盘、手柄或者在使用Rviz工具栏中的**2D Nav Goal**按钮选择Goal，进行环境建图。
建图完成后保存地图,并将地图放入autolabor_pro1_nav/maps文件夹下备用。
```shell
$ rosrun map_server map_saver -f map_name
```

# 8. AMCL定位
建好地图之后, ROS 提供**amcl**包 (adaptive
Monte Carlo localization) 根据当前laser输入和/odom自动定位机器人在地图中位置。配合move_base包可以使机器人实现在地图中自主导航。
amcl配置文件为`autolabor_pro1_nav/launch/auto_amcl.launch`。

* 启动amcl:
```shell
$ roslaunch autolabor_pro1_nav auto_pro_amcl.launch
```
启动程序后一般需要在Rviz中通过**2D Pose Estimate**按钮设定amcl定位初值，大致标示机器人在地图中的位置，加快粒子群收敛。

![amcl0](https://github.com/kinglintianxia/autolabor_pro1/blob/master/autolabor_pro1/img/amcl0.png)

最终，会看到机器人在地图中很好的定位。
![amcl1](https://github.com/kinglintianxia/autolabor_pro1/blob/master/autolabor_pro1/img/amcl1.png)

# 9. Behavior Trees 任务规划
使用行为树（Behavior Trees）进行机器人任务规划。详细教程见`ros_by_example_indigo_volume_2`-3.9和3.10节。由于现在ROS中没有现成的行为树库，使用第三方库`pi_trees`实现。
```shell
$ sudo apt-get install graphviz-dev libgraphviz-dev \
python-pygraph python-pygraphviz gv
$ cd ~/auto_ws/src
$ git clone -b indigo-devel https://github.com/pirobot/pi_trees.git
$ cd ~/auto_ws && catkin_make
```
在地图中使用**Publish Point**按钮选择几个点作为巡逻点，同时监控底盘电池电量，任务规划图：
![pi_trees](https://github.com/kinglintianxia/autolabor_pro1/blob/master/autolabor_pro1/img/pi_tree.png)
添加启动任务规划：
```shell
$ roslaunch autolabor_pro1_nav auto_pro_amcl.launch
$ rosrun autolabor_pro1_nav patrol_tree.py
```
最终，机器人实现自主导航视频：

[![auto_nav](https://github.com/kinglintianxia/autolabor_pro1/blob/master/autolabor_pro1/img/auto_nav.png)
](http://v.youku.com/v_show/id_XMzYyNDc3NTgxMg==.html?spm=a2hzp.8244740.0.0)

# 10. 使用TX2作为主控
Jetson TX2采用 NVIDIA Maxwell™ 架构、256 颗 NVIDIA CUDA® 核心 和 64 位 CPU，并且其设计非常节能高效(7.5W)。此外，它还采用了深度学习、 计算机视觉、GPU 计算和图形方面的新技术，非常适合嵌入式 AI 计算。适合机器人、无人机、智能摄像机和便携医疗设备等智能终端设备。
将程序部署到TX2上，会遇到rplidar插入USB不能识别串口的问题，需要重新编译内核，加入CP210x串口驱动支持，解决的方法见[博客](https://blog.csdn.net/gzj2013/article/details/77069803)。
TX2部署视频：

[![TX2部署视频](https://github.com/kinglintianxia/autolabor_pro1/blob/master/autolabor_pro1/img/tx2.png)](http://v.youku.com/v_show/id_XMzYyOTk5MDE0OA==.html?spm=a2hzp.8244740.0.0)
