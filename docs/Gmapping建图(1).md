## Gmapping建图

注：树莓派需要与microROS控制板ROS_DOMAIN_ID需要一致，可以查看【MicroROS控制板参数配置】来设置microROS控制板ROS_DOMAIN_ID，查看教程【连接MicroROS代理】判断ID是否一致。

### 1、Gmapping简介

- gmapping只适用于单帧二维激光点数小于1440的点，如果单帧激光点数大于1440，那么就会出【[mapping-4] process has died】 这样的问题。
- Gmapping是基于滤波SLAM框架的常用开源SLAM算法。
- Gmapping基于RBpf粒子滤波算法，即时定位和建图过程分离，先进行定位再进行建图。
- Gmapping在RBpf算法上做了两个主要的改进：改进提议分布和选择性重采样。

优点：Gmapping可以实时构建室内地图，在构建小场景地图所需的计算量较小且精度较高。

缺点：随着场景增大所需的粒子增加，因为每个粒子都携带一幅地图，因此在构建大地图时所需内存和计算量都会增加。因此不适合构建大场景地图。并且没有回环检测，因此在回环闭合时可能会造成地图错位，虽然增加粒子数目可以使地图闭合但是以增加计算量和内存为代价。

![image-20231218203929783](image-20231218203929783.png)

### 2、程序功能说明

小车连接上代理，运行程序，rviz中会显示建图的界面，用键盘或者手柄去控制小车运动，直到建完图。然后运行保存地图的指令保存地图。

### 3、查询小车信息

#### 3.1、启动并连接代理

树莓派成功开机后，打开终端输入以下指令打开代理，

```
sh ~/start_agent_rpi5.sh
```

![image-20240122162257264](image-20240122162257264.png)

按下microROS控制板的复位按键，等待小车连接上代理，连接成功如下图所示。

![image-20240122162429192](image-20240122162429192.png)

#### 3.2、进入小车docker

打开另一个终端输入以下指令进入docker，

```
sh ros2_humble.sh 
```

出现以下界面就是进入docker成功，现在即可通过指令控制小车，

![image-20240201151537578](image-20240201151537578.png)

### 4、启动程序

首先启动小车处理底层数据程序，终端输入，

```
ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py
```

![image-20240125192911040](image-20240125192911040.png)

然后，启动rviz可视化建图，输入这个指令之前需要进入同一个docker终端，步骤如下：

在树莓派的中重新打开一个新终端输入指令，

```
docker ps -a
```

能看到你进入docker的ID号以及docker版本，看到up哪一项就是当前启动的docker。

![image-20240319163141675](image-20240319163141675.png)

根据这个id号就能进入同一个docker，注意每次进入docker的id号都是不一样的，输入指令，

```py
docker exec -it ef0e1b7da319 /bin/bash
```

![image-20240319163032633](image-20240319163032633.png)

出现这个画面我们就是进入了同一个docker。

输入指令启动rviz可视化建图

```
ros2 launch yahboomcar_nav display_launch.py
```

![image-20240125192958974](image-20240125192958974.png)

此时还没运行建图节点，所以没有数据。接下来运行建图节点，终端输入**（和上面一样需要进入同一个docker终端）**，

```
ros2 launch yahboomcar_nav map_gmapping_launch.py
```

![image-20240126105953402](image-20240126105953402.png)

然后运行手柄控制或者键盘控制，二选一，终端输入**（和上面一样需要进入同一个docker终端）**，

```py
#键盘
ros2 run yahboomcar_ctrl yahboom_keyboard
#手柄
ros2 run yahboomcar_ctrl yahboom_joy
ros2 run joy joy_node
```

然后控制小车，缓慢的走完需要建图的区域，建图完毕后，输入以下指令保存地图，终端输入**（和上面一样需要进入同一个docker终端）**，

![image-20240126105926852](image-20240126105926852.png)

```
ros2 launch yahboomcar_nav save_map_launch.py
```

会保存一个命名为yahboom_map的地图，这个地图保存在，

```
/root/yahboomcar_ws/src/yahboomcar_nav/maps
```

会有两个文件生成，一个是yahboom_map.pgm，一个是yahboom_map.yaml，看下yaml的内容，

```
image: yahboom_map.pgm
mode: trinary
resolution: 0.05
origin: [-10, -10, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

- image：表示地图的图片，也就是yahboom_map.pgm

- mode：该属性可以是trinary、scale或者raw之一，取决于所选择的mode，trinary模式是默认模式

- resolution：地图的分辨率， 米/像素

- origin：地图左下角的 2D 位姿(x,y,yaw), 这里的yaw是逆时针方向旋转的（yaw=0 表示没有旋转）。目前系统中的很多部分会忽略yaw值。

- negate：是否颠倒 白/黑 、自由/占用 的意义（阈值的解释不受影响）
- occupied_thresh：占用概率大于这个阈值的的像素，会被认为是完全占用。
- free_thresh：占用概率小于这个阈值的的像素，会被认为是完全自由。

### 5、查看节点通讯图

终端输入，

```
ros2 run rqt_graph rqt_graph
```

![image-20240125193219729](image-20240125193219729.png)

如果一开始没有显示，选择【Nodes/Topics(all)】，然后点击左上角的刷新按钮。

### 6、查看TF树

终端输入，

```
ros2 run tf2_tools view_frames
```

![image-20240126113822497](image-20240126113822497.png)

运行完毕后，会在终端的目录下生成两个文件分别是.gv和.pdf文件，其中的pdf文件就是TF树。

![image-20240125193245956](image-20240125193245956.png)



### 7、代码解析

这里只说明建图的map_gmapping_launch.py，这个文件路径是，

```
/root/yahboomcar_ws/src/yahboomcar_nav/launch
```

map_gmapping_launch.py

```
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    slam_gmapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('slam_gmapping'), 'launch'),
         '/slam_gmapping.launch.py'])
    )

    base_link_to_laser_tf_node = Node(
     package='tf2_ros',
     executable='static_transform_publisher',
     name='base_link_to_base_laser',
     arguments=['-0.0046412', '0' , '0.094079','0','0','0','base_link','laser_frame']
    )
    
    return LaunchDescription([slam_gmapping_launch,base_link_to_laser_tf_node])

```

这里启动了一个launch文件-slam_gmapping_launch和一个发布静态变换的节点-base_link_to_laser_tf_node。详细看下slam_gmapping_launch，该文件位于，

```
/root/gmapping_ws/src/slam_gmapping/launch
```

slam_gmapping.launch.py，

```
from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
import launch.actions
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
	return LaunchDescription([
        launch_ros.actions.Node(
            package='slam_gmapping', 
            executable='slam_gmapping', 
            output='screen', 
            parameters=[os.path.join(get_package_share_directory("slam_gmapping"), "params", "slam_gmapping.yaml")]),
    ])
```

这里启动了slam_gmapping的节点，加载了slam_gmapping.yaml参数文件，该文件位于（以配套虚拟机为例），

```
/root/gmapping_ws/src/slam_gmapping/params
```

slam_gmapping.yaml

```py
/slam_gmapping:
  ros__parameters:
    angularUpdate: 0.5
    astep: 0.05
    base_frame: base_footprint
    map_frame: map
    odom_frame: odom
    delta: 0.05
    iterations: 5
    kernelSize: 1
    lasamplerange: 0.005
    lasamplestep: 0.005
    linearUpdate: 1.0
    llsamplerange: 0.01
    llsamplestep: 0.01
    lsigma: 0.075
    lskip: 0
    lstep: 0.05
    map_update_interval: 5.0
    maxRange: 6.0
    maxUrange: 4.0
    minimum_score: 0.0
    occ_thresh: 0.25
    ogain: 3.0
    particles: 30
    qos_overrides:
      /parameter_events:
        publisher:
          depth: 1000
          durability: volatile
          history: keep_all
          reliability: reliable
      /tf:
        publisher:
          depth: 1000
          durability: volatile
          history: keep_last
          reliability: reliable
    resampleThreshold: 0.5
    sigma: 0.05
    srr: 0.1
    srt: 0.2
    str: 0.1
    stt: 0.2
    temporalUpdate: 1.0
    transform_publish_period: 0.05
    use_sim_time: false
    xmax: 10.0
    xmin: -10.0
    ymax: 10.0
    ymin: -10.0
```





































