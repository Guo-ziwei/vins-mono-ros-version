vins-mono-ros-version

**描述**：
这是一个用于[深蓝学院](<http://www.shenlanxueyuan.com/>)教学的[代码](https://github.com/HeYijia/VINS-Course)，她基于 VINS-Mono 框架，但不依赖 ROS, Ceres, G2o。这个代码非常基础，目的在于演示仅基于 Eigen 的后端 LM 算法，滑动窗口算法，鲁棒核函数等等 SLAM 优化中常见的算法。本人将其借口改为ROS的接口并替换掉pangolin用Rviz作为显示
该代码支持 Ubuntu

### 安装依赖项：

1. ros

2. Eigen

3. Ceres: vins 初始化部分使用了 ceres 做 sfm，所以我们还是需要依赖 ceres. 

### 编译代码

Clone the repository and catkin_make:

``` bash
cd ~/catkin_ws/src
git clone https://github.com/Guo-ziwei/vins-mono-ros
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash
```

### 运行

``` bash
rosrun vins_mono vins_mono_node

rosbag play ...
```

