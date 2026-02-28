

# F1TENTH ROS2 智能车项目

基于ROS2 Humble的F1TENTH智能车视觉巡线项目。实现了从仿真环境搭建、图像处理到PID控制的完整自动驾驶巡线功能。



## 功能特性

✅ Ubuntu 22.04 + ROS2 Humble 环境搭建

✅ Gazebo仿真场景搭建（自定义贴图赛道）

✅ 改进的车模型（带摄像头支柱，尺寸优化）

✅ Python图像处理节点（白色赛道检测）

✅ PID控制节点（偏差计算与转向控制）

✅ 可视化调试（rqt_image_view）

✅ 完整巡线功能实现



## 文件结构

<img width="744" height="826" alt="image" src="https://github.com/user-attachments/assets/2df8ae0d-c5f5-4f23-a00e-b0a7d6a65f25" />





## 启动流程

```
cd ~/ros2_ws_car  

source install/setup.bash
```

启动Gazebo

```
ros2 launch gazebo_ros gazebo.launch.py 
```

加载赛道

```
ros2 run gazebo_ros spawn_entity.py -file \$(pwd)/src/f1tenth_simulator/urdf/levine/model.sdf -entity levine_track
```

加载车辆

```
ros2 run gazebo_ros spawn_entity.py -file $(pwd)/src/f1tenth_simulator/urdf/model.sdf -entity racecar -x 0 -y 0 -z 0.1
```

启动图像处理

```
source install/setup.bash 

ros2 run image_processing image_processor
```

打开可视化摄像头

```
ros2 run rqt_image_view rqt_image_view
```

启动PID控制器

```
source install/setup.bash

ros2 run pid_controller pid_controller
```




