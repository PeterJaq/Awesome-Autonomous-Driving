# Autonomous-Driving-101

作者: 牛肉咖喱饭 

定期一个自动驾驶相关领域小项目或者小组件的试用分享！

## 参数标定

[camera-calibration](https://github.com/LittleAprilFool/camera-calibration)
**入门** 能够比较好的阐述相机标定具体步骤和原理的

[CameraCalibration](https://github.com/dyfcalid/CameraCalibration)
**实用工具** 这个项目集合了相机标定相关的多个脚本工具，便于完成完整的车载环视相机标定流程

[ros-camera-lidar-calibration](https://github.com/swyphcosmo/ros-camera-lidar-calibration)
**实用工具** 相机内参标定与相机lidar外参标定

[lidar_IMU_calib](https://github.com/APRIL-ZJU/lidar_IMU_calib)
**实用工具** Lidar IMU 的标定工具

[sync_gps_lidar_imu_cam](https://github.com/nkliuhui/sync_gps_lidar_imu_cam)
**硬件方案** lidar-imu-cam-GPS时间戳硬件同步方案

## 2D检测

### 车道与车道线检测
[Advanced-Lane-Detection](https://github.com/uranus4ever/Advanced-Lane-Detection)
一个非常适合新人的车道检测任务的小demo

[RESA](https://github.com/ZJULearning/resa)

[LaneDet](https://github.com/Turoad/lanedet)

[CondLaneNet](https://github.com/aliyun/conditional-lane-detection)

[Focus on Local: Detecting Lane Marker from Bottom Up via Key Point](https://openaccess.thecvf.com/content/CVPR2021/papers/Qu_Focus_on_Local_Detecting_Lane_Marker_From_Bottom_Up_via_CVPR_2021_paper.pdf)

[LaneNet-Lane-Detection](https://github.com/MaybeShewill-CV/lanenet-lane-detection)

### 目标检测与追踪

[YOLOR](https://github.com/WongKinYiu/yolor)
提出了在网络模型中引入隐知识的概念，将隐知识和显知识同时作用于模型训练，通过核函数对齐，预测精修以及多任务同时学习，让网络表征出一种统一化的特征。

[YOLOX](https://github.com/Megvii-BaseDetection/YOLOX)
Anchor-free 版本的YOLO，堆砌了解耦头，simOTA等，达到了SOTA

[Lite.ai](https://github.com/DefTruth/lite.ai)
该项目提供了一系列轻量级的目标检测语义分割任务的整合框架支持 YOLOX🔥, YoloR🔥, YoloV5, YoloV4, DeepLabV3, ArcFace, CosFace, RetinaFace, SSD, etc.

[Yolov5_DeepSort_Pytorch](https://github.com/mikel-brostrom/Yolov5_DeepSort_Pytorch)
基于yolo-v5的目标追踪

### 语义分割

[Cam2BEV](https://github.com/ika-rwth-aachen/Cam2BEV)
Cam2BEV一个将多路周视摄像头的语义分割结果融合在一个鸟瞰图的工具，并且该方法不需要手工对鸟瞰图进行标注通过合成的数据进行训练。

### 多头任务检测检测

[YOLOP](https://github.com/hustvl/YOLOP)
来自华中科技大学的作品，也是yolo系列的另一力作，本项目提出额一种高效的多任务网络，可以联合处理自动驾驶中的多个任务（目标检测，可行驶区域分割与车道检测三个关键任务），值得注意的是在BDD100K中该方法实现了SOTA的情况下还保持了嵌入式友好。

## 3D检测
[Voxelnet](https://github.com/steph1793/Voxelnet)

[Complex-YOLO](https://github.com/maudzung/Complex-YOLOv4-Pytorch)

[PointRCNN](https://github.com/sshaoshuai/PointRCNN)

[3D-BoundingBox](https://github.com/skhadem/3D-BoundingBox)
基于单目的3D bbox估计

## 数据迭代与闭环

[Discriminative Active Learning] https://github.com/dsgissin/DiscriminativeActiveLearning
**主动学习**


## SLAM与定位
[DeepLIO](https://github.com/ArashJavan/DeepLIO)
**Lidar + IMU** 一款基于深度学习的lidar IMU融合里程计

[hdl_localization](https://github.com/koide3/hdl_localization)
**Lidar + IMU** 基于卡尔曼滤波的位置估计使用了激光雷达，IMU, 可以做到实时估计。

[hdl_graph_slam](https://github.com/koide3/hdl_graph_slam)
**Lidar + IMU + GPS** 它基于三维图形SLAM，具有基于NDT扫描匹配的测距估计和循环检测。它还支持几个约束，如GPS、IMU。

[LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)
**Lidar + IMU + GPS** 基于激光雷达，IMU和GPS多种传感器的因子图优化方案，以及在帧图匹配中使用帧-局部地图取代帧-全局地图。

[LVI-SAM](https://github.com/TixiaoShan/LVI-SAM)
**Lidar + Camera** 基于视觉+激光雷达的惯导融合

[LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM)
**Lidar** LeGO-LOAM是以LOAM为框架而衍生出来的新的框架。其与LOAM相比，更改了特征点的提取形式，添加了后端优化，因此，构建出来的地图就更加的完善。

[SC-LeGO-LOAM](https://github.com/irapkaist/SC-LeGO-LOAM)
**Lidar** LeGO-LOAM的基于全局描述子Scan Context的回环检测

[SC-LIO-SAM](https://github.com/gisbi-kim/SC-LIO-SAM)
**Lidar + Camera** LIO-SAM的基于全局描述子Scan Context的回环检测

## 行为预测

[An Auto-tuning Framework for Autonomous Vehicles] (https://arxiv.org/pdf/1808.04913.pdf)

## 规划与控制

[自动驾驶中的决策规划算法概述](https://www.jiqizhixin.com/articles/2019-07-22)

[PID](https://en.wikipedia.org/wiki/PID_controller)

[有限状态机](https://en.wikipedia.org/wiki/Finite-state_machine)

[MPC](https://en.wikipedia.org/wiki/Model_predictive_control)

## 框架与解决方案

[Apollo](https://github.com/ApolloAuto/apollo)
百度apollo构建的平台

[Autoware.auto]

[Autoware.ai]


## 其他组件

[Carla-birdeye-view](https://github.com/deepsense-ai/carla-birdeye-view)
可以对接carla的自动驾驶鸟瞰图组件。

[Uber AVS](https://avs.auto/#/)
自动驾驶可视化前端组件 xviz 与 streetscape.gl 

[Cruise](https://webviz.io/worldview/#/)
Cruise 开源的一款自动驾驶前端可视化套件

## 数据集与比赛
[KITTI](http://www.cvlibs.net/datasets/kitti/)

[BDD100k](https://www.bdd100k.com/)

[UrbanNav](https://github.com/weisongwen/UrbanNavDataset)

[ONCE](https://once-for-auto-driving.github.io)

[SODA10M](https://soda-2d.github.io/)

**数据集** 一个在亚洲城市峡谷（包括东京和香港）收集的开源本地化数据集,主要用于解决定位算法的各种问题。

## 其他更好的分享

[awesome-3D-object-detection] (https://github.com/Tom-Hardy-3D-Vision-Workshop/awesome-3D-object-detection)

[3D-ObjectDetection-and-Pose-Estimation](https://github.com/littlebearsama/3D-ObjectDetection-and-Pose-Estimation)
物体检测与位姿估计