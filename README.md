# Awesome-Autonomous-Driving [![Awesome](https://awesome.re/badge-flat.svg)](https://awesome.re)

Author: 牛肉咖喱饭(PeterJaq)

Update：2024/04/20

This project will be periodically updated with quality projects and papers related to autonomous driving.

## Update
* [2024/04/20] Update Arxiv 2024 02 Monthly ADAS Paper List!! [Arxiv-202403](https://github.com/PeterJaq/Awesome-Autonomous-Driving/blob/main/arxiv/202403.md)
* [2024/03/31] Update CVPR 2024 Paper List in [CVPR2024 Autonoumous Driving](https://github.com/PeterJaq/Awesome-Autonomous-Driving/blob/main/conferences/CVPR2024.md)
* [2024/03/23] Update HD Map Groudtruth and Oneline Paper List !!
* [2024/3/14] Update Arxiv 2024 02 Monthly ADAS Paper List!! [Arxiv-202402](https://github.com/PeterJaq/Awesome-Autonomous-Driving/blob/main/arxiv/202402.md)
* [2024/2/11] Update Arxiv 2024 01 Monthly ADAS Paper List!! [Arxiv-202401](https://github.com/PeterJaq/Awesome-Autonomous-Driving/blob/main/arxiv/202401.md)
* [2024/1/13] Add Other Awesome List.
* [2024/1/1] Update Arxiv 2023 12 Monthly ADAS Paper List!! [Arxiv-202312](https://github.com/PeterJaq/Awesome-Autonomous-Driving/blob/main/arxiv/202312.md)
* [2023/12/3] Add Daily ADAS Arxiv Paper List!! in [ADAS-Arxiv-Daily](https://github.com/PeterJaq/adas-arxiv-daily)
* [2023/12/2] Update Arxiv 2023 11 Monthly ADAS Paper List!! [Arxiv-202311](https://github.com/PeterJaq/Awesome-Autonomous-Driving/blob/main/arxiv/202311.md)
* [2023/11/20] Update NeurIPS 2023 ADAS Paper List!! [NeurIPS2023](https://github.com/PeterJaq/Awesome-Autonomous-Driving/blob/main/conferences/NeurIPS2023.md)
* [2023/09/27] Update ICCV 2023 ADAS Paper List！！

## Contents
- [Contents](#contents)
- [1. Autonomous Driving Midleware and Integrated Solutions(中间件与解决方案)](#1-autonomous-driving-midleware-and-integrated-solutions)
  - [1.1 Midelware(中间件)](#11-midelware)
  - [1.2 Integrated Solutions(解决方案)](#12-integrated-solutions)
- [2. Sensor and Calibration Tools(传感器与参数标定)](#2-sensor-and-calibration-tools)
  - [2.1 Sensor Hardware(传感器硬件)](#21-sensor-hardware)
  - [2.2 Calibration Tools(参数标定工具)](#22-calibration-tools)
- [3. Perception](#3-perception)
  - [3.1 Detection](#31-detection)
    - [3.1.1 Vision based](#311-vision-based)
    - [3.1.2 Lidar based](#312-lidar-based)
    - [3.1.3 Radar based](#313-radar-based)
    - [3.1.4 Multimodal Fusion](#314-multimodal-fusion)
  - [3.2 Tracking](#32-tracking)
- [4. Prediction](#4-prediction)
- [5. Localization and SLAM(定位与SLAM)](#5-localization-and-slam)
- [6. Planning](#6-planning)
- [7. Control](#7-control)
- [8. Dataset and Competition(数据集与竞赛)](#8-Dataset-and-Competition)
- [9. Visualization(可视化工具)](#9-Visualization)
- [10. Data Loop(数据闭环)](#10-Data-Loop)
- [11. Simulation(仿真)](#11-Simulation)
- [12. Others(其他更好的)](#12-Others)

## 1. Autonomous Driving Midleware and Integrated Solutions

### 1.1 Midelware
*中间件*
- [ROS](https://github.com/ros) - A set of software libraries and tools that help you build robot applications. 
- [ROS-2](https://github.com/ros2) - A set of software libraries and tools that help you build robot applications. 
- [Cyber](https://github.com/storypku/CyberRT) - High performance runtime framework designed specifically for autonomous driving (AD) scenarios from [baidu](www.baidu.com).
### 1.2 Integrated Solutions
*解决方案*

- [Apollo](https://github.com/ApolloAuto/apollo) - The intergrated solution from [baidu](www.baidu.com).
- [Autoware.ai](https://github.com/Autoware-AI/) - Open-source software for self-driving vehicles known as Autoware-1.
- [Autoware.auto](https://gitlab.com/autowarefoundation/autoware.auto) - Open-source software for self-driving vehicles known as Autoware-2.
- [AutowareArchitectureProposal.proj](https://github.com/tier4/AutowareArchitectureProposal.proj) - Manages several projects related to self-driving vehicles. 
- [self-driving-ish_computer_vision_system](https://github.com/iwatake2222/self-driving-ish_computer_vision_system) - This project generates images you've probably seen in autonomous driving demo.
- [Aslan](https://github.com/project-aslan/Aslan) - An open-source full-stack software based on ROS framework.
- [AutoC2X-AW](https://github.com/esakilab/AutoC2X-AW) - Extension for Autoware and OpenC2X.

## 2. Sensor and Calibration Tools
### 2.1 Sensor Hardware
*传感器硬件*

  **LiDAR** 
  
  - [velodyne](https://github.com/ros-drivers/velodyne) - velodyne lidar driver for ros.
  - [livox_ros_driver](https://github.com/Livox-SDK/livox_ros_driver) - livox (a low cost lidar form [DJI](https://www.dji.com/cn)) lidar driver.
  - [rslidar_sdk](https://github.com/RoboSense-LiDAR/rslidar_sdk) - lidar driver from [Robosense](https://www.robosense.ai).
  - [ros2_ouster_drivers](https://github.com/ros-drivers/ros2_ouster_drivers) - ROS2 Drivers for the [Ouster](https://www.ouster.com) OS-0, OS-1, and OS-2 Lidars. 

 **Camera** 
 
  - [miivii_gmsl_camera](https://github.com/MiiViiDynamics/miivii_gmsl_camera) - [米文](https://www.miivii.com/)摄像头
  - [sensing](https://www.sensing-world.com/) - [森云](https://www.sensing-world.com/) - 森云摄像头
  - [Hikvision](https://www.hikvision.com) - You can download [SDK](https://www.hikvision.com/en/support/download/sdk/).
  - [usb_cam](https://github.com/ros-drivers/usb_cam) - all most ros1 usb camera driver you can buy from Taobao/Aliexpress.
  - [ros2_usb_camera](https://github.com/klintan/ros2_usb_camera) - all most ros2 usb camera driver you can buy from Taobao/Aliexpress.

 **GPS/IMU** 
 
  - [huace](https://www.huace.cn) - 华测组合导航产品
  - [novatel_gps_driver](https://github.com/swri-robotics/novatel_gps_driver) - C++ ROS driver for NovAtel GPS / GNSS Receivers.
 
 **MCU**
 
  - [STM32Cube_MCU_Overall_Offer](https://github.com/STMicroelectronics/STM32Cube_MCU_Overall_Offer) - The open source offer for the STM32 MCU products.


### 2.2 Calibration Tools
*参数标定工具*

- [OpenCalib](https://github.com/PJLab-ADG/SensorsCalibration) - 
**ALL in One** 商汤开源的自动驾驶多传感器的一个开源标定工具箱，基本涵盖了大部分的自动驾驶标定场景。
- [camera-calibration](https://github.com/LittleAprilFool/camera-calibration) -
能够比较好的阐述相机标定具体步骤和原理的
- [CameraCalibration](https://github.com/dyfcalid/CameraCalibration) - 
这个项目集合了相机标定相关的多个脚本工具，便于完成完整的车载环视相机标定流程
- [ros-camera-lidar-calibration](https://github.com/swyphcosmo/ros-camera-lidar-calibration) -
相机内参标定与相机lidar外参标定
- [lidar_IMU_calib](https://github.com/APRIL-ZJU/lidar_IMU_calib) -
Lidar IMU 的标定工具
- [sync_gps_lidar_imu_cam](https://github.com/nkliuhui/sync_gps_lidar_imu_cam) -
lidar-imu-cam-GPS时间戳硬件同步方案

## 3. Perception
### 3.1 Detection

*检测与分割*

### 3.1.1 Vision based
*基于视觉*

  **BackBone**
  - [Next-ViT](https://arxiv.org/pdf/2207.05501.pdf) 来自字节面向工业界的新一代Transform模型部署。
  - [CoAtNet](https://arxiv.org/pdf/2106.04803.pdf)
  - [FocalsConv](https://github.com/dvlab-research/FocalsConv) Focal Sparse Convolutional
  - [PoolFormer](https://github.com/sail-sg/poolformer) [CVPR2022] MetaFormer Is Actually What You Need for Vision. 证明Transformer模型的能力，而不是设计复杂的token mixer来实现SOTA性能
  - [ConvNext](https://arxiv.org/abs/2201.03545) [CVPR2022] A ConvNet for the 2020s. 用设计transformer的思想构建卷积。
  - [Mobile-Former](https://arxiv.org/abs/2108.05895) [CVPR2022] 微软提出Mobile-Former，MobileNet和Transformer的并行设计，可以实现局部和全局特征的双向融合，在分类和下游任务中，性能远超MobileNetV3等轻量级网络！
  - [Up to 31](https://arxiv.org/abs/2203.06717) Revisiting Large Kernel Design in CNNs. 大Kernel =? SOTA 这篇文章给你答案！

  **Occupancy**
  - [Occupancy Networks](https://link.zhihu.com/?target=https%3A//github.com/LMescheder/Occupancy-Networks) Learning 3D Reconstruction in Function Space.
  - [Pyramid Occupancy Network](https://link.zhihu.com/?target=https%3A//github.com/tom-roddick/mono-semantic-maps) Predicting Semantic Map Representations from Images using Pyramid Occupancy Networks.
  - [MonoScene](https://github.com/cv-rits/MonoScene) Monocular 3D Semantic Scene Completion.
  - [OccDepth](https://github.com/megvii-research/OccDepth) A Depth-Aware Method for 3D Semantic Scene Completion.
  - [VoxFormer](https://github.com/NVlabs/VoxFormer) Sparse Voxel Transformer for Camera-based 3D Semantic Scene.
  - [TPVFormer](https://github.com/wzzheng/TPVFormer) Tri-Perspective View for Vision-Based 3D Semantic Occupancy Prediction.
  - [SurroundOcc](https://github.com/weiyithu/SurroundOcc) Multi-Camera 3D Occupancy Prediction for Autonomous Driving.
  - [A Comprehensive Review of Occupancy](https://arxiv.org/abs/2303.01212) A summary of the current research trend and provide some probable future outlooks for occupancy.
  
  **数据增强**
  - [TeachAugment](https://github.com/DensoITLab/TeachAugment) [CVPR2022] Data Augmentation Optimization Using Teacher Knowledge
  - [AlignMixup](https://github.com/DensoITLab/TeachAugment)  [CVPR2022] Improving Representations By Interpolating Aligned Features
  - [rising](https://github.com/PhoenixDL/rising) 基于pytorch的GPU数据预处理transform模块，实测好用！
  
**Lane Detection**
   
  - [Advanced-Lane-Detection](https://github.com/uranus4ever/Advanced-Lane-Detection) - 一个非常适合新人的车道检测任务的小demo
  - [RESA](https://github.com/ZJULearning/resa)
  - [LaneDet](https://github.com/Turoad/lanedet)
  - [CondLaneNet](https://github.com/aliyun/conditional-lane-detection)
  - [Focus on Local: Detecting Lane Marker from Bottom Up via Key Point](https://openaccess.thecvf.com/content/CVPR2021/papers/Qu_Focus_on_Local_Detecting_Lane_Marker_From_Bottom_Up_via_CVPR_2021_paper.pdf)
  - [LaneNet-Lane-Detection](https://github.com/MaybeShewill-CV/lanenet-lane-detection)
  - [urban_road_filter](https://github.com/jkk-research/urban_road_filter) 一种实时的道路边缘检测分割工具
  - [Cam2BEV](https://github.com/ika-rwth-aachen/Cam2BEV) - Cam2BEV一个将多路周视摄像头的语义分割结果融合在一个鸟瞰图的工具，并且该方法不需要手工对鸟瞰图进行标注通过合成的数据进行训练。
  - [YOLOP](https://github.com/hustvl/YOLOP) - 来自华中科技大学的作品，也是yolo系列的另一力作，本项目提出额一种高效的多任务网络，可以联合处理自动驾驶中的多个任务（目标检测，可行驶区域分割与车道检测三个关键任务），值得注意的是在BDD100K中该方法实现了SOTA的情况下还保持了嵌入式友好。
 
  **Object Detection**
  
  - [YOLOR](https://github.com/WongKinYiu/yolor) - 提出了在网络模型中引入隐知识的概念，将隐知识和显知识同时作用于模型训练，通过核函数对齐，预测精修以及多任务同时学习，让网络表征出一种统一化的特征。
  - [YOLOX](https://github.com/Megvii-BaseDetection/YOLOX) - Anchor-free 版本的YOLO，堆砌了解耦头，simOTA等，达到了SOTA
  - [3D-BoundingBox](https://github.com/skhadem/3D-BoundingBox)
  - [Pseudo_Lidar_V2](https://github.com/mileyan/Pseudo_Lidar_V2) - Accurate Depth for 3D Object Detection in Autonomous Driving.
  - [Pseudo_lidar](https://github.com/mileyan/pseudo_lidar.git) - Pseudo-LiDAR from Visual Depth Estimation: Bridging the Gap in 3D Object Detection for Autonomous Driving.
  - [https://arxiv.org/abs/2203.10981](https://arxiv.org/abs/2203.10981) MonoDTR: Monocular 3D Object Detection with Depth-Aware Transformer 基于单目的Depth-Aware Transformer 的3D检测.
  - [BoxeR](https://github.com/kienduynguyen/BoxeR)  Box-Attention for 2D and 3D Transformers. 从鸟瞰平面生成判别信息，用于 3D 端到端对象检测。该项目同样也提出了2D上的Detection 解决方案。
 

### 3.1.2 Lidar based

*基于激光雷达*

  **Object Detection**
  
  - [Voxelnet](https://github.com/steph1793/Voxelnet)
  - [Complex-YOLO](https://github.com/maudzung/Complex-YOLOv4-Pytorch)
  - [PointRCNN](https://github.com/sshaoshuai/PointRCNN) 
  - [CenterPoint](https://github.com/tianweiy/CenterPoint) - 3D Object Detection and Tracking using center points in the bird-eye view.
  - [PartA2-Net](https://github.com/sshaoshuai/PartA2-Net) - From Points to Parts: 3D Object Detection from Point Cloud with Part-aware and Part-aggregation Network.
  - [CIA-SSD](https://github.com/Vegeta2020/CIA-SSD) - Confident IoU-Aware Single Stage Object Detector From Point Cloud.
  - [3DIoUMatch-PVRCNN](https://github.com/THU17cyz/3DIoUMatch-PVRCNN) - 3DIoUMatch: Leveraging IoU Prediction for Semi-Supervised 3D Object Detection.
  - [SFA3D](https://github.com/maudzung/SFA3D) - Super Fast and Accurate 3D Object Detection based on 3D LiDAR Point Clouds.
  - [Auto4D](https://arxiv.org/pdf/2101.06586v1.pdf) - Auto4D: Learning to Label 4D Objects from Sequential Point Clouds. 
  - [3DAL](https://arxiv.org/abs/2103.05073) - Offboard 3D Object Detection from Point Cloud Sequences
  - [LIFT](https://openaccess.thecvf.com/content/CVPR2022/html/Zeng_LIFT_Learning_4D_LiDAR_Image_Fusion_Transformer_for_3D_Object_CVPR_2022_paper.html)  [CVPR2022]  LIFT: Learning 4D LiDAR Image Fusion Transformer for 3D Object Detection
  - [FSD](https://github.com/TuSimple/SST) [CVPR2022] Fully Sparse 3D Object Detection & SST: Single-stride Sparse Transformer 来自图森的 Sparse Transformer. 
  - [VoxelNext](https://cvpr2023.thecvf.com/virtual/2023/poster/22493) [CVPR2023] VoxelNeXt: Fully Sparse VoxelNet for 3D Object Detection and Tracking
  - [PillarNext](https://cvpr2023.thecvf.com/virtual/2023/poster/21961) [CVPR2023] Rethinking Network Designs for 3D Object Detection in LiDAR Point Clouds
  - [LargeKernel3D](https://cvpr2023.thecvf.com/virtual/2023/poster/22447) [CVPR2023] LargeKernel3D: Scaling Up Kernels in 3D Sparse CNNs
  - [LinK](https://cvpr2023.thecvf.com/virtual/2023/poster/22454) [CVPR2023] Linear Kernel for LiDAR-Based 3D Perception
  - [Spherical Transformer](https://cvpr2023.thecvf.com/virtual/2023/poster/22665) [CVPR2023]spherical Transformer for LiDAR-Based 3D Recognition
  - [Unspervised 3D OD](https://cvpr2023.thecvf.com/virtual/2023/poster/22263) [CVPR2023]Towards Unsupervised Object Detection From LiDAR Point Clouds
  - [Benchmarking robustness of 3D OD](https://cvpr2023.thecvf.com/virtual/2023/poster/22236) [CVPR2023] Benchmarking Robustness of 3D Object Detection to Common Corruptions
  - [Bi3D](https://cvpr2023.thecvf.com/virtual/2023/poster/21758) [CVPR2023] Bi-Domain Active Learning for Cross-Domain 3D Object Detection
  - [Density-Insensitive](https://cvpr2023.thecvf.com/virtual/2023/poster/21626) [CVPR2023] Density-Insensitive Unsupervised Domain Adaption on 3D Object Detection
  - [UniDistill](https://cvpr2023.thecvf.com/virtual/2023/poster/22863) [CVPR2023] UniDistill: A Universal Cross-Modality Knowledge Distillation Framework for 3D Object Detection in Bird’s-Eye View
  - [MSF](https://cvpr2023.thecvf.com/virtual/2023/poster/23069) [CVPR2023] MSF: Motion-Guided Sequential Fusion for Efficient 3D Object Detection From Point Cloud Sequences
  - [OcTr](https://cvpr2023.thecvf.com/virtual/2023/poster/22293) [CVPR2023] OcTr: Octree-Based Transformer for 3D Object Detection
  - [SlowLiDAR](https://cvpr2023.thecvf.com/virtual/2023/poster/21242) [CVPR2023] Increasing the Latency of LiDAR-Based Detection Using Adversarial Examples
  - [Uni3D](https://cvpr2023.thecvf.com/virtual/2023/poster/21959) [CVPR2023] Uni3D: A Unified Baseline for Multi-Dataset 3D Object Detection
  - [DetZero](https://arxiv.org/abs/2306.06023) [ICCV2023] Rethinking Offboard 3D Object Detection with Long-term Sequential Point Clouds
  - [FocalFormer3D](https://arxiv.org/abs/2308.04556) [ICCV2023] Focusing on Hard Instance for 3D Object Detection
  - [GPA-3D](https://arxiv.org/abs/2308.08140) [ICCV2023] Geometry-aware Prototype Alignment for Unsupervised Domain Adaptive 3D Object Detection from Point Clouds
  - [KECOR](https://arxiv.org/abs/2307.07942) [ICCV2023] KECOR: Kernel Coding Rate Maximization for Active 3D Object Detection
  - [Once Detected, Never Lost](https://arxiv.org/abs/2304.12315) [ICCV2023] Once Detected, Never Lost: Surpassing Human Performance in Offline LiDAR based 3D Object Detection
  - [PARTNER](https://arxiv.org/abs/2308.03982) [ICCV2023] PARTNER: Level up the Polar Representation for LiDAR 3D Object Detection
  - [PG-RCNN](https://arxiv.org/abs/2307.12637) [ICCV2023] PG-RCNN: Semantic Surface Point Generation for 3D Object Detection
  - [Domain-Adaptive](https://arxiv.org/abs/2307.07944) [ICCV2023]Revisiting Domain-Adaptive 3D Object Detection by Reliable, Diverse and Class-balanced Pseudo-Labeling
    
  **Lidar Ground Segmentation**
  - [patchwork](https://github.com/LimHyungTae/patchwork) Patchwork 主要由三部分组成：基于同心带模型（CZM）的极坐标网格表示、区域地平面拟合（R-GPF）和地面似然估计（GLE) IROS2021
  - [patchwork++](https://github.com/url-kaist/patchwork-plusplus) 与Patchwork不同，Patchwork++由称为反射噪声去除（RNR）、区域垂直平面拟合（R-VPF）、自适应GLE（A-GLE）和空间地面恢复（TGR）的新模块组成。Patchwork++具有更高的精确度和召回率。此外，新的Patchwork++具有较低的召回标准差。
  - [TRAVEL](https://github.com/url-kaist/TRAVEL.git) 他使用三维点云的图形表示，同时进行可穿越的地面检测和物体聚类, 为了分割可穿越的地面，点云被编码为一个图结构，即三网格场，它将每个三网格视为一个节点。IROS 2022

 **Lidar Segmentation**
  - [RangeView](https://arxiv.org/abs/2303.05367) [ICCV2023] Rethinking Range View Representation for LiDAR Segmentation
  
### 3.1.2 Multi Sensor Fusion

  **3D Object Detection**

 - [PERF](https://github.com/megvii-research/petr) PETR encodes the position information of 3D coordinates into image features, producing the 3D position-aware features.
 - [PERFv2](https://github.com/megvii-research/petr) Based on PETR, PETRv2 explores the effectiveness of temporal modeling, which utilizes the temporal information of previous frames to boost 3D object detection. 
 - [BEVFusion](https://github.com/mit-han-lab/bevfusion.git) BEVFusion is fundamentally task-agnostic and seamlessly supports different 3D perception tasks with almost no architectural changes.
 - [BEVDepth](https://github.com/Megvii-BaseDetection/BEVDepth.git) BEVDepth resolves this by leveraging explicit depth supervision.
 - [BEVFormer](https://github.com/fundamentalvision/BEVFormer.git) BEVFormer learns unified BEV representations with spatiotemporal transformers to support multiple autonomous driving perception tasks.
 - [ST-P3](https://github.com/OpenPerceptionX/ST-P3) End-to-end Vision-based Autonomous Driving via Spatial-Temporal Feature Learning
 - [SpatialDETR](https://github.com/cgtuebingen/SpatialDETR) Robust Scalable Transformer-Based 3D Object Detection from Multi-View Camera Images with Global Cross-Sensor Attention
 - [BEVDet](https://github.com/HuangJunJie2017/BEVDet) High-Performance Multi-Camera 3D Object Detection in Bird-Eye-View.
 - [BEVDet4D](https://arxiv.org/abs/2203.17054) Exploit Temporal Cues in Multi-camera 3D Object Detection.
 - [M2BEV](https://arxiv.org/abs/2204.05088) Multi-Camera Joint 3D Detection and Segmentation with Unified Birds-Eye View Representation.
 - [BEVerse](https://github.com/zhangyp15/BEVerse) Unified Perception and Prediction in Birds-Eye-View for Vision-Centric Autonomous Driving.
 - [PolarDETR](https://arxiv.org/abs/2206.10965) Polar Parametrization for Vision-based Surround-View 3D Detection.
 - [PolarFormer](https://github.com/fudan-zvg/PolarFormer) Multi-camera 3D Object Detection with Polar Transformers.
 - [CrossDTR](https://github.com/sty61010/CrossDTR) Cross-view and Depth-guided Transformers for 3D Object Detection.
 - [Sim-BEV](https://github.com/aharley/simple_bev) A Simple Baseline for BEV Perception Without LiDAR.
 - [AeDet](https://github.com/fcjian/AeDet) AeDet: Azimuth-invariant Multi-view 3D Object Detection.
 - [DFKF](https://cvpr2023.thecvf.com/virtual/2023/poster/22921) [CVPR2023]Distilling Focal Knowledge From Imperfect Expert for 3D Object Detection
 - [Understand BEV](https://cvpr2023.thecvf.com/virtual/2023/poster/22414)[CVPR2023] Understanding the Robustness of 3D Object Detection With Bird’s-Eye-View Representations in Autonomous Driving
 - [Focal Knowledge Form](https://cvpr2023.thecvf.com/virtual/2023/poster/22921) [CVPR2023] Distilling Focal Knowledge From Imperfect Expert for 3D Object Detection
 - [BEVHeight](https://cvpr2023.thecvf.com/virtual/2023/poster/21525) [CVPR2023] BEVHeight: A Robust Framework for Vision-Based Roadside 3D Object Detection
 - [BEV-SAN](https://cvpr2023.thecvf.com/virtual/2023/poster/22737) [CVPR2023] BEV-SAN: Accurate BEV 3D Object Detection via Slice Attention Networks
 - [Collaboration Overtake LiDAR](https://cvpr2023.thecvf.com/virtual/2023/poster/21634) [CVPR2023] Collaboration Helps Camera Overtake LiDAR in 3D Detection
 - [MSMDFusion](https://cvpr2023.thecvf.com/virtual/2023/poster/22992) [CVPR2023] MSMDFusion: Fusing LiDAR and Camera at Multiple Scales With Multi-Depth Seeds for 3D Object Detection
 - [BEV-Guided](https://cvpr2023.thecvf.com/virtual/2023/poster/21232) [CVPR2023] BEV-Guided Multi-Modality Fusion for Driving Perception
 - [BEV-DC](https://cvpr2023.thecvf.com/virtual/2023/poster/23193) [CVPR2023] BEV@DC: Bird’s-Eye View Assisted Training for Depth Completion
 - [Ada3D](https://arxiv.org/abs/2307.08209) [ICCV2023] Ada3D : Exploiting the Spatial Redundancy with Adaptive Inference for Efficient 3D Object Detection
 - [Cross Modal Transformer](https://arxiv.org/abs/2301.01283) [ICCV2023] Cross Modal Transformer: Towards Fast and Robust 3D Object Detection
 - [Object-Centric Temporal Modeling](https://arxiv.org/abs/2303.11926) [ICCV2023] Exploring Object-Centric Temporal Modeling for Efficient Multi-View 3D Object Detection
 - [QD-BEV](https://arxiv.org/abs/2308.10515) [ICCV2023] Quantization-aware View-guided Distillation for Multi-view 3D Object Detection
 - [MetaBEV](https://arxiv.org/abs/2304.09801) [ICCV2023] Solving Sensor Failures for BEV Detection and Map Segmentation
 - [Perceiver](https://arxiv.org/abs/2304.01289) [ICCV2023] Monocular 3D Object Detection with Bounding Box Denoising in 3D by Perceiver
 - [MonoNeRD](https://arxiv.org/abs/2308.09421) [ICCV2023] NeRF-like Representations for Monocular 3D Object Detection
 - [Object as Query](https://arxiv.org/abs/2301.02364) [ICCV2023] Object as Query: Lifting any 2D Object Detector to 3D Detection
 - [Predict to Detect](https://arxiv.org/abs/2306.08528) [ICCV2023] Predict to Detect: Prediction-guided 3D Object Detection using Sequential Images
 - [Pepresentation Disparity-aware](https://arxiv.org/abs/2308.10308) [ICCV2023] Representation Disparity-aware Distillation for 3D Object Detection
 - [SA-BEV](https://arxiv.org/abs/2307.11477) [ICCV2023] SA-BEV: Generating Semantic-Aware Bird's-Eye-View Feature for Multi-view 3D Object Detection
 - [SparseBEV](https://arxiv.org/abs/2308.09244) [ICCV2023] SparseBEV: High-Performance Sparse 3D Object Detection from Multi-Camera Videos
 - [SparseFusion](https://arxiv.org/abs/2308.14340) [ICCV2023] SparseFusion: Fusing Multi-Modal Sparse Representations for Multi-Sensor 3D Object Detection
 - [SupFusion](https://arxiv.org/abs/2309.07084) [ICCV2023] SupFusion: Supervised LiDAR-Camera Fusion for 3D Object Detection
 - [3DPPE](https://arxiv.org/abs/2211.14710) [ICCV2023] 3DPPE: 3D Point Positional Encoding for Multi-Camera 3D Object Detection Transformers
 - [MonoDETR](https://arxiv.org/abs/2203.13310) [ICCV2023] MonoDETR: Depth-guided Transformer for Monocular 3D Object Detection
 - [PETRv2](https://arxiv.org/abs/2206.01256) [ICCV2023] PETRv2: A Unified Framework for 3D Perception from Multi-Camera Images
 - [UpCycling](https://arxiv.org/abs/2211.11950) [ICCV2023] UpCycling: Semi-supervised 3D Object Detection without Sharing Raw-level Unlabeled Scenes
 - [ICCV2023] Not Every Side Is Equal: Localization Uncertainty Estimation for Semi-Supervised 3D Object Detection
  **Lane Detection** 

- [Repainting and Imitating Learning for Lane Detection](https://arxiv.org/abs/2210.05097)
- [WS-3D-Lane: Weakly Supervised 3D Lane Detection With 2D Lane Labels](https://arxiv.org/abs/2209.11523)
- [CurveFormer: 3D Lane Detection by Curve Propagation with Curve Queries and Attention](https://arxiv.org/abs/2209.07989)
- [PriorLane: A Prior Knowledge Enhanced Lane Detection Approach Based on Transformer]( https://arxiv.org/abs/2209.06994 )
- [M^2-3DLaneNet: Multi-Modal 3D Lane Detection](https://arxiv.org/abs/2209.05996)
- [RCLane: Relay Chain Prediction for Lane Detection](https://arxiv.org/abs/2207.09399)  ECCV 2022
- [PersFormer: 3D Lane Detection via Perspective Transformer and the OpenLane Benchmark](https://arxiv.org/abs/2203.11089)(https://github.com/OpenPerceptionX/PersFormer_3DLane)  [OpenLane Dataset](https://github.com/OpenPerceptionX/OpenLane)  ECCV 2022 Oral
- [Reconstruct from Top View: A 3D Lane Detection Approach based on Geometry Structure Prior](https://arxiv.org/abs/2206.10098)
- [Multi-level Domain Adaptation for Lane Detection](https://arxiv.org/abs/2206.10692)
- [Ultra Fast Deep Lane Detection with Hybrid Anchor Driven Ordinal Classification](https://arxiv.org/abs/2206.07389)  [github](https://github.com/cfzd/Ultra-Fast-Lane-Detection-v2)  TPAMI 2022
- [ONCE-3DLanes: Building Monocular 3D Lane Detection](https://arxiv.org/abs/2205.00301) CVPR 2022
- [A Keypoint-based Global Association Network for Lane Detection](https://arxiv.org/abs/2204.07335)   CVPR 2022
- [Eigenlanes: Data-Driven Lane Descriptors for Structurally Diverse Lanes](https://arxiv.org/abs/2203.15302
- [SDLane  Dataset](https://www.42dot.ai/akit/dataset/)  CVPR 2022
- [Towards Driving-Oriented Metric for Lane Detection Models](https://arxiv.org/abs/2203.16851)   CVPR 2022
- [CLRNet: Cross Layer Refinement Network for Lane Detection](https://arxiv.org/abs/2203.10350)  CVPR 2022
- [Rethinking Efficient Lane Detection via Curve Modeling](https://arxiv.org/abs/2203.02431)   CVPR 2022
- [Lane detection with Position Embedding](https://arxiv.org/abs/2203.12301)
- [AtrousFormer：Lane Detection with Versatile AtrousFormer and Local Semantic Guidance](https://arxiv.org/abs/2203.04067)
- [Laneformer: Object-Aware Row-Column Transformers for Lane Detection](https://www.aaai.org/AAAI22Papers/AAAI-6756.HanJ.pdf)  AAAI 2022
- [RONELDv2: A faster, improved lane tracking method](https://arxiv.org/abs/2202.13137)
- [BEV-LaneDet](https://cvpr2023.thecvf.com/virtual/2023/poster/22300) An Efficient 3D Lane Detection Based on Virtual Camera via Key-Points.
- [Anchor3DLane](https://openaccess.thecvf.com/content/CVPR2023/papers/Huang_Anchor3DLane_Learning_To_Regress_3D_Anchors_for_Monocular_3D_Lane_CVPR_2023_paper.pdf) [CVPR2023] Learning to Regress 3D Anchors for Monocular 3D Lane Detection
- [FrustumFormer](https://cvpr2023.thecvf.com/virtual/2023/poster/23100) [CVPR2023] Adaptive Instance-Aware Resampling for Multi-View 3D Detection

  **Segmentation**
- [LaserMix](https://cvpr2023.thecvf.com/virtual/2023/poster/22996) [CVPR2023] LaserMix for Semi-Supervised LiDAR Semantic Segmentation
- [PC Forecasting as Proxy](https://cvpr2023.thecvf.com/virtual/2023/poster/22022) [CVPR2023] Point Cloud Forecasting as a Proxy for 4D Occupancy Forecasting
- [Less is More](https://cvpr2023.thecvf.com/virtual/2023/poster/23130) [CVPR2023] Reducing Task and Model Complexity for 3D Point Cloud Semantic Segmentation
- [ISBNet](https://cvpr2023.thecvf.com/virtual/2023/poster/21952) [CVPR2023] A 3D Point Cloud Instance Segmentation Network With Instance-Aware Sampling and Box-Aware Dynamic Convolution

### 3.2 Tracking
*追踪算法*

- [SimpleTrack](https://github.com/TuSimple/SimpleTrack) - [3D-MOT] Simple yet Effective 3D Multi-object Tracking.
- [CAMO-MOT](https://arxiv.org/pdf/2209.02540.pdf) - [3D-MOT]  This paper propose an occlusion head to select the best object appearance features multiple times effectively, reducing the influence of occlusions
- [EagerMot](https://github.com/aleksandrkim61/EagerMOT) - [3D-MOT] Improve your online 3D multi-object tracking performance by using 2D detections to support tracking when 3D association fails.
- [OGR3MOT](https://arxiv.org/pdf/2104.11747.pdf) - [3D-MOT] This paper provides a natural way for track initialization and handling of false positive detections.
significantly improving track stability
- [ImmortalTracker](https://github.com/ImmortalTracker/ImmortalTracker) - Our mismatch ratio is tens of times lower than any previously published method.
- [Yolov5_DeepSort_Pytorch](https://github.com/mikel-brostrom/Yolov5_DeepSort_Pytorch) - 基于yolo-v5的目标追踪
- [3D Multi-Object Tracking in Point Clouds Based on Prediction Confidence-Guided Data Association](https://ieeexplore.ieee.org/abstract/document/9352500) 引入了一种新的配对代价计算方式，以利用点云中物体的特征来实现更快、更准确的数据关联。
- [Label Metric for Multi-Class Multi-Target Tracking under Hierarchical Multilevel Classification](https://www.mdpi.com/1424-8220/22/22/8613/pdf)  提出了一种
层次化多级分类标签，用于层次化多级目标跟踪。
 - [Tracking ROS](https://github.com/HuangCongQing/3D-LIDAR-Multi-Object-Tracking) 一个很好的ROS Tracking节点，方便大家参考。
 - [CXTrack](https://cvpr2023.thecvf.com/virtual/2023/poster/22624) Improving 3D Point Cloud Tracking With Contextual Information
 - [Monocular-Tracking](https://arxiv.org/abs/2308.11607) [ICCV2023] Delving into Motion-Aware Matching for Monocular 3D Object Tracking
 - [MBPTrack](https://arxiv.org/abs/2303.05071) [ICCV2023] Improving 3D Point Cloud Tracking with Memory Networks and Box Priors
 - [Synchronize Feature Extracting and Matching](https://arxiv.org/abs/2308.12549) [ICCV2023] Synchronize Feature Extracting and Matching: A Single Branch Framework for 3D Object Tracking

### 3.3 Map & Topo

 - [Survey](https://dl.acm.org/doi/10.1145/3627160) Data Issues in High-Definition Maps Furniture – A Survey
 - [MapNeXt](https://arxiv.org/abs/2401.07323) MapNeXt: Revisiting Training and Scaling Practices for Online Vectorized HD Map Construction
 - [PolyRoad](https://ieeexplore.ieee.org/document/10364754) PolyRoad: Polyline Transformer for Topological Road-Boundary Detection
 - [Survey](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=10184094) High-Definition Maps Construction Based on Visual Sensor: A Comprehensive Survey
 - [VMA](https://arxiv.org/abs/2304.09807) VMA: Divide-and-Conquer Vectorized Map Annotation System for Large-Scale Driving Scene
 - [Lane Graph as Path](https://arxiv.org/abs/2303.08815) Lane Graph as Path: Continuity-preserving Path-wise Modeling for Online Lane Graph Construction
 - [PivotNet](https://arxiv.org/abs/2308.16477) PivotNet: Vectorized Pivot Learning for End-to-end HD Map Construction
 - [E2E Map](https://arxiv.org/abs/2306.09700) End-to-End Vectorized HD-map Construction with Piecewise Bezier Curve
 - [LATR](https://arxiv.org/abs/2308.04583) LATR: 3D Lane Detection from Monocular Images with Transformer
 - [TopoReas](https://arxiv.org/abs/2304.05277) Graph-based Topology Reasoning for Driving Scenes
 - [TopoMLP](https://arxiv.org/abs/2310.06753) TopoMLP: A Simple yet Strong Pipeline for Driving Topology Reasoning
 - [Neural Map Prior](https://arxiv.org/abs/2304.08481) Neural Map Prior for Autonomous Driving
 - [Construction using Geometry](https://arxiv.org/abs/2312.03341) Online Vectorized HD Map Construction using Geometry
 - [MapTRv2](https://arxiv.org/abs/2308.05736) MapTRv2: An End-to-End Framework for Online Vectorized HD Map Construction
 - [InstaGraM](https://arxiv.org/abs/2301.04470) InstaGraM: Instance-level Graph Modeling for Vectorized HD Map Learning
 - [PolyMerge](https://arxiv.org/abs/2310.184160) PolyMerge: A Novel Technique aimed at Dynamic HD Map Updates Leveraging Polylines
 - [MapSeg](https://arxiv.org/abs/2311.02503) MapSeg: Segmentation guided structured model for online HD map construction
 - [Efficient](https://ieeexplore.ieee.org/document/10161331) Efficient and Hybrid Decoder for Local Map Construction in Bird'-Eye-View
 - [Mind the map!](https://arxiv.org/abs/2311.10517) Mind the map! Accounting for existing map information when estimating online HDMaps from sensor data
 - [ScalableMap](https://arxiv.org/abs/2310.13378) ScalableMap: Scalable Map Learning for Online Long-Range Vectorized HD Map Construction
 - [TopoNet](https://onlinelibrary.wiley.com/doi/abs/10.1111/cgf.14496) TopoNet: Topology Learning for 3D Reconstruction of Objects of Arbitrary Genus
 - [SuperFusion](https://arxiv.org/abs/2211.15656) SuperFusion: Multilevel LiDAR-Camera Fusion for Long-Range HD Map Generation
 - [MapTR](https://arxiv.org/abs/2208.14437) MapTR: Structured Modeling and Learning for Online Vectorized HD Map Construction
 - [VectorMapNet](https://arxiv.org/abs/2206.08920) VectorMapNet: End-to-end Vectorized HD Map Learning
 - [csBoundary](https://arxiv.org/abs/2111.06020) csBoundary: City-Scale Road-Boundary Detection in Aerial Images for High-Definition Maps
 - [Topo-boundary](https://arxiv.org/abs/2103.17119) Topo-boundary: A Benchmark Dataset on Topological Road-boundary Detection Using Aerial Images for Autonomous Driving
 - [HDMapNet](https://arxiv.org/abs/2107.06307) HDMapNet: An Online HD Map Construction and Evaluation Framework


### 3.4 High Performance Inference
*高性能推理*

**视觉系列**
- [Lite.ai](https://github.com/DefTruth/lite.ai) -
该项目提供了一系列轻量级的目标检测语义分割任务的整合框架支持 YOLOX🔥, YoloR🔥, YoloV5, YoloV4, DeepLabV3, ArcFace, CosFace, RetinaFace, SSD, etc.
- [multi-attention -> onnx](https://github.com/liudaizong/CSMGAN/blob/51348c805e83cf4b1c791592d329851a8e2186aa/code/modules_/multihead_attention.py) -   
提供了一个多头注意力机制支持onnx部署的方式
- [TRT ViT](https://arxiv.org/pdf/2205.09579.pdf) 字节跳动提出的面向工业界部署的ViT

**LiDAR Pillars系列**

- [CUDA-PointPillars](https://github.com/NVIDIA-AI-IOT/CUDA-PointPillars) - NV官方PointPillars部署方案
- [nutonomy_pointpillars](https://github.com/SmallMunich/nutonomy_pointpillars) - PointPillars
- [mmdet3d_onnx_tools](https://github.com/speshowBUAA/mmdet3d_onnx_tools) - PointPillars
- [CenterPoint](https://github.com/CarkusL/CenterPoint) - CenterPoint-PonintPillars 
- [PointPillars_MultiHead_40FPS](https://github.com/hova88/PointPillars_MultiHead_40FPS) - MultiHead PointPillars
- [我自己的 ROS Lidar Perception TensorRT部署](https://github.com/PeterJaq/lidar_perception)
- [CenterPoint](https://github.com/Abraham423/CenterPoint) - CenterPoint 推理方案 ROS

## 4. Prediction

- [An Auto-tuning Framework for Autonomous Vehicles] (https://arxiv.org/pdf/1808.04913.pdf)
- [VectorNet](https://github.com/Liang-ZX/VectorNet.git) -
来自[VectorNet: Encoding HD Maps and Agent Dynamics from Vectorized Representation](https://arxiv.org/abs/2005.04259)利用高精地图 -与目标物信息进对目标进行行为预测。apollo在7.0版本的行为预测部分的encoder利用了这个vectornet.
- [TNT](https://github.com/Henry1iu/TNT-Trajectory-Predition) - TNT是一种基于历史数据（即多代理和环境之间交互）生成目标的轨迹状态序列方法，并基于似然估计得到紧凑的轨迹预测集。
- [DESIRE](https://github.com/tdavchev/DESIRE) - DESIRE: Distant Future Prediction in Dynamic Scenes with Interacting Agents 
- [TNT: Target-driveN Trajectory Prediction](https://arxiv.org/pdf/2008.08294.pdf) apollo在7.0版本的行为预测模块inter-TNT的轨迹生成利用了TNT的方法.
- [MultiPath++](https://github.com/stepankonev/waymo-motion-prediction-challenge-2022-multipath-plus-plus) - Efficient Information Fusion and Trajectory Aggregation for Behavior Prediction.
- [MotionCNN](https://github.com/kbrodt/waymo-motion-prediction-2021) - A Strong Baseline for Motion Prediction in Autonomous Driving.
- [WAT](https://github.com/wei-mao-2019/wat) - Weakly-supervised Action Transition Learning for Stochastic Human Motion Prediction.
- [BEVerse](https://github.com/zhangyp15/beverse) - Unified Perception and Prediction in Birds-Eye-View for Vision-Centric Autonomous Driving.
- [ParkPredict+](https://github.com/xushenlz/parksim) - Vehicle simualtion and behavior prediction in parking lots.
- [HiVT](https://github.com/ZikangZhou/HiVT) - Hierarchical Vector Transformer for Multi-Agent Motion Prediction
- [FEND](https://cvpr2023.thecvf.com/virtual/2023/poster/21711) A Future Enhanced Distribution-Aware Contrastive Learning Framework for Long-Tail Trajectory Prediction
- [EqMotion](https://cvpr2023.thecvf.com/virtual/2023/poster/22896) Equivariant Multi-Agent Motion Prediction With Invariant Interaction Reasoning
- [EigenTrajectory](https://arxiv.org/abs/2307.09306) [ICCV2023] EigenTrajectory: Low-Rank Descriptors for Multi-Modal Trajectory Forecasting
- [Temporal Enhanced](https://arxiv.org/abs/2304.00967) [ICCV2023] Temporal Enhanced Training of Multi-view 3D Object Detector via Historical Object Prediction
- [TrajectoryFormer](https://arxiv.org/abs/2306.05888) [ICCV2023] TrajectoryFormer: 3D Object Tracking Transformer with Predictive Trajectory Hypotheses



  
## 5 Localization and SLAM
   *Localization*
  
  - [hdl_localization](https://github.com/koide3/hdl_localization) - **Lidar + IMU** 基于卡尔曼滤波的位置估计使用了激光雷达，IMU, 可以做到实时估计。
  
  *SLAM*
- [PaGO-LOAM](https://github.com/url-kaist/AlterGround-LeGO-LOAM) 一个基于LeGO-LOAM的LiDAR测距框架，在这个框架中，测试地面分割算法是否有助于提取特征和改善SLAM性能是很容易和直接的。
- [Quatro-LeGO-LOAM](https://github.com/url-kaist/Quatro-LeGO-LOAM) 在城市环境中避免退化的鲁棒性global registration方法 + LeGO-LOAM
- [AVP-SLAM](https://arxiv.org/abs/2007.01813)来自2020IROS的AVP定位方案：AVP-SLAM: Semantic Visual Mapping and Localization for Autonomous Vehicles in the Parking Lot(IROS 2020),主要是通过BEV视角对停车场中的车道线车库线以及标识进行检测并利用其进行稀疏定位。
最近有两位大佬提供了仿真和定位的开源方案：[AVP-SLAM-SIM](https://github.com/TurtleZhong/AVP-SLAM-SIM) [AVP-SLAM-PLUS](https://github.com/liuguitao/AVP-SLAM-PLUS)
- [DeepLIO](https://github.com/ArashJavan/DeepLIO) - **Lidar + IMU** 一款基于深度学习的lidar IMU融合里程计
- [hdl_graph_slam](https://github.com/koide3/hdl_graph_slam) - **Lidar + IMU + GPS** 它基于三维图形SLAM，具有基于NDT扫描匹配的测距估计和循环检测。它还支持几个约束，如GPS、IMU。
- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) - **Lidar + IMU + GPS** 基于激光雷达，IMU和GPS多种传感器的因子图优化方案，以及在帧图匹配中使用帧-局部地图取代帧-全局地图。
- [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM) - **Lidar + Camera** 基于视觉+激光雷达的惯导融合
- [LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM) - **Lidar** LeGO-LOAM是以LOAM为框架而衍生出来的新的框架。其与LOAM相比，更改了特征点的提取形式，添加了后端优化，因此，构建出来的地图就更加的完善。
- [SC-LeGO-LOAM](https://github.com/irapkaist/SC-LeGO-LOAM) - **Lidar** LeGO-LOAM的基于全局描述子Scan Context的回环检测
- [SC-LIO-SAM](https://github.com/gisbi-kim/SC-LIO-SAM) - **Lidar + Camera** LIO-SAM的基于全局描述子Scan Context的回环检测
- [Livox-Mapping](https://github.com/PJLab-ADG/Livox-Mapping) - **Livox + IMU + SC  ** 一款基于Livox的mapping工具包，在先前的工具上添加了SC和Fastlio的一些特性 
- [Fast-LIO](https://github.com/hku-mars/FAST_LIO) - 目前最好用的前端里程计之一，几乎同时兼具速度与鲁棒性
- [Faster-LIO](https://github.com/gaoxiang12/faster-lio) - 比Fast LIO快1-1.5倍的前端里程计
- [FAST_LIO_SLAM](https://github.com/gisbi-kim/FAST_LIO_SLAM) Scancontext + 现在的SOTA里程计(fast lio)
- [SC-A-LOAM](https://github.com/gisbi-kim/SC-A-LOAM) - Scancontext + 现在的SOTA里程计(Lego-loam, fast lio, a loam etc.)
- [FAST_LIO_LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION) Fast lio 系列建图完成后依赖这些执行定位.
- [Deep Functional Maps](https://cvpr2023.thecvf.com/virtual/2023/poster/22894) Understanding and Improving Features Learned in Deep Functional Maps
- [vMap](https://cvpr2023.thecvf.com/virtual/2023/poster/21837) vMAP: Vectorised Object Mapping for Neural Field SLAM
- [DeepLSD](https://openaccess.thecvf.com/content/CVPR2023/papers/Pautrat_DeepLSD_Line_Segment_Detection_and_Refinement_With_Deep_Image_Gradients_CVPR_2023_paper.pdf) DeepLSD: Line Segment Detection and Refinement with Deep Image Gradients
- [EgoLoc](https://arxiv.org/abs/2212.06969) [ICCV2023] EgoLoc: Revisiting 3D Object Localization from Egocentric Videos with Visual Queries


## 6. Planning
*规划*
- [自动驾驶中的决策规划算法概述](https://www.jiqizhixin.com/articles/2019-07-22)
- [有限状态机](https://en.wikipedia.org/wiki/Finite-state_machine)
- [MPC](https://en.wikipedia.org/wiki/Model_predictive_control)
- [PathPlanning](https://github.com/zhm-real/PathPlanningt)
- [pacmod](https://github.com/astuff/pacmod) -  Designed to allow the user to control a vehicle with the PACMod drive-by-wire system.
- [rrt](https://github.com/RoboJackets/rrt) - C++ RRT (Rapidly-exploring Random Tree) implementation.
- [HypridAStarTrailer](https://github.com/AtsushiSakai/HybridAStarTrailer) - A path planning algorithm based on Hybrid A* for trailer truck.
- [path_planner](https://github.com/karlkurzer/path_planner) - Hybrid A* Path Planner for the KTH Research Concept Vehicle.
- [fastrack](https://github.com/HJReachability/fastrack) - A ROS implementation of Fast and Safe Tracking (FaSTrack).
- [commonroad](https://commonroad.in.tum.de/) - Composable benchmarks for motion planning on roads.
- [traffic-editor](https://github.com/osrf/traffic-editor) - A graphical editor for robot traffic flows.
- [steering_functions](https://github.com/hbanzhaf/steering_functions) - Contains a C++ library that implements steering functions for car-like robots with limited turning radius.
- [moveit](https://moveit.ros.org/) - Easy-to-use robotics manipulation platform for developing applications, evaluating designs, and building integrated products.
- [flexible-collision-library](https://github.com/flexible-collision-library/fcl) - A library for performing three types of proximity queries on a pair of geometric models composed of triangles.
- [aikido](https://github.com/personalrobotics/aikido) - Artificial Intelligence for Kinematics, Dynamics, and Optimization.
- [casADi](https://github.com/casadi/casadi) - A symbolic framework for numeric optimization implementing automatic differentiation in forward and reverse modes on sparse matrix-valued computational graphs.
- [ACADO Toolkit](https://github.com/acado/acado) - A software environment and algorithm collection for automatic control and dynamic optimization.
- [CrowdNav](https://github.com/vita-epfl/CrowdNav) - Crowd-aware Robot Navigation with Attention-based Deep Reinforcement Learning.
- [ompl](https://github.com/ompl/ompl) - Consists of many state-of-the-art sampling-based motion planning algorithms.
- [openrave](https://github.com/rdiankov/openrave) - Open Robotics Automation Virtual Environment: An environment for testing, developing, and deploying robotics motion planning algorithms.
- [teb_local_planner](https://github.com/rst-tu-dortmund/teb_local_planner) - An optimal trajectory planner considering distinctive topologies for mobile robots based on Timed-Elastic-Bands.
- [pinocchio](https://github.com/stack-of-tasks/pinocchio) - A fast and flexible implementation of Rigid Body Dynamics algorithms and their analytical derivatives.
- [rmf_core](https://github.com/osrf/rmf_core) - The rmf_core packages provide the centralized functions of the Robotics Middleware Framework (RMF).
- [global_racetrajectory_optimization](https://github.com/TUMFTM/global_racetrajectory_optimization) - This repository contains multiple approaches for generating global racetrajectories.
- [toppra](https://github.com/hungpham2511/toppra) - A library for computing the time-optimal path parametrization for robots subject to kinematic and dynamic constraints.
- [tinyspline](https://github.com/msteinbeck/tinyspline) - TinySpline is a small, yet powerful library for interpolating, transforming, and querying arbitrary NURBS, B-Splines, and Bézier curves.
- [dual quaternions ros](https://github.com/Achllle/dual_quaternions_ros) - ROS python package for dual quaternion SLERP.
- [mb planner](https://github.com/unr-arl/mbplanner_ros) - Aerial vehicle planner for tight spaces. Used in DARPA SubT Challenge.
- [ilqr](https://github.com/anassinator/ilqr) - Iterative Linear Quadratic Regulator with auto-differentiatiable dynamics models.
- [EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner) - A lightweight gradient-based local planner without ESDF construction, which significantly reduces computation time compared to some state-of-the-art methods.
- [pykep](https://github.com/esa/pykep) - A scientific library providing basic tools for research in interplanetary trajectory design.
- [am_traj](https://github.com/ZJU-FAST-Lab/am_traj) - Alternating Minimization Based Trajectory Generation for Quadrotor Aggressive Flight.
- [GraphBasedLocalTrajectoryPlanner](https://github.com/TUMFTM/GraphBasedLocalTrajectoryPlanner) - Was used on a real race vehicle during the Roborace Season Alpha and achieved speeds above 200km/h.
- [Ruckig](https://ruckig.com) - Instantaneous Motion Generation. Real-time. Jerk-constrained. Time-optimal.

## 7. Control
*控制*
- [PID](https://en.wikipedia.org/wiki/PID_controller)
- [Open Source Car Control](https://github.com/PolySync/oscc) -  An assemblage of software and hardware designs that enable computer control of modern cars in order to facilitate the development of autonomous vehicle technology.
- [control-toolbox](https://github.com/ethz-adrl/control-toolbox) - An efficient C++ library for control, estimation, optimization and motion planning in robotics.
- [mpcc](https://github.com/alexliniger/MPCC) - Model Predictive Contouring Controller for Autonomous Racing.
- [open_street_map](https://github.com/ros-geographic-info/open_street_map) - ROS packages for working with Open Street Map geographic information.
- [autogenu-jupyter](https://github.com/mayataka/autogenu-jupyter) - This project provides the continuation/GMRES method (C/GMRES method) based solvers for nonlinear model predictive control (NMPC) and an automatic code generator for NMPC.
- [OpEn](https://github.com/alphaville/optimization-engine) - A solver for Fast & Accurate Embedded Optimization for next-generation Robotics and Autonomous Systems.

## 9.  Dataset and Competition
*数据集与竞赛*

- [KITTI](http://www.cvlibs.net/datasets/kitti/)
- [BDD100k](https://www.bdd100k.com/)
- [UrbanNav](https://github.com/weisongwen/UrbanNavDataset) - 一个在亚洲城市峡谷（包括东京和香港）收集的开源本地化数据集,主要用于解决定位算法的各种问题。
- [ONCE](https://once-for-auto-driving.github.io)
- [SODA10M](https://soda-2d.github.io/)
- [OPV2V](https://mobility-lab.seas.ucla.edu/opv2v/) - 首个大型自动驾驶协同感知数据集 + banchmark代码框架, 由UCLA提供

## 10. Data Loop & Model Loop
*数据闭环*

**NAS**
- [Beta-DARTS](https://github.com/Sunshine-Ye/Beta-DARTS) Beta-Decay Regularization for Differentiable Architecture Search
- [ISNAS-DIP](https://arxiv.org/abs/2111.15362) Image-Specific Neural Architecture Search for Deep Image Prior

**主动学习**

- [Discriminative Active Learning](https://github.com/dsgissin/DiscriminativeActiveLearning)
- [AL-FM](https://openaccess.thecvf.com/content/CVPR2022/papers/Parvaneh_Active_Learning_by_Feature_Mixing_CVPR_2022_paper.pdf) [CVPR2022]基于特征混合的主动学习
- [LfOSA](https://arxiv.org/pdf/2201.06758.pdf) [CVPR2022] 面向Open set的 active learning 框架


**Coner case & Long-tail**
- [RAC](https://arxiv.org/abs/2202.11233) Retrieval Augmented Classification for Long-Tail Visual Recognition*

**数据挖掘**
- [AirDet](https://arxiv.org/abs/2112.01740) Few-Shot Detection without Fine-tuning for Autonomous Exploration. 这篇文章把他放在数据挖掘方面是思考有没有可能用极少样本不用fine-tuning 后可以从原有自动驾驶数据湖中挖掘出更多的样本。

**Data Requirement**
- [Data Requirement](https://cvpr2023.thecvf.com/virtual/2023/poster/21020) A Meta-Learning Approach to Predicting Performance and Data Requirements
- [Independent Componenet Alignment MT](https://cvpr2023.thecvf.com/virtual/2023/poster/21126) Independent Component Alignment for Multi-Task Learning
- [Data-Efficient](https://cvpr2023.thecvf.com/virtual/2023/poster/21084) Data-Efficient Large Scale Place Recognition With Graded Similarity Supervision
- [Data Deterrministic Uncertainty](https://cvpr2023.thecvf.com/virtual/2023/poster/21132)Deep Deterministic Uncertainty: A New Simple Baseline

**OOD**
- [Rethink OOD](https://cvpr2023.thecvf.com/virtual/2023/poster/21132) Rethink OOD
## 11. Visualization
*可视化工具*

- [Carla-birdeye-view](https://github.com/deepsense-ai/carla-birdeye-view) - 可以对接carla的自动驾驶鸟瞰图组件。
- [Uber AVS](https://avs.auto/#/) - 自动驾驶可视化前端组件 xviz 与 streetscape.gl 
- [Cruise](https://webviz.io/worldview/#/) - Cruise 开源的一款自动驾驶前端可视化套件

# 12. Simulation
- [UniSim](https://cvpr2023.thecvf.com/virtual/2023/poster/22272) [CVPR2023] A Neural Closed-Loop Sensor Simulator
- [LiDar-in-the-loop](https://cvpr2023.thecvf.com/virtual/2023/poster/21850) [CVPR2023] LiDAR-in-the-Loop Hyperparameter Optimization
- [Compact Representation](https://cvpr2023.thecvf.com/virtual/2023/poster/22264) [CVPR2023] Learning Compact Representations for LiDAR Completion and Generation
- [MixSim](https://cvpr2023.thecvf.com/virtual/2023/poster/22001) [CVPR2023] MixSim: A Hierarchical Framework for Mixed Reality Traffic Simulation
- [The Differentiable Lens](https://cvpr2023.thecvf.com/virtual/2023/poster/22808) [CVPR2023] Compound Lens Search Over Glass Surfaces and Materials for Object Detection

## 13. Others
*其他更好的分享*
- [awesome-3D-object-detection](https://github.com/Tom-Hardy-3D-Vision-Workshop/awesome-3D-object-detection)
- [3D-ObjectDetection-and-Pose-Estimation](https://github.com/littlebearsama/3D-ObjectDetection-and-Pose-Estimation) -物体检测与位姿估计
- [Awesome-Knowledge-Distillation-for-Autonomous-Driving](https://github.com/linyanAI/Awesome-Knowledge-Distillation-for-Autonomous-Driving) Knowledge Distillation for AD

