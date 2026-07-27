# 依赖库版本说明

本项目使用的第三方库版本如下（截止到：2026年7月5日）。

---

## 核心依赖

- **Eigen**：3.4.0  
- **Ceres Solver**：2.2.0  
- **Sophus**：1.24.6  
- **g2o**：1.0.0  
- **PCL**：1.12.1  
---

## 说明


- **Sophus**  
  由于新版Sophus的SO3用的四元组, 所以我们需要使用`EIGEN_DONT_VECTORIZE`取消对齐


## ROS 1 到 2 的转换
ros1 to ros2 convert: rosbags-convert --src /home/seamanj/Software/dataset/sad/wxb/test1.bag \
--dst /home/seamanj/Software/dataset/sad/wxb/test1_ros2


rosbags-convert --src /home/seamanj/Software/dataset/sad/nclt/20120115.bag \
--dst /home/seamanj/Software/dataset/sad/nclt/20120115_ros2


rosbags-convert --src /home/seamanj/Software/dataset/sad/avia/HKU_MB_2020-09-20-13-34-51.bag \
--dst /home/seamanj/Software/dataset/sad/avia/HKU_MB_2020-09-20-13-34-51_ros2