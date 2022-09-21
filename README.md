# RoboMaster2022 IRobot 步兵视觉代码开源

## 1. 写在前面

本代码为IRobot 战队 2022年RM比赛中的步兵视觉代码，包含了能量机关击打实现。如有问题可咨询本人`摸王nya（QQ:2495217894）`。

这是我从本赛季入队后就一直写到赛季结束的代码，里面的架构（太屎了）、注释、命名等多多少少可能会存在着一些问题，各种模块耦合也是粗茶淡饭，以及缺少系统的设计。虽然实战效果并不是很好，因为很多技术问题（反小陀螺、测距不稳等）并没有突破，但却确实也实现了一些功能（运动预测、能量机关）。关于能量机关的技术报告我在我的另一篇帖子里已经放出了，但由于是一个晚上赶出来的，可能会存在着一些勘误，在`Algorithm`目录下也有。

最近这段时间也是在忙着各种实验作业、新人培训、读论文之类的琐事，腾不出时间来对这份代码作出什么修改。只能说希望看的人不要找上门骂我呜呜呜。

## 2. 文件树

```
├── Algorithm
├── CMakeLists.txt
├── Configure					# 参数文件
│   ├── Debug.xml
│   ├── param
│   ├── RuneParam.xml
│   └── Settings.xml
├── include
│   ├── Armor					# 装甲板识别模块
│   │   ├── ArmorBox.h
│   │   ├── ArmorDetector.h
│   │   ├── classfier.h			# Eigen数字识别
│   │   ├── CV_Classifier.h		# 弃用
│   │   └── SpinDetector.h
│   ├── Const					# 常量及一些结构体
│   │   ├── const.h
│   │   └── rm_types.h
│   ├── Debug					# 调试模块
│   │   ├── Debug.h
│   │   └── DebugParam.h
│   ├── Extend
│   │   ├── Mat_time.h
│   │   └── Tools.h
│   ├── ImageProgress			# 多线程
│   │   └── ImageProgress.h
│   ├── mercure					# 大恒mer系列相机驱动
│   │   ├── DxImageProc.h
│   │   ├── GxIAPI.h
│   │   └── mercure_driver.h
│   ├── Rune					# 能量机关
│   │   ├── 打符追踪状态处理.png
│   │   ├── Fitting.h
│   │   ├── README.md
│   │   ├── RuneBox.h
│   │   └── RuneDetector.h
│   ├── Serial					# 通信
│   │   ├── CanSerial.h
│   │   └── Serial.h		# 弃用
│   └── Solver
│       ├── CoordConvert.h
│       ├── EKF.h			# 弃用
│       ├── Predictor.h
│       └── Solver.h
├── README.md
└── src
    ├── Armor
    │   ├── ArmorBox.cpp
    │   ├── ArmorDetector.cpp
    │   ├── classfier.cpp			
    │   ├── CV_Classifier.cpp		# 弃用
    │   └── SpinDetector.cpp
    ├── Debug
    │   └── Debug.cpp
    ├── Extend
    │   ├── Mat_time.cpp
    │   └── Tools.cpp
    ├── ImageProgress
    │   └── ImageProgress.cpp
    ├── main.cpp
    ├── mercure
    │   ├── CMakeLists.txt
    │   ├── libgxiapi.so
    │   └── mercure_driver.cpp
    ├── Rune
    │   ├── Fitting.cpp
    │   ├── RuneBox.cpp
    │   └── RuneDetector.cpp
    ├── Serial
    │   ├── CanSerial.cpp			
    │   └── Serial.cpp					# 弃用
    └── Solver
        ├── EKF.cpp						# 弃用
        ├── Predictor.cpp
        └── Solver.cpp
```

