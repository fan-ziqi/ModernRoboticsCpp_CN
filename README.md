# C++ 库: 现代机器人学：机构、规划与控制(中文版)

## 更新

2022.10.06 增加`MRTest`例程，位于`MRLib`路径下。更改main函数内容，加入UR5测试例程。

2022.10.05 第八章已更完，增加旋转矩阵，更新`main.c`测试例程

## 简介

本仓库为[_Modern Robotics: Mechanics, Planning, and Control(现代机器人学：机构、规划与控制)_](http://modernrobotics.org)  的C++版本代码库。

代码库位于`MRLib`路径下，[用户手册](https://github.com/NxRLab/ModernRobotics/blob/master/doc/MRlib.pdf) 在 [主仓库](https://github.com/NxRLab/ModernRobotics/) 的`doc`路径下。

本代码库目前支持的语言：

* C++
* [Python](https://github.com/NxRLab/ModernRobotics/tree/master/packages/Python)
* [MATLAB](https://github.com/NxRLab/ModernRobotics/tree/master/packages/Matlab)
* [Mathematica](https://github.com/NxRLab/ModernRobotics/tree/master/packages/Mathematica)

在每个函数上面都有一个注释部分，解释其功能、输入和输出。

注意：本库的主要目的是易于阅读和教育，强化书中的概念。该代码既没有针对效率也没有针对健壮性进行优化。

## 改进

增加旋转矩阵`rotx`、`roty`、`rotz`。

## 依赖

本代码库依赖[Eigen库](https://eigen.tuxfamily.org/index.php?title=Main_Page) （仓库中已包含了3.4.0版本，位于`Eigen`路径下）
