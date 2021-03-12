<!--
 * @Date: 2021-03-13 01:23:08
 * @Author: Zhiqi Feng
 * @LastEditors: feng 
 * @LastEditTime: 2021-03-13 01:40:44
 * @FilePath: /learnning/planning_learnning/Frenet2Cartesian/Frenet.md
-->
<!-- TOC -->

- [Frenet坐标系与Cartesian坐标系互转](#frenet坐标系与cartesian坐标系互转)
  - [一、Frenet坐标系与Cartesian坐标系的转换公式简单推导](#一frenet坐标系与cartesian坐标系的转换公式简单推导)

<!-- /TOC -->
# Frenet坐标系与Cartesian坐标系互转
Frenet坐标系使用道路的中心线作为Base frame，使用参考线的切线向量和法线向量建立坐标系。相比笛卡尔坐标系，Frenet坐标系简化了路径规划问题。

参考文献：

[Apollo项目坐标系研究](https://blog.csdn.net/davidhopper/article/details/79162385)

[Frenet坐标系与Cartesian坐标系互转](https://blog.csdn.net/u013468614/article/details/108748016)


![](images/2021-03-13-01-27-41.png)

## 一、Frenet坐标系与Cartesian坐标系的转换公式简单推导
1.1 Frenet公式
下图显示了一条3D空间中一条连续可微的曲线K，P为曲线K上的一个点，黄色平面为曲线K在点P处的运动平面。T ⃗ \vec{T} 
T
 为曲线K在点P处的切向量，N ⃗ \vec{N} 
N
 为K在P处的法向量（N ⃗ \vec{N} 
N
 与T ⃗ \vec{T} 
T
 在同一运动平面）,B ⃗ \vec{B} 
B
 为曲线K在P处的副法向量（N ⃗ \vec{N} 
N
 垂直于运动平面）。


