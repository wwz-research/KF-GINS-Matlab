   ## KF-GINS-Matlab
   
[[中]](./README_CN.md) &ensp; [[EN]](./README.md)

## Matlab实现的基于扩展卡尔曼滤波的GNSS/INS组合导航软件（Matlab版KF-GINS）

### 简介

虽然[KF-GINS](https://github.com/i2Nav-WHU/KF-GINS)只实现了经典的GNSS/INS松组合导航解算，适合初学者学习理解组合导航算法，但是作为一套基于C++的组合导航软件，KF-GINS在代码调试和修改扩展方面对初学者仍有一定的挑战。因此，我们开源了一套基于Matlab实现的组合导航软件 KF-GINS-Matlab，作为KF-GINS的Matlab版本，更加适合于教学实验和初学者入门。除了实现Matlab版的KF-GINS外，KF-GINS-Matlab还增加了GNSS速度修正算法，并添加了ODO/NHC更新的框架。(注意：ODO/NHC观测更新的部分并没有完整实现，ODO比例因子误差也没有增广。欢迎大家根据文档中的算法自己填上，作为一种练习。)

**单位:** 武汉大学卫星导航定位技术研究中心[多源智能导航实验室](http://www.i2nav.com/)

**相关资料:**

- X. Niu, L. Wang, Q. Chen, H. Tang, Q. Zhang, and T. Zhang, “KF-GINS: an open-sourced software for GNSS/INS integrated navigation,” GPS Solut, vol. 29, no. 4, p. 202, Oct. 2025, doi: 10.1007/s10291-025-01967-w.
- 牛小骥, 陈起金, 张全, "惯性导航原理与算法设计", 高等学校导航工程专业规划教材. 武汉: 武汉大学出版社, 2025.
- 牛小骥, 陈起金, "[惯性导航原理与GNSS/INS组合导航课程讲义](http://www.i2nav.com/index/newListDetail_zw.do?newskind_id=13a8654e060c40c69e5f3d4c13069078&newsinfo_id=40f3c65b158742c099ba3b600c983aa1)", 武汉大学多源智能导航实验室, 2022.
- 牛小骥, 陈起金, "[惯性导航原理与GNSS/INS组合导航课程视频](https://www.bilibili.com/video/BV1na411Z7rQ?spm_id_from=333.999.0.0&vd_source=a417ebe0768fc96919fe8e34c55ed591)", 武汉大学多源智能导航实验室, 2022.
- X. Niu, Q. Zhang, L. Gong, C. Liu, H. Zhang, C. Shi, J. Wang and M. Coleman (2014). "Development and evaluation of GNSS/INS data processing software for position and orientation systems." Survey Review 2014; 47(341), 87-98.
- 严恭敏, 翁浚, 捷联惯导算法与组合导航原理. 西北工业大学出版社, 2019.
- E.-H. Shin, "Estimation techniques for low-cost inertial navigation," Ph.D. dissertation, Dept. Geomatics Eng., Univ. of Calary, AB, Cabada, 2005
- [Savage, P.G.](http://www.strapdownassociates.com/), "Strapdown analytics, Part I", Maple Plain, MN: Strapdown Associates, 2000.
- Titterton David, JohnL. Weston, and John Weston, "Strapdown inertial navigation technology", The Institution of Electrical Engineers, 2004.
- P. D. Groves, "Principles of GNSS, Inertial, and Multi-sensor Integrated Navigation Systems", 2nd ed., vol. 39. Artech House, 2013.

**如果你使用这个软件进行学术研究，请添加如下致谢，并引用我们的[相关文档和论文](./ref.bib)**

```
中文模板：“本文作者感谢武汉大学卫星导航定位技术研究中心多源智能导航实验室(i2Nav)牛小骥教授团队开源的KF-GINS-Matlab软件平台。”
English version: “The authors would like to acknowledge the team of Prof. Xiaoji Niu of the Integrated and Intelligent Navigation (i2Nav) group from GNSS Research Center of Wuhan University for providing the open-source KF-GINS-Matlab software that was used in the paper.”
```

**联系:**

- 有任何相关的技术问题，可以向王立强(wlq@whu.edu.cn)发送邮件，或在仓库中开启一个议题。
- 中国使用者可以加入QQ交流群(481173293)进行讨论，进群需要提供所在机构和真实姓名。

## 1.KF-GINS-Maltab文件介绍

### 1.1 本仓库在原版基础上的扩展

- **基于原项目**：本仓库在武汉大学卫星导航定位技术研究中心多源智能导航实验室（i2Nav）开源的官方 `KF-GINS-Matlab` 项目基础上进行扩展和实验开发，继续采用 **GPLv3** 许可，并沿用原项目在论文中的致谢/引用要求。
- **新增数据集与配置**：增加 `dataset5` 与 `ProcessConfig5.m`，用于车辆组合导航实验；支持二进制 IMU 数据 `IMU.bin`，以及 RTKLIB 风格的 GNSS 解算结果文件（如 `GNSSsolution.pos`, `MOV_*.pos`），通过 `ReadBinaryIMU.m` 与 `ReadGNSSData.m` 进行解析。
- **ODO / NHC 组合实现**：实现 `GetOdoVel.m` 和统一的 `ODONHCUpdate.m`，在 EKF 中将里程计比例因子作为第 22 维扩展状态进行估计，实现 1 维 ODO、2 维 NHC 和 3 维 ODO+NHC 三种观测模式，并在 `Initialize.m` 与 `ErrorFeedback.m` 中完成相应协方差与反馈更新。
- **零速 / 零航向角速率更新 (ZVT)**：新增 `ZVTUpdate.m`，并在 `kf_gins.m` 与 `ProcessConfig5.m` 中加入零速时间段与更新类型配置，实现零速约束（ZUPT）和零航向角速率约束的联合更新，同时将新息输出到 `ZuptInnov.txt` 与 `ZyrInnov.txt` 以便后处理分析。
- **角速度日志与后处理分析**：在主循环中计算并输出 \(\omega_{nb}^b\) 到 `Omega_nb_b.txt`，结合扩展的 `plot-function/` 绘图脚本，对杆臂速度补偿、ODO/NHC/ZVT 新息、里程计比例因子估计以及多组合模式误差进行可视化分析。
- **敏感性实验脚本**：增加 `exp_gyrbias_sensitivity.m`、`exp_gyrbias_processnoise.m` 等实验脚本，在 `dataset5` 上研究初始陀螺零偏协方差与过程噪声设置对滤波收敛性与稳态精度的影响。

如果你在论文或项目中使用本仓库，**请同时引用原 KF-GINS / KF-GINS-Matlab 的论文和文档，并说明本仓库是在原版基础上实现了 ODO/NHC/ZVT 及相关敏感性实验的扩展版本**。

### 1.2 原版文件一览

| 文件名                        | 主要功能描述                        |
| ----------------------------- | ----------------------------------- |
| kf_gins.m                     | 程序入口，组合算法处理主循环        |
| Initialize.m                  | 程序初始化                          |
| InsMech.m                     | 捷联惯导机械编排算法                |
| InsPropagate.m                | 状态向量预测和协方差传播            |
| GNSSUpdate.m                  | GNSS位置/速度观测更新               |
| GetOdoVel.m                   | 处理ODO原始数据，获取ODO测速结果    |
| ODONHCUpdate.m                | ODO/NHC速度更新函数(观测更新未实现) |
| ErrorFeedback.m               | 误差反馈                            |
| ProcessConfig.m               | 处理程序的参数配置                  |
| ProcessConfig1.m              | GNSS位置/INS测试数据程序参数配置    |
| ProcessConfig2.m              | GNSS速度/INS测试数据程序参数配置    |
| ProcessConfig3.m              | ODO/NHC测试数据程序参数配置         |
| Initialize.m                  | 程序初始化                          |
| ProcessConfig.m               | 测试数据参数配置                    |
| function/Param.m              | 处理程序中的地理参数                |
| function/*.m                  | 旋转参数变换库/地理参数转换库       |
| plot-function/plot_result.m   | 绘制导航结果曲线                    |
| plot-function/plot_std.m      | 绘制估计的导航状态STD               |
| plot-function/plot_imuerror.m | 绘制估计的IMU误差                   |
| plot-function/calc_error.m    | 计算定位结果的误差并绘制误差曲线    |
| dataset1                      | GNSS位置/INS组合测试数据            |
| dataset2                      | GNSS速度/INS组合测试数据            |
| dataset3                      | ODO/NHC组合测试数据                 |

## 2.KF-GINS-Matlab运行

### 2.1 运行环境和程序下载

KF-GINS-Matlab需要在Matlab软件中运行，不依赖额外的扩展包，不依赖系统环境，不特殊要求Matlab版本。

可以通过如下git命令下载KF-GINS-Matlab，也可以直接在github上下载压缩包进行解压。

```
git clone https://github.com/i2Nav-WHU/KF-GINS-Matlab.git ~/
```

### 2.2  KF-GINS-Matlab运行

1）在Matlab中打开KF-GINS-Matlab文件夹

2）双击kf-gins.m文件进入kf-gins.m编辑器，点击主菜单栏“编辑器”->"运行"，或者命令行窗口输入kf-gins(默认运行ProcessConfig1.m)

3）等待命令行提示程序运行结束

### 2.3 显示运行结果

1）将plot-function文件夹添加到工作区：右键单击plot-function文件夹，选择“添加到路径”->“选定的文件夹和子文件夹”

2）展开plot-function文件夹，执行plot_result.m绘制定位结果，执行clac_error.m绘制定位误差

### 2.4 注意事项

ODO/NHC 更新框架中，根据更新频率设置ODO更新时间。每次ODO更新时，使用GetOdoVel函数从原始数据中获取ODO速度。

GetOdoVel.m函数中，对更新时刻前后各10个历元的ODO数据平均得到用于更新的ODO速度，减小量化噪声。

### 2.5 运行其他测试数据

1）运行其他测试数据时，需要修改kf-gins.m中的cfg值；绘制其他数据结果时，需要修改绘图脚本中的文件路径

2）运行自己的测试数据时，建议重新自定义ProcessConfig.m，并修改文件路径和配置信息

## 3 测试数据

### 3.1 测试数据

KF-GINS-Matlab在原版基础上提供了三套经典测试数据，并在本仓库中新增一套扩展测试数据 `dataset5`。其中 dataset1 为 GNSS 位置/INS 测试数据（与 KF-GINS 中的测试数据相同）；dataset2 为 GNSS 速度/INS 测试数据；dataset3 为 ODO/NHC 测试数据；dataset5 为车辆 GNSS/INS/ODO 组合导航扩展测试数据，用于 ODO/NHC/ZVT 以及敏感性实验。

| 数据文件夹 | 文件名          | 描述             |
| ---------- | --------------- | ---------------- |
| dataset1   | Leador-A15.txt  | IMU文本文件      |
|            | GNSS-RTK.txt    | GNSS位置文件     |
|            | truth.nav       | 参考真值文件     |
| dataset2   | ADIS16465.txt   | IMU文本文件      |
|            | GNSS-POSVEL.txt | GNSS位置速度文件 |
|            | truth.nav       | 参考真值文件     |
| dataset3   | ADIS16465.txt   | IMU文本文件      |
|            | GNSS-POS.txt    | GNSS位置文件     |
|            | ODO.txt         | ODO速度文件      |
|            | truth.nav       | 参考真值文件     |
| dataset5   | IMU.bin         | IMU 二进制文件   |
|            | GNSSsolution.pos / MOV_*.pos | GNSS 解算结果文件（RTK/PPP/SPP），由 `ReadGNSSData` 解析 |
|            | ODO.txt（可选） | 里程计前向速度文件（开启 ODO/NHC 时使用） |
|            | REF_NAV.nav     | 高精度参考导航结果 |

### 3.2 数据格式

IMU文件、GNSS位置文件、参考真值，以及生成的导航结果、IMU误差文件和STD文件的具体格式请参考KF-GINS/README.md。GNSS位置速度文本文件，ODO文本文件格式如下：

- GNSS位置速度文本文件格式定义为:

| 列数  | 数据描述                   | 单位    |
| ----- | -------------------------- | ------- |
| 1     | GNSS 周内秒                | $s$   |
| 2     | 纬度                       | $rad$ |
| 3     | 经度                       | $rad$ |
| 4     | 椭球高                     | $m$   |
| 5~7   | 位置标准差(北向-东向-地向) | $m$   |
| 8~10  | 三维速度(北向-东向-地向)   | $m/s$ |
| 11~13 | 速度标准差(北向-东向-地向) | $m/s$ |

- ODO文本文件格式如下：

| 列数 | 数据描述   | 单位    |
| ---- | ---------- | ------- |
| 1    | GNSS周内秒 | $s$   |
| 2    | ODO速度    | $m/s$ |

## 4 许可

KF-GINS-Matlab 源代码在 GPLv3 许可下发布。

商业用途请联系牛小骥教授(xjniu@whu.edu.cn)。
