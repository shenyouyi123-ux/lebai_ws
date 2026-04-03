[乐白机械臂 Python SDK 图解操作指南.md](https://github.com/user-attachments/files/26458584/Python.SDK.md)
# **乐白机械臂 Python SDK 图解操作指南**

本文档严格按照乐白官方 Python SDK 文档的章节顺序（对应图1至图8）整理，详细说明了每个模块的功能、API 调用方法以及核心参数。

## **第一章：Python SDK 简介（对应图 1）**

官方 SDK 提供了两种不同的运行模式，对 Python 版本有基本要求：

* **版本要求**：仅支持 **Python 3.7 及以上版本**。  
* **常规调用方式（同步模式）**：  
  这是最常用的模式，适合一般的脚本编写和控制。  
  pip install lebai\_sdk

* **异步协程调用方式（Async 模式）**：  
  如果你的项目使用了 AsyncIO 及 async/await 语法，需要安装异步版本的 SDK。  
  pip install lebai\_sdk\_asyncio

## **第二章：初始化连接（对应图 2）**

本章介绍了如何发现设备并与机械臂建立通信连接。

### **1\. 设备发现 (discover\_devices)**

基于 mDNS 技术，用于在局域网内寻找机械臂的 IP 和 MAC 地址。

* **注意**：此功能在跨网段或前端 JavaScript SDK 中不支持。

import lebai\_sdk  
\# 持续扫描局域网设备 3 秒  
devices \= lebai\_sdk.discover\_devices(3)  
\# 返回值示例：\[{'name': 'Lebai-Robot', 'ip': '10.20.17.1', 'mac': 'xx:xx:xx...'}\]

### **2\. 初始化对象 (LebaiRobot)**

通过设备的 IP 地址实例化控制对象。

robot\_ip \= "10.20.17.1"  
\# 第二个参数为 is\_sim (bool)，False 代表连接真实机械臂，True 代表仿真模式  
robot \= lebai\_sdk.LebaiRobot(robot\_ip, False)

### **3\. 获取连接状态与等待断开**

\# 判断当前是否处于连接状态 (返回 True/False)  
is\_connected \= robot.is\_connected()

\# 阻塞当前线程，直到机械臂连接断开  
robot.wait\_disconnected()

## **第三章：状态数据（对应图 3）**

用于获取机械臂的系统级运行状态和急停报警信息。

### **1\. 机器人状态获取 (get\_robot\_state)**

返回机械臂当前所处的状态枚举。

state \= robot.get\_robot\_state()  
\# 常见状态包括：  
\# DISCONNECTED(断开), ESTOP(急停), BOOTING(启动中), ROBOT\_OFF(关机)  
\# ROBOT\_ON(待机), IDLE(空闲), MOVING(运动中), PAUSED(暂停) 等。

### **2\. 获取急停原因 (get\_estop\_reason)**

当 get\_robot\_state() 为 ESTOP 时，可通过此接口排查原因。

reason \= robot.get\_estop\_reason()  
\# 常见急停原因包括：None(无), System(系统), Master(主控), Collision(碰撞) 等。

### **3\. 获取其他状态数据 \[需补充\]**

*(注：原图 3 下半部分较模糊，推测包含控制柜数据和关节传感器数据)*

\# \[推测\] 获取控制柜/底层数据，通常包含控制箱温度、电压等  
\# data \= robot.get\_cabinet\_data() 或类似接口

## **第四章：机器人系统控制（对应图 4）**

控制机械臂控制柜的电源和系统状态。

### **1\. 启动手臂 (start\_sys)**

系统上电并松开电机抱闸。**必须在执行运动指令前调用。**

robot.start\_sys()

### **2\. 停止手臂 (stop\_sys)**

停止手臂运动并锁死电机抱闸。此时系统仍开机，但处于安全锁定状态。

robot.stop\_sys()

### **3\. 关机 (powerdown)**

软关机指令，控制柜将保存数据并切断主电源。

robot.powerdown()

### **4\. 急停 (estop)**

软件触发急停。机器人会立即停止所有运动，并切断电机动力。**恢复时需要旋开急停（如果是物理按钮）并重新调用 start\_sys()。**

robot.estop()

## **第五章：运动（对应图 5）**

控制机械臂运动的核心模块，包含关节运动和笛卡尔空间运动。

所有运动指令共享以下可选参数：

* **a (float)**: 加速度 (关节运动为 rad/s^2，直线运动为 m/s^2)  
* **v (float)**: 速度 (关节运动为 rad/s，直线运动为 m/s)  
* **t (float)**: 运行时间 (秒)。设为 0 时，由 a 和 v 自动规划。  
* **r (float)**: 交融半径 (米)。设为 0 表示精准停在目标点。大于 0 时用于连续多点运动的平滑过渡。

### **1\. 关节运动 (movej)**

以各关节的最优路径移动，末端轨迹在空间中通常**不是**直线。

\# p 可以是 JointPose(关节弧度) 或 CartesianPose(空间位姿)  
robot.movej(p, a=0.5, v=0.2, t=0, r=0)

### **2\. 直线运动 (movel)**

强制末端工具中心点 (TCP) 在空间中走绝对直线。

\# p 必须是 CartesianPose (x, y, z, rx, ry, rz)  
robot.movel(p, a=0.1, v=0.05, t=0, r=0)

### **3\. 圆弧运动 (movec) \[需补充\]**

*(注：图 5 中存在长篇幅运动指令，推测包含圆弧插补)*

\# 通常包含途经点 (via) 和 终点 (to)  
\# robot.movec(via\_pose, to\_pose, rad=0, a=0.1, v=0.05, t=0, r=0)

### **4\. 停止运动 (stop\_move)**

停止当前的运动队列。

robot.stop\_move()

## **第六章：位置和位姿（对应图 6）**

用于读取机械臂的空间坐标，以及手动进行运动学解算。

### **1\. 正解 (kinematics\_forward)**

输入 6 个关节的角度（JointPose），计算出对应的末端笛卡尔位姿（CartesianPose）。

cartesian\_pose \= robot.kinematics\_forward(joint\_pose)

### **2\. 逆解 (kinematics\_inverse)**

输入期望的末端位姿（CartesianPose），计算出达到该位置需要的关节角度（JointPose）。由于可能有多个解，需要传入参考关节角度（通常是当前角度）。

target\_joint\_pose \= robot.kinematics\_inverse(target\_cart\_pose, current\_joint\_pose)

### **3\. 获取实际状态 (get\_actual\_\*)**

获取机械臂**当前真实的**物理位置。

\# 获取实际关节位置 (返回 JointPose，单位：弧度)  
actual\_joints \= robot.get\_actual\_joint\_positions()

\# 获取实际末端位姿 (返回 CartesianPose，单位：米, 弧度)  
actual\_tcp \= robot.get\_actual\_tcp\_pose()

### **4\. 获取目标状态 (get\_target\_\*)**

获取机械臂在运动规划器中**正在前往的**理论目标位置。

target\_joints \= robot.get\_target\_joint\_positions()  
target\_tcp \= robot.get\_target\_tcp\_pose()

## **第七章：夹爪（对应图 7）**

用于控制直接安装在机械臂末端法兰上的兼容夹爪。

### **1\. 初始化夹爪 (init\_claw)**

**非常重要**：设备重新上电后，必须执行一次初始化，夹爪会自动闭合并张开以确认行程。

\# must=True 表示强制重新初始化  
robot.init\_claw(must=True)

### **2\. 夹爪控制 (set\_claw)**

控制夹爪的抓取动作。

\# force: 力度 (0-100)  
\# amplitude: 幅度/位置 (0-100，通常 0为完全闭合，100为完全张开)  
robot.set\_claw(force=100, amplitude=50) \# 以最大力度移动到 50% 开合度

### **3\. 获取夹爪状态 (get\_claw)**

返回当前夹爪的物理状态字典。

claw\_status \= robot.get\_claw()  
\# 打印出的字典包含：{'force': xx, 'amplitude': xx, 'weight': xx, 'hold\_on': True/False}  
print(claw\_status\['amplitude'\])

## **第八章：程序任务控制（对应图 8）**

用于通过代码去触发或管理在 Web 控制台（Dashboard）中已经编写好的任务流程脚本。

### **1\. 启动任务 (start\_task)**

运行指定的任务组。

\# task\_id 为 Web 控制台中对应任务的唯一标识符  
robot.start\_task(task\_id)

### **2\. 暂停/恢复/停止任务**

\# 暂停正在运行的任务  
robot.pause\_task()

\# 恢复被暂停的任务  
robot.resume\_task()

\# 强制停止当前正在运行的任务  
robot.stop\_task()  
