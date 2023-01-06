---
title: How to control Franka Panda by ROS
author: LinTao
date: 2023-01-06 14:10:00 +0800
categories: [Tutorial]
tags: [ROS, Franka]
render_with_liquid: false
---
<a name="UNZP5"></a>
## 导言
franka_ros是Ros controller框架下一套通讯控制接口，用户需要编写自己的controller类，继承franka_ros提供的接口(franka_hw::  )来实现传感器读取和控制。

**Ros controller的相关资料**<br />[https://zhuanlan.zhihu.com/p/433214417](https://zhuanlan.zhihu.com/p/433214417)<br />[https://blog.csdn.net/gongdiwudu/article/details/124153274](https://blog.csdn.net/gongdiwudu/article/details/124153274)

**关于franka_ros的相关资料**<br />官方FCL文档(可以看有哪些接口，但没有技术细节)<br />[http://www.franka.cn/FCI/franka_ros.html#franka-hw](http://www.franka.cn/FCI/franka_ros.html#franka-hw)<br />franka_hw技术文档<br />[http://docs.ros.org/en/kinetic/api/franka_hw/html/annotated.html](http://docs.ros.org/en/kinetic/api/franka_hw/html/annotated.html)<br />**关于libfranka的相关资料**<br />franka_ros是在libfranka的基础上做了一层封装，实际可以通过c++直接对机械臂进行控制，建议大概看一下libfranka，了解机械臂的控制架构。<br />[http://www.franka.cn/FCI/libfranka.html](http://www.franka.cn/FCI/libfranka.html)<br />下面以显扬科技的**末端笛卡尔坐标位置控制**的示例代码(cartesian_pose_example_controller.cpp),借助显扬科技原有的包文件，讲解如何实现自己的控制器。
<a name="C9DzM"></a>
## 一、编写my_controller.cpp
源码为了管理，在.h文件中声明变量，在.cpp文件中写核心代码，为了便于讲解这里合并到一起，实际也可以只写一个.cpp文件，在/src文件夹下创建my_controller.cpp。
<a name="rDDfi"></a>
### 1.1 头文件部分
这一部分建议参考示例代码引用，关于向量、矩阵等库不做讲解。<br />重点关注franka_hw，首先确定自己需要获得什么传感器数据，做哪个层级的控制，然后在技术文档里查看相关接口被包含于哪个头文件。<br />机械臂有四种控制模式，分别是**关节位置控制|关节速度控制|末端位置控制|末端速度控制**，同一时间只选择一种模式，并且可以**直接对关节力矩进行控制**(力控，可选项，不编写就是官方自带的控制方式)。<br />如果我们想对关节角度进行控制，可以引用ros control官方的头文件hardware_interface/joint_command_interface.h，因为电机接口(转速、转角)是通用的，而机械臂的功能接口不统一，是由各个厂家自己编写的。<br />本示例代码对机械臂末端位置进行控制，并且希望得到机械臂初始角度数据，查看文档之后，确定我们的类需要继承两个接口(interface)[FrankaPoseCartesianInterface](http://docs.ros.org/en/kinetic/api/franka_hw/html/classfranka__hw_1_1FrankaPoseCartesianInterface.html)和[FrankaStateInterface](http://docs.ros.org/en/kinetic/api/franka_hw/html/classfranka__hw_1_1FrankaStateInterface.html)，分别包含于[franka_cartesian_command_interface.h](http://docs.ros.org/en/kinetic/api/franka_hw/html/franka__cartesian__command__interface_8h_source.html)和[franka_state_interface.h](http://docs.ros.org/en/kinetic/api/franka_hw/html/franka__state__interface_8h_source.html)之中。

```cpp
#include <cmath>
#include <array>
#include <memory>
#include <mutex>
#include <vector>
#include <stdexcept>
#include <string>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/node_handle.h>
#include <geometry_msgs/Vector3.h>

#include <controller_interface/controller_base.h>
#include <controller_interface/multi_interface_controller.h>

#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_cartesian_command_interface.h>

#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.h>
```
<a name="oRNW5"></a>
### 1.2 整体架构
我们需要在一个命名空间之下，声明MyController，继承我们需要的传感器和控制接口，在私有属性中声明传感器和控制的handler(具体见init部分和update部分源码)。<br />然后实现controller_interface的init、starting、update三个方法。<br />init方法在MyController被加载的时候会调用，在这之中我们需要初始化一些变量。<br />starting是MyController在启动/重启时会被调用一次，做一些类似初始化的事情。<br />update将会在每个控制周期内执行一次，也就是程序的主循环，核心代码放于此处。<br />ros control整体框架围绕这三个方法展开，我们只需要编写算法即可，具体怎么执行是ROS负责的。<br />最后在命名空间外，将MyController导出。
```cpp
/*头文件部分*/
namespace franka_hit_controllers_ns {//命名空间可以随便起名，建议以controllers_ns结尾

class MyController : public controller_interface::MultiInterfaceController<franka_hw::FrankaPoseCartesianInterface,franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
  ros::Duration elapsed_time_;
  std::array<double, 16> initial_pose_{};//satrt pose
  std::array<double, 16> new_pose;//the last pose
  ros::Subscriber sub_equilibrium_pose_;//subscriber for topic moveXYZ
  void moveXYZCallback(const geometry_msgs::Vector3::ConstPtr& msg);
  std::mutex pose_mutex;
  bool reached=true;//reach the target position
  std::vector<double> vectorXYZ;//the distance for XYZ
  double run_time = 0.0;                                                            
};//MyController类声明结束

//编写三个方法
bool MyController::init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle){}
void MyController::starting(const ros::Time& /* time */) {}
void MyController::update(const ros::Time& /* time */,const ros::Duration& period){}

}//命名空间结束

PLUGINLIB_EXPORT_CLASS(franka_hit_controllers_ns::MyController,
                       controller_interface::ControllerBase)//注册类，不需要加分号
```
<a name="DC7p8"></a>
### 1.3 init部分编写
init方法有两个参数，第一个robot_hardware用于获取相关的接口，第二个node_handle用于发布订阅话题、获得我们再外部yaml文件中定义的参数。<br />首先需要通过robot_hardware的get方法拿到相关接口interface，然后通过interface的gethandle方法拿到控制和读数据所需要的句柄(handle)。只有通过handle的方法我们才能读取数据或者是控制，所以我们需要声明相关的handler变量，在此处赋值，在update()中使用。
```cpp
my_controller:
    type: franka_hit_controllers_ns/MyController
    arm_id: panda
```
```cpp
bool MyController::init(hardware_interface::RobotHW* robot_hardware,ros::NodeHandle& node_handle) {
  vectorXYZ.resize(3,0.0);
  //拿到FrankaPoseCartesianInterface接口
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianPoseExampleController: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }
  
  //订阅moveXYZ话题，moveXYZCallback是获得话题数据后执行的回调函数
  sub_equilibrium_pose_ = node_handle.subscribe("/moveXYZ", 20, &CartesianPoseExampleController::moveXYZCallback, this);
  //getParam拿到了yaml文件中的arm_id，也就是panda(单臂默认为panda，多臂才会给不同id)
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianPoseExampleController: Could not get parameter arm_id");
    return false;
  }
  //通过FrankaPoseCartesianInterface接口,使用getHandle()
  //得到了标识为“arm_id_robot”的机械臂的FrankaCartesianPoseHandle
  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseExampleController: Exception getting Cartesian handle: " << e.what());
    return false;
  }
  //得到FrankaStateInterface
  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianPoseExampleController: Could not get state interface from hardware");
    return false;
  }
  //得到了标识为“arm_id_robot”的机械臂的FrankaStateHandle
  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");
    //panda默认的起始姿态
    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
    //通过handle的getRobotState()方法读取状态，建议看技术文档得知更多细节！！！
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_WARN("please make sure the robot does not crash into something ");
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseExampleController: Exception getting state handle: " << e.what());
    return false;
  }

  return true;
}
```
<a name="OzD9g"></a>
### 1.4 starting部分编写
在启动时被自动调用一起，之后可能会复位重启，也会被调用，参考示例代码可以将一些初始化赋值放在此处。
```cpp
void MyController::starting(const ros::Time& /* time */) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_c;
  new_pose = initial_pose_;
  elapsed_time_ = ros::Duration(0.0);  
}
```
<a name="FYqDn"></a>
### 1.5 update部分编写
对算法不做讲解，主要关注如何发布控制指令。<br />在前文我们获得了对末端位置进行控制的handle，我们可以通过setCommand发布指令。
```cpp
void MyController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
    pose_mutex.lock();
    if (reached == true)
    {
      cartesian_pose_handle_->setCommand(new_pose);
      //ROS_INFO("please dont wait robot reach the target position");
       pose_mutex.unlock();
        return;
    }
 

  elapsed_time_ += period;

  //double radius = 0.2;
  //reached = false;
  if(elapsed_time_.toSec()>run_time)
  {
    reached = true;
    //发布控制指令!!!
    cartesian_pose_handle_->setCommand(new_pose);
    pose_mutex.unlock();
    return ;
  }
  double angle = M_PI / 4 * (1 - std::cos(M_PI / run_time * elapsed_time_.toSec()));
  
  double delta_x = vectorXYZ[0] * std::sin(angle);
  double delta_y = vectorXYZ[1] * std::sin(angle);
  double delta_z = vectorXYZ[2] * std::sin(angle);
  new_pose = initial_pose_;
  new_pose[12] += delta_x;
  new_pose[13] += delta_y;
  new_pose[14] += delta_z;
  //std::cout<<"the new pose[14] is "<<new_pose[14]<<std::endl;
  cartesian_pose_handle_->setCommand(new_pose);

  pose_mutex.unlock();
}
```
<a name="sIeVx"></a>
## 二、编译
<a name="pv5AZ"></a>
### 2.1 生成插件描述文件
MyController相当于一个插件，需要使用在包文件下新建一个.xml文件进行描述，此处只需要在原有文件内新增文本。
```cpp
<library path="lib/libfranka_hinyeung_controllers">
.....
  <class name="franka_hinyeung_controllers/MyController" type="franka_hit_controllers_ns::MyController" base_class_type="controller_interface::ControllerBase">
      <description>
          my first controller!
      </description>
  </class>
.....
</library>
```
<a name="kAAcc"></a>
### 2.2 修改package.xml文件
将插件描述文件加入工程，这一部分不用做，但需要了解。
```cpp
  <export>
    <controller_interface plugin="${prefix}/franka_hinyeung_controllers_plugin.xml"/>
  </export>
```
<a name="foedm"></a>
### 2.3 修改CMakeList.txt文件
```cpp
add_library(franka_hinyeung_controllers
    src/my_controller.cpp
)
```
<a name="jjaIe"></a>
### 2.4 修改yaml文件
在config/franka_hinyeung_controllers.yaml文件中添加如下文本
```cpp
my_controller:
    type: franka_hinyeung_controllers/MyController
    arm_id: panda
```
<a name="uUMFO"></a>
### 2.5 新建launch文件
建议熟读launch文件，学习需要启动那些文件。<br />在launch文件夹下复制一份cartesian_pose_example_controller.launch文件，改名为my_controller.launch文件然后将其中的args="cartesian_pose_example_controller"换成args="my_controller"。
<a name="HKFoB"></a>
### 2.6 编译
在workspace下开启终端输入catkin_make指令进行编译。<br />然后就可以通过roslaunch调用my_controller.launch文件启动程序，控制机械臂末端位置了。
