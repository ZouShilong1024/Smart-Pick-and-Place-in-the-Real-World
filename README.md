# Smart-Pick-and-Place-in-the-Real-World
## 整体功能
语音监听用户输入并进行解析，将解析内容用于后续操作任务，实现指定物体抓取和指定位置放置，支持任务的连续执行，整体成功率稳定在 90% 以上

## 优化内容
- 解决 RealMan RM75-B move_pose 控制方式不够鲁棒、部分抓取位姿不可达、真机执行抓取成功率低等问题，引入仿真推演方法，利用 pybullet 提供的 IK 求解器进行 IK 求解，并进行可达性检测和碰撞检测，最后将满足该条件的 pose 发给机械臂进行执行 [仿真推演-具体代码]()
- 提供了快速验证相机外参是否准确的工具，通过将 anygrasp 获取到的 pose 变换到世界坐标系与真实机械臂的 endeffector 位置对比，以实现快速验证 [抓取pose可视化-具体代码]()
- 实现了更鲁棒的 placement 方法，计算 yolo-world 获取到的 bounding box 框内的平均深度，以用于计算相机坐标点，而不是只利用中心点的信息 [2dto3d-具体代码]()
- 解决 Anygrasp 生成好的抓取 pose 不合理的问题，由于 Anygrasp 方法主要是基于夹爪末端执行器进行的设计，迁移到灵巧手上时会导致灵巧手反抓，因此添加额外的变换解决灵巧手反抓的问题 [灵巧手反抓-具体代码]()

## 快速复现
> 硬件环境：相机 realsense D455、机械臂 RealMan RM75-B、灵巧手 Inspire Hand、工控机 3090Ti、麦克风

> 软件环境：ROS-noetic、Anygrasp、open vocabulary model（YOLO-World / Grounding Dino v2）

#### 1. ROS 一键安装
```python3
wget http://fishros.com/install -O fishros && . fishros
```
#### 2. 配置 Anygrasp 环境 [Anygrasp_Env](https://pan.baidu.com/s/1_mCB-7P2NAHVcZDsBmJ4ig?pwd=k5un)
```python3
mkdir ~/anaconda3/envs/anygrasp
tar -zxvf anygrasp.tar.gz -C ~/anaconda3/envs/anygrasp
conda info -e
conda activate anygrasp
```

#### 3. 配置其他环境
```
pip install -r requirements.txt
```
#### 4. 下载涉及到的预训练权重
- Whisper checkpoint download link: [Whisper](https://pan.baidu.com/s/1zp654dQ1kLwabmHKPGfMgQ?pwd=ca4t)
- YOLO-World checkpoint download link: [YOLO-World](https://pan.baidu.com/s/1jRMHvZE9V3vRx7GtPWp_Ww?pwd=9duz)
- Anygrasp checkpoint download link: [Anygrasp](https://pan.baidu.com/s/1U5oknCINRR5j0QfnVe9n7Q?pwd=4qab)

#### 5. Start
分别开启三个终端，依次执行下面三行命令
```python3
bash start1.bash
bash start2.bash
bash start3.bash
```

## 使用自己的设备
#### 1. 机械臂
修改 `smart_pick_and_place_ws/src/rm_65_pkg/src/rm_65_pkg` 下有关机械臂设置的相关文件：
- arm_75_rm.py: 机械臂底层 api，实现了对睿尔曼(RM)系列机械臂进行连接、配置、本体状态获取以及基础控制功能，包括 move_joint (直接通过关节控制到指定位置，适合只有一条轨迹的情形)、move_joint_follow (与前面 api 功能一致，能够支持多条轨迹连续执行)、get_current_arm_state (获取机械臂本体状态信息，包括 ee position 和 joint position，用途： ① 相机标定 ② 实现 rviz 中关节与真机同步)
- arm_75_bringup.py: 在上面代码的基础上进一步封装，支持 socket 服务及 publish 机械臂本体的关节信息 (rviz 可视化)

修改 `smart_pick_and_place_ws/src/rm_65_pkg/src/rm_description` 下机械臂相关的 urdf 文件，这部分文件主要用途有两个: ① rviz 中可视化 ② pybullet 中进行逆解的求解

修改 `robot_config.json` 文件中与机械臂相关的配置信息

#### 2. 灵巧手 (修改内容同上)
修改 `smart_pick_and_place_ws/src/rm_65_pkg/src/rm_65_pkg` 下有关灵巧手设置的相关文件：
- hand_controller_modbus.py: 灵巧手控制底层 api，实现了对因时灵巧手进行连接、配置、本体状态获取及基础控制功能
- inspire_hand_bringup.py: 在上面代码基础上进一步封装，支持 socket 服务及 publish 灵巧手本体的关节信息 (rviz 可视化)

修改 `smart_pick_and_place_ws/src/rm_65_pkg/src/rm_description` 下灵巧手相关的 urdf 文件，这部分文件主要用途有一个: rviz 中可视化。注意这里不需要将灵巧手的导入 pybullet 中，在进行逆解求解时，会结合 tf 将灵巧手 endeffector 的位置信息转换到机械臂的最后一个 link


## 代码说明
- camera.py：相机类
- armcontroller.py: 机械臂类
- audio.py: 语音类
- parser.py: 用户语音输入解析类
- planner.py: 任务规划类
- transformation.py: 获取指定 link 间 transformation
- utils.py: 功能函数

## 任何问题
如果你对具身智能感兴趣或者有一些有意思的想法，欢迎随时与我进行交流，个人邮箱: zoushilong@nudt.edu.cn

## 致谢
[AEG-bot](https://davit666.github.io/AEG-bot/)、[Anygrasp](https://github.com/graspnet/anygrasp_sdk)、[YOLO-World](https://github.com/AILab-CVC/YOLO-World)