[English](README.md) | [简体中文]

# 2D|3D BEV ros流水线（Unity + ROS）

本仓库包含：
- Unity 仿真可执行程序
- Python 工具链，支持 **PINHOLE | KB | UCM | EUCM | DS | OCAM** 相机模型，以及 **2D|3D BEV（鸟瞰图）** 生成
- **ROS 集成**：用于接收/同步鱼眼相机话题利用查找表快速生成BEV视图

## 2D BEV 演示

![2d_bev](resource/2d_bev.gif)

## 3D BEV 演示

![3d_bev](resource/3d_bev.gif)

## 原始鱼眼图片

![cam1](python_scripts/test_sample/cam1.png)
![cam2](python_scripts/test_sample/cam2.png)
![cam3](python_scripts/test_sample/cam3.png)
![cam4](python_scripts/test_sample/cam4.png)

## 项目结构

- Unity 游戏目录：[unity_game/](unity_game/)
- Python 脚本目录：[python_scripts/](python_scripts/)

主要 Python 文件：
- BEV LUT 构建流程：[python_scripts/build_bev.py](python_scripts/build_bev.py)
- 全局配置：[python_scripts/config.py](python_scripts/config.py)
- 相机标定配置：[python_scripts/calibration/calibration.yaml](python_scripts/calibration/calibration.yaml)
- BEV ROS节点：[python_scripts/test_bev_ros.py](python_scripts/test_bev_ros.py)
- 多相机模型实现：[python_scripts/camera_models/](python_scripts/camera_models/)

## 环境依赖

- Python 3.8+
- Numpy
- opencv-python
- [ROS](https://www.ros.org/)
- [ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint)

## 快速开始

### 1）启动 Unity 游戏

```bash
cd unity_game
./unity_game.x86_64
```

运行后可看到场景：

![scene](resource/scene.jpg)

### 2）启动 [ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint)
注意游戏脚本默认ros ip 127.0.0.1:10000，若与ROS-TCP-Endpoint的配置不一样，需要修改ROS-TCP-Endpoint的launch文件。

```bash
cd ROS-TCP-Endpoint
source devel/setup.bash
roslaunch ros_tcp_endpoint endpoint.launch
```

### 3）构建 BEV 查找表（LUT）与输出

在仓库根目录执行：

```bash
cd python_scripts
python build_bev.py
```

### 4）运行 test_bev_ros 节点

```bash
cd python_scripts
python test_bev_ros.py --mode both
```

模式说明：
- `--mode 2d`：仅启用 2D BEV
- `--mode 3d`：仅启用 3D BEV
- `--mode both`：同时启用 2D + 3D BEV（可能较卡）

默认订阅话题：
- `/front_left/compressed`
- `/front_right/compressed`
- `/back_right/compressed`
- `/back_left/compressed`

如果你的话题名不同，请修改 [python_scripts/test_bev_ros.py](python_scripts/test_bev_ros.py)。

## 标定/数据路径

在 [python_scripts/](python_scripts/) 下会使用/生成以下目录：
- `calibration/intrinsics/params`
- `calibration/extrinsics/params`
- `data/bev_2d/luts`
- `data/bev_2d/debug`
- `data/bev_3d/luts`
- `data/bev_3d/debug`

# 致谢

[Open-3D-Surround-View](https://github.com/nick8592/Open-3D-Surround-View) 大部分代码来自该仓库，该仓库包含了2d|3d bev完整的教程。

[YOPO-Sim](https://github.com/TJU-Aerial-Robotics/YOPO-Sim) 该仓库提供了集成ros和多传感器的unity仿真环境。

# 许可 [MIT License](LICENSE)

