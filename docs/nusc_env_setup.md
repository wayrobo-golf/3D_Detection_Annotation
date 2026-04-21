# `nusc_env` 新机安装说明

本文档用于把当前仓库迁移到新机器，并尽快恢复一个可运行的 `nusc_env`。

## 推荐基线

- Ubuntu 22.04
- ROS 2 Humble
- Conda / Mambaforge
- Python 3.10

原因：项目脚本里直接写死了 `source /opt/ros/humble/setup.bash`，并且 `cv_bridge_with_opencv411` 会跟随当前 Python 小版本编译，因此建议和 ROS 2 Humble 默认的 Python 3.10 保持一致。

## 1. 安装系统依赖

先安装 ROS 2 Humble 与编译依赖：

```bash
sudo apt update
sudo apt install -y \
  curl \
  git \
  build-essential \
  cmake \
  pkg-config \
  libeigen3-dev \
  libpcl-dev \
  libyaml-cpp-dev \
  libcurl4-openssl-dev \
  libjsoncpp-dev \
  libtinyxml2-dev \
  libopencv-dev \
  python3-opencv \
  libboost-dev \
  libboost-python-dev \
  python3-colcon-common-extensions \
  ros-humble-desktop \
  ros-humble-pcl-conversions \
  ros-humble-tf2-eigen \
  ros-humble-tf2-geometry-msgs \
  ros-humble-rclcpp-lifecycle
```

如果新机还没装 ROS 2 Humble，请先按 ROS 官方方式完成 `/opt/ros/humble` 安装，再执行上面的依赖安装。

## 2. 创建 Conda 环境

在仓库根目录执行：

```bash
conda env create -f environment/nusc_env.yml
conda activate nusc_env
```

如果环境已存在，更新即可：

```bash
conda env update -n nusc_env -f environment/nusc_env.yml --prune
```

## 3. 编译工作区

每次新开 shell，建议按这个顺序：

```bash
source /opt/ros/humble/setup.bash
conda activate nusc_env
colcon build --symlink-install
source install/setup.bash
```

如果你希望 `colcon` 明确使用当前 conda 里的 Python 3.10，也可以这样编译：

```bash
source /opt/ros/humble/setup.bash
conda activate nusc_env
colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python
source install/setup.bash
```

## 4. 最小验证

先验证 Python 环境：

```bash
conda run -n nusc_env python -c "import numpy, scipy, yaml, cv2, open3d; print('python env ok')"
```

再验证仓库测试：

```bash
source /opt/ros/humble/setup.bash
conda run -n nusc_env pytest pre_annotation_factory/tests -q
```

最后验证主流程命令能拉起：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
conda run -n nusc_env python pre_annotation_factory/scripts/annotationctl.py --help
```

## 5. 运行前额外准备

仓库根目录需要 `.env`，至少包含：

```dotenv
XTREME1_TOKEN=<your_token>
```

另外还要确认以下外部资源在新机路径上可用：

- `automatic_annotation/config/default.yaml` 中引用的地图、XML、外参
- rosbag 数据目录
- 如果使用 QoS 配置，`pre_annotation_factory/scripts/replay_rosbag_main.py` 中引用的 QoS yaml

## 6. 最快迁移流程

如果旧机器环境基本正常，推荐按下面顺序迁移：

1. 在新机安装 Ubuntu 22.04、ROS 2 Humble、Conda。
2. 拉取本仓库。
3. 执行 `conda env create -f environment/nusc_env.yml`。
4. 执行 `source /opt/ros/humble/setup.bash && conda activate nusc_env && colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python`。
5. 补齐 `.env`、地图、XML、bag、外参等外部数据。
6. 运行 `conda run -n nusc_env pytest pre_annotation_factory/tests -q` 和 `annotationctl.py --help` 做冒烟验证。

## 7. 常见问题

### `cv_bridge_with_opencv411` / Boost.Python 编译失败

优先检查：

- 当前 `conda` 环境是不是 Python 3.10
- 是否安装了 `libboost-python-dev`
- 编译时是否传了 `-DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python`

### `ros2 run automatic_annotation automatic_annotation_node` 找不到

通常是以下原因：

- 没有执行 `colcon build --symlink-install`
- 没有 `source install/setup.bash`
- 没有先 `source /opt/ros/humble/setup.bash`

### Python 能跑，但预标注流程失败

这类问题多数不是 `nusc_env` 本身，而是外部运行资源缺失：

- 地图 `.pcd`
- `tracklet_labels.xml`
- bag 数据
- Xtreme1 Token
- 配置里的绝对路径仍指向旧机器
