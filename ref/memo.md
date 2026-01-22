## Install

### install with brew

```bash
brew install pixi
```

### (FYI) update with brew

```bash
brew upgrade pixi
```

## Create Workspace

```bash
pixi init my_ros2_project -c robostack-jazzy -c conda-forge
cd my_ros2_project
pixi add ros-jazzy-ros2run
```

## Add examples

```bash
pixi add ros-jazzy-desktop ros-jazzy-turtlesim
```

## Execute

### Turtlesim example

```bash
pixi run ros2 run turtlesim turtlesim_node
```

You can ignore bellow error: 

```bash
2026-01-19 11:25:12.223 [SYSTEM Error] Problem to set affinity of thread with id [0x16c083000,dds.udp.7000] to value 0. Error 'Protocol family not supported' -> Function configure_current_thread_affinity
```

You can operate turtle with teleop_key (in new terminal)

```bash
pixi run ros2 run turtlesim turtle_teleop_key
```

### rviz2

```bash
pixi run rviz2
```

## Create custom package

### Python publisher / subscriber

```bash
pixi run ros2 pkg create --build-type ament_python --destination-directory src --node-name talker pub_sub_python
```

```bash
pixi run colcon build
```

```bash
# Publisherを起動（ターミナル1）
pixi run ros2 run pub_sub_python talker

# Subscriberを起動（ターミナル2）
pixi run ros2 run pub_sub_python listener
```


### C++ publisher / subscriber

```bash
pixi run ros2 pkg create --build-type ament_cmake --destination-directory src --node-name talker pub_sub_cpp
```

```bash
pixi run build --cmake-args -G Ninja
```

```bash
pixi run ros2 run pub_sub_cpp talker

pixi run ros2 run pub_sub_cpp listener
```