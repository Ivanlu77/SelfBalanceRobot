# LD06 LiDAR + Hector SLAM System (Raspberry Pi + Ubuntu)

This is a complete usage guide for running the LD06 LiDAR sensor on a Raspberry Pi (ROS master) and executing Hector SLAM with RViz visualization on a remote Ubuntu PC. Suitable for manual operation and documentation backup.

---

## System Overview

| Device           | Role                                                 |
| ---------------- | ---------------------------------------------------- |
| **Raspberry Pi** | Runs ROS Master and LD06 LiDAR driver node           |
| **Ubuntu PC**    | Runs Hector SLAM + RViz, connects to the Pi remotely |

---

# 1. Raspberry Pi Operations (ROS Master + LiDAR)

### 1. Start Docker container

```bash
docker start -ai ros_ld06
```

### 2. Compile and source LiDAR workspace

```bash
cd /root/sdk_ld06_raspberry_ros
catkin_make
source devel/setup.bash
```

### 3. Start ROS master

```bash
roscore
```

> Keep this terminal open. Use a new terminal for the next steps.

### 4. Launch LD06 driver node

```bash
cd /root/sdk_ld06_raspberry_ros
source devel/setup.bash
roslaunch ldlidar ld06.launch

```

> Make sure the launch file name and path are correct.

---

# 2. Ubuntu Operations (Hector SLAM + RViz)

### 1. Set up ROS network variables

```bash
export ROS_MASTER_URI=http://192.168.137.217:11311
export ROS_IP=192.168.128.129
```

> Recommend adding to `~/.bashrc`:

```bash
echo 'export ROS_MASTER_URI=http://192.168.137.217:11311' >> ~/.bashrc
echo 'export ROS_IP=192.168.128.129' >> ~/.bashrc
source ~/.bashrc
```

### 2. Compile and source Hector SLAM workspace

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 3. Launch Hector SLAM

```bash
roslaunch my_hector_slam ld06_hector.launch
```

Make sure the launch file includes:

```xml
<param name="scan_topic" value="/LiDAR/LD06" />
<param name="base_frame" value="lidar_frame" />
<param name="map_frame" value="map" />
<param name="odom_frame" value="lidar_frame" />
```

### 4. Start RViz in a new terminal

```bash
rviz
```

Set the following inside RViz:

| Option          | Value                            |
| --------------- | -------------------------------- |
| Fixed Frame     | `map`                            |
| LaserScan topic | `/LiDAR/LD06`                    |
| Map topic       | `/map`                           |
| Display modules | `Map`, `LaserScan`, `TF`, `Axes` |

---

# 3. Useful Debug Commands

| Purpose              | Command                             |
| -------------------- | ----------------------------------- |
| Check ROS topics     | `rostopic list`                     |
| View LiDAR data      | `rostopic echo /LiDAR/LD06`         |
| View TF transforms   | `rosrun tf tf_echo map lidar_frame` |
| Generate TF tree PDF | `rosrun tf view_frames`             |

---

# 4. Quick Summary Workflow

## 🧾 On Raspberry Pi

```bash
docker start -ai ros_ld06
cd /root/sdk_ld06_raspberry_ros
catkin_make
source devel/setup.bash
roscore
# In another terminal:
roslaunch ldlidar ldlidar.launch
```

## 💻 On Ubuntu

```bash
export ROS_MASTER_URI=http://192.168.137.217:11311
export ROS_IP=192.168.128.129
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch my_hector_slam ld06_hector.launch
# In another terminal:
rviz

