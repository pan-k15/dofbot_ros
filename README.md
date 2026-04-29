# 🤖 dofbot_ros

ROS 1 (Noetic) packages for the **Yahboom DOFBOT** — a 6-DOF AI Vision Robotic Arm.  
This repository provides robot description, hardware control, and MoveIt motion-planning support for DOFBOT running on **ROS Noetic**.

---

## 📦 Packages

| Package | Description |
|---|---|
| `dofbot_config` | URDF/SRDF robot description, joint configuration, and display launch files |
| `dofbot_control` | Hardware interface and servo control node for real robot operation |
| `dofbot_moveit` | MoveIt motion planning configuration, trajectory execution, and demo launch |
| `dofbot_tasks` | High-level task execution: pick and place, box stacking |
| `dofbot_vision` | Computer vision pipeline: QR code reading, object detection, object pose estimation |

---

## 🛠️ Prerequisites

| Requirement | Version |
|---|---|
| OS | Ubuntu 20.04 (Focal) |
| ROS | Noetic Ninjemys |
| MoveIt | `ros-noetic-moveit` |
| Build tool | `catkin` |
| Python | 3.8+ |
| C++ | 14 or later |

Install ROS Noetic dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-moveit \
  ros-noetic-ros-control \
  ros-noetic-ros-controllers \
  ros-noetic-joint-state-publisher \
  ros-noetic-joint-state-publisher-gui \
  ros-noetic-robot-state-publisher \
  ros-noetic-xacro \
  python3-catkin-tools
```

---

## 🚀 Installation

```bash
# 1. Create a catkin workspace
mkdir -p ~/dofbot_ws/src
cd ~/dofbot_ws/src

# 2. Clone this repository
git clone https://github.com/pan-k15/dofbot_ros.git

# 3. Install ROS dependencies
cd ~/dofbot_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. Build with catkin
catkin_make

# 5. Source the workspace
source devel/setup.bash
```

Add to your shell profile to avoid repeating step 5:

```bash
echo "source ~/dofbot_ws/devel/setup.bash" >> ~/.bashrc
```

---

## ▶️ Usage

### Visualize the Robot (RViz)

Launch the robot model in RViz with an interactive joint slider:

```bash
roslaunch dofbot_config display.launch
```

### MoveIt Demo (Simulated)

Open RViz with the MoveIt Motion Planning plugin. Drag the interactive marker to set a goal pose, then click **Plan & Execute**:

```bash
roslaunch dofbot_moveit demo.launch
```

### Run on Real Hardware

Start the hardware control node to communicate with the physical servos:

```bash
roslaunch dofbot_control dofbot_control.launch
```

Then launch MoveIt to control the real arm:

```bash
roslaunch dofbot_moveit moveit_planning_execution.launch
```

### Vision Pipeline

Start the camera and object detection node:

```bash
roslaunch dofbot_vision dofbot_vision.launch
```

For QR code reading only:

```bash
rosrun dofbot_vision qr_reader.py
```

### Task Execution

Run pick and place:

```bash
roslaunch dofbot_tasks pick_and_place.launch
```

Run box stacking:

```bash
roslaunch dofbot_tasks box_stacking.launch
```

---

## 📂 Repository Structure

```
dofbot_ros/
├── dofbot_config/          # Robot description package (catkin)
│   ├── urdf/               # URDF/XACRO robot model
│   ├── config/             # Joint limits, controller parameters
│   ├── launch/             # Display and bringup launch files
│   └── CMakeLists.txt
├── dofbot_control/         # Hardware control package (catkin)
│   ├── src/                # C++ servo driver / control node
│   ├── scripts/            # Python helper scripts
│   ├── launch/             # Hardware bringup launch files
│   └── CMakeLists.txt
├── dofbot_moveit/          # MoveIt configuration package (catkin)
│   ├── config/             # SRDF, kinematics.yaml, planning pipeline
│   ├── launch/             # MoveIt demo and execution launch files
│   └── CMakeLists.txt
├── dofbot_tasks/           # High-level task package (catkin)
│   ├── src/                # Pick and place, box stacking nodes
│   ├── scripts/            # Python task scripts
│   ├── launch/             # Task launch files
│   └── CMakeLists.txt
├── dofbot_vision/          # Vision package (catkin)
│   ├── src/                # Object detection, pose estimation nodes
│   ├── scripts/            # QR code reader, vision pipeline scripts
│   ├── launch/             # Vision launch files
│   └── CMakeLists.txt
├── CMakeLists.txt          # Catkin toplevel (symlink to noetic)
└── README.md
```

---

## 🦾 About DOFBOT

DOFBOT is a 6-DOF serial bus servo robotic arm developed by **Yahboom**, designed for AI and robotics education. Supported capabilities include:

- Forward / inverse kinematics
- MoveIt motion planning and simulation
- Cartesian path planning
- Collision detection
- Real robot servo control
- AI vision tasks (color recognition, gesture control, object sorting, face tracking)

---

## 📄 License

This project is open source. See the [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgements

- [Yahboom Technology](https://www.yahboom.net) — DOFBOT hardware and reference software
- [MoveIt](https://moveit.ros.org) — Motion planning framework for ROS
- [ROS Noetic](https://wiki.ros.org/noetic) — Robot Operating System