# Drone Node Launch Instructions (Jetson Orin Nano - Ubuntu 22.04, JetPack 6.2)

This guide provides step-by-step instructions to launch the drone's camera, LiDAR, micro-ROS agent, and main subscriber node on a Jetson Orin Nano running Ubuntu 22.04 with JetPack 6.2.

---

## Prerequisites

- Jetson Orin Nano with Ubuntu 22.04 + JetPack 6.2
- `solardrone_ws` ROS 2 workspace properly built
- Intel RealSense camera connected
- Teraranger Tower Evo X4 LiDAR connected
- Micro-ROS agent installed

---

## 1. Start Depth Camera Node

```bash
cd ~/solardrone_ws
source install/setup.bash

# Launch the RealSense camera node
ros2 launch realsense realsense.launch.py
```

### ✅ Tip: Verify Camera Feed
In another terminal, check if the camera is publishing:

```bash
ros2 topic echo /camera/realsense_node/color/image_raw
```

> 🔄 If no continuous messages appear, try rerunning `realsense.launch.py`.

---

## 2. Start LiDAR Node (Teraranger Tower Evo X4)

```bash
cd ~/solardrone_ws/src/teraranger_evo/teraranger_evo/
python3 teraranger_evo_tower_x4.py
```

---

## 3. Start Micro-ROS Agent

```bash
MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600
```

---

## 4. Start Main Drone Program (Subscriber Node)

```bash
cd ~/solardrone_ws
source install/setup.bash

# Run the main subscriber node
ros2 run realsense realsense_subscriber
```

---

## Notes

- Make sure each step is run in a **separate terminal**.
- Ensure all hardware devices (RealSense camera, Teraranger LiDAR, micro-ROS node) are properly connected before starting.
- If you face issues, re-check USB permissions and device connectivity.

---
