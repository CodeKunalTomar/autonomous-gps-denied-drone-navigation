# Quick Start Guide - GPS-Denied Drone Navigation

## 🚀 System Requirements Met
✅ Ubuntu 22.04 + ROS2 Humble  
✅ OpenCV + Computer Vision packages  
✅ PX4 message support  
✅ All dependencies installed and tested  

## ⚡ Immediate System Test

### **1. Verify Installation**
```bash
cd ~/drone_nav_ws
source install/setup.bash
ros2 pkg list | grep -E "drone_|px4_msgs"
```
> **Expected:** drone_obstacle_detection, drone_path_planning, drone_visual_slam, px4_msgs

### **2. Test Obstacle Detection (Terminal 1)**
```bash
ros2 run drone_obstacle_detection obstacle_detector_debug
```
> **Expected:** "🚁 DEBUG Obstacle Detector started - Safety distance: 2.0m"

### **3. Test Path Planning (Terminal 2)**
```bash
ros2 run drone_path_planning path_planner
```
> **Expected:** "🗺️ Path Planner started - GPS-denied navigation mode"

### **4. Simulate Camera Input (Terminal 3)**
```python
python3 -c "
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
rclpy.init()
node = Node('test_camera')
pub = node.create_publisher(Image, '/camera/depth/image_raw', 10)
# Safe environment (3m depth = no obstacles)
depth_data = np.full((480, 640), 3000, dtype=np.uint16)
msg = Image()
msg.height, msg.width = 480, 640
msg.encoding = '16UC1'
msg.step = 1280
msg.data = depth_data.tobytes()
rate = node.create_rate(10)
while rclpy.ok():
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'camera_link'
    pub.publish(msg)
    rate.sleep()
"
```

### **5. Monitor System (Terminal 4)**
Check PX4 trajectory commands:
```bash
ros2 topic echo /fmu/in/trajectory_setpoint
```
> **Expected:** Position and velocity setpoints for autonomous navigation

## 🎯 Expected Results
- **Terminal 1:** No obstacles detected (3m > 2m safety distance)  
- **Terminal 2:** Waypoint navigation to (0,0,-2) -> (5,0,-2) -> (5,5,-2) -> (0,5,-2) -> (0,0,-2)  
- **Terminal 3:** Publishing safe depth data at 10Hz  
- **Terminal 4:** PX4 trajectory setpoints with position [0,0,-2] and calculated velocities  

## 🚨 Emergency Stop Test

### **Test Obstacle Response:**
In Terminal 3, replace safe data with an emergency obstacle:
```python
# Replace the line in the simulation script:
depth_data = np.full((480, 640), 500, dtype=np.uint16) # 0.5m = EMERGENCY!
```

> **Expected Results:**
> - **Terminal 1:** "🚨 OBSTACLE DETECTED! Distance: 0.500m" + "🛑 EMERGENCY STOP COMMAND SENT!"
> - **Terminal 2:** "⚠️ Obstacle detected! Pausing path planning for safety"
> - **Terminal 4:** Emergency stop commands: `linear: {x: 0.0, y: 0.0, z: 0.0}`

## 🔧 System Configuration

### **Key Parameters:**
- **Safety Distance:** 2.0m (configurable in `obstacle_detector.cpp`)
- **Emergency Distance:** 1.0m (immediate stop threshold)
- **Navigation Speed:** 1.0 m/s maximum (adaptive based on distance)
- **Processing Rate:** 10Hz (real-time performance)

### **Waypoint Pattern:**
```
(0,0,-2) ──5m──> (5,0,-2)
   ↑               ↓
   │               │ 5m
   │               ↓
(0,5,-2) <──5m── (5,5,-2)
```

## 🛟 Troubleshooting

### **No Obstacle Detection:**
- Check camera topic: `ros2 topic hz /camera/depth/image_raw`
- Verify data format: `ros2 topic echo /camera/depth/image_raw --once`

### **No Path Planning:**
- Check PX4 topics: `ros2 topic list | grep fmu`
- Verify trajectory output: `ros2 topic hz /fmu/in/trajectory_setpoint`

### **System Not Responding:**
- Restart all terminals in sequence (1→2→3→4)
- Check build status: `colcon build --packages-select drone_obstacle_detection drone_path_planning`

## 🎮 Ready for Phase 4
**Your system is now ready for:**
- Gazebo 3D simulation environments
- Visual terrain navigation testing
- Complex obstacle course scenarios
- Real-world deployment preparation

---
**🏆 Congratulations! You have a complete, working autonomous GPS-denied drone navigation system!**
