# Gocator_ROS1

This ROS package provides a minimal `read_depth` node for Gocator 2330D laser profiler sensor, migrated from [here](https://github.com/robotsorcerer/gocator).

## Features

- Real-time depth profile data acquisition from Gocator 2330D sensor
- Publishes point cloud data as ROS PointCloud2 messages
- Includes pre-built SDK libraries and headers
- Robot-agnostic URDF model for easy integration

## Requirements

- Ubuntu 20.04 LTS
- ROS Noetic
- PCL (Point Cloud Library)
- Ethernet connection to Gocator sensor

## 1. Installation

### 1.1 Quick Start (Recommended)
This package already includes the necessary SDK and `.so` library files. If you want to use the included libraries, skip to section 1.3.

### 1.2 Custom SDK Installation (Optional)
If you need to use a newer SDK version:

1. [Download the latest SDK](https://lmi3d.com/product-downloads/?searchdownloads=&productcategory=43&productcategoryone=48&productcategorytwo=52&resourcecategory=%20&resourceindustry=%20&resourcefiletype=%20&softwarerelease=%20&action=mydownloadfilter)
   - Example: "GoPxL SDK - Version 1.2.30.56" (Asia Pacific)
2. Extract the archive and refer to the documentation in `Go_SDK/doc` folder (GoSdk.html, kApi.html)
3. Build the SDK following the instructions in GoSdk.html:
   ```bash
   cd GO_SDK/Gocator
   make -f GoSdk-Linux_X64.mk
   ```
   - Built libraries will be generated in `GO_SDK/lib/linux_x64d`

#### Copy Headers and Libraries
- Copy GoSdk headers from `GO_SDK/GoSdk/GoSdk` to `include/GoSdk/`
- Copy kApi headers from `GO_SDK/Platform/kApi/kApi` to `include/kApi/`
- Copy library files (`libGoSdk.so`, `libkApi.so`) to `gocator_ros/lib/`

### 1.3 Firewall Configuration
1. Check firewall status:
   ```bash
   sudo ufw status
   ```
2. If subnet 192.168.1.0/24 is not allowed, enable it:
   ```bash
   sudo ufw allow from 192.168.1.0/24
   ```
3. Verify the configuration:
   ```bash
   sudo ufw status
   # Should show: Anywhere ALLOW 192.168.1.0/24
   ```

## 2. Hardware Setup

### 2.1 Sensor Connection
- Connect the Gocator sensor directly to your PC using an Ethernet cable
- If network configuration doesn't match and connection fails, use the provided script:
```bash
chmod +x set_ip.sh
bash ./set_ip.sh
```

### 2.2 Web Configuration Interface
- Open a web browser (Chrome recommended) and navigate to `192.168.1.10`

### 2.3 Manual Network Configuration (Troubleshooting)

1. Check current network interfaces:
   ```bash
   ifconfig
   # Example output:
   # enx00e04c680015: ...
   ```
2. Set your PC's IP to the same subnet as the sensor (e.g., 192.168.1.100/24):
   ```bash
   sudo ip addr add 192.168.1.100/24 dev enx00e04c680015
   ```
3. Test connectivity:
   ```bash
   ping 192.168.1.10
   ```

## 3. Usage

### 3.1 Build and Run
```bash
cd ~/YOUR_PATH/catkin_ws
catkin_make
source devel/setup.bash
roslaunch gocator_ros read_depth.launch
```

### 3.2 View Point Cloud Data
```bash
rostopic echo /gocator_profile_pcd
```

## 4. Configuration

### 4.1 Sensor Settings
- To change sensor IP or topic name, modify the parameters in `src/read_depth.cpp`
- Access the sensor's web interface at `192.168.1.10` for advanced configuration

### 4.2 Coordinate System Notes
This package has been tested on Ubuntu 20.04/ROS Noetic. Please note the following coordinate system adjustments in `read_depth.cpp`:

```cpp
double z_new = z - 130; // Gocator 2330D sets zero point 130mm below the sensor
cloud->points.emplace_back(-x / 1000.0, 0.0, z_new / 1000.0); // x axis is inverted to align with standard coordinate frame
```

## 5. Topics

- `/gocator_profile_pcd` (sensor_msgs/PointCloud2): Point cloud data from the laser profiler

## 6. Troubleshooting

- Ensure the sensor is properly connected via Ethernet
- Check firewall settings allow communication on the 192.168.1.0/24 subnet
- Verify network interface configuration using the provided `set_ip.sh` script
- All the setting/Congifuration from Chrome(192.168.1.10) web interface will override/affect on this packages. 