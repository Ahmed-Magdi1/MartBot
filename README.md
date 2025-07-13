# MartBot - Supermarket Assistant Robot

## Overview

MartBot is an autonomous mobile robot designed specifically to assist elderly and disabled customers in supermarket environments. This graduation project combines advanced robotics, artificial intelligence, and user-centered design to create an accessible shopping companion. The robot provides intelligent navigation guidance through store aisles, delivers comprehensive product information via an intuitive touchscreen interface, and features an AI-powered self-checkout system using computer vision technology. Built on the ROS framework with autonomous navigation and multi-sensor integration, MartBot aims to enhance retail accessibility and promote independent shopping experiences for users with mobility challenges and special needs.

## Package Descriptions

<div align="center">

| Package | Purpose | Key Components |
|:-------:|:-------:|:--------------:|
| **`martbot_bringup`** | System initialization | Launch files, sensor startup |
| **`martbot_description`** | Robot modeling | URDF files, 3D meshes, joint configs |
| **`martbot_nav`** | Autonomous navigation | Path planning, obstacle avoidance |
| **`martbot_slam`** | Mapping & localization | Gmapping, AMCL, EKF, map files |
| **`martbot_gui_final`** | User interface | Touch-screen GUI, product database |
| **`hoverboard-driver`** | Motor control | Differential drive, speed control |
| **`realsense-ros`** | Depth perception | RGB-D camera, point clouds |
| **`rplidar_ros`** | Environmental scanning | 360° laser, obstacle detection |
| **`yolov5_ros`** | Object recognition | AI-powered product detection |

</div>

## System Architecture

<div align="center">
  <img src="images/system_architecture.png" alt="MartBot System Architecture" width="800"/>
  <p><b>Complete System Architecture and Hardware Integration</b></p>
</div>

The system architecture demonstrates MartBot's comprehensive hardware integration across four main layers:

### **User Interface Layer**
- **7-inch Touchscreen**: Primary user interaction interface with HDMI and USB connectivity
- **Status LED Strip**: Visual feedback system for operational status and user guidance

### **Processing Layer**
- **On-board Computer**: Central processing unit managing all robot operations
- **Multiple USB Ports**: Facilitating communication with sensors and peripherals

### **Sensor Layer**
- **RPLiDAR A1**: 360° laser scanning for SLAM and obstacle detection
- **Intel RealSense D455**: RGB-D camera for depth perception and object recognition
- **Adafruit BNO055 Absolute Orientation Sensor**: 9-axis inertial navigation system for orientation tracking
- **Monocular Camera**: Additional visual input for product identification

### **Motor Control & Power Layer**
- **Differential Drive System**: Two BLDC motors with hoverboard controller integration
- **FTDI Communication**: Serial interface for motor control commands
- **Emergency Stop Switch**: Safety mechanism for immediate system shutdown
- **Dual Power System**: 
  - 36V battery for motor operations
  - 12V battery for the on-board computer 
- **Power Management**: Integrated inverter and adapter system with proper grounding

---

## Design
MartBot features a differential drive configuration optimized for supermarket environments:

- **Functional Form Factor:** Designed to navigate standard supermarket aisles (typically 100–150 cm wide)
- **Integrated Shopping Basket:** Built-in compartment for grocery transportation
- **Touchscreen Interface:** Mounted on the basket for intuitive user interaction, integrated with an onboard camera for product scanning
- **Sensor Mounts:** Dedicated slots for depth camera and LiDAR placement, with an optimized height and field of view

<div align="center">

<table>
  <tr>
    <td>
      <img src="images/Robot Base.png" alt="MartBot Front View" width="300"/>
    </td>
    <td>
      <img src="images/Actuator Mounting.png" alt="MartBot Side View" width="300"/>
    </td>
  </tr>
</table>

<p><b>MartBot Chassis</b></p>

</div>

<div align="center">

<table>
  <tr>
    <td align="center">
      <img src="images/CAD Model.PNG" alt="MartBot CAD Model" width="240"/>
    </td>
    <td align="center">
      <img src="images/Martbot.PNG" alt="MartBot Prototype" width="250"/>
    </td>
  </tr>
</table>

<p><b>MartBot CAD Model and Prototype</b></p>

</div>
