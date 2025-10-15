HV Bot — Intelligent Mobile Manipulator (currently on progress)

HV Bot is an advanced mobile manipulator robot combining autonomous navigation, manipulation, object detection, and AI integration.
It serves as a multi-purpose robotic system capable of performing pick-and-place, object tracking, and environment understanding using modern perception and machine learning techniques.

KEY FEATURES
Mobile Manipulator: Differential drive base with a 6-DOF arm.
Multi-Camera System: Equipped with onboard cameras for perception and tracking.
Object Detection & Pose Estimation:
Uses OpenCV for computer vision tasks.
PointNet and ML algorithms for object shape and pose understanding.
ArUco marker–based pick-and-place currently implemented.
Autonomous Navigation:
SLAM and path planning with Nav2.
Sensor fusion using EKF (robot_localization).

Swarm System Integration:
Designed to coordinate with multiple HV bots for collaborative tasks.
Fire Detection:
Integrted machine learning models to detect fire or hazardous conditions.

LLM Integration:
Uses a Large Language Model (LLM) for natural-language-based task commands, reasoning, and mission planning.

Modular Design:
Each subsystem (navigation, manipulation, vision, swarm) works independently yet communicates via ROS 2.

🧠 Tech Stack
ROS 2 (Humble / Jazzy)
Gazebo / RViz2
OpenCV for vision processing
PointNet for 3D object and pose estimation
Python + C++ ROS nodes
Nav2, slam_toolbox, and robot_localization

LLM integration (for reasoning and task control)

ArUco marker tracking for pick-and-place calibration
