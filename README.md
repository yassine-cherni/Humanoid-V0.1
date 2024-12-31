# Humanoid Robot Self-Balancing with ROS2 Jazzy, Isaac Sim, Path Planning, and AI  

Welcome to the **Self-Balancing Humanoid Robot Project**, a comprehensive and cutting-edge solution for humanoid robot control and navigation. This repository brings together **ROS2 Jazzy**, **NVIDIA Isaac Sim**, **Path Planning**, and **AI-powered Retrieval-Augmented Generation (RAG)** models to develop a highly intelligent and adaptive robotic system. It’s designed to bridge the gap between simulation and real-world deployment, empowering humanoid robots to navigate and balance in dynamic environments seamlessly.  

---

## Key Features  

### 🚀 **Real-Time Control with ROS2 Jazzy**  
Efficient actuator and sensor management using the latest ROS2 Jazzy framework ensures precise control and adaptability.  

### 🌌 **Photorealistic Simulation with Isaac Sim**  
Experience high-fidelity simulations using NVIDIA Isaac Sim, featuring realistic physics and environmental interactions for training and testing.  

### 🤖 **Advanced Deep Reinforcement Learning (DRL)**  
Train your robot to balance and recover from disturbances using state-of-the-art DRL algorithms, including Proximal Policy Optimization (PPO) and Soft Actor-Critic (SAC).  

### 🧭 **Integrated Path Planning**  
Navigate seamlessly using robust global and local path planning algorithms like A* and Dynamic Window Approach (DWA).  

### 🧠 **AI with RAG Models**  
Enhance decision-making capabilities by integrating Retrieval-Augmented Generation (RAG) models, combining the power of LLMs with real-time contextual data retrieval.  

### 🌍 **Sim-to-Real Transfer**  
Achieve robust real-world performance with domain randomization techniques and optimized DRL policies.  

### 🔗 **Multi-Sensor Fusion**  
Leverage IMU, force/torque sensors, encoders, and RGB-D cameras for highly accurate state estimation and environmental awareness.  

---

## Repository Structure  

```plaintext
humanoid_self_balancing/
├── config/                  # Configuration files for ROS2 controllers, sensors, and planners
├── description/             # URDF/Xacro files for the humanoid robot
├── isaac_sim/               # Isaac Sim scripts for simulation
├── launch/                  # Launch files for simulation and deployment
├── path_planning/           # Global and local path planning algorithms
├── rag/                     # AI-driven RAG model scripts
├── scripts/                 # Training and policy deployment scripts
├── src/                     # Custom ROS2 controllers in C++
├── models/                  # 3D robot models (DAE, STL)
├── worlds/                  # Simulation worlds for Isaac Sim and Gazebo
└── logs/                    # Training and runtime logs

