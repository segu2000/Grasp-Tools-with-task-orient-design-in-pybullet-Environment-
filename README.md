
# 🤖 Grasp Tools with Task-Oriented Design in PyBullet Environment

## 📌 Overview

This project focuses on **task-oriented robotic grasping** in the **PyBullet simulation environment**. The objective is to enable a robotic hand to efficiently **grasp, hold, and place objects** in a designated tray using a combination of **forward and inverse kinematics**. The project integrates **sensor-based object detection, grasp planning, and precision control algorithms** to enhance robotic manipulation capabilities.

## 🎯 Features

- **Environment Setup**: Creates a **realistic simulation environment** for robotic grasping.
- **Object Detection**: Uses **vision-based algorithms** for recognizing objects.
- **Grasp Planning**: Implements **task-oriented grasping strategies**.
- **Robotic Control**: Uses **inverse kinematics** to precisely control robotic hand movements.
- **Data Collection & Fine-Tuning**: Allows manual adjustments to improve grasp efficiency.
- **Object Placement**: Ensures accurate object placement in a tray.

## 📸 Screenshots

![Picture21](https://github.com/user-attachments/assets/be1555df-4015-46c5-beca-f86add5eb6a7)
![Picture20](https://github.com/user-attachments/assets/6ac903f1-180a-4e95-8f34-64da8d88f603)
![Picture19](https://github.com/user-attachments/assets/ab271279-e5a2-4c53-8593-b8f6fe589fe3)
![Picture18](https://github.com/user-attachments/assets/56f14985-8c70-4e0d-9263-792870b37eeb)
![Picture17](https://github.com/user-attachments/assets/75d30dff-34a2-4ba6-820b-fe812a714ad0)
![Picture16](https://github.com/user-attachments/assets/557b3146-1b0f-4cc8-9501-4bbef07ae75e)
![Picture15](https://github.com/user-attachments/assets/d8516cfe-b340-435a-86a5-b69221f7ac82)
![Picture14](https://github.com/user-attachments/assets/8b61f4de-6ab7-4440-bfd4-c4acf844f78f)
![Picture13](https://github.com/user-attachments/assets/9abdc41b-6df1-44ae-98cd-e7665c695348)


## 🚀 Installation & Setup

### Prerequisites

Ensure you have the following installed:

- **Python** (Recommended: Python 3.8+)
- **PyBullet** (Physics simulation for robotics)
- **NumPy** (For data processing)
- **Anaconda (Optional, for virtual environment management)**
- **A Python IDE (e.g., PyCharm, VS Code)**

### Steps to Run the Project

1. **Clone the Repository**
   ```sh
   git clone https://github.com/your-username/your-repo-name.git
   cd your-repo-name
   ```

2. **Create a Virtual Environment (Optional)**
   ```sh
   conda create --name grasping_robot python=3.8
   conda activate grasping_robot
   ```

3. **Install Dependencies**
   ```sh
   pip install pybullet numpy
   ```

4. **Run the Simulation**
   ```sh
   python main.py
   ```

## 🛠️ Algorithm & Implementation

### 📌 Workflow
1. **Environment Configuration**:
   - Initializes PyBullet simulation.
   - Sets up robotic hand, objects, and tray.
   
2. **Object Detection**:
   - Uses **sensor-based algorithms** to detect object position and orientation.
   
3. **Grasp Planning**:
   - Computes the optimal grasp approach based on object characteristics.
   - Determines **grasp angle, position, and stability factors**.

4. **Execution (Grasp & Placement)**:
   - Moves the robotic hand towards the object.
   - Grasps the object using **inverse kinematics**.
   - Moves the object to a predefined tray position.
   - Releases the object and resets.

### 📌 Control Functions:
```python
for finger in fingers:
    p.setJointMotorControl2(hand, finger, p.POSITION_CONTROL, targetPosition=grasp_angle)

p.setJointMotorControl2(hand, palm, p.POSITION_CONTROL, targetPosition=palmP)
```
- The **grasp angle** and **palm position** are dynamically adjusted for precise grasping.

## 📈 Results & Observations

- Successfully **grasps and places objects** using **task-specific parameters**.
- **Fine-tuned grasping approach** improves performance and stability.
- **Realistic physics simulation** using PyBullet for validation.
- **Challenges** include grasping objects of **different sizes and shapes**, requiring further improvements.

## 🛠️ Technologies Used

- **Python** - Programming language
- **PyBullet** - Physics simulation engine
- **NumPy** - Data processing
- **URDF** - 3D modeling for robots

## 📜 References

1. **Intuitive Control of Robotic Hand with Pneumatic Feedback** - [IEEE Link](https://ieeexplore.ieee.org/)
2. **Robot Grasping and Manipulation** - [IEEE Spectrum](https://spectrum.ieee.org/robots-getting-a-grip-on-general-manipulation)

