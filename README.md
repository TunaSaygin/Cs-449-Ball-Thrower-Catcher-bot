# Dynamic Object Throwing with Precision Constraints and RRT Path Planning


https://github.com/user-attachments/assets/abb6d717-bedc-4d91-a6ae-471612713866


## 🚀 **Overview**
This project showcases the development of a **ball-throwing and catching robotic system**. The system is powered by the **Robotic library (`ry`)** (version 0.1.10) for simulation and control, and features:
- A **thrower robot** that calculates optimal trajectories and velocities to throw a ball.
- A **catcher robot** that dynamically adjusts its position to catch the thrown ball.

This project demonstrates cutting-edge robotics concepts, including:
- 🎯 **Trajectory Optimization**
- 🤖 **Kinematics and Dynamics Simulation**
- 🧑‍🤝‍🧑 **Multi-Robot Coordination**
- 🛠️ **Object Tracking and Manipulation**



## ✨ **Features**
![ProjDevFlow](https://github.com/user-attachments/assets/79e0f992-e24a-475f-b823-f2c77709e4ca)

### **Thrower Robot**
- 📐 **Optimal Velocity Calculation**: Computes the throwing velocity using kinematic and dynamic constraints.
- 🌌 **Trajectory Simulation**: Models ball trajectories accounting for gravity and release parameters.
- 🔄 **Inversion Handling**: Manages trajectory flipping for complex bin placements.

### **Catcher Robot**
- 👁️ **Vision-Based System**: Captures ball trajectories with precision.
- 🏃 **Dynamic Positioning**: Adjusts in real-time to intercept the ball.

### **Path Finding**
- 📏 Checks if the thrower is too far/close to the bin or catcher.
- 🌳 Uses **RRT (Rapidly-Exploring Random Trees)** to reposition the thrower robot optimally.

### **Simulation Environment**
- 🛠️ Built using the **[robotic](https://github.com/MarcToussaint/robotic/) library (`ry`)** by [Marc Toussaint](https://github.com/MarcToussaint). Thanks Marc!
- 🎥 **Visualization Support**: Displays trajectories and robot movements in real-time.
- 🛑 Debugging markers to track robot states and ball positions.



## 🗂️ **Project Structure**

### **Core Components**

#### `thrower_generic.py`
Core functions for the thrower robot:
- **`find_velocity`**: Calculates the velocity required to reach the bin.
- **`pick_last_object_if_valid`**: Determines the initial ball position for a valid throw.
- **`grasp_object`**: Simulates the robot grasping the ball.
- **`throw_object`**: Executes the throwing action and updates the trajectory.
- **`check_in_the_bin`**: Verifies if the ball lands in the target bin.

#### `catcher_robot.py`
Defines the catcher robot’s behavior:
- **Vision-Based Point Cloud Capture**: Tracks the ball’s trajectory.
- **Position Adjustment**: Dynamically moves to intercept the ball.

#### `thrower_catcher_integration.py`
Coordinates the interaction between the thrower and catcher robots:
- **Threaded Integration**: Synchronizes throwing and catching actions.
- **Real-Time Visualization**: Provides an interactive simulation environment.

#### **Utility Scripts**
- **`my_utils.py`**: Contains helper functions for quaternion and rotation calculations.
- **`velocity_finder.py`**: Implements trajectory optimization and velocity calculations.

### **Data Files**
- **`catching.g`**: Simulation configuration for the catcher robot.
- **`throwing_bare.g`**: Simulation configuration for the thrower robot.



## 📖 **Documentation**
- To access the detailed documentation, switch to the **`doc`** branch and open `index.html` in your browser.



## 💡 **Key Highlights**
- A perfect demonstration of **real-time robotics simulation** and **multi-robot coordination**.
- Integrates advanced algorithms like **RRT** and **trajectory optimization** for dynamic adaptability.
- **Modular and extensible** design for further enhancements and experimentation.



## 🏆 **Future Work**
- Enhance the vision system to include **machine learning-based tracking** for better ball trajectory estimation.
- Optimize RRT algorithms for faster and more efficient pathfinding.
- Expand the simulation to include **collaborative robots** for multi-object manipulation.


## ✅ **Installation** 
- Clone the repository
```
git clone https://github.com/TunaSaygin/Cs-449-Ball-Thrower-Catcher-bot.git
```
- Navigate to the project directory and install dependencies
```
cd Cs-449-Ball-Thrower-Catcher-bot
pip install -r requirements.txt
```

- Run Simulation
```
python3 thrower_catcher_projectile.py
```

## 👥 Team members:

<table align="center">
  <tbody>
    <tr>
      <td align="center" valign="top" width="20%"><a href="https://github.com/TunaSaygin"><img src="https://avatars.githubusercontent.com/u/91385993?v=4" width="100px;" alt="Tuna Saygın"/><br /><b>Tuna Saygın</b></a><br/>22102566</td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/betuldogrul"><img src="https://avatars.githubusercontent.com/u/91432039?v=4" width="100px;" alt="Betül Doğrul"/><br /><b>Betül Doğrul</b></a><br/>22003559</td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/SCORPIA2004"><img src="https://avatars.githubusercontent.com/u/62741526?v=4" width="100px;" alt="Muhammad Shayan Usman"/><br/><b>Muhammad Shayan Usman</b></a><br/>22101343</td>

  </tbody>
</table>
