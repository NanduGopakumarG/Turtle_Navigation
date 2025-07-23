# Turtle_Navigation

# 🐢 Turtle Navigation using ROS 2 – PID Controlled Motion

This repository showcases **turtle navigation tasks** using the `turtlesim` package in **ROS 2**. The focus is on implementing controlled navigation using linear and angular velocities, with an emphasis on **PID control** for optimized motion.

---

# GUI Control using rqt_robot_steering

Launch the graphical slider-based interface:

      rqt_robot_steering
      
Select topic: /turtle1/cmd_vel

Use sliders to adjust:

- Linear velocity

- Angular velocity

This allows fine-tuned real-time control of turtle movement via a user-friendly GUI.

# File Structure

      turtle_navigation/
      
        ├── goal_1_spawn_turtle.py
        ├── goal_1_pid_controller.py 
        ├── goal_1_pid_cont_plot.py
        ├── README.md
        └── figures/
        └── pid_plot_example.png        # Example performance graph
    
# Dependencies

Make sure the following packages are installed:

- rclpy

- turtlesim

- rqt_robot_steering

- matplotlib (for plotting)

- numpy

Install missing Python dependencies via:

      pip install matplotlib numpy

---

## Goal 1: Control Turtle

---

### Objective

- Spawn a turtle at a **random location** in a 2D environment
- Navigate it to a **specified goal position**
- Implement a **PID controller** to:
  - Minimize overshoot
  - Improve response time
- **Analyze performance** variations by tuning PID parameters and plotting key data

---


### Launch Turtlesim Node

Launch the simulation environment:

      ros2 run turtlesim turtlesim_node
      
### Spawn a Turtle at a Random Location

Run the spawning script:

      python3 goal_1_spawn_turtle.py
      
This will create a new turtle at a random coordinate within the Turtlesim window.

### Manual Control of the Turtle

Use rqt_robot_steering to control the turtle interactively:

      rqt_robot_steering
      
Adjust the linear and angular velocity sliders to move the turtle manually.

### Implement PID Controller for Autonomous Movement

Activate PID control with:

      python3 goal_1_pid_controller.py
      
Modify Kp, Ki, and Kd values in the script to observe changes in movement performance.

Set a custom goal position in the script.

### Analyze and Plot PID Performance Variations

Visualize the effect of different PID settings:

      python3 goal_1_pid_cont_plot.py

This script plots key data such as position error over time.

Analyze the impact of PID tuning on system behavior.

---

## Goal 2: Making a Grid with Acceleration and Deceleration Constraints

---

### Objective

The aim of this goal is to simulate **realistic turtle movement** in the `turtlesim` environment by incorporating:

- Acceleration and deceleration constraints
- Grid-following path with smooth transitions
- Velocity profile analysis (accel/decel)
- Evaluation of whether PID gains need tuning for smooth tracking

---

### Implementing Velocity Constraints


Run the velocity control script with acceleration and deceleration limits:

      python3 goal_2_velocity_constraints.py

Parameters:

- max_accel: Maximum allowed acceleration

- max_decel: Maximum allowed deceleration

- The turtle’s velocity is gradually increased or decreased based on the set constraints to mimic real-world motion.

### Plotting Velocity Profiles

      python3 goal_2_velocity_const_plot.py
      
Plots:

- Velocity

- Acceleration

- Deceleration


### Creating and Following a Grid Pattern

Make the turtle follow a grid using velocity-constrained motion:

      python3 goal_2_grid_pattern.py

Computes grid waypoints

Commands turtle to move in X and Y directions

Respects the acceleration/deceleration limits during transitions

### Analyzing PID Gains

- By observing the turtle's ability to follow the grid accurately and respond smoothly, assess:

- Whether existing Kp, Ki, Kd gains are sufficient

- If retuning is required to counteract inertia from accel/decel dynamics

- Tuning the PID may be necessary to maintain consistent performance under different motion constraints.

---

## Goal 3: Rotate Turtle in Circle

---

### Objective

The goal of this task is to control the turtle in the `turtlesim` simulator to:

- Move in a **circular path** with configurable speed and radius
- Publish the **real pose** of the turtle to `/rt_real_pose`
- Publish a **noisy pose** (Gaussian noise, σ = 10 units) to `/rt_noisy_pose`
- Publish both pose topics **every 5 seconds**

---

### Write the Circle Control Script

Create a Python node:

      goal_3_circle_turtle.py

Commands circular motion by adjusting:

- Linear velocity (v)

- Angular velocity (ω = v / radius)

Publishes:

- Real pose to /rt_real_pose

- Noisy pose to /rt_noisy_pose (Gaussian noise with standard deviation = 10)

- Publishes every 5 seconds using a ROS 2 timer.

### Run the Circle Turtle Node

In another terminal:

      python3 goal_3_circle_turtle.py
      
- The turtle will start moving in a circle.

- You can modify the script to change speed or radius.

- Expect noisy values around the real position, simulating sensor imperfections.

---

## Goal 4: Chase Turtle Fast – Robber vs. Police Simulation

---

### Objective

The aim of this task is to simulate a **chasing scenario** in the `turtlesim` environment using ROS 2, where:

- **Robber Turtle (RT)** moves in a circular path
- **Police Turtle (PT)** chases RT with:
  - Acceleration/deceleration constraints
  - Faster max speed than RT
- The chase ends when PT is **≤ 3 units** from RT

---

### Implement the PT Chasing Logic

Run the Python script:

      goal_4_chase.py
      
- Subscribes to /rt_real_pose to get RT's current position (published every 5 sec)

- Spawns PT at a random location 10 seconds after RT

- Computes the distance between PT and RT

- Publishes velocity commands for PT to /PT/cmd_vel

- Applies acceleration and deceleration constraints

- Stops PT when it gets within 3 units of RT

### Set Up and Run the Simulation

In another terminal:

      python3 goal_4_chase.py

---

## Goal 5: Chase Turtle Slow – Planning with Speed Constraints

---

### Objective

This task simulates a **chase with a planning constraint** where:

- **Random Turtle (RT)** moves in a circular path
- **Pursuer Turtle (PT)** tries to **catch RT**, but:
  - PT moves at **half the speed** of RT
- The task explores if a **planning or predictive strategy** is necessary for successful interception

---

### Implement the Slow Chase Logic

Create and run the script:

      goal_5_chase.py

Script Logic:
Subscribes to:

  - /turtle1/pose (RT pose)

  - /turtle2/pose (PT pose)

Computes:

- Relative distance and angle between PT and RT

- Publishes velocity commands to:

- /turtle2/cmd_vel with half the speed of RT

- Chases RT using geometric guidance while respecting speed limitations

### Run the Script

Run in a new terminal:

      python3 goal_5_chase.py

Component	Description

- Pose Subscribers	/turtle1/pose and /turtle2/pose
  
- Velocity Command	/turtle2/cmd_vel
  
- Speed Constraint	PT’s linear & angular velocities = ½ RT’s velocities
  
- Movement Logic	Based on relative pose, heading, and direction to target
  
- Delay Spawn	PT spawns after 10 seconds using spawn_pt.py
  
---

## Goal 6: Chase Turtle Noisy – Dealing with Uncertainty

---

### Objective

This task extends the turtle chasing challenge by introducing **uncertainty and latency**:

- **Runner Turtle (RT)** moves as before.
- 
- **Pursuer Turtle (PT)** only receives a **noisy position** of RT **every 5 seconds** via a custom topic:
- 
  - `/rt_noisy_pose`

The challenge is to **effectively chase RT** with **limited and inaccurate** information, and to explore whether an **estimator** like a Kalman Filter is necessary.

---

### Implement the Noisy Chase Logic

Create:

      goal_6_chase.py

Script Logic

- Subscribes to: /rt_noisy_pose (custom noisy RT pose topic)

- The RT's pose is updated only every 5 seconds

- The pose includes random noise

- Calculates the direction from PT to the noisy RT position

- Publishes movement commands to /turtle2/cmd_vel

- Unlike Goal 5, the PT must navigate between noisy updates, often guessing RT's position.

### Run the Script

      python3 goal_6_chase.py

Make sure that rt_noisy_pose is being published by another node/script (noisy_pose_publisher.py), simulating the 5-second noisy updates.

---

# License

---

This project is licensed under the MIT License.

---
