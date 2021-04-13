# Programming for robots and manipulators (VRM)

<p align="center">
<img src=https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/blob/main/images/lab.png width="800" height="475">
</p>

## Requirements:

**Software:**
```bash
Robot Operating System (ROS)
RoboStudio ABB
Automation Studio B&R
Linux Ubuntu (16.04 or 18.04)
Unity3D and Vuforia
```

**Programming Language:**
```bash
Python and/or C/C++, C#
```

**Other:**
```bash
Algorithmization, Programming, Mathematics and Optimization
```
## Description:

The VRM (Programming for Robots and Manipulators) course enables students to acquire skills and knowledge in programming industrial / mobile robots and manipulators. This course also expands skills in advanced system integration and deployment in real-world robotic applications. The aim of the VRM course is to introduce students to modern approaches to robotic technology with a focus on programming, kinematics / dynamics solutions, motion planning, Industry 4.0 and the use of artificial intelligence (AI).

The main focus is on students practical skills in laboratory exercises, which include several blocks:
1. RobotStudio ABB
2. Forward/Inverse kinematics
3. Robotic operating system (ROS) extended by advanced industrial capabilities ROS-Industrial (ROS-I)
4. Virtual / digital twin using Unity3D extended by system integration with B&R Automation PLC via OPC UA
5. A simple demonstration of augmented reality based on robotics

These few blocks are extended by theoretical knowledge, which students acquire in the form of lectures.

Link: [Detailed description of the Syllabus (Czech)](https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/blob/main/Course_description/Description.pdf)

Link: [Course descrition - FME, BUT](https://www.fme.vutbr.cz/en/studenti/predmety/235124)

## Detailed description of the Syllabus:

**Week 1 (8. 2. 2021):**
- Introduction to the course, main goals, methods and evaluation criteria, etc.
- Introduction to the issue, development and definition of robots, manipulators.
- Introduction of an advanced robotic production line called Industry 4.0 (i4C).

Link: [Lecture 1](https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/tree/main/Lecture/1)

**Week 2 (14. 2. 2021):**
- Stationary industrial robots and single-purpose manipulators. Specific constructions of industrial robots, parallel structures. Programmable logic controllers (PLC) and use in robotics.
- Control and programming of industrial robots. Introduction of basic tools for creating robotic simulations.
- Assignment of seminar paper.

Link: [Lecture 2](https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/tree/main/Lecture/2)

**Week 3 (22. 2. 2021):**
- End-effectors and their adaptability.
- ABB RobotStudio - Workshop (Part 1: Introduction, Create tool, Simple task with an industrial robot, etc.)
- Assignment of project.

Link: [Lecture 3](https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/tree/main/Lecture/3)

Link: [Laboratory 1](https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/tree/main/Lab/1)

<p align="center">
<img src=https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/blob/main/images/RS_LAB_1.PNG width="650" height="350">
</p>

**Week 4 (1. 3. 2021):**
- ABB RobotStudio - Workshop (Part 2: Simple task with an collaborative robot, Conveyor control, Smart gripper, Sync., etc.)

Link: [Laboratory 2](https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/tree/main/Lab/2)

<p align="center">
<img src=https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/blob/main/images/RS_LAB_2.PNG width="650" height="350">
</p>

**Week 5 (8. 3. 2021):**
- Forward / Inverse Kinematics.
- Demonstration of Forward / Inverse kinematics on a two-link simple manipulator. Creation of a working envelope of a specified robotic construction.

Link: [Lecture 4](https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/tree/main/Lecture/4)

Link: [Laboratory 3](https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/tree/main/Lab/3)

<p align="center">
<img src=https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/blob/main/images/FK_IK_LAB_3.PNG width="650" height="300">
</p>

**Week 6 (15. 3. 2021):**
- Differential Kinematics and Robotic Dynamics.
- BRNO INDUSTRY 4.0 | 2021 ([online](https://brno-industry-40-online.b2match.io)): 5th International B2B Conference about Production Digitization and Smart Technologies for Industry
- Demonstration of Differential Kinematics on a two-link simple manipulator. Example of dynamics calculation using Euler-Lagrange equation.

Link: [Lecture 5](https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/tree/main/Lecture/5)

Link: [Laboratory 4](https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/tree/main/Lab/4)

**Week 7 (22. 3. 2021):**
- Motion planning in robotics (mobile, industrial robots) using classical Joint / Cartesian interpolation and other planning methods such as RRT (Rapidly-exploring random tree), PRM (Probabilistic roadmap) and Reinforcement / Deep-Reinforcement learning.
- Bezier curves (Linear, Quadratic, Cubic).
- Demonstration of simple motion planning using Joint / Cartesian interpolation on a two-link manipulator. Trajectory smoothing using Bézier curves. Animation of the resulting trajectory, check of reachable points, etc.

Link: [Lecture 6](https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/tree/main/Lecture/6)

Link: [Laboratory 5](https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/tree/main/Lab/5)

<p align="center">
 <img src=https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/blob/main/images/manipulator.png width="400" height="325">
 <img src=https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/blob/main/images/bezier.png width="400" height="325">
</p>

**Week 8 (29. 3. 2021):**
- ROS (Robot Operating System), ROS-I (Industrial) Introduction. 
- ROS installation (melodic distribution), package configuration, explanation of basic concepts (topics, services, messages, etc.)
- A simple example of TurtleSim motion control and working with a terminal.
- Creating a ROS workspace for simple control of TurtleSim motion using the Python programming language (catkin, rospy, launch file, etc.)

Link: [Laboratory 6](https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/tree/main/Lab/6)

<p align="center">
 <img src=https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/blob/main/images/turtle1.png width="375" height="325">
 <img src=https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/blob/main/images/turtle2.png width="375" height="325">
</p>

**Week 9 (5. 4. 2021):**
- National Holiday (Easter Monday)


**Week 10 (12. 4. 2021):**
- Simple demonstration of robot motion control and trajectory planning via the ROS system using several simulation tools (RVIZ, gazebo, etc.)
- Controlling the movement of multiple industrial robots (ABB, Fanuc, Universal Robots, etc.) using the Python programming language (catkin, rospy, launch file, etc.)
- Presentation of students' Bachelor's theses (ROS, robotics, system integration, etc.)

Link: [Laboratory 7](https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/tree/main/Lab/7)

<p align="center">
 <img src=https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/blob/main/images/rviz_1.png width="355" height="275">
 <img src=https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/blob/main/images/rviz_2.png width="355" height="275">
 <img src=https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/blob/main/images/rviz_4.png width="355" height="275">
</p>

**Week 11 (19. 4. 2021):**
- Unity3D as a tool for creating digital / virtual twins, connection with B&R Automation Studio (follow-up project from the VPL course).
- Introduction to augmented reality and a simple demonstration of the application in the real world.

<p align="center">
 <img src=https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/blob/main/images/lin_ax_1.png width="355" height="275">
 <img src=https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/blob/main/images/lin_ax_2.png width="355" height="275">
</p>

**Week 12 (26. 4. 2021):**
- Introduction to the concept of Industry 4.0. 
- Industry 5.0, 6.0 and automation a few years later.

<p align="center">
<img src=https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/blob/main/images/i4C_m.png width="800" height="475">
</p>

**Week 13 (3. 5. 2021):**
- Presentation of team projects.

## Assessment Methodology:

**Description:**
- Active participation in laboratory exercises and lectures: 10 points
- Seminar paper: 20 points [[Link](https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/tree/main/Project/1_Seminar_paper)]
- Project no. 1: 30 points [[Link](https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/tree/main/Project/2_Project_1)]
- Project no. 2 (Team project): 40 points [[Link](https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/tree/main/Project/2_Project_2)]

The condition for writing a seminar paper is the use of LaTex (e.g., Overleaf -> Online LaTeX Editor). Projects are submitted via GitHub, which will contain a folder of all relevant files for each project and a short description in English.

The penalty equation for late submission of a project is defined as:

<p align="center">
$\Large p_s = \lvert \frac{\Delta t}{24}e^{\frac{1}{2}} \rvert + \delta_p,$
</p>

where $\Delta t$ is defined as the difference between the date of deadline and the date of assignment of the project (in hours), and $\delta_p$ is the project error factor defined as $\frac{s_{max}}{10}$.

The maximum possible score is defined as:

<p align="center">
$\Large s = s_{max} - p_s,$
</p>

where $s_{max}$ is the initial maximum score, and $p_s$ is a penalty.

The script for the calculation can be found at [[Link](https://github.com/rparak/Programming-for-robots-and-manipulators-VRM/tree/main/Course_description/project_score_calculator)].

## Resources and Literature:
**Textbooks:**
1. **Introduction to AI Robotics**, Robin R. Murphy
2. **Roboty a robotizované výrobní technologie**, Zdeněk Kolíbal
3. **Handbook of Robotics**, Bruno Siciliano
4. **Modern Robotics: Mechanics, Planning, and Control**, Kevin M. Lynch and Frank C. Park
5. **Robotics, Vision and Control**, Peter Corke
6. **Planning Algorithms**, Steven M. LaValle
7. **Industrial Robotics: Theory, Modelling and Control**, Sam Cubero
8. **Mathematics for Computer Graphics**, John Vince

**Other:**
1. [IEEE Xplore](https://ieeexplore.ieee.org/Xplore/home.jsp)
2. [Science Direct](https://www.sciencedirect.com)
3. [Springer - International Publisher Science](https://www.springer.com/gp)

## Contact Info:
Roman.Parak@vutbr.cz or Microsoft Teams (recommended)

## License
[MIT](https://choosealicense.com/licenses/mit/)
