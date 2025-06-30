# Multi‑Robot Autonomous Exploration

This ROS package, prepared as a final assignment, performs autonomous mapping of a maze in the Gazebo simulation environment using **four TurtleBot3 robots**.  
The package automates exploration with a **frontier‑based exploration** method, intelligently directing robots toward unknown regions of the map.

## How the Project Works

The `multi_robot_explorer` node is the heart of the exploration logic:

1. **Map Analysis** – Continuously listens to the unified `/map` topic built from all robots’ sensor data.  
2. **Frontier Detection** – Detects the borders between known free space and unexplored areas; these borders are the most rational exploration targets.  
3. **Target Optimisation** – Because many frontier points can be very close, they are grouped with a *clustering* algorithm, and the centroid of each cluster becomes a candidate target.  
   *In testing, robots tended to get stuck without clustering, so it is enabled by default.*  
4. **Intelligent Task Assignment** – Each idle robot is assigned the nearest frontier target that is not already claimed by another robot.  
5. **Blacklist Mechanism** – A key feature. If a robot cannot reach an assigned target (e.g. the goal is behind a wall or the robot is trapped), that target is temporarily **black‑listed**.  
   This prevents the node from repeatedly assigning an unreachable goal, avoiding error loops and reducing situations where robots spin in place or collide with walls.  
6. **Task Monitoring** – Goals are sent via the ROS `actionlib` interface, allowing reliable tracking of whether the robot has reached the goal or encountered problems.

---

## Dependencies

Install the following packages before running the project.

### System packages (install with APT)

* `ros-noetic-multirobot-map-merge` – merges maps from different robots.  
* `ros-noetic-dwa-local-planner` – local planner used by the `move_base` navigation stack.

### ROS packages (clone with Git)

* `micromouse_maze` – contains the simulation world and RViz configuration.  
* `turtlebot3` – core TurtleBot3 packages.  
* `turtlebot3_simulations` – Gazebo simulation packages for TurtleBot3.

---

## Installation Steps

1. **Create a workspace** (if you do not already have one):

   ```bash
   mkdir -p ~/robotlar_ws/src
   cd ~/robotlar_ws/
   catkin_make
   ```

2. **Install missing system packages**:

   ```bash
   sudo apt-get update
   sudo apt-get install ros-noetic-multirobot-map-merge ros-noetic-dwa-local-planner
   ```

3. **Clone the required repositories** into the workspace `src` folder:

   ```bash
   cd ~/robotlar_ws/src
   git clone https://gitlab.com/blm6191_2425b/blm6191/micromouse_maze.git
   git clone https://gitlab.com/blm6191_2425b/blm6191/turtlebot3.git
   git clone https://gitlab.com/blm6191_2425b/blm6191/turtlebot3_simulations.git
   ```

   Copy this `multi_robot_explorer` package into `~/robotlar_ws/src` as well.

4. **Build the workspace**:

   ```bash
   cd ~/robotlar_ws
   rosdep install --from-paths src --ignore-src -r -y
   catkin_make
   ```

   If you encounter build errors, remove the `build` and `devel` folders (`rm -rf build devel`) and run `catkin_make` again.

5. **Set environment variables** – add the following lines to `.bashrc`:

   ```bash
   echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
   echo "source ~/robotlar_ws/devel/setup.bash" >> ~/.bashrc
   echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
   ```

   **Close and reopen** all terminals or run `source ~/.bashrc`.

---

## Launch Instructions

Start each of the following commands in **its own terminal**, waiting for the current terminal to finish initialisation before moving to the next.

1. **Terminal 1 – Gazebo simulation**  
   (Loads the maze and 4 robots.)

   ```bash
   roslaunch micromouse_maze micromouse_maze3_multi.launch
   ```

2. **Terminal 2 – Map merge**  
   (Merges the 4 individual maps into `/map`.)

   ```bash
   roslaunch turtlebot3_gazebo multi_map_merge.launch
   ```

3. **Terminal 3 – SLAM (mapping)**  
   (Starts `gmapping` for each robot.)

   ```bash
   roslaunch turtlebot3_gazebo multi_turtlebot3_slam.launch
   ```

4. **Terminal 4 – Navigation**  
   (Activates `move_base` planners and action servers for each robot.)

   ```bash
   roslaunch turtlebot3_navigation multi_move_base.launch
   ```

5. **Terminal 5 – RViz (visualisation)**  
   (Opens the interface to view robots, map and sensor data.)

   ```bash
   roslaunch micromouse_maze multi_robot_rviz.launch
   ```

6. **Terminal 6 – Autonomous exploration node**  
   (Start after all infrastructure is running.)

   ```bash
   roslaunch multi_robot_explorer explorer.launch
   ```

### Example Run

_When translating, some Turkish characters did not render correctly in the console; that is why “??” may appear in some screenshots. This was later fixed, so you should not encounter the issue._

With all six terminals running, you will first see the robots placed in the maze and the merged map beginning in RViz:

![Start](images/baslangic.png)

![Start RViz](images/baslangic_rviz.png)

If the code is working, the robots receive their first tasks immediately, and **Terminal 6** displays:

![Initial Assignments](images/robot_atama_baslangic.png)

You can see coordinate assignments for Robots 0‑3:

![Gazebo Progress](images/gazebo_ilerleme.png)

Robots begin moving as shown above. If a robot cannot find a clear path (e.g. the robot in the centre), it waits a short time, then automatically proceeds once the path is open:

![Path Cleared](images/orta_duzgunlesme.png)

The console shows `ABORTED` when a path is unreachable. The goal is black‑listed, a new goal is assigned, and exploration continues without deadlock.

Both RViz and Gazebo show planned paths (green lines) and robot motion:

![RViz Progress](images/rviz_ilerleme.png)

![Group Progress](images/toplu_1.png)

You can monitor path assignments in the terminal, RViz and Gazebo:

![Path Exploration](images/yol_kesif.png)

After letting the system explore for a while, the map reaches its final state:

![Final Exploration](images/final.png)

### Expected Outcome

After launching the final command:

* **Terminal 6** – Logs like `New goal assigned to Robot X...` appear.  
* **RViz** – Robots automatically draw green paths toward unexplored areas.  
* If a robot cannot reach its goal, you will see `could not reach goal` followed by `goal added to blacklist`. The robot immediately switches to another target instead of getting stuck.  

Exploration ends automatically when no new frontier points remain and all robots have completed their tasks.

### Extra

To change the number of robots, edit the `num_robots` parameter in  
`multi_robot_explorer/launch/explorer.launch`:

```xml
<param name="num_robots" value="4" />
```
