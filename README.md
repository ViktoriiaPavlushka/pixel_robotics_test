**Safety Node**

**Description**
This is a safety node for a robot. It uses LIDAR data to prevent crashes. If an obstacle is 0.5 meters or closer, the robot stops moving towards it. 

**How to Run Locally**
1. Start the simulation inside the Docker container:
   `ros2 launch my_diff_robot robot.launch.py`
3. Open a second terminal, enter the container, and run the safety node:
   `wsl -d Ubuntu
    docker exec -it ros_gz_gui bash
    python3 src/my_diff_robot/scripts/safety_node.py`
4. Open a third terminal, enter the container, and run the teleop script:
   `wsl -d Ubuntu
    docker exec -it ros_gz_gui bash
    python3 src/my_diff_robot/scripts/simple_teleop.py`

**Design Choices & Assumptions**
* **Middleman Setup:** The safety node listens to the teleop script on `/cmd_vel_raw`. It checks the LIDAR data and only sends safe commands to the robot's wheels on `/diff_drive_controller/cmd_vel`.
* **Smart Collision Check:** The code uses the robot's exact width. It stops the robot if an obstacle is directly in its path.
* **Directional Safety:** If the robot is blocked in the front, the code still allows the user to drive backward or rotate to get away.
* **Data Filtering:** The code safely ignores `inf` and `NaN` values of lidar`s data so the program does not crash in empty rooms.
* **Assumption:** The `simple_teleop.py` file is modified to publish commands to the `/cmd_vel_raw` topic.

**Demo Video**
https://drive.google.com/file/d/1i94apxRrW00yEo0_FTBiPTq-BtqNFkMu/view?usp=sharing
