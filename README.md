# ROS2 Navigation for Unitree GO2 (Foxy).
Author: Sayantani Bhattacharya
</br></br>
Adapted for Foxy by @jamission
</br></br>
This is a standalone package, that has code and launch files for manual navigation operation on Unitree GO2.
</br></br>
Using: 
- Unitree Go2 in high level mode  [wrapper over Unitree's SDK](https://github.com/Sayantani-Bhattacharya/unitree_ros2)
- ROS2 Foxy
- RTAB-Map
- Nav2 stack.

## Features:
Package Name: unitree_go2_nav_foxy
- Transform publishers.
- Robot State publishers.
- Rtabmap pkgs generating odom tf and occupancy grid using GO2's 4d Lidar.
- Rviz visulaizations with modified URDF.
- Nav-to-pose to enable high level control with the GO2's APIs.
- Nav2 based manual goal subscription.

## Setup   
  1.     mkdir src && cd src
         git clone https://github.com/jamission/unitree_go2_nav_foxy.git
         cd unitree_go2_nav
     
  2. Pull the Unitree SDK Wrapper submodule.
    
         cd unitree_ros2       
         git clone git@github.com:Sayantani-Bhattacharya/unitree_ros2.git
     
  3. Follow the setup and build instructions mentioned in the unitree_ros2 package ReadMe. This will get you connected to the real Robot.
  4.     cd ../
         source /opt/ros/foxy/setup.bash
         colcon build
         source install/setup.bash
  5. Make sure rtabmap is installed:
       `sudo apt install ros-foxy-rtabmap-ros`
     
      and verify install (this should return a path if successful)
          `ros2 pkg prefix rtabmap_slam`
      

## Directions:

1. Complete the above setup.
2. Verfiy that unitree topics are visible in your current workspace.

       ros2 topic list
3. For mapping:

        ros2 launch unitree_go2_nav mapping.launch.py

   For the map to be generated initally, the robot needs to move a bit.

4. For navigation and mapping:
   </br>  
   Terminal 1:

       ros2 launch unitree_go2_nav navigation.launch.py

   Terminal 2:
   
       cd unitree_ros
       source /opt/ros/foxy/setup.bash
       source install/setup.bash
       ./install/unitree_ros2_example/bin/high_level_ctrl

   This will start Rviz, where you can use the 2D goal pose UI button to give GO2 commands.
   Or, you can publish ros2 topic as well.
   

## Known issues, that I am currently working on:
  - Modifing the URDF joint states to make it similar to the real robot.
    

## For developing autonomy packages with this as a base package:
Add this to your repository as a submodule

      git submodule add https://github.com/jamission/unitree_go2_nav_foxy.git
      git submodule update --init --recursive 
      git commit -m "Added submodule unitree_go2_nav" 
      git push origin main 

For reference: https://github.com/Sayantani-Bhattacharya/Multi-Hetero-Agent-Exploration-on-UnitreeGOs/tree/go2_auto_nav
