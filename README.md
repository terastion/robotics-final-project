# robotics-final-project

## Project Description

## System Architecture

## ROS Node Diagram

## Execution
Generate an OpenAI API Key, put the API key in `client = OpenAI(api_key = '<YOUR_API_KEY_HERE>'` within both `main_node.py` and `action_manager.py`.

Replace the `music.mp3`'s absolute path in `play_music.py` with its absolute path in your environment
  
`roscore`

`ssh pi@<YOUR_TURTLEBOT_IP>` and `bringup`

`ssh pi@<YOUR_TURTLEBOT_IP>` and `bringup_cam`

`roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch`

`roslaunch turtlebot3_manipulation_moveit_config move_group.launch`

`roslaunch robocourier action.launch`

> **Note:** If you encounter any troubles, it is likely due to missing libraries. Use  `pip instal XXX` command to install necessary libraries.

## Challenges, Future Work, and Takeaways
