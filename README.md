# IMU CONTROL

Launch drivers for all 8 IMUs with faster implementation:
roslaunch fly_jacket all_imu_launch_fast.launch

Run pose estimation
rosrun fly_jacket pose_imu.py 

Run control node
rosrun fly_jacket control_node.py 
