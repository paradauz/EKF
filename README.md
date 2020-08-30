# ekf
This is the implementation of an extended Kalman filter for the localization of the MoonRanger rover. It fused reading from gyroscope, aceelerometer and wheel encoders to predict the position and orientation of the rover.

To launch type roslaunch ekf ekf.launch
Visulization of the AHRS can be done in Rviz:
  Launch Rviz 
  Add two axis  
  Set one to "ENU" and the second one to "quat" 

Position estimation is published to /odom topic

Useful cheat sheet for Eigen library:
https://gist.github.com/gocarlos/c91237b02c120c6319612e42fa196d77
