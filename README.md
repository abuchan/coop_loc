#Cooperative Multi-Robot Localization with a Low Cost Heterogenous Team
This repository was factored out of the project at https://github.com/ucberkeley-vip/coop\_slam

## Heterogeneous Team
1. Robot Team [n]
  * [1]   Observer Robot - {Camera, IMU}
  * [n-1] Picket robots  - {IMU}
2. Measurements
  * Relative Pose (Observer-> Picket)
  * IMU

## Custom Messages:
1. IMUwithMoving
  * header
  * angular velocity
  * linear acceleration
  * ismoving

