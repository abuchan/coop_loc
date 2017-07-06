from numpy import matrix

#-----------------------------------------------------------------------
# Global Indexes: Vector/Matrix Lengths
#-----------------------------------------------------------------------
SL = 16 # State
PL = 15 # Covariance
RL = 6  # Residual
ZL = 7  # Camera Measurement
UL = 6  # IMU Measurement
QL = 12 # Process Noise

#-----------------------------------------------------------------------
# Global Indexes: State Vector Indexes 
#-----------------------------------------------------------------------
QUAT = matrix('0;1;2;3') # Quaternion
POS = matrix('4;5;6')    # Position
VEL = matrix('7;8;9')    # Velocity
BG = matrix('10;11;12')  # Bias gyroscope
BA = matrix('13;14;15')  # Bias accelerometer
NONQUAT = matrix('4;5;6;7;8;9;10;11;12;13;14;15') # Non-quaternion

#-----------------------------------------------------------------------
# Global Indexes: Covariance Matrix Indexes 
#-----------------------------------------------------------------------
PTHETA = matrix('0;1;2')      # Theta
PPOS = matrix('3;4;5')        # Position
PPOSE = matrix('0;1;2;3;4;5') # Pose
PVEL = matrix('6;7;8')        # Velocity
PBG = matrix('9;10;11')       # Bias gyroscope
PBA = matrix('12;13;14')      # Bias accelerometer
POBS = matrix('0;1;2;3;4;5')  # Observation
PNONQUAT = matrix('3;4;5;6;7;8;9;10;11;12;13;14') # Non-quaternion

#-----------------------------------------------------------------------
# Global Indexes: IMU
#-----------------------------------------------------------------------
UG = matrix('0;1;2')       # Rotational velocity
UA = matrix('3;4;5')       # Linear acceleration
U  = matrix('0;1;2;3;4;5') # UA & UG

#-----------------------------------------------------------------------
# Global Indexes: Pose
#-----------------------------------------------------------------------
Z    = matrix('0;1;2;3;4;5;6') # Pose
ZCOV = matrix('0;1;2;3;4;5')   # Pose Covarance

#-----------------------------------------------------------------------
# Global Indexes: IMU Noise Characteristics 
#-----------------------------------------------------------------------
NG = matrix('0;1;2')    # Noise gyroscope
NA = matrix('3;4;5')    # Noise accelerometer
NWG = matrix('6;7;8')   # Random walk process gyroscope
NWA = matrix('9;10;11') # Random walk process accelerometer

#-----------------------------------------------------------------------
# Global Types: Measurements
#-----------------------------------------------------------------------
IMU  = 0 # IMU Measurement
POSE = 1 # Relative Pose Measurement

#-----------------------------------------------------------------------
# Global Constants:
#-----------------------------------------------------------------------
g = matrix('0;0;-9.81') # Gravity vector