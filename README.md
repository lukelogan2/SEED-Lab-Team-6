# SEED-Lab-Mini-Project

## Organization of this Repository
- Motor Control contains the code uploaded to the Arduino to control the motor spinning the wheel
- Motor Control Old contains a version of Motor Control with a PI controller that doesn't function as well
- seed_CV_v2.py contains the code ran on the Raspberry Pi to execute the computer vision on the pi camera and send results to the Arduino
- Transfer_Function_Test_v1 contains the code ran on the Arduino to collect samples of velocity vs time to estimate the transfer function
- Transfer Function Test v2 implements the same test as v1, but uses a longer period to calculate velocity
