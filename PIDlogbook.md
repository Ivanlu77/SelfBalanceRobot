# PID control logbook

## 1.Construct the robot, installing platform IO and test given code
  &emsp; To test the correctness of wiring and structure of the robot, targetAndgle in the given code was  changed to 3. The motor rotated in expected direction.
  ### Next target: Coding and implement attitude control loop
  
## 2.Attitude Control loop
  &emsp; Attitude control loop uses difference between target angle and current angle as the input and output acceleration value to motor control function. The current angle was obtained by a MPU5060 and a complementary filter. MPU5060 contains a accelerometer and a gyroscope, the outputs of the two components are dealed with the complementary filter to get tilting angle. Target angle was set manully. It equivalents to the natrual equilibrium tilt angle of the robot.
  
  &emsp; At the beginning the design, the output of the PID controller was velocity of the motor. However, However, after testing a wide range of Kp values, the robot still showed no signs of balancing. Then code was changed so that PID output values were used as acceleration. kp was increased from 10, and robot started to show sign of balancing after 400 and could stand on itself at 700 with low frequency medium sway. 
  
