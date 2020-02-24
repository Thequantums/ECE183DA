READ ME FOR LAB 2 found below # line below

READ ME FOR LAB 1:
This lab involveed setting up hardware so that data could be gathered. This meant we had to build the car, read up on the specs
of various components, and calibrate them to get accurate readings. After this, a mathematical model was created and this served as
the foundation for a simulator implemeneted in python. The following is a description of the files created during this project

Lab 1 Report: Lab Report for this lab. Refer to this document for details on every aspect of the project

PWM Mapping: Excel Data Used to map pwm values to angular velocities. Five trials measuring time it took the wheel to turn a full cycle
were run at PWM values separated by increments of 10. From this an average was created for each value which was then converted to an
angular velocity. Further trials were run to clearly define a deadzone, and then all these values were graphed.

Read Me: This file

Values for Magnetometer Calibration: Five trials were run using calibration code and from these five values an average bias and scalar
were computed that would then be hard coded into senser data gathering code. The magnitude of magnetic strength was gathered by looking
at the average value of data gathered when using calibrated values

pwm_data.txt: File where input for simulation is stored. Values are a pair of inputs, one for left PWM and righ PWM. Currently contains
values that correspond to a continuos right turn centered on the right wheel

simulation.py: The implementation of the mathematical model. For derivation of the model refer to lap report and for further details on
the simulation refer to documentation on this file.

simulation_data.txt: File where output from simulation is stored. Format is a six-tuple, with the five data measurements followed by
a simulated timestamp

paperbot->Software->paperbot->paperbot.ino: Synthesis of provided code and generated code, meant to send controls to the car and gather
data from the five sensors in a continuous fashion

paperbot->Software->paperbot->mpu9250_sensor_test->mpu9250_sensor_test.ino: Code used to calibrate magnetometer. General method has been
commented out in loop function and currently calibrated hardcoded values are functionally present. General method was created with help
from kriswiner calibration code. Reference to this gitHub can be found in report

All other code and documents were given to us and were used to complete this lab

#######################################################################################################################################

README For Lab 2

This lab was dedicated to exploring the use of an Extended Kalman Filter. This involved creating an EKF for this specific system and
and comparing it hardware and simulator states. The accuracy of the estimate and the certainty around that estimate were the main points
of interest were the main varaibles of interest. Due too much technical diffculties we could onlt implement a volatile EKF with some
success and we did not generate hardware data to compare to. That being said, many of our issues were with implementing the EKF on code,
as for the mathematical understanding, we were able to develop a good understanding of it. The following is a description of the files 
used. If a file is not mentioned, then it was not updated and you can refer to README 1 for its description

EFK_states: File where output from state estimator is stored. Format is a four-tuple, which represents the four state estimates of x y 
theta and theta dot

Lab2_Report.pdf:Lab Report for this lab. Refer to this document for details on every aspect of the project

Readme.md: This file

StateEstimator.py: Implementation of EKF. For derivation of EKF refer to report and for implementation details refer to documentation on
this file

pwm_data.txt: File where input for simulation is stored. Values are a pair of inputs, one for left PWM and right PWM.

simulation.py: Update of simulator using models prescribed be Sabah during lab. In general, behaviour is the same except range finder 
code has less conditionals, theta dot included as state, and states are printed in simulation_states.txt.

simulation_data.txt: File where output from simulation is stored. Updated format is an eight-tuple, with both inputs followed by the 
five data measurements followed by a simulated timestamp

simulation_states.txt: Output file for simulation.py. Format is a four-tuple, which represents the four states x y theta theta_dot

paperbot->Software->paperbot.ino: Attempt to add wireless data transmission functionality to paperbot. Incomplete, other modified files
in Software folder are attempts at same implmention and are also incomplete

