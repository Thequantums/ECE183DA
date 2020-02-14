README For Lab 2

This lab was dedicated to exploring the use of an Extended Kalman Filter. This involved creating an EKF for this specific system and and comparing it hardware and simulator states. The accuracy of the estimate and the certainty around that estimate were the main points of interest were the main varaibles of interest. Due too much technical diffculties we could onlt implement a volatile EKF with some success and we did not generate hardware data to compare to. That being said, many of our issues were with implementing the EKF on code, as for the mathematical understanding, we were able to develop a good understanding of it. The following is a description of the files used. If a file is not mentioned, then it was not updated and you can refer to README 1 in lab1 folder for its description

EFK_states: File where output from state estimator is stored. Format is a four-tuple, which represents the four state estimates of x y theta and theta dot

Lab2_Report.pdf:Lab Report for this lab. Refer to this document for details on every aspect of the project

Readme.md: This file

StateEstimator.py: Implementation of EKF. For derivation of EKF refer to report and for implementation details refer to documentation on this file

pwm_data.txt: File where input for simulation is stored. Values are a pair of inputs, one for left PWM and right PWM.

simulation.py: Update of simulator using models prescribed be Sabah during lab. In general, behaviour is the same except range finder code has less conditionals, theta dot included as state, and states are printed in simulation_states.txt.

simulation_data.txt: File where output from simulation is stored. Updated format is an eight-tuple, with both inputs followed by the five data measurements followed by a simulated timestamp

simulation_states.txt: Output file for simulation.py. Format is a four-tuple, which represents the four states x y theta theta_dot

paperbot->Software->paperbot.ino: Attempt to add wireless data transmission functionality to paperbot. Incomplete, other modified files in Software folder are attempts at same implmention and are also incomplete
