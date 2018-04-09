
# PMC Controller

This project is a C++ implementation of a PMC controller for controlling steering angle and throttle of a simulated car.  The car simulation program provides position, speed, and steering angle as inputs to the controlling program.  CTE is the distance the car is from the center line.  The simulation accepts throttle and steering angle back from the controlling program.  This communication with the program controlling the car is via a raw socket where the program acts as the server and accepts requests for commands from the simulation.  It is the job of the controlling C++ program to accept the inputs and formulate a correct steering angle and throttle so that the car stays as close to the center line as possible.  The program also provides waypoints and planned path for visualizaiton in the simulation.  See image below.


<br /><br />
<p align="center">
<img src="https://github.com/TheOnceAndFutureSmalltalker/mpc_project/blob/master/img/simulator.JPG" width="802px" /><br /><b>Car on Track Simulator</b></p>
<br />
