
# PMC Controller

This project is a C++ implementation of a PMC controller for controlling steering angle and throttle of a simulated car.  The car simulation program provides position, speed, and steering angle as inputs to the controlling program.  CTE is the distance the car is from the center line.  The simulation accepts throttle and steering angle back from the controlling program.  This communication with the program controlling the car is via a raw socket where the program acts as the server and accepts requests for commands from the simulation.  It is the job of the controlling C++ program to accept the inputs and formulate a correct steering angle and throttle so that the car stays as close to the center line as possible.  The program also provides waypoints and planned path for visualizaiton in the simulation.  See image below.


<br /><br />
<p align="center">
<img src="https://github.com/TheOnceAndFutureSmalltalker/mpc_project/blob/master/img/simulator.JPG" width="802px" /><br /><b>Car on Track Simulator</b></p>
<br />

## PMC Controller

## Approach

I used the solution code provided in the Udacity Lesson 20 Model Predictive Controller.  This provided the basic outline for implementing the MPC solution.  It uses the external library CppAd for calculating derivatives of functions and the Ipopt library for solving an optimization problem.  


I also followed the Udacity walkthrough video for this project provided by <a href="https://youtu.be/bOQuhpz3YfU?list=PLAwxTw4SYaPnfR7TzRZN-uxlxGbqxhtm2">this link</a>.  The main point of interest in this approach is that the waypoints provided by the simulation are transposed from map coordinates to car coordinates with the car at (0,0) and zero angle of orientation.  The entire problem is then solved in car coordinates.  This makes it easy providing the waypoints and planned path back to the simulation (which must be in car coordinates) since we are already in car coordinates and no translation is necessary.  
