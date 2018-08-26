
# MPC Project

This project is a C++ implementation of a Model Predictive Controller - MPC - for controlling steering angle and throttle of a simulated car.  The car simulation program provides position, speed, and steering angle as inputs to the controlling program.  The simulation accepts throttle and steering angle back from the controlling program.  This communication with the program controlling the car is via a raw socket where the program acts as the server and accepts requests for commands from the simulation.  It is the job of the controlling C++ program to accept the inputs and formulate a correct steering angle and throttle so that the car stays as close to the center line as possible.  The program also provides waypoints and planned path for visualizaiton in the simulation.  See image below.


<br /><br />
<p align="center">
<img src="https://github.com/TheOnceAndFutureSmalltalker/mpc_project/blob/master/img/simulator.JPG" width="802px" /><br /><b>Car on Track Simulator</b></p>
<br />

## Car Model Predictive Controller

A Model Predictive Controller is a controller that issues commands to a car to get it to follow a predetermined path.  The path is given as a set of points on a map - called waypoints.  The commands given to the car by the controller are usualy throttle, braking, and steering angle - the typical things controlled by a driver of a car.  

The car cannot usually follow the path precisely due to intertia, turn angle, etc.  The controller tries to get the car to follow the path as closely as possible however, the exact predetermined path being considered optimal.

The MPC calculates a predicted path for the car to follow based on the given predetermined waypoints and current car state - position, speed, bearing.  The goal of the MPC is to calculate a predicted path that follows the predetermined path as closely as possible and keeps car's bearing oriented with predetermined path as closely as possible.  Some secondary goals of the controller are to not change speed or steering angle too abrubtly and keep speed as close to a preferred value as possible.  Additional goals may be added as well.

Finding the best predicted path is constrained by the current state of car, the physics of the car's motion, and limits of the car's control mechanisms (max steering angle, max throttle, etc.).  

Since the equations involved are non-linear and we are trying to optimize a goal with given constraints, the MPC problem is classified as a non-linear optimization problem.  These are not trivial to solve.  

## Approach

I used the solution code provided in the Udacity Lesson 20 Model Predictive Controller.  This provided the basic outline for implementing the MPC solution.  It uses the external library CppAd for calculating derivatives of functions and the Ipopt library for solving an optimization problem.  


I also followed the Udacity walkthrough video for this project provided by <a href="https://youtu.be/bOQuhpz3YfU?list=PLAwxTw4SYaPnfR7TzRZN-uxlxGbqxhtm2">this link</a>.  The main point of interest in this approach is that the waypoints provided by the simulation are transposed from map coordinates to car coordinates with the car at (0,0) and zero angle of orientation.  The entire problem is then solved in car coordinates.  This makes it easy providing the waypoints and planned path back to the simulation (which must be in car coordinates) since we are already in car coordinates and no translation is necessary.  

## Rubric Elements

Here I will cover the rubric points.

### The Model

The model used to predict car behavior from one state to the next is a standard kinematic model.  First we consider the car's current state.  This includes the car's x and y position in map coordinates, its angle psi from the map x axis, and its current speed, v (note that even though we use v for speed, it does NOT denote velocity since it does not itself include direction, which is psi in our case.)  
What moves the car from one state to the next are the actuators acceleration, a, (or throttle and may include braking or negative acceleration) and steering angle, delta.  

A car's next state is a function of its current state and can be calculated using the following kinematic equations:

x1 = x0 + v0 cos(psi) * dt
y1 = y0 + v0 sin(psi) * dt
psi1 = psi0 + v0/Lf * delta0 * dt
v1 = v0 + a * dt

The implementation of this model is found in the file MPC.cpp, lines 119 - 130.

### Timestep Length and Elapsed Duration

N is the number of states we intend to predict out into the future.  The larger N is, the closer we can get to the true path.  However, we pay a computatational penalty for larger N and since this is a real time calculation, and very numerically intensive, we run the risk of not completing the necessary calculations in time.  So I went with the original recommendation of N=10 and this seemed to work fine.

The time between the N states we are predicting is dt.  We do not need to predict too far into the future so a dt = 100 mS is fine giving us a full future prediction of 10 * 0.1 = 1 second.  

### Polynomial Fitting and MPC Preprocessing

At each state, the car has an intended path defined by a set of points in map coordinates.  A polynomial needs to be fitted to these points that describes the path mathematically.  This polynomial equation then allows us to numerically find a predicted path for the car.

As mentioned, the car's state and the waypoints are provided in map coordinates.  The car, however, prefers to operate within its own coordinate system where it is always at (0,0) and the x axis extends out directly in front of it.  Therefore, we need to transpose the waypoints and the car's state from map coordinates to vehicle coordinates before we find the optimal predicted path.

The waypoints are transposed as follows:

vehicle_x = (map_x - x0) * cos(-psi) - (map_y - y0) * sin(-psi)
vehicle_y = (map_x - x0) * sin(-psi) + (map_y - y0) * cos(-psi)

This is done prior to fitting the polynomial.  This results in the initial state of the car being x=0, y=0. psi=0, and v is unchanged.  The implementation of these equations are found in the file main.cpp, lines 98 - 110.

### Model Predictive Control with Latency

When we issue a command to the car to turn or accelarate, there is a delay in response time.  We call this latency.  The result is that by the time the car actually responds to the commands, it is no longer at its initial state.  To compensate for this, we can apply the kinematic model described above to the current state for the period of the latency to arrive at the state which will actually be in effect once the steering and acceleration commands are realized.  This state, in car coordinates, is calculated as below:

latent_x = 0 + v * dt;
latent_y = 0
latent_psi = 0 + v * -delta/Lf * dt;
latent_v = v + a * dt;

For the purposes of this project, the latency is 100 mS.  The implementation of latency is found in main.cpp, lines 135 - 151. 

## Results

I was able to get the car to perform up to 150 mph.  I was able to do this by increasing the cost coefficient of the CTE term so that the car stayed closer to the center.  However, it slowed down considerably on turns and never came close to 150 mph!  It maybe got to 95 mph or so.  This is due to lack of a good straight away.

The curve after the bridge was the most difficult.  I ran into the second cement block there several times in tuning the program.

I also ran it at 10 mph and found some strange results.  It zig-zagged quite a bit, stopped several times, and even went in reverse!

Finally, regardless of reference speed, at the outset, the car would always zig zag back and forth for a few seconds before it seemed to get on track.  I don't know why this is.  Whatever the reason is, it may be related to why the car does not perform at very low reference speeds as mentioned above. 

In the current version of code, the reference velocity is set at 100 mph.

## Recommendations

A higher cost coefficient for CTE allowed the car to have higher reference speeds.  This made for successful runs, however the car would slow down considerably on curves in order not to incur the high cost of CTE.  At lower speeds however, this penalty was too much and the car could have gone faster on the curves.  This leads me to believe that the cost coefficients could perhaps be functions of reference velocity.

The predicted trajectory at slow speeds was much shorter than the waypoints trajectory and at high speeds, extended far out in front of it and the first few segments lacked smoothness.  I think the distance and increments of the plot of the predicted trajectory could also be a function of reference velocity.

I would also like to make reference velocity a command line argument with a default of say, 60 mph.  This would make testing a little easier.
