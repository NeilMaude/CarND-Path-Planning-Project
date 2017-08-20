# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

Neil Maude, August 2017

## Overview
This project navigates a car around the Udacity path planning test track.  The objective is to plot a path which remains within the set constraints (speed limit, acceleration, jerk, avoiding other cars, remaining in lane unless changing lane), whilst making progress around the test loop.

## Creating a Path
The main loop of the application receives co-ordinates and sensor fusion data, then responds with a series of points which make up the path along which the vehicle will drive.  Un-used points are also passed to the main loop, allowing a continuous path to be created by adding to previously un-used points.

Path planning is done using Frenet co-ordinates (s = distance along the roadway, d = distance from the roadway centre line) which allow planning using the required `lane` (0, 1, 2 from left to right) at a given point along the roadway - either the car remaining in the current lane or moving to a new lane.  Code within main.cpp uses the `spline.h` library to create points along this desired trajectory, which are appended to points that have been planned previously but not yet visited.   

This spline-based process creates a smooth path for the vehicle.  By inspection, it was found that a path generated in this way from a lane to an adjacent lane at 50mph and covering approx 30m forward distance would not violate the jerk constraint and therefore this process can be used safely to generate lane-change paths.

## Acceleration/Deceleration
The path planner uses sensor fusion data to detect cars in the current lane and applies a deceleration to avoid collisions.  The deceleration is fixed at ~5m/s2 to remain within the acceleration constraints of the project.

## Boundary Conditions
The car initially starts at rest in lane 1(!).  Therefore setting the speed to 50mph would show as an instantaneous acceleration, violating the jerk constraint.  Therefore speed is started at 0mph and gradually increased.  So long as the lane ahead of the car is clear of vehicles, the car will accelerate at ~5m/s2 until the 50mph speed limit is reached (the max velocity is set at 49.5mph, to avoid risk of breaching the speed limit constraint).

## Lane Selection and Scoring (Cost Function)
If a car is detected in the current lane, the car will slow to avoid a collision, but will also seek an alternative lane.  This is done by a simple lane-scoring based on a weighted sum of distances to the cars in each lane.  

Each car in a lane contributes to the cost function for that lane by:
`(30 - distance_to_car) * weight`
where the weight is a negative value

Therefore a car which is nearby will have a large negative weight.  

The preferred lane is the lane with the highest score.  The score for an empty lane will be 0 (no vehicles), with the right-most lane being preferred in the event of a tie.  If no lane is empty, the lane with the most distant car will be chosen.

If the preferred lane is 2 lanes away from current (i.e. a move from 0 to 2 or 2 to 0 is chosen), the preferred lane will be set to lane 1 as an intermediate state (plotting a spline across 2 lanes will likely break the jerk constraint).  The preferred lane will be re-selected after this first lane change is completed.

## Finite States
The car has 5 possible states:
1. Driving ahead
2. Waiting to move left
3. Waiting to move right
4. Moving left
5. Moving right

The finite state machine starts off with the car driving ahead in the current lane and remains so until an obstructing car is found.

At this point, the scoring process will select a lane and set the state to waiting to move left or right.

The adjacent lane is then checked for obstructions - if there are none, the state is moved on to moving left/right.  Otherwise the state remains waiting and the car continues driving in the current lane until the lane change is safe.

If the lane change is safe, points are plotted to make the chosen maneouver.

Once a lane change is complete, the car state is set to driving in the current lane once more, until an obstruction is found.

Note: it may be the case that there is lots of traffic in the chosen lane - therefore the car will remain in the waiting state for only approx. 5 seconds (250 points consumed) before scrapping that plan and re-scoring the lanes.

## Output
This path planning implementation drives the car smoothly around the test track, remaining within the speed/acceleration/jerk constraints and avoiding other cars.  The car is occasionally slowed due to being boxed-in by other cars, which could be improved by some of the enhancements suggested below.



## Potential Enhancements
The following are potential enhancements to the path planning process:
* Variable acceleration and deceleration - these are set to a fixed rate, but could be tuned to the speed of vehicles in front of and to the side of the car.  For example this could be a proportional controller using the speed of the car ahead and distance to the car ahead as inputs, outputting a smoother deceleration for the path planner.
* In general it is assumed that other cars will continue at a fixed speed in their current lane.  In the real world this will not be the case and a predictive solution would be an improvement over this simple assumption.
* The lane scoring cost function is simple and effective, but could be improved.  For example, this function does not consider the speeds of cars in adjacent lanes and may select a lane with a slow car rather than a lane with a faster, but nearby, car - this second lane would make for better progress.
* The lane scoring function only looks at cars which are within the 30m horizon used for speed detection.  Sometimes this means that the car prefers a lane which will shortly be occupied, over a lane which is clear for some distance.  This could be addressed by extending the horizon for the cost function.

# Udacity Instructions Follow:

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
