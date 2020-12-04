# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

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

## Path Planner

The path planner of the self-driving car (SDV) uses a combination of its current and future positions and speeds and the sensor fusion information to get the relative positions of the cars around it to figure out when it's safe and prudent to make lane changes.
When there aren't any cars ahead in the SDV's lane it will continue cruising at a target speed of 49.5 mph (main.cpp line 17) to avoid going above the speed limit of 50 mph. On the other hand, when the SDV detects (from the sensor fusion inputs) cars ahead in its lane, it will use a cost function to determine whether a left or right lane shift is advisable. In the following sections we cover the basic functionality and details of the path planner.

### Basic Motion
Before implementing the more advanced path planning logic for our SDV, we first have to solve the basic problem of getting the car moving along the highway in the direction of traffic. This was achieved by incrementing the `s` coordinate of the SDV (in Frenet coordinate system) in such a way that it's making forward progress with the desired speed of just below 50 mph.

### Spline Interpolation
Simply incrementing the SDV's `s` coordinate would result in jerky movement since the waypoints are spaced apart and linear interpolation would create sharp turns at each waypoint. In order to make the car follow a smooth trajectory, we use spline interpolation (main.cpp line 350) between evenly spaced out points along the highway. The spacing between the points (main.cpp line 326) was hand-tuned in such a way so as to keep the car in its lane when it's not making any lane changes (the larger the spacing the more the car tends to drift within its lane due to the approximate nature of interpolation) and to avoid jerky lane shifts that exceed the maximum allowed acceleration (the smaller the spacing the more aggressive the lane shift maneuvers appear). In fact, we switch between two different spacings for lane keeping and lane shifting (main.cpp lines 18-19). Here we also use a counter variable (`last_lane_shift`) that prescribes the amount of cycles the car should keep the lane change spacing. The same variable is also used to make the car wait a few cycles between consecutive lane changes (otherwise, the double lane change can be too abrupt and result in excessive acceleration).

### Collision Avoidance
Next in order to have the car successfully drive in traffic, we need to at least consider the cars driving in our lane ahead of us and slow down if we get too close. This is achieved by iterating through all the detected vehicles provided in the sensor fusion input and checking that the distance to the closest car in our lane ahead of us is no less than the safe distance of 30m (main.cpp lines 32, 263).
In theory, this alone would be enough to make the car go infinitely around the track without issues albeit it would be going less than its target speed when stuck behind a slow car. Thus, the SDV needs a way of taking actions that would keep it safe (avoiding collisions) and at the same time keep it moving as close to the target speed as possible. In order to address this tradeoff between speed and safety, we need to design a cost function that would take several inputs into consideration.

### Cost Function

Here we detail the logic that goes into computing the cost function which helps the SDV determine the best course of action.
First, let's list the constraints that we need to satisfy and the actions that we can choose from.

Constraints:
1. Avoid collisions
2. Drive as close to the target speed (of 49.5mph) as possible without violating speed limit (of 50mph)

Available Actions:
1. Stay in the current lane (action: stay)
2. Change lanes to the left (action: left)
3. Change lanes to the right (action: right)
4. Accelerate
5. Slow down

#### Cost function design
In order to put the above two constrains together we need a way to express them in similar units. One way of doing it is by considering the space available for the SDV to advance forward in a given lane.
First, let's separate the lane choice actions (1-3) and accelerate actions (4-5) separately by only considering actions 4-5 when we are staying in the current lane.
Thus for each of the lane choice actions (stay, left, right), we need to compute how much space is available for the SDV to advance forward and to shift to the lane with the most available space to move ahead.
Besides considering the cars ahead for deciding whether it's worth changing lanes, we also need to consider the cars that are next to us or behind us to in order to avoid collisions.
For example, if there is a car to our right going slightly behind us, then we shouldn't consider changing lanes to the right even if the right lane is empty otherwise. Such undesirable dangerous maneuvars can be encoded in the cost function by setting it to infinity (lines 234, 242) when the available space is 0 (lines 182, 195).
In practice it wasn't enough to consider only the distances to the cars around us, since a car that's behind us could be moving much faster than us and if we were to suddenly change lanes, then this could lead to a collision. In the code we are using both the current position and the car's relative velocity to estimate our distance to this vehicle 1 second into the future (line 164).
Now that we have considered the hard constraint of collision avoidance, we can get to the more subtle constraint of trying to keep the target speed.
For this we want our car to be able to change lanes when it expects to have more space to drive freely (i.e. without having to slow down behind traffic) in a different lane. At first I implemented logic that would only look for the vehicles in the neighboring lanes and decrease the action cost if there was more space to the closest car ahead (lines 236, 244). However, this lead to situations where the car would get easily boxed in (in leftmost or rightmost lanes) by two slow moving cars although there was a way to go freely ahead by changing two lanes. At this point, I added extra logic that would also look at the skip lanes to decide whether a (double) lane shift is desirable (lines 228, 236). Upon tweaking some of the parameters to make the SDV reasonably aggressive while avoiding collisions, I was able to achieve good performance. The SDV is able to make decisions that help it advance forward by weaving through the traffic fairly safely. The two important parameters that control how aggressive the car is are `min_dist` and `clear_dist` (main.cpp lines 27, 30), which are used to decide whether it's safe for the SDV to change into the nearby lane by considering the min distance to the cars in that lane at the current moment (`min_dist`) and 1 second into the future (`clear_dist`). Empirically these parameters were set to 10m and 20m respectively (smaller values would lead to occasional collisions when the SDV tried to perform lane shifts in tight spaces).
One interesting thing that can obvserved from the SDV's behavior is that it sometimes appears to shift lanes for no reason. This, however, isn't true in most cases. Since the resolution of the simulator is lower than the SDV's sensor information, the SDV tends to make lane shifts in advance if it sees that its current lane provides less free runway than another lane. One could argue that such early lane shifts aren't necessary and in some cases could even lead to changing lanes too often (e.g. if other cars in the traffic tended to change lanes often). However, this doesn't appear to be an issue since most cars in the simulator don't change lanes.

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

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

