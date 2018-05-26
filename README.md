# CarND-Path-Planning-Project REPORT
Self-Driving Car Engineer Nanodegree Program
Student Name:Kushal Virupakshappa
   
### Simulator.
Simulator has been provided by UDACITY and can be found at [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### RUBICS based observations
1. Compilation: The code can compile and run without errors as per the current version.
2. Driving Without Incident : As per ten tests conducted the car could drive 4.32 miles or above without incident. The inital start time is bit slow.
3. The car doesn't undergo jerks or unwanted acceleration and can drive under speed limit. In very rare and unforeseeable circumstances there may be jerks.
4. It stays in Lane and avoids collusion for upto 4.32 miles as observed.
5. Car is able to change lanes.


## Model Details.

The main focus of this project being Behavioural Planning and Trajectory Generation.

### Behavioural Planning

1. Three states have been incorporated Keep Lane, Prepare Lane change Left and Prepare Lane change right.
2. Keep Lane State: Responsible for generating trajectories drivable in the lane in which the car is present.
3. Prepare Lane change Left: Responsible of Checking if there is a lane change possible to the left and if yes lane change trajectory is generated else the trajectory for the current lane is generated.
4. Prepare Lane Change Right: Same explaination as in Prepare Lane change Left state but the direction is right.
5. The transition is from Keep Lane to Prepare Lane Change Right is there is a vehicle ahead.
6. If the Lane change to left fails the lane change to right happens and finally if lane change is not possible and if the velocity is too low then Keep lane is chosen.
7. If lane changes in respective lane change state happens then also the state is changed to Keep lane.


### Trajectory Generatioin

1.  Frenet Co-ordinates are used select the next way points.
2.  SPLINE algorithm is used to interpolate the co-efficients needed for calculating x and y co-ordinates.
3.  Library for SPLINE algorithm has been used as dicussed in the course.


## Results.
![Alt text](Path_Planning_Project/comp.jpg?raw=true "Snapshot of completing ")
