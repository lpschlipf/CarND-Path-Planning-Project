# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
The goal of this project is to build a path planner that creates smooth, safe trajectory for cars driving on a three lane highway. This project generates a trajectory that aims to drive on the middle lane while overtaking slower vehicles to the left or right, while avoiding collisions and staying inside "smooth" trajectory parameters.

![image](https://user-images.githubusercontent.com/52862737/156771163-6017b7a2-6ac5-4f1b-8ce9-1da1e363e349.png)

Above shown is the architecture of the self driving vehicle software as a whole. We will use data from the sensor fusion and localisation to predict vehicle positions in order to decide on the optimal behavior. This behaviour will then generate a trajectory that is input to the motion controller in order to steer the vehicle.

## Prediction
Predictions come directly from the sensor fusion [id, x, y, vx, vy, s, d]. Here, x and y values are in global map coordinates, and the vx, vy values are the velocity components in global coordinates. Using this information, we can judge if we are too close to another car in the ego lane by checking if it's position is within 30 meters in front of the ego vehicle. Additionally we can decide if the lanes to the left or the right can be considered free, by checking whether any vehicle is within 30 meters to the front or the back of the vehicle in the respective lane.

## Behavior
According to the prediction results, we decide what the best behavior is.
- when there is no car in proximity on the ego lane, keep accelerating with 0.25m/h^2 until we have reached the target speed of ~ 50mph.
- When there is a car in close proximity do (order is relevant):
   - Change lane to the left if possible and lane is free.
   - Change lane to the right if possible and lane is free.
   - Otherwise decelerate with 0.25 m/h^2 to avoid a collision with target vehicle.

## Trajectory
Depending on the above behaviour, a trajectory is built up that consists of 50 points. For this the remaining points that are left from the last cycle are kept and points are appended at the end according the optimal behavior. To generate the points spline fitting is used from an external library (from Project Q&A of Aaron Brown). For easier calculations the global trajectory points are transformed to the ego coordinate system so that the first point is at the origin (position of the ego vehicle) and the trajectory has an angle of zero degrees there. Now spline points are generated under the consideration that the ego vehicle can travel at the target velocity. Finally the trajectory points are transformed back to the global coordinate system.
