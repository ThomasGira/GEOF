# GEOF: Good Enough Odometry Frame

## Description
I'm trying to be better so this is my attempt at writing 2d slam. I'm actually not going to start off by doing this and instead am trying to write a neural net that can drive a car around a track. I think a lot of the tool that I will develop while doing this will lend well to creating an infrastructure to develop slam on. The key components for this project will be a visualizer, a simulator and the neural net.

## Visualizer
The visualizer will use open cv and will simply need to display a map, a car and sensor information.

Functions:
- Draw map
- Draw car
- Draw Lidar


## Simulator
The simulator will be pretty basic, using linear dynamics.
It will contain the map, the vehicle and sensors.

Functions:
- visualize sim
- store robot position


### The Map
The map will be relatively simple. Just a binary occupied or not. This will need to handle collision for oboth the vehicle and for any sensors (ie. lidar)

### Vehicle
Input:
 - linear acceleration
 - Rotational acceleration

Output:
 - translation

Stored info:
 - Velocity
 - sesnor data

Functions:
 - Update velocity
 - Set acceleration
 - Check collisions

 ### Sensors
 The only sensor that I will have currently is a multi beam planar lidar.

Parameters:
 - Range
 - Number of beams
 - FOV

Functions:
 - Check beam

Sored info:
 - Collision points of beams

### Utils:
 - vector:
    - Contains:
        - Pose x 
        - Pose y
        - Theta
    - Functions:
        - world2robot
        - robot2world