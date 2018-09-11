# Introduction
My robot has been kidnapped and transported to a new location. Luckily it has `a map` of this location, a (noisy) `GPS estimate of its initial location`, and lots of `noisy sensor` and control data.
To Save this robot, I implemented a `2 dimensional particle filter` in C++. This filter is given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data.

__What I have: Initial Localization from GPS, Observation, Velocity and Yaw Rate Data with Noise__ 
* Observation data are the measured distance of landmarks

__What my GOALs are: Implementing the Particle Filter to Localize the Robot__
1. Initialize `50 particles` based on x, y, theta from GPS with random Gaussian noise
2. In each time step, predict states of particles with velocity and yaw_rate input, using `CTRV`(Constant Trun Rate and Velocity) model
3. Associate data with observations through `nearest neighbor` method, then update the particles weights by calculating the product of each measurement's `Multivariate-Gaussian probability density`
4. Resample the particles based on the new weights

>**Here is the flow chart of particle filter**
![Particle Filter Flow Chart](https://github.com/WenHsu1203/Kidnapped-Vehicle-Project/blob/master/Photo/Particle%20Filter.png?raw=true)

# Result
In the simulator you can see the path that the robot drives along with all of its `landmark measurements(⨷) `represented by `green lines`. The estimation of the position from particle is the `blue circle` around the robot

__*Demo Video☟*__
[![Video](http://img.youtube.com/vi/0rbb8DnsY6g/0.jpg)](http://www.youtube.com/watch?v=0rbb8DnsY6g "Particle Filter")

# Download
Simulator can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)


#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id




