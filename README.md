# Multiple_Turtlebot3_SLAM_RRT_Path_Planning
In this Project, Five Turtlebot3 has been spawned in a custom world and SLAM (Simultaneous Localization And Mapping) will done by RRT (Rapidly Exploring Random Tree) path planning algorithm.

## SLAM - Simultaneous Localization And Mapping

Simultaneous localization and mapping (SLAM) is the computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of an agent's location within it. While this initially appears to be a chicken-and-egg problem there are several algorithms known for solving it, at least approximately, in tractable time for certain environments. Popular approximate solution methods include the particle filter, extended Kalman filter, Covariance intersection, and GraphSLAM. SLAM algorithms are based on concepts in computational geometry and computer vision, and are used in robot navigation, robotic mapping and odometry for virtual reality or augmented reality.

### Localization

Robot localization is the process of determining where a mobile robot is located with respect to its environment. Localization is one of the most fundamental competencies required by an autonomous robot as the knowledge of the robot's own location is an essential precursor to making decisions about future actions. 

Robot localization provides an answer to the question: Where is the robot now? A reliable solution to this question is required for performing useful tasks, as the knowledge of current location is essential for deciding what to do next.

In this Project ```Particle Filter``` Localiation algorithm has been implied.

#### Particle Filter Localization

Monte Carlo localization (MCL), also known as particle filter localization, is an algorithm for robots to localize using a particle filter.Given a map of the environment, the algorithm estimates the position and orientation of a robot as it moves and senses the environment.The algorithm uses a particle filter to represent the distribution of likely states, with each particle representing a possible state, i.e., a hypothesis of where the robot is.The algorithm typically starts with a uniform random distribution of particles over the configuration space, meaning the robot has no information about where it is and assumes it is equally likely to be at any point in space. Whenever the robot moves, it shifts the particles to predict its new state after the movement. Whenever the robot senses something, the particles are resampled based on recursive Bayesian estimation, i.e., how well the actual sensed data correlate with the predicted state. Ultimately, the particles should converge towards the actual position of the robot

Consider a robot with an internal map of its environment. When the robot moves around, it needs to know where it is within this map. Determining its location and rotation (more generally, the pose) by using its sensor observations is known as robot localization.

Because the robot may not always behave in a perfectly predictable way, it generates many random guesses of where it is going to be next. These guesses are known as particles. Each particle contains a full description of a possible future state. When the robot observes the environment, it discards particles inconsistent with this observation, and generates more particles close to those that appear consistent. In the end, hopefully most particles converge to where the robot actually is.

Particle Filter Localization ALgorithm:

![Demo Image](https://github.com/kri98nan/Multiple_Turtlebot3_SLAM_RRT_Path_Planning/blob/main/Algorithm-of-particle-filter.png)

### Mapping

Robotic mapping is a discipline related to computer vision and cartography. The goal for an autonomous robot is to be able to construct (or use) a map (outdoor use) or floor plan (indoor use) and to localize itself and its recharging bases or beacons in it. Robotic leg is that branch which deals with the study and application of ability to localize itself in a map / plan and sometimes to construct the map or floor plan by the autonomous robot.

Evolutionarily shaped blind action may suffice to keep some animals alive. For some insects for example, the environment is not interpreted as a map, and they survive only with a triggered response. A slightly more elaborated navigation strategy dramatically enhances the capabilities of the robot. Cognitive maps enable planning capacities and use of current perceptions, memorized events, and expected consequences.

In this Project Occupancy Grid Mapping has been implied.

#### Occupancy Grid Mapping

Occupancy Grid Mapping refers to a family of computer algorithms in probabilistic robotics for mobile robots which address the problem of generating maps from noisy and uncertain sensor measurement data, with the assumption that the robot pose is known.

The basic idea of the occupancy grid is to represent a map of the environment as an evenly spaced field of binary random variables each representing the presence of an obstacle at that location in the environment. Occupancy grid algorithms compute approximate posterior estimates for these random variables.

There are four major components of occupancy grid mapping approach. They are:

    1)Interpretation
    2)Integration
    3)Position estimation
    4)Exploration
 
Occupancy Grid Mapping Algorithm:

![OCG_image](https://github.com/kri98nan/Multiple_Turtlebot3_SLAM_RRT_Path_Planning/blob/main/OCG.jpg)

## RRT Path Planning
A rapidly exploring random tree (RRT) is an algorithm designed to efficiently search nonconvex, high-dimensional spaces by randomly building a space-filling tree. The tree is constructed incrementally from samples drawn randomly from the search space and is inherently biased to grow towards large unsearched areas of the problem. Algortihm for RRT as follows 

RRT Algorithm 
```
Algorithm BuildRRT
    Input: Initial configuration qinit, number of vertices in RRT K, incremental distance Δq)
    Output: RRT graph G

    G.init(qinit)
    for k = 1 to K do
        qrand ← RAND_CONF()
        qnear ← NEAREST_VERTEX(qrand, G)
        qnew ← NEW_CONF(qnear, qrand, Δq)
        G.add_vertex(qnew)
        G.add_edge(qnear, qnew)
    return G
```
An RRT grows a tree rooted at the starting configuration by using random samples from the search space. As each sample is drawn, a connection is attempted between it and the nearest state in the tree. If the connection is feasible (passes entirely through free space and obeys any constraints), this results in the addition of the new state to the tree. With uniform sampling of the search space, the probability of expanding an existing state is proportional to the size of its Voronoi region. As the largest Voronoi regions belong to the states on the frontier of the search, this means that the tree preferentially expands towards large unsearched areas.

The length of the connection between the tree and a new state is frequently limited by a growth factor. If the random sample is further from its nearest state in the tree than this limit allows, a new state at the maximum distance from the tree along the line to the random sample is used instead of the random sample itself. The random samples can then be viewed as controlling the direction of the tree growth while the growth factor determines its rate. This maintains the space-filling bias of the RRT while limiting the size of the incremental growth.

RRT growth can be biased by increasing the probability of sampling states from a specific area. Most practical implementations of RRTs make use of this to guide the search towards the planning problem goals. This is accomplished by introducing a small probability of sampling the goal to the state sampling procedure. The higher this probability, the more greedily the tree grows towards the goal.


## Software Requirmenets
Ubuntu 16.04 LTS

ROS Kinetic

Gazebo

Rviz

## Execution
### Setting up workspace
```
cd catkin_ws (optional)
git clone https://github.com/kri98nan/Multiple_Turtlebot3_SLAM_RRT_Path_Planning.git
cd Multiple_Turtlebot3_SLAM_RRT_Path_Planning
catkin_make
```
### Defining the turtlebot and sourcing
```
export TURTLEBOT3_MODEL=burger
source devel/setup.bash
```
### Setting up Gazebo Environment
```
gnome-terminal --tab --title="test" --command="bash -c 'roslaunch turtlebot3_gazebo multi_turtlebot3.launch;source devel/setup.bash; export TURTLEBOT3_MODEL=burger; $SHELL'"
```
### Establishing Gmapping
Note: Execute this command only after the gazebo is fully loaded
```
bash gmapping.sh
```
### RRT Path Planning
```
gnome-terminal --command="bash -c 'roslaunch rrt_exploration navigation.launch;source devel/setup.bash; export TURTLEBOT3_MODEL=burger; $SHELL'"
```

After all the programs loaded publish the points in four corners of the map and one inside the map to start RRT exploration.

## Working Video
https://youtu.be/8phYOpBw0_I

The project is still under development. Any changes made will be updated in this repository.
