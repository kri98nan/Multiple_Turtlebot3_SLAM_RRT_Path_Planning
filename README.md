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

