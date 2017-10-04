# Jerk-Optimization-of-Timed-Elastic-Band-Algorithm

## Introduction
[Time elastic band algorithm(Rosmann,2012)](http://ieeexplore.ieee.org/abstract/document/6309484/) solves the local planning problem using a weighted multi-objective optimization framwork. Then the optimization is transformed into a hypergraph and solved with [G2O](http://ieeexplore.ieee.org/abstract/document/5979949/). The TEB algorithm is avalible as a ROS package developed by Rosmann, you can download the whole package[here](https://github.com/rst-tu-dortmund/teb_local_planner).
To make the trajectory produced by the TEB algorithm less jerky, an item considering jerk (derivative of acceleration) is added to the original optimazation framwork.

## Algorithm
The modified hypergraph is shown below. 
<img width="600" height="250" src="https://github.com/ZRZheng/Jerk-Optimization-of-Timed-Elastic-Band-Algorithm/blob/master/HyperGraph.PNG">

## Requirement
* [ROS kinetic](http://wiki.ros.org/kinetic/Installation)
* Download the original [teb package](https://github.com/rst-tu-dortmund/teb_local_planner)
* Add the edge_jerk.h file in the /include/teb_local_planner/g2o_types
* Replace the optimal_planner.h and teb_config.h in the /include/teb_local_planner
* Replace the optimal_planner.cpp and teb_config.cpp in the /src
