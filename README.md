# 2D Omni Robot Navigation
**This repository belongs to HKU RoboMaster ICRA Challenge Team, serves as material for 2020 DJI ICRA AI Challenge.**
## Introduction:
Efficient navigation in narrow space and facing multiple dynamic obstacles has always been a problem in robotics by the nature that the environment is often complex and onboard computational power is limited. 

There has been many attempts to tackle the problem from different perspectives. Traditional optimal planning method including A* and Dijkstra algorithm can guarantee an optimal solution but time the complexity in high dimension and resolution grid map is not realistic in small platform like wheeled robot. Probabilistic method has been invented by Lydia E. Kavraki and branched out to be probabilistic roadmap method and rapid exploring random tree(RRT). **These method accelerate the planning problem significantly in high dimensional configuration space.However,since these methods are only probabilisticly complete and doesn't guarantee optimal, naively apply these methods in motion planning problem is likely to fail when planning through narrow space or suffers from very wiggly trajectories.Heuristic Sampling around obstacle edges prefers narrow space planning, but tend to yield wiggly result in a more general map** 

Based on this observation  we combine the idea of parallel computation, heuristic sampling rrt and developed an frame work based on RRT and is able to plan efficiently under all kinds of maps and is able to plan trajectories even in very narrow space. To make the Navigation complete we further integrated and modified  Reciprocal Velocity Obstacle Avoidance(RVO),trick of conditional replanning in our frame work in order to avoid collision with other robots or dynamic obstacles.
## Installation and testing:
### Environment:
- Ubuntu 18.04
- Intel Xeon E5-2600v4 x 2
### Dependency:
```
sudo apt install mpich
pip install opecv-python matplotlib numpy mpi4py
```
### Usage
#### Isolated RVO test:
```
python3 local_rvo_example.py
```
#### Isolated Original RRT test:
```
python3 rrt_example.py <icra_new/narrow>
```
#### Isolated Our RRT test:
```
mpiexec -n 10 python3 bias_rrt_example.py <icra_new/narrow>
```
#### Navigation Fully functional test:
```
mpiexec -n 10 python3 main.py icra_new
```
- Select a start point on the displayed map.
- Select a goal point on the displayed map.

