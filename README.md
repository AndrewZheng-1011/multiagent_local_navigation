# Multiagent Local Navigation
In this work, I analyze classical multi agent local navigation techniques. The following algorithms
were simulated and compared with one another:
- Social Force Model (SFM) (from "Simulating Dynamical Features of Escape Panic" by Dirk Helbing, et. al.)
- Predictive Time to Collision (TTC) Algorithms
- Velocity Obstacles Algorithm (VO)


## Review of each Algorithm
We review the nature of each of the algorithm and later compare the algorithm in a multi-agent navigation
scenario ranging from 3 or 8 agents.

### Social Force Model
The SFM is a reactive based motion planning algorithm in which the crux of the algorithm relies on the attractive force
from the goal and the interaction force from neighboring agents/pedestrians (i.e. distance between agents). The authors
modeled the interaction force by 3 core components:
1. Repulsive interaction force (i.e. force that repels the closer the agent gets to another agent) denoted by $A_i