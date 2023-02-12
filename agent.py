# ageny.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the author.
# 
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)
#

import numpy as np
import math
from abc import ABC, abstractmethod

""" 
    Abstract class for agents
"""
class AbstractAgent(ABC):

    def __init__(self, inputParameters):
        """ 
            Takes an input line from the csv file,  
            and initializes the agent
        """
        self.id = int(inputParameters[0]) # the id of the agent
        self.gid = int(inputParameters[1]) # the group id of the agent
        self.pos = np.array([float(inputParameters[2]), float(inputParameters[3])]) # the position of the agent 
        self.vel = np.zeros(2) # the velocity of the agent
        self.goal = np.array([float(inputParameters[4]), float(inputParameters[5])]) # the goal of the agent
        self.prefspeed = float(inputParameters[6]) # the preferred speed of the agent
        self.gvel = self.goal-self.pos # the goal velocity of the agent
        self.gvel = self.gvel/(np.sqrt(self.gvel.dot(self.gvel )))*self.prefspeed       
        self.maxspeed = float(inputParameters[7]) # the maximum sped of the agent
        self.radius = float(inputParameters[8]) # the radius of the agent
        self.atGoal = False # has the agent reached its goal?
     
    @abstractmethod
    def computeAction(self, neighbors=[]):
        """
            Performs a sense and act simulation step.
        """
        pass

    @abstractmethod    
    def update(self, dt):
        """
            Updates the state of the character, given the time step of the simulation.
        """
        pass

""" 
    Agent class that implements the 2000 Social Force Model by Helbing et al.
    See the paper for details  
"""
class SFMAgent(AbstractAgent):

    def __init__(self, inputParameters, goalRadius=1, dhor = 10, ksi=0.5, A=2000, B=0.08, k=1.2e5, kappa=2.4e5, mass = 1, maxF = 10):
        """ 
           Initializes the agent
        """
        super().__init__(inputParameters)
        self.atGoal = False # has the agent reached its goal?
        self.goalRadiusSq = goalRadius*goalRadius # parameter to determine if agent is close to the goal
        self.dhor = dhor # the sensing radius
        self.ksi = ksi # the relaxation time used to compute the goal force
        self.A = A # scaling constant for the repulsive force
        self.B = B # safe distance that the agent prefers to keep
        self.k = k # scaling constant of the body force in case of a collision
        self.kappa = kappa # scaling constant of the sliding friction foce in case of a collision
        self.F = np.zeros(2) # the total force acting on the agent
        self.maxF = maxF # the maximum force that can be applied to the agent
        self.mass = mass


    def computeAction(self, neighbors=[]):
        """ 
            Using SFM algorithm, computes a reactive force for the respective agent

            Inputs:
            -------
            neighbors: List
            List of neighboring agent objects    
        """
        f_avoid = 0

        # 1a) Compute force due to goal
        f_goal = self.mass*(self.gvel - self.vel)/self.ksi

        # 1b) Determine all neighbors of agent that are less than d_H away from agent
        for neighbor in neighbors:
            # Compute if neighbors are within the agent's sensing radius
            if self.detectCloseNeighbor(neighbor):
                # Estimate corresponding repulsive force that is exerted to the agent
                f_avoid += self.computeAvoidanceForce(neighbor)


        if not self.atGoal:
            self.F = f_goal + f_avoid
        else:
            self.F = np.zeros(2)
            

    def update(self, dt):
        """ 
            Code to update the velocity and position of the agents.  
            as well as determine the new goal velocity 
        """
        if not self.atGoal:
            # 2) Sum of avoidance force, saturate force to max
            if np.linalg.norm(self.F) > self.maxF:
                self.F = self.maxF*(self.F/np.linalg.norm(self.F))
            self.vel += self.F/self.mass*dt     # update the velocity
            self.pos += self.vel*dt   #update the position
        
            # compute the goal velocity for the next time step. Do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq: 
                self.atGoal = True  # goal has been reached
            else: 
                self.gvel = self.gvel/np.sqrt(distGoalSq)*self.prefspeed  

    def detectCloseNeighbor(self, neighbor):
        """
        Detect if neighbor within close proximity of the agent

        Inputs:
        --------
        neighbor : Object
            Neighbor object

        Outputs:
        ---------
        Boolean
            Return if neighbor is close
        """

        # Return true if:
        #   1) Neighbor is within sensing radius
        #   2) Distance from agent to neighbor is not itself (Due to neighbors containing all agents including itself)

        d_AB = np.linalg.norm(self.pos - neighbor.pos) - self.radius - neighbor.radius
        if self.dhor >= d_AB and self.id != neighbor.id and not neighbor.atGoal:
            # print("Detected close obstacle at location [%0.2f, %0.2f] w.r.t. self pos [%0.2f, %0.2f]"
            # % (neighbor.pos[0], neighbor.pos[1], self.pos[0], self.pos[1]))
            return True
        return False

    def computeAvoidanceForce(self, neighbor):
        """
        Compute a reactive avoidance force (eq. 2 of
        "Simulating dynamical features of escape panic" by
        Dirk Helbing, et al)
        
        Inputs:
        -------
        neighbor : Object
            Neighbor agent object
        
        Outputs:
        --------
        f_avoid : ndarry
            Force vector to avoid neighbor k
        """
        f_repulsive, f_body, f_sliding_friction = 0, 0, 0

        # Reactive based approach
        d_AB = np.linalg.norm(self.pos - neighbor.pos)
        n_AB = (self.pos - neighbor.pos)/d_AB
        r = self.radius + neighbor.radius # Scalar

        # Compute repulsive
        f_repulsive = self.A*math.exp((r - d_AB)/self.B)*n_AB

        g_x = lambda r, d: max(r-d, 0)

        # Compute body
        f_body = self.k*g_x(r,d_AB)*n_AB

        # Compute sliding friction force
        t_AB = np.array(-n_AB[1], n_AB[0]) # Tangential vector | Limits to 2d
        delta_v = np.dot((neighbor.vel - self.vel), t_AB)
        f_sliding_friction = self.kappa*g_x(r, d_AB)*delta_v*t_AB

        return f_repulsive + f_body + f_sliding_friction


""" 
    Agent class that implements the TTC force-based approach  
"""
class TTCAgent(AbstractAgent):

    def __init__(self, inputParameters, goalRadius=1, dhor = 10, ksi=0.5, timehor=5, epsilon=0, maxF = 10, mass = 1):
        """ 
           Initializes the agent
        """
        super().__init__(inputParameters)
        self.atGoal = False # has the agent reached its goal?
        self.goalRadiusSq = goalRadius * goalRadius # parameter to determine if agent is close to the goal
        self.dhor = dhor # the sensing radius
        self.ksi = ksi # the relaxation time used to compute the goal force
        self.timehor = timehor # the time horizon for computing avoidance forces
        self.epsilon = epsilon # the error in sensed velocities
        self.F = np.zeros(2) # the total force acting on the agent
        self.maxF = maxF # the maximum force that can be applied to the agent
        self.mass = mass # Mass of the agent


    def computeAction(self, neighbors=[]):
        """ 
        Using TTC algorithm, computes a predictive reactive force for the respective agent

        Inputs:
        -------
        neighbors: List
            List of neighboring agent objects    
        """

        '''
        Compute TTC w/ quadratic formula: 
        Assumptions:
        1) Agents have linear motion
        2) Agents maintain their current velocities
        '''
        # Initialize variable
        f_avoid = 0 # Initialize avoidance force to be 0

        # 1a) Compute force due to goal
        f_goal = self.mass*(self.gvel - self.vel)/self.ksi
    
        # 1b) Determine all neighbors of agent that are less than d_H away from agent
        for neighbor in neighbors:
            # Compute if neighbors are within the agent's sensing radius
            if self.detectCloseNeighbor(neighbor):
                # Estimate corresponding repulsive force that is exerted to the agent
                f_avoid += self.computeAvoidanceForce(neighbor)
        
        
        if not self.atGoal:
            self.F = f_goal + f_avoid
            # self.F = np.zeros(2)
        else: # If at goal, no force
            self.F = np.zeros(2)
            

    def update(self, dt):
        """ 
            Code to update the velocity and position of the agents.  
            as well as determine the new goal velocity 
        """
        if not self.atGoal:
            # 2) Sum of avoidance force, saturate force to max
            if np.linalg.norm(self.F) > self.maxF:
                self.F = self.maxF*(self.F/np.linalg.norm(self.F))
            self.vel += self.F/self.mass*dt     # update the velocity
            self.pos += self.vel*dt   #update the position
        
            # compute the goal velocity for the next time step. Do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq: 
                self.atGoal = True  # goal has been reached
            else: 
                self.gvel = self.gvel/np.sqrt(distGoalSq)*self.prefspeed 

    def detectCloseNeighbor(self, neighbor):
        """
        Detects if neighbor within close proximity of the agent

        Inputs:
        --------
        neighbor : Object
            Neighbor object

        Outputs:
        ---------
        Boolean
            Return if neighbor is close
        """

        # Return true if:
        #   1) Neighbor is within sensing radius
        #   2) Distance from agent to neighbor is not itself (Due to neighbors containing all agents including itself)

        d_AB = np.linalg.norm(self.pos - neighbor.pos) - self.radius - neighbor.radius
        if self.dhor >= d_AB and self.id != neighbor.id and not neighbor.atGoal:
            # print("Detected close obstacle at location [%0.2f, %0.2f] w.r.t. self pos [%0.2f, %0.2f]"
            # % (neighbor.pos[0], neighbor.pos[1], self.pos[0], self.pos[1]))
            return True
        return False
    

    def computeTTC(self, neighbor):
        """
        Compute time to collision (TTC)
        
        Inputs:
        --------
        neighbor : Object
            Neighboring agents

        Outputs:
        ---------
        tau : Float
            Time to collision
        """
        # Compute parameters for TTC
        x = self.pos - neighbor.pos
        v = self.vel - neighbor.vel
        r = self.radius + neighbor.radius # Scalar

        a = np.dot(v, v) - self.epsilon**2
        b = np.dot(x, v) - self.epsilon*r
        c = np.dot(x,x) - r**2
        d = b**2 - a*c

        if c < 0: # Agents colliding
            print("Agent colliding!!!")
            return 0

        if b > 0: # Agents diverging
            return float('inf')

        if d <= 0: # No collision
            return float('inf')

        # Since b < 0 and b^2 >> ac -> Then smallest positive root is of the form:
        tau = c/(-b + np.sqrt(d))
        if tau < 0:
            return float('inf')
        return tau
        
    def computeAvoidanceForce(self, neighbor):
        """
        Compute an avoidance force of the form:
            F_avoid = max(d_H - d_AB, 0)/d_AB^n_hat
        
        Inputs:
        -------
        neighbor : Object
            Neighbor agent object
        
        Outputs:
        --------
        f_avoid : ndarry
            Force vector to avoid neighbor k
        """

        # Initialize parameters
        x = self.pos - neighbor.pos
        v = self.vel - neighbor.vel
        r = self.radius + neighbor.radius # Scalar


        # Predictive force-based approach
        f_avoid = 0

        tau = self.computeTTC(neighbor)
        if tau == float('inf'): # No collision -> No avoidance force
            return f_avoid

        n_AB = (x+v*tau)/np.linalg.norm(x+v*tau) # Can now calculate normal when tau != inf
        if tau == 0: # If collision, use max force
            # f_avoid = self.maxF*n_AB
            return f_avoid
        f_avoid = max(self.timehor - tau, 0)/tau*n_AB

        return f_avoid


""" 
    Agent class that implements a sampling-based VO approach  
"""
class VOAgent(AbstractAgent):

    def __init__(self, inputParameters, goalRadius=1, dhor = 5, epsilon=0, N = 100, alpha = 1, beta = 1, gamma = 5):
        """ 
           Initializes the agent
        """
        super().__init__(inputParameters)
        self.atGoal = False # has the agent reached its goal?
        self.goalRadiusSq = goalRadius*goalRadius # parameter to determine if agent is close to the goal
        self.dhor = dhor # the sensing radius
        self.epsilon = epsilon # the error in sensed velocities
        self.vnew = np.zeros(2) # the new velocity of the agent
        self.alpha = alpha # Scaling constant for difference in cand vel & goal vel
        self.beta = beta  # Scaling constant for difference in cand and current vel
        self.gamma = gamma # Scaling constant for collision
        self.N = N # Sampling number
   
    def computeAction(self, neighbors=[]):
        """ 
        Using VO algorithm, samples a predictive reactive velocity for the respective agent

        Inputs:
        -------
        neighbors: List
            List of neighboring agent objects    
        """ 
        cand_vel, cost = self.getBestVel(neighbors)        
        


        if not self.atGoal:
            # self.vnew[:] = self.gvel[:]   # here I just set the new velocity to be the goal velocity 
            self.vnew[:] = cand_vel


    def update(self, dt):
        """ 
            Code to update the velocity and position of the agent  
            as well as determine the new goal velocity 
        """
        if not self.atGoal:
            self.vel[:] = self.vnew[:]
            self.pos += self.vel*dt   #update the position
        
            # compute the goal velocity for the next time step. Do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq: 
                self.atGoal = True  # goal has been reached
            else: 
                # self.gvel = self.gvel/np.sqrt(distGoalSq)*self.prefspeed
                self.gvel = self.gvel/np.linalg.norm(self.gvel)*self.prefspeed

    def detectCloseNeighbor(self, neighbor):
        """
        Detect if neighbor within close proximity of the agent

        Inputs:
        --------
        neighbor : Object
            Neighbor object
        gamma : Float
            Small gamma value to validate neighbor position is not itself

        Outputs:
        ---------
        Boolean
            Return if neighbor is close
        """

        # Return true if:
        #   1) Neighbor is within sensing radius
        #   2) Distance from agent to neighbor is not itself (Due to neighbors containing all agents including itself)

        d_AB = np.linalg.norm(self.pos - neighbor.pos) - self.radius - neighbor.radius
        if self.dhor >= d_AB and self.id != neighbor.id and not neighbor.atGoal:
            # print("Detected close obstacle at location [%0.2f, %0.2f] w.r.t. self pos [%0.2f, %0.2f]"
            # % (neighbor.pos[0], neighbor.pos[1], self.pos[0], self.pos[1]))
            return True
        return False

    def getRandCirc(self, radius):
        """
        Samples an admissable point from a unit circle of radius "radius". The formulation
        uses the following form:
        [x] = [r*cos(theta), r*sin(theta)]

        The following implementation is currently limited to 2d

        Inputs:
        -------
        self : Object
            Contains object info for max admissable velocity
        radius : Float
            Radius of the unit ball

        Outputs:
        ---------
        x : ndarray
            Admissable point w/in unit ball R^(nx1)
        """
        r = np.sqrt(np.random.random())*radius # Random radius
        theta = np.random.random()*2*math.pi # Random angle

        return np.array([r*math.cos(theta), r*math.sin(theta)])
    
    def sampleAdmissbleVel(self, N):
        """
        Samples N points from a unit circle and returns the admissible velocity using
        the following formulation:
        [v_x, v_y] = [v*cos(theta), v*sin(theta)]
        
        Inputs:
        -------
        self : Object
            Contains object info for max admissable velocity
        N : int
            Number of candidate velocity to sample

        Outputs:
        ---------
        vel : List of ndarray
            Admissable velocity [[v_x1, v_y1],...,[v_xN, v_yN]]
        """
        # Initialize admissible velocity list
        n = 2 # Hard code dimension of navigation space
        vel = np.zeros((n,N))
        for i in range(N):
            vel[:,i] = self.getRandCirc(self.maxspeed)
        return vel

    def computeTTC(self, neighbor, cand_v=[]):
        """
        Compute time to collision (TTC) of neighboring agent w.r.t. self
        
        Inputs:
        --------
        neighbor : Object
            Neighboring agents
        epsilon : Float
            Sensory noise for the estimated velocity
        cand_v : ndarray (Optional)
            Time to collision given a candidate velocity instead

        Outputs:
        ---------
        tau : Float
            Time to collision
        """
        # Compute parameters for TTC
        x = self.pos - neighbor.pos
        v = self.vel - neighbor.vel # Relative velocity between two agents is same as true relative velocity
        if cand_v != []:
            v = cand_v - neighbor.vel
        r = self.radius + neighbor.radius # Scalar

        a = np.dot(v, v)-self.epsilon**2
        b = np.dot(x, v) - self.epsilon*r
        c = np.dot(x,x) - r**2
        d = b**2 - a*c

        if c < 0: # Agents colliding
            return 0

        if b > 0: # Agents diverging
            return float('inf')

        if d <= 0: # No collision
            return float('inf')

        # Since b < 0 and b^2 >> ac -> Then smallest positive root is of the form:
        tau = c/(-b + np.sqrt(d))
        if tau < 0:
            return float('inf')
        return tau

    def getBestVel(self, neighbors):
        """
        Samples candidate velocity and finds the corresponding candidate velocity with the minimum
        cost function:

                alpha*||v_cand - v_goal|| + beta*||v_cand - v|| + gamma/tc

        where alpha, beta, and gamma are tuning parameters, and tc is the minimum time to collision
        for the corresponding candidate velocity

        Inputs:
        -------
        self : Object
            Object containing information of the agent's position, velocity, and radius
        
        neighbor : Object
            Neighbor agent containing information of the neighbor's position, velocity, and radius
        
        Output:
        -------
        best_cand_vel : ndarray
            Best candidate velocity
        min_cost : float
            Minimum cost for given candidate velocities
        """

        # Check cost of each candidate velocity
        # TODO: Optimize cost evaluation s.t. not stacking arrays
        cost = []


        # 1b) Randomly sample admissable velocities
        cand_vels = self.sampleAdmissbleVel(self.N)
        for cand_vel in cand_vels.T:
            # 1a) Determine all neighbors that are less than d_H m away from agent,
            #       if so, compute time to collision with candidate velocity
            ttc_neighbors = [] # TTC list for a CERTAIN candidate velocity
            for neighbor in neighbors:
                # Check if neighbor is close
                if self.detectCloseNeighbor(neighbor):
                    ttc_neighbors.append(self.computeTTC(neighbor, cand_vel))
            
            if np.size(ttc_neighbors) == 0 : # If no neighbors, append a dummy ttc with infinite ttc
                ttc_neighbors.append(float('inf'))
            
            min_ttc_neighbors = min(ttc_neighbors) # Get minimum time to collision for a kTH cand_vel

            # Get corresponding cost to kth candidate velocity
            # 1c) Evaluate the fitness of each of the N candidate velocities using the following cost function
            if min_ttc_neighbors != 0: # if ttc is not 0, than use traditional cost function
                cost.append((self.alpha*np.linalg.norm(cand_vel - self.gvel)) +
                self.beta*np.linalg.norm(cand_vel - self.vel) + self.gamma/min_ttc_neighbors)
            else: # ttc = 0 -> cost is 
                cost.append(float('inf'))
        
        # 1d) Select new velocity for agent w/ candidate vel that has min cost
        # Find minimum cost
        min_cost = min(cost)
        min_index = cost.index(min_cost)
        best_cand_vel = cand_vels[:,min_index]
        # print(best_cand_vel)

        return best_cand_vel, min_cost

        