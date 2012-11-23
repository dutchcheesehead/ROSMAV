import sys
import math

def get_reward(state):
    goal_highx=2
    goal_lowx=1.2
    goal_highy=3
    goal_lowy=2.1
    goal_lowtheta=-3.145
    goal_hightheta= 3.145
    
    x=state['/position']['x']
    y=state['/position']['y']
    theta=state['/position']['theta']
    
    if goal_lowx <= x <= goal_highx and goal_lowy <= y <= goal_highy and goal_lowtheta <= theta <=goal_hightheta:
        reward=10
    else:
        reward=-1
#        reward=-1*math.sqrt(math.pow(10-x, 2)+math.pow(0-y,2))
    #       expression=lambda x, y, theta: -1*math.sqrt(math.pow(10-x, 2)+math.pow(0-y,2))
            
#        reward=expression(x, y, theta)
    
    print "reward" 
    print reward
    return reward 
		
		
def check_termination_conditions(state):
    goal_highx=2
    goal_lowx=1.2
    goal_highy=2.5
    goal_lowy=2.1
    goal_lowtheta=-3.145
    goal_hightheta= 3.145

            
    x=state['/position']['x']
    y=state['/position']['y']
    theta=state['/position']['theta']
    

    if goal_lowx <= x <= goal_highx and goal_lowy <= y <= goal_highy and goal_lowtheta <= theta <=goal_hightheta:
        terminate=True
    else:
        terminate=False
    
    print terminate
    return terminate
