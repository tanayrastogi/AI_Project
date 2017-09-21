import numpy as np

#----------------------------------------------
# Classes for RRT
#----------------------------------------------
class State:
    def __init__(self,pos,vel = None, heading = None):
        self.pos = pos
        self.vel = vel
        if(heading != None):
            if (heading > 2.0*np.pi):
                self.heading = heading-2.0*np.pi
            elif (heading < -2.0*np.pi):
                self.heading = heading+2.0*np.pi
            else:
                self.heading = heading
        else:
            self.heading = heading

class Node:
    def __init__(self, state,parent,time = 0):
        self.state = state
        self.parent = parent
        self.childgren = 0
        self.time = time
        self.path = None
        self.pathLength = None

    def add_path(self,states):
        self.path = states
