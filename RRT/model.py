import numpy as np
from math import *
import time
from node import Node,State
from kineticcar import KinCarmodel
from dynpoint import DynPointmodel
from diffdrive import DifferentialDrivemodel
from dyncar import DynCarmodel
from frictioncar import FrictionCarmodel

class Model:
    def __init__(self,data,type = 'kin-point'):
        self.phi_max = data.phi_max
        self.a_max = data.a_max
        self.omega_max = data.omega_max
        self.v_max = data.v_max
        self.L = data.L_car
        self.k_friction = data.k_friction
        self.type = type

        self.kinCar = KinCarmodel(self.v_max,self.phi_max,self.L)
        self.dynPoint = DynPointmodel(self.v_max,self.a_max)
        self.diffDrive = DifferentialDrivemodel(self.v_max,self.omega_max)
        self.dynCar = DynCarmodel(self.a_max,self.v_max,self.phi_max,self.L)
        self.frictionCar = FrictionCarmodel(self.a_max,self.v_max,self.phi_max,self.L,self.k_friction)

    def getRandomControl(self):
        if (self.type == 'dyn-point'):
            return self.dynPoint.randomControl()
        elif (self.type == 'diff-drive'):
            return self.diffDrive.randomControl()
        elif (self.type == 'kin-car'):
            return self.kinCar.randomControl()
        elif (self.type == 'dyn-car'):
            return self.dynCar.randomControl()
        elif (self.type == 'friction-car'):
            return self.frictionCar.randomControl()
        return None

    def getNewState(self,currentState,control,dt):
        if (self.type == 'dyn-point'):
            return self.dynPoint.nextState(currentState,control,dt)
        elif (self.type == 'diff-drive'):
            return self.diffDrive.nextState(currentState,control,dt)
        elif (self.type == 'kin-car'):
            return self.kinCar.nextState(currentState,control,dt)
        elif (self.type == 'dyn-car'):
            return self.dynCar.nextState(currentState,control,dt)
        elif (self.type == 'friction-car'):
            return self.frictionCar.nextState(currentState,control,dt)
        return None

    #goTowards
    #Use the motion model to go towards a target
    #Does not take anything except direction into account
    #Will go towards the target as fast as possible
    def goTowards(self,startState,targetState,dt,time):
        #print("Go towards \ Model")
        if (self.type == 'kin-point'):
            # 1) angle towards the target
            alpha = atan2(targetState.pos[1]-startState.pos[1],targetState.pos[0]-startState.pos[0])
            path = list()
            for i in np.linspace(0,self.v_max*time,num=int(time/dt)):
                pos = [startState.pos[0] + i*np.cos(alpha),startState.pos[1] + i*np.sin(alpha)]
                v = self.v_max
                h = targetState.heading
                path.append(State(pos,v,h))
            return path,time
        elif (self.type == "kin-car"):
            return self.kinCar.goTowards(startState,targetState,dt,time)
        elif (self.type == 'dyn-point'):
            return self.dynPoint.goTowards(startState,targetState,dt,time)
        else:
            return None,0

    #getPath:
    # StartState: type: State
    # endState: type: State
    # ds : type float, step size
    def getPath(self,startState,endState,dt):
        #print("Get path \ Model")
        if(self.type == 'kin-point'):
            d = np.linalg.norm(np.array(startState.pos)-np.array(endState.pos))
            n = 2+(d/(dt*startState.vel))
            #print("steps: " + str(n))
            path = list()
            for i in np.linspace(0, 1, num=n):
                x = startState.pos[0] + i* (endState.pos[0]-startState.pos[0])
                y = startState.pos[1] + i* (endState.pos[1]-startState.pos[1])
                v = self.v_max
                h = endState.heading
                path.append(State([x,y],v,h))
            return path,d/self.v_max
        elif (self.type == "kin-car"):
            return self.kinCar.goToState(startState,endState,dt)
        elif (self.type == 'dyn-point'):
            return self.dynPoint.goToState(startState,endState,dt)
        else:
            return None,0

def angle_differance(alpha,beta):

    # 1) make sure both angles are between 0 and 2pi
    if(alpha<0):
        while(alpha<0):
            alpha += 2*np.pi
    elif(alpha>2*np.pi):
        while(alpha>2*np.pi):
            alpha -= 2*np.pi
    if(beta<0):
        while(beta<0):
            beta += 2*np.pi
    elif(beta>2*np.pi):
        while(beta>2*np.pi):
            beta -= 2*np.pi
    #print("Calculate differance between angles: " + str(alpha) + " and " + str(beta))
    return min(max(alpha,beta)-min(alpha,beta) , 2*np.pi + min(alpha,beta) - max(alpha,beta))
