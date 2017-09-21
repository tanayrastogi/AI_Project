import time
import numpy as np
from node import State
import random

class DynCarmodel:
    def __init__(self,amax,vmax,phimax,L):
        self.v_max = vmax
        self.a_max = amax
        self.phi_max = phimax
        self.L = L

    #Randomize a control input
    #This object will be used in nextState function
    def randomControl(self):
        print("Get random control / dynamic car")
        a = 2*(random.random()-0.5)*self.a_max
        phi = 2*(random.random()-0.5)*self.phi_max
        return [a,phi]

    def nextState(self,currentState,control,dt):
        a = control[0] #F/M #mass of car  = 1
        phi = control[1]

        x = currentState.pos[0]
        y = currentState.pos[1]
        theta = currentState.heading
        v = currentState.vel

        v_new = v + a*dt
        theta_new = theta + (v_new/self.L)*np.tan(phi)*dt
        x_new = x+v*np.cos(theta_new)*dt
        y_new = y+v*np.sin(theta_new)*dt

        if(abs(v_new) > self.v_max):
            return None

        return State([x_new,y_new],v_new,theta_new)


    def integrate(self,currentState,controlInput,dt):
        dTheta = dt*currentState.vel/self.L * np.tan(controlInput)
        Theta = currentState.heading+dTheta
        v = self.v_max
        p = currentState.pos
        return State([p[0]+dt*v*np.cos(Theta),p[1]+dt*v*np.sin(Theta)],v,Theta)

    #Drive towards the target. Stop if t_max is exceded or target is reached
    def goTowards(self,current,target,dt,t_max):
        t = 0
        path = list()
        #newState = current
        currentState = current
        nextState = None
        while(t<t_max):
            dist = -1#np.linalg.norm(np.linalg.norm(newState.pos)-np.array(goal.pos))
            for steeringInput in np.linspace(-self.phi_max, self.phi_max, num=21):
                S = self.integrate(currentState,steeringInput,dt)
                d = np.linalg.norm(np.array(S.pos)-np.array(target.pos))
                if(d<dist or dist==-1):
                    dist = d
                    nextState = S
            currentState = nextState
            path.append(currentState)
            t+=dt
            if(d<self.v_max * dt):
                print("point reached \kineticCarModel")
                break;

        #print("Time: " + str(t))
        #print("steps: " + str(len(path)))
        return path,t

    #Drive towards the target and arrive at the correct state
    def goToState(self,current,targetState,dt):
        t = 0
        path = list()

        #TODO crosstrack
        #TODO better crosstrack
        #TODO handle points which are hard to reach
        #TODO bonus if algorithm works for more than kin-car model

        t=0
        tmax = 10

        currentState = current
        while(t<tmax):

            #Calculate point to aim at
            P = np.array(targetState.pos)
            v = [np.cos(targetState.heading),np.sin(targetState.heading)]
            q = np.array(currentState.pos)-P
            #project q on v, then add P
            proj = np.dot((np.dot(q,v)/np.dot(v,v)),v)
            #if(np.dot(proj,v)) > 0:
            #    return self.getNextState(current,goal,dt,XTRACK=False)
            target = np.dot(0.8,proj)+P
            vel = currentState.vel
            #print("target: " + str(target))

            newState = None
            dist = -1#np.linalg.norm(np.linalg.norm(newState.pos)-np.array(goal.pos))
            for steeringInput in np.linspace(-self.phi_max, self.phi_max, num=21):
                S = self.integrate(currentState,steeringInput,dt)
                d = np.linalg.norm(np.array(S.pos)-target)
                #print("d: " + str(d))
                if(d<dist or dist==-1):
                    dist = d
                    newState = S
            currentState = newState
            path.append(currentState)
            t +=dt
            distance_to_goal = np.linalg.norm(np.array(targetState.pos)-np.array(currentState.pos))
            if(distance_to_goal<self.v_max*dt):
                #print("Close to goal " + str(dist))
                break


        return path ,t
