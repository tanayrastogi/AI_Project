import time
import numpy as np
from node import State
from math import atan2
import random

class DynPointmodel:

    def __init__(self,vmax,amax):
        self.v_max = vmax
        self.a_max = amax

    #Randomize a control input
    #This object will be used in nextState function
    def randomControl(self):
        a_mag = random.random()*self.a_max
        a_dir = random.random()*np.pi*2
        return [a_mag,a_dir]

    def nextState(self,currentState,control,dt):
        x = currentState.pos[0]
        y = currentState.pos[1]
        vx = np.cos(currentState.heading)*currentState.vel
        vy = np.sin(currentState.heading)*currentState.vel
        ax = np.cos(control[1])*control[0]
        ay = np.sin(control[1])*control[0]

        x_new = x+vx*dt + 0.5*dt*dt*ax
        y_new = y+vy*dt + 0.5*dt*dt*ay
        vx_new = vx + ax*dt
        vy_new = vy + ay*dt

        direction = atan2(vy_new,vx_new)
        vel = np.linalg.norm(np.array([vx_new,vy_new]))

        if(vel > self.v_max):
            #print("Too fast! " + str(vel))
            return None

        #print("distance: " + str(np.linalg.norm([x_new-x,y_new-y])))

        return State([x_new,y_new],vel,direction)

    def integrate(self,currentState,controlInput,dt):
        vx = np.cos(currentState.heading)*currentState.vel
        vy = np.sin(currentState.heading)*currentState.vel

        dvx = controlInput[1]*np.cos(controlInput[0])*dt
        dvy = controlInput[1]*np.sin(controlInput[0])*dt

        vx = vx + dvx
        vy = vy + dvy
        v = np.linalg.norm([vx,vy])
        Theta = atan2(vy,vx)

        p = currentState.pos
        return State([p[0]+vx*dt,p[1]+dt*vy],v,Theta)

    #Drive towards the target. Stop if t_max is exceded or target is reached
    def goTowards(self,current,target,dt,t_max):
        t = 0
        path = list()
        #newState = current
        currentState = current
        nextState = None


        while(t<t_max):
            dist = -1#np.linalg.norm(np.linalg.norm(newState.pos)-np.array(goal.pos))
            for steeringInput in [currentState.heading+np.pi/2,currentState.heading-np.pi/2]:
                S = self.integrate(currentState,[steeringInput,self.a_max],dt)
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

        t=0
        tmax = 10

        vel_target = 1

        currentState = current


        while(t<tmax):

            #Calculate point on the line
            P = np.array(targetState.pos)
            v = [np.cos(targetState.heading),np.sin(targetState.heading)]
            q = np.array(currentState.pos)-P
            #project q on v, then add P
            proj = np.dot((np.dot(q,v)/np.dot(v,v)),v)
            #if(np.dot(proj,v)) > 0:
            #    return self.getNextState(current,goal,dt,XTRACK=False)
            P_line = proj+P
            vel = currentState.vel
            x = P_line-np.array(currentState.pos)

            distance_from_line = np.linalg.norm(x)
            print("Distance: " + str(distance_from_line))
            ax = np.linalg.norm(x)

            vel_error = vel_target-vel
            a_v = np.dot(vel_error,v)
            print("Velocity: " + str(vel))
            print("Velocity error : " + str(vel_error))
            print("a: " + str(a_v) + " | v: " + str(v))


            a = a_v# + a_x
            angle = atan2(a[1],a[0])
            #print("target: " + str(target))
            currentState = self.integrate(currentState,[angle,self.a_max],dt)

            path.append(currentState)
            t +=dt
            distance_to_goal = np.linalg.norm(np.array(targetState.pos)-np.array(currentState.pos))
            if(distance_to_goal<self.v_max*dt):
                #print("Close to goal " + str(dist))
                break


        return path ,t
