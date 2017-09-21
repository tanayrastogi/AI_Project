from jsonreader import Data
from math import *
import math, sys, pygame, random
from pygame import *
import numpy as np
from model import Model,angle_differance
from node import Node,State
import time


#----------------------------------------------
# Pygame stuff
#----------------------------------------------
XDIM = 750
YDIM = 750
windowSize = [XDIM, YDIM]
pygame.init()
fpsClock = pygame.time.Clock()
screen = pygame.display.set_mode(windowSize)
white = 255, 255, 255
black = 0, 0, 0
red = 255, 0, 0
green = 0, 255, 0
blue = 0, 0, 255
cyan = 0,180,105
dark_green = 0, 102, 0
colors = [red,green,cyan,dark_green]

#----------------------------------------------
# scaling for visualization
#----------------------------------------------
X_tr = 0 # Translation along X-axis
Y_tr = 0 # Translation along Y axis
scale = 10.0
minX = 0
maxX = 75
minY = 0
maxY = 75

#----------------------------------------------
# Varables for the tree
#----------------------------------------------
nodes = list()
GOAL_RADIUS = 1.5
GOAL_DIRECTIONTOL = 7#np.pi/20
GOAL_TRYDISTANCE = 2
OPTIMIZE_DISTANCE = 2
data = None
model = None


#Draw obsticles and boundary polygon, and calculate the scaling parameters
def initObsticles(data):
    global X_tr
    global Y_tr
    global scale
    global minX
    global maxX
    global minY
    global maxY

    #find out how much the boundary polygon must be translated
    minX = data.boundary[0][0]
    minY = data.boundary[0][1]
    maxX = data.boundary[0][0]
    maxY = data.boundary[0][1]
    for i in range(0,len(data.boundary)):
        if(data.boundary[i][0] < minX):
            minX = data.boundary[i][0]
        if(data.boundary[i][1] < minY):
            minY = data.boundary[i][1]
        if(data.boundary[i][0] > maxX):
            maxX = data.boundary[i][0]
        if(data.boundary[i][1] > maxY):
            maxY = data.boundary[i][1]
    X_tr = -minX
    Y_tr = -minY
    scale = min( XDIM/(maxX-minX) , XDIM/(maxY-minY) )
    draw_polygons()

    '''
    print("Y_tr " + str(Y_tr))
    print("scale " + str(scale))
    print("minX " + str(minX))
    print("maxX " + str(maxX))
    print("minY " + str(minY))
    print("maxY " + str(maxY))
    print("Y_tr " + str(Y_tr))
    time.sleep(10)
    '''

def draw_polygons():
    boundary_polygon = list()
    for i in range(len(data.boundary)):
        x = scale*(X_tr+data.boundary[i][0])
        y = YDIM - scale*(Y_tr+data.boundary[i][1])
        boundary_polygon.append([int(x),int(y)])
    pygame.draw.polygon(screen, white, boundary_polygon, 0)

    #print("obsticles: " + str(data.obsticles))
    for j in range(len(data.obsticles)):
        #print("J: " + str(j))
        polygon = list()
        for i in range(len(data.obsticles[j])):
            x = scale*(X_tr+data.obsticles[j][i][0])
            y = YDIM - scale*(Y_tr+data.obsticles[j][i][1])
            #print("X: " + str(scale)+'*('+str(X_tr)+'+'+str(data.obsticles[j][i][0])+')')
            #print("(x,y) = " + str(x) + ',' + str(y))
            polygon.append([int(x),int(y)])
        pygame.draw.polygon(screen, red, polygon, 0)
        #print("Polygon: " + str(polygon))

def draw_point(P,radius,color,showHeading = False):
    x = scale*(X_tr+P.state.pos[0])
    y = YDIM - scale*(Y_tr+P.state.pos[1])
    pygame.draw.circle(screen, color, [int(x),int(y)], radius)
    if(not P.state.heading is None and showHeading):
        draw_line(P.state.pos,[P.state.pos[0] + 0.5*np.cos(P.state.heading),P.state.pos[1] + 0.5*np.sin(P.state.heading)],red,1)

def draw_line(p1,p2,color,thickness):
    x1 = scale*(X_tr+p1[0])
    y1 = YDIM - scale*(Y_tr+p1[1])
    x2 = scale*(X_tr+p2[0])
    y2 = YDIM - scale*(Y_tr+p2[1])
    pygame.draw.line(screen,color,[int(x1),int(y1)],[int(x2),int(y2)],thickness)

def drawPath(N):
    current = N
    #print("DrawPath: current:" + str(current))
    lastPoint = None
    while(current.parent != None):
        #travelTime += current.time
        if(current.path is None):
            current = current.parent
            continue
        if(not lastPoint is None):
            draw_line(lastPoint.pos,current.state.pos,red,2)
        for i in range(1,len(current.path)):
            draw_line(current.path[i-1].pos,current.path[i].pos,red,2)
        draw_line(current.state.pos,current.path[-1].pos,red,2)
        lastPoint = current.path[0]
        current = current.parent

def pathLength(N):
    current = N
    L = 0
    P = np.array(current.state.pos)
    while(current.parent != None):
        if current.path != None:
            for i in range(len(current.path)-2,0,-1):
                P2 = np.array(current.path[i].pos)
                L+=np.linalg.norm(P2-P);
                P = P2
        current = current.parent

    L += np.linalg.norm(P-np.array(current.state.pos))
    return L



def randomsize_point(pathLength = None, start = None, stop = None):
    if((pathLength is None or start is None or stop is None)):
        return minX + (maxX-minX)*random.random() ,minY + (maxY-minY)*random.random()

    #print("randomsize_point")
    p1 = np.array(start)
    p2 = np.array(stop)
    d = np.linalg.norm(p2-p1)
    L = max(pathLength,d)
    C = 0.5*(p2-p1)
    width = 2*L-d
    height = sqrt((0.5*L)*(0.5*L) - (0.5*d)*(0.5*d) )

    n1 = C/np.linalg.norm(C)
    n2 = np.cross(n1,np.array([0,0,1]))
    n2 = n2[:2]


    #print("pathLength: " +str(L))
    #print("p1: " + str(p1))
    #print("p2: " + str(p2))
    #print("d: " + str(d))
    #print("C: " + str(C))
    #print("width: " + str(width))
    #time.sleep(1)
    #print("height: " + str(height))
    #print("n1: " + str(n1))
    #print("n2: " + str(n2))

    while(True):
        r1 = (random.random()-0.5)*n1*width
        r2 = 2*(random.random()-0.5)*n2*height
        P = C+ r1+r2
        k1 = np.linalg.norm(P-p1)
        k2 = np.linalg.norm(P-p2)

        if(k1+k2 <= L):
            return P

    return minX + (maxX-minX)*random.random() ,minY + (maxY-minY)*random.random()

    #return minX + (maxX-minX)*random.random() ,minY + (10)*random.random()


def isInside(poly,P):
    c = False
    L = len(poly)
    for i in range(0,L):
        if( ((poly[(i+1)%L][1]>P[1]) != (poly[i][1]>P[1])) and (P[0] < (poly[i][0]-poly[(i+1)%L][0]) * (P[1]-poly[(i+1)%L][1]) / (poly[i][1]-poly[(i+1)%L][1]) + poly[(i+1)%L][0]) ):
            c = not c
    return c


#Check if a point is valid
def isValid(P):
    global data
    #check if it is inside the boundary_polygon
    if(not isInside(data.boundary,P)):
        return False

    for i in range(len(data.obsticles)):
        if(isInside(data.obsticles[i],P)):
            return False
    return True

#Check if it might be possible to reach the targetPoint
def canReach(Pstart,Pgoal):
    #project the startPoint on the line of the goal
    P = np.array(Pgoal.state.pos)
    v = [np.cos(Pgoal.state.heading),np.sin(Pgoal.state.heading)]
    q = np.array(Pstart.state.pos)-P
    #project q on v,
    proj = np.dot((np.dot(q,v)/np.dot(v,v)),v)
    return np.dot(proj,v) < -0.5


def find_NN(pos):
    global nodes
    nearest = None
    dist = 0
    for i in range(len(nodes)):
        d = sqrt( (nodes[i].state.pos[0]-pos[0])*(nodes[i].state.pos[0]-pos[0]) + (nodes[i].state.pos[1]-pos[1])*(nodes[i].state.pos[1]-pos[1]) )
        if(nearest == None or d <= dist):
            nearest = nodes[i]
            dist = d
            #print(d)
    return nearest

#Extend tree by addning a new branch
def extend(startNode,targetState,extend = True):
    global nodes
    global data
    global model

    #print("Extending between: " + str(startNode.state.pos) + " <-> " +str(targetState.pos))
    #time.sleep(0.1)

    dt = 0.05

    #Extend: Just drive/ go towards the target. Don't attempt to havce any
    #special state when you get there
    steps = 50
    if (extend):
        bestPath = None
        bestDistance = -1
        travelTime = -1

        for itteration in range(10):
            current = startNode.state
            path = list()
            path.append(current)

            #Randomize control varaibles
            random_control = model.getRandomControl()

            #print("StartNode: " + str(current.pos))

            failed = False
            d = 0
            for i in range(steps):
                #calculate the next step
                s = model.getNewState(current,random_control,dt)
                #check if the state is colliding with any obsticles
                if((s is None) or (not isValid(s.pos))):
                    #collision
                    #print("collision or state is None")
                    failed = True
                    break

                d += np.linalg.norm(np.array(current.pos)-np.array(s.pos))
                current = s
                #print(d)
                path.append(s)

            if(failed):
                continue

            distance_to_target = np.linalg.norm(np.array(targetState.pos)-np.array(path[-1].pos))
            #save the best path
            if((distance_to_target < bestDistance or bestDistance == -1)):
                bestDistance = distance_to_target
                bestPath = path
                travelTime = dt*len(path)
                #print("Distance: " + str(bestDistance))

        if(bestPath is None):
            return None

        child = Node(bestPath[-1],startNode,time = travelTime)
        child.add_path(bestPath)
        return child

    return None
    '''
    path,travelTime = model.getPath(startNode.state,targetState,dt)
    if(path is None):
        return None

    if(not (np.linalg.norm(np.array(path[-1].pos)-np.array(targetState.pos)) < GOAL_RADIUS)):
        print("Not close enough")
        #return None
    if(not (angle_differance(targetState.heading,path[-1].heading) < GOAL_DIRECTIONTOL)):
        print("bad headning")
        #return None

    for s in path:
        #print("path: " + str(s.pos))
        if(not isValid(s.pos)):
            #collision
            print("collision")
            #return None

    child = Node(targetState,startNode,time = travelTime)
    child.add_path(path)
    return child
    '''
#optimize the tree by changing edges
def optimize(Node):
    #return False

    global nodes
    global model
    global data
    changed = False
    #1) Calculate total time for Node
    current = Node
    time_Node = 0
    while(not current is None):
        time_Node +=current.time
        current = current.parent

    # 2) find points in a radius around the node
    neighbours = list()
    distance = list()
    for i in range(len(nodes)):
        #d = sqrt( (nodes[i].state.pos[0]-Node.state.pos[0])*(nodes[i].state.pos[0]-Node.state.pos[0]) + (nodes[i].state.pos[1]-Node.state.pos[1])*(nodes[i].state.pos[1]-Node.state.pos[1]) )
        d = np.linalg.norm(np.array(nodes[i].state.pos) - np.array(Node.state.pos))
        if(d<OPTIMIZE_DISTANCE and nodes[i] != Node): #Node is close enough
            neighbours.append(nodes[i])
            distance.append(d)
    #print("Found " + str(len(neighbours)) + " nodes to try to optimize")
    #for it in distance:
        #print("Distance: " + str(it))
    #print("Max speed:" + str(model.v_max))

    if len(neighbours) == 0:
        return False

    #print("Time for N: " + str(time_Node))
    # Try to optimize
    improvements = np.zeros(len(neighbours))
    for i in range(len(neighbours)):
        current = neighbours[i]
        time_neihgbour = 0
        while(not current is None):
            time_neihgbour +=current.time
            current = current.parent
        improvements[i] = time_neihgbour - (time_Node+(distance[i]/model.v_max))
        '''
        print("\nTime for Node: " + str(time_Node))
        print("Time for neighbour: " + str(time_neihgbour))
        print("Distance between node and neighbour: " + str(distance[i]))
        print("Time between node and neighbour: " + str(distance[i]/model.v_max))
        print("Time for path through Node: " + str((time_Node+(distance[i]/model.v_max))))
        print("improvement: " + str(improvements[i]) + "\n")
        '''
    for i in range(len(improvements)):
        #print("canReach(Node,neighbours[i]): " + str(canReach(Node,neighbours[i])))

        if(improvements[i] > 0):# and canReach(Node,neighbours[i])): #it is possible to improve

            if (model.type == 'kin-car' and not canReach(Node,neighbours[i])):
                continue

            temp = extend(Node,neighbours[i].state,extend=False)
            if(temp == None):
                continue
            #print("Time: " + str(temp.time))

            neighbours[i].parent = Node
            neighbours[i].path = temp.path
            neighbours[i].time = temp.time
            changed = True
            for i in range(1,len(temp.path)):
                draw_line(temp.path[i-1].pos,temp.path[i].pos,blue,2)
            #pygame.display.update()
            #drawPath(neighbours[i])
            #pygame.display.update()
            #time.sleep(2)

        else:
            pass
                #print("Did not find a better path")
    #if(changed):
        #time.sleep(0.05)
    return changed

def optimeizePath(Node):
    current = Node
    while current.parent != None:
        optimize(current)
        current = current.parent

def optimeizeTest():
    global nodes
    global data
    global model
    data = Data('problem_A.json')
    #init obsticles and boundary polygon
    initObsticles(data)
    goal_vel = np.linalg.norm(np.array(data.goal_vel))
    goal_heading = atan2(data.goal_vel[1],data.goal_vel[0])
    goalPoint = Node(State(data.goal_pos,goal_vel,goal_heading), None)

    #settings
    model = Model(data,type = 'kin-point')

    p1 = [0,0]
    p2 = State([1,2],model.v_max,np.pi/2)
    p3 = State([0,2],model.v_max,np.pi/2)
    p4 = State([0,1],model.v_max,np.pi/2)

    P1 = Node(State(p1,0.1,np.pi/2),None)
    nodes.append(P1)

    P2 = extend(P1,p2,extend = False)
    nodes.append(P2)
    drawPath(P2)

    P3 = extend(P2,p3,extend = False)
    nodes.append(P3)
    drawPath(P3)

    P4 = extend(P1,p4,extend = False)
    nodes.append(P4)
    drawPath(P4)

    pygame.display.update()
    time.sleep(5)
    optimize(P4)

    screen.fill(black)
    draw_polygons()
    for i in range(len(nodes)):
        draw_point(nodes[i],1,dark_green)
        if(nodes[i].path is None):
            continue
        for j in range(1,len(nodes[i].path)):
            draw_line(nodes[i].path[j-1].pos,nodes[i].path[j].pos,green,1)
    pygame.display.update()
    time.sleep(1)


def xTrackTest():
    global nodes
    global data
    global model
    data = Data('maps/problem_A_niklas2.json')
    #init obsticles and boundary polygon
    initObsticles(data)


    #settings
    #model = Model(data,type = 'kin-point')
    model = Model(data,type = 'dyn-point')
    #model = Model(data,type = 'kin-car')

    #Create start and goal states
    initial_vel = 2
    initial_heading = 1*np.pi/2
    initialPoint = Node(State([1,-1],initial_vel,initial_heading), None)

    goal_vel = 1
    goal_heading = np.pi/2
    goalPoint = Node(State([1,1.5],goal_vel,goal_heading), None)

    #Draw start and goal states
    draw_point(initialPoint,5,black,showHeading=True)
    draw_point(goalPoint,5,black,showHeading=True)

    Start = initialPoint
    Stop = goalPoint.state

    child = extend(Start,Stop,extend=False)
    if (child is None): #target could not be reached
        print("Child is None")

    #list with all nodes is a global varaible
    nodes.append(initialPoint)
    counter = 0
    imagenr = 0
    repaint = False
    while True:
        #time.sleep(0.1)
        #counter = counter+1
        #if counter%10 == 0:
        #    pass
        #    pygame.image.save(screen,'images\RRT-STAR'+str(imagenr)+'.jpg')
        #    imagenr +=1
        #distance_to_goal = np.linalg.norm(np.array(N.state.pos) - np.array(goalPoint.state.pos))

        #--------------------------EXPAND TREE---------------------------------
        #P = randomsize_point()
        #draw_line(P,P,red,2)


        #Draw all states to get a nice image!
        draw_line(child.parent.state.pos,child.path[0].pos,green,1)
        for i in range(1,len(child.path)):
            draw_line(child.path[i-1].pos,child.path[i].pos,green,1)
        draw_point(child,1,dark_green)
        #nodes.append(child)


        if(repaint or len(nodes) % 1000 == 999):
            repaint = False
            #Redraw tree
            #print("Redraw")
            print("Redrawing " + str(len(nodes)) + " nodes")
            screen.fill(black)
            draw_polygons()
            draw_point(initialPoint,5,black,showHeading=True)
            draw_point(goalPoint,5,black,showHeading=True)
            for i in range(len(nodes)):
                if(nodes[i].path is None):
                    continue
                for j in range(1,len(nodes[i].path)):
                    draw_line(nodes[i].path[j-1].pos,nodes[i].path[j].pos,green,1)
                #Draw all states to get a nice image!
                if(child.parent != None):
                    draw_line(nodes[i].parent.state.pos,nodes[i].path[0].pos,green,1)
                draw_line(nodes[i].state.pos,nodes[i].path[-1].pos,green,1)
                draw_point(nodes[i],1,dark_green)
                if(not goalPoint.parent is None):
                    drawPath(goalPoint)
            pygame.display.update()
            #time.sleep(10)


        #handle events
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Exiting")
            if (e.type == KEYUP and e.key == K_SPACE):
                repaint = True

        if(len(nodes) % 50 == 0):
            print("Nodes: " + str(len(nodes)))

        pygame.display.update()
        fpsClock.tick(10000)
        time.sleep(1)


def main():
    global nodes
    global data
    global model
    data = Data('maps/problem_B.json')
    #init obsticles and boundary polygon
    initObsticles(data)


    #settings
    #model = Model(data,type = 'kin-point')
    #model = Model(data,type = 'dyn-point')
    #model = Model(data,type = 'diff-drive')
    #model = Model(data,type = 'kin-car')
    #model = Model(data,type = 'dyn-car')
    model = Model(data,type = 'friction-car')

    #Create start and goal states
    initial_vel = np.linalg.norm(np.array(data.start_vel))
    initial_heading = atan2(data.start_vel[1],data.start_vel[0])
    initialPoint = Node(State(data.start_pos,initial_vel,initial_heading), None)

    goal_vel = np.linalg.norm(np.array(data.goal_vel))
    goal_heading = atan2(data.goal_vel[1],data.goal_vel[0])
    goalPoint = Node(State(data.goal_pos,goal_vel,goal_heading), None)
    #goalPoint = Node(State([5,5],goal_vel,initial_heading), None)

    #Draw start and goal states
    draw_point(initialPoint,5,black,showHeading=True)
    draw_point(goalPoint,5,black,showHeading=True)


    #list with all nodes is a global varaible
    nodes.append(initialPoint)
    counter = 0
    imagenr = 0
    repaint = False
    while True:
        #time.sleep(0.1)
        #counter = counter+1
        #if counter%10 == 0:
        #    pass
        #    pygame.image.save(screen,'images\RRT-STAR'+str(imagenr)+'.jpg')
        #    imagenr +=1
        #distance_to_goal = np.linalg.norm(np.array(N.state.pos) - np.array(goalPoint.state.pos))

        #--------------------------EXPAND TREE---------------------------------
        #for i in range(200):
        #    P = randomsize_point(pathLength=10,start=initialPoint.state.pos, stop=goalPoint.state.pos)
        #    draw_line(P,P,red,2)
        #time.sleep(2)


        if(goalPoint.parent != None):
            P = randomsize_point(pathLength=pathLength(goalPoint),start=initialPoint.state.pos, stop=goalPoint.state.pos)
        else:
            P = randomsize_point()
        draw_line(P,P,red,2)

        randomState = State(P,random.random()*model.v_max,random.random()*2*np.pi)

        N = find_NN(P)
        #if(optimize(N)):
        #    print("improvement")
        child = extend(N,randomState,extend=True)
        if (child is None): #target could not be reached
            print("Child is None")
            continue

        #optimize the tree like RRT-star
        #if(optimize(child)):
        #    print("WOHO it has improved")

        #Optimize a random node
        #if(optimize(nodes[np.random.randint(len(nodes))])):
        #    print("Random improvement")

        #Draw all states to get a nice image!
        draw_line(child.parent.state.pos,child.path[0].pos,green,1)
        for i in range(1,len(child.path)):
            draw_line(child.path[i-1].pos,child.path[i].pos,green,1)
        draw_point(child,1,dark_green)
        nodes.append(child)


        #Redraw everything
        if(repaint or len(nodes) % 1000 == 999):
            repaint = False
            #Redraw tree
            #print("Redraw")
            print("Redrawing " + str(len(nodes)) + " nodes")
            screen.fill(black)
            draw_polygons()
            draw_point(initialPoint,5,black,showHeading=True)
            draw_point(goalPoint,5,black,showHeading=True)
            for i in range(len(nodes)):
                if(nodes[i].path is None):
                    continue
                for j in range(1,len(nodes[i].path)):
                    draw_line(nodes[i].path[j-1].pos,nodes[i].path[j].pos,green,1)
                #Draw all states to get a nice image!
                if(child.parent != None):
                    draw_line(nodes[i].parent.state.pos,nodes[i].path[0].pos,green,1)
                draw_line(nodes[i].state.pos,nodes[i].path[-1].pos,green,1)
                draw_point(nodes[i],1,dark_green)
                if(not goalPoint.parent is None):
                    drawPath(goalPoint)
            pygame.display.update()
            #time.sleep(10)


        #Try to extend to the goal if the node is close
        if(np.linalg.norm(np.array(child.state.pos) - np.array(goalPoint.state.pos)) < GOAL_TRYDISTANCE):
            print("Trying to reach goal")
            #attempt = extend(child,goalPoint.state,extend = False) #------------------------
            attempt = child
            if (attempt is None): #target could not be reached
                continue

            distance_diff = np.linalg.norm(np.array(attempt.state.pos)-np.array(goalPoint.state.pos))
            heading_diff = angle_differance(attempt.state.heading,goalPoint.state.heading)
            print("distance is off by: " + str(distance_diff))
            print("heading is off by : " + str(180.0*heading_diff/np.pi))
            if(distance_diff < GOAL_RADIUS and heading_diff < GOAL_DIRECTIONTOL):
                print("Goal reached!")
                if(goalPoint.parent is None):
                    #time for new Node
                    c_c = 0
                    current = child
                    while(not current is None):
                        c_c +=current.time
                        current = current.parent
                    goalPoint.parent = attempt
                    goalPoint.pathLength = c_c
                    nodes.append(attempt)
                    nodes.append(goalPoint)
                    drawPath(goalPoint)
                else:
                    # Current time for goalPoint
                    c_t = 0
                    current = goalPoint
                    while(not current is None):
                        c_t +=current.time
                        current = current.parent
                    #time for new Node
                    c_c = 0
                    current = attempt
                    while(not current is None):
                        c_c +=current.time
                        current = current.parent
                    if(c_c<c_t):
                        print("Best path has been improved")
                        goalPoint.parent = attempt
                        goalPoint.pathLength = c_c
                        nodes.append(attempt)
                        drawPath(goalPoint)
                #drawPath(goalPoint)
                #pygame.display.update()


        #handle events
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Exiting")
            if (e.type == KEYUP and e.key == K_SPACE):
                repaint = True
            if (e.type == KEYUP and e.key == K_o):
                l = len(nodes)
                print("Optimizing " + str(l) + " nodes,  this might take some time")
                for i in range(l):
                    if(i%20 == 0):
                        print("optimizing node: " +str(i))
                    optimize(nodes[i])
            if (e.type == KEYUP and e.key == K_p):
                if(goalPoint.parent != None and random.random() < 0.2):
                    print("Optimize best path")
                    optimeizePath(goalPoint)
            if (e.type == KEYUP and e.key == K_s):
                if(goalPoint.parent != None):
                    print("Save path!: ")
                    steps = 0
                    current = goalPoint

                    while(current.parent != None):
                        steps +=1
                        if current.path != None:
                            steps += len(current.path)
                        current = current.parent
                    print("Steps to goal: " + str(steps))

                    current = goalPoint
                    output = np.zeros((steps,4))
                    index = 0
                    while(current.parent != None):
                        output[index][0] = current.state.pos[0]
                        output[index][1] = current.state.pos[1]
                        output[index][2] = current.state.vel
                        output[index][3] = current.state.heading
                        index = index+1

                        if current.path != None:
                            for i in range(len(current.path)-2,0,-1):
                                output[index][0] = current.path[i].pos[0]
                                output[index][1] = current.path[i].pos[1]
                                output[index][2] = current.path[i].vel
                                output[index][3] = current.path[i].heading
                                index+=1

                        current = current.parent

                    output[index][0] = current.state.pos[0]
                    output[index][1] = current.state.pos[1]
                    output[index][2] = current.state.vel
                    output[index][3] = current.state.heading
                    index = index+1
                    output = output[:index,:]

                    np.savetxt("TEST-dynamiccar-friction.csv", output, delimiter=",")
                    print("saving path")
                    time.sleep(2)


        if(len(nodes) % 50 == 0):
            print("Nodes: " + str(len(nodes)))

        pygame.display.update()
        fpsClock.tick(10000)
        #time.sleep(0.5)


if __name__ == '__main__':
    main()
    #xTrackTest()
    #optimeizeTest()
