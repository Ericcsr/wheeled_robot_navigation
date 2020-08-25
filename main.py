import numpy as np
from matplotlib import pyplot as plt
from matplotlib import cm
from cv2 import imread
import cv2 as cv
import random, sys, math, os.path, time
from mpi4py import MPI
import sys
import conf

if len(sys.argv) == 2:
    task = sys.argv[1]
else:
    print("Usage: mpiexec -n <workernumber(10)> python3 bias_rrt.py <task>")

comm = MPI.COMM_WORLD
rank = comm.Get_rank()
print('This is:{0}'.format(rank))
size = comm.Get_size()

MAP_IMG = './maps/'+task+'.bmp'  # Black and white image for a map
DISPLAY = './maps/'+task+'_orig'+'.bmp' # Comment this line if test narrow planning
STEP_DISTANCE = 5+3*(rank%5)
SEED = rank  # For random numbers

class Workpack:
    def __init__(self,map,start,goal):
        self.image = map
        self.points = (start,goal)

class Resultpack:
    def __init__(self,path):
        self.path = path
        self.length = compute_length(path)
    def __gt__(self,other):
        return self.length > other.length
    def __eq__(self, other):
        return self.length == other.length

def compute_length(path):
    length = 0.0
    for i in range(len(path)-1):
        start = (path[i][0],path[i][1])
        end = (path[i+1][0],path[i+1][1])
        length += math.sqrt((start[0]-end[0])**2+(start[1]-end[1])**2)
    return length

def rapidlyExploringRandomTree(img, start, goal, seed=None):
    ls_time = time.time()
    hundreds = 100
    random.seed(seed)
    points = []
    graph = []
    points.append(start)
    graph.append((start, []))
    occupied = True
    phaseTwo = False

    # Phase two values (points 5 step distances around the goal point)
    minX = max(goal[0] - 10 * STEP_DISTANCE, 0)
    maxX = min(goal[0] + 10 * STEP_DISTANCE, len(img[0]) - 1)
    minY = max(goal[1] - 10 * STEP_DISTANCE, 0)
    maxY = min(goal[1] + 10 * STEP_DISTANCE, len(img) - 1)
    # ======= Biased Sampling =======
    if rank // 5==0:
        thresh1 = np.load('./distributions/'+task+'prob.npy')
        rows,cols = thresh1.shape
        xP = thresh1.flatten() / np.sum(thresh1)
    #yP = np.sum(thresh1, axis=1) / np.sum(thresh1)
        x = np.arange(rows*cols)
    #y = np.arange(117)
    # blacks_x = np.random.choice(x,size=1,replace=True, p=xP)
    # blacks_y = np.random.choice(y,size=1,replace=True, p=yP)

    i = 0
    while (goal not in points) and (len(points) < conf.MAX_NUM_VERT) and i <= 10000:
        if (i % 100) == 0:
            print
            i, 'points randomly generated'

        if (len(points) % hundreds) == 0:
            print
            len(points), 'vertex generated'
            hundreds = hundreds + 100

        while (occupied):
            if phaseTwo and (random.random() > 0.8):
                point = [random.randint(minX, maxX), random.randint(minY, maxY)]
            else:
                if rank //5 == 1:
                    point = [random.randint(0, len(img[0]) - 1), random.randint(0, len(img) - 1)]
                else:
                    p = np.random.choice(x, size=1, replace=True, p=xP)[0]
                    point = [p%cols,p//cols]

            if (img[point[1]][point[0]][0] == 255):
                occupied = False

        occupied = True

        nearest = findNearestPoint(points, point)
        newPoints = connectPoints(point, nearest, img)
        addToGraph(graph, newPoints)
        newPoints.pop(0)  # The first element is already in the points list
        points.extend(newPoints)
        #plt.draw()
        i = i + 1

        if len(points) >= conf.MIN_NUM_VERT:
            if not phaseTwo:
                print
                'Phase Two'
            phaseTwo = True

        if phaseTwo:
            nearest = findNearestPoint(points, goal)
            newPoints = connectPoints(goal, nearest, img)
            addToGraph(graph, newPoints)
            newPoints.pop(0)
            points.extend(newPoints)
            #plt.draw()

    if goal in points and i<= 10000:
        print
        'Goal found, total vertex in graph:', len(points), 'total random points generated:', i
        path = searchPath(graph, start, [start])
        '''
        for i in range(len(path) - 1):
            ax.plot([path[i][0], path[i + 1][0]], [path[i][1], path[i + 1][1]], color='g', linestyle='-', linewidth=2)
            plt.draw()
        '''
        #print('Showing resulting map')
        #print('Final path:', path)
        #print('The final path is made from:', len(path), 'connected points')
    else:
        path = None
        #print('Reached maximum number of vertex and goal was not found')
        #print('Total vertex in graph:', len(points), 'total random points generated:', i)
        #print('Showing resulting map')
    #print("Taken:{0} second to converge".format(time.time() - ls_time))
    #plt.show()
    return path

def searchPath(graph, point, path):
    for i in graph:
        if point == i[0]:
            p = i

    if p[0] == graph[-1][0]:
        return path

    for link in p[1]:
        path.append(link)
        finalPath = searchPath(graph, link, path)

        if finalPath != None:
            return finalPath
        else:
            path.pop()

def addToGraph( graph, newPoints):
    if len(newPoints) > 1:  # If there is anything to add to the graph
        for p in range(len(newPoints) - 1):
            nearest = [nearest for nearest in graph if (nearest[0] == [newPoints[p][0], newPoints[p][1]])]
            nearest[0][1].append(newPoints[p + 1])
            graph.append((newPoints[p + 1], []))
        '''
            if not p == 0:
                ax.plot(newPoints[p][0], newPoints[p][1], '+k')  # First point is already painted
            ax.plot([newPoints[p][0], newPoints[p + 1][0]], [newPoints[p][1], newPoints[p + 1][1]], color='k',
                    linestyle='-', linewidth=1)

        if point in newPoints:
            ax.plot(point[0], point[1], '.g')  # Last point is green
        else:
            ax.plot(newPoints[p + 1][0], newPoints[p + 1][1], '+k')  # Last point is not green
        '''

def connectPoints(a, b, img):
    newPoints = []
    newPoints.append([b[0], b[1]])
    step = [(a[0] - b[0]) / float(STEP_DISTANCE), (a[1] - b[1]) / float(STEP_DISTANCE)]

    # Set small steps to check for walls
    pointsNeeded = int(math.floor(max(math.fabs(step[0]), math.fabs(step[1]))))

    if math.fabs(step[0]) > math.fabs(step[1]):
        if step[0] >= 0:
            step = [1, step[1] / math.fabs(step[0])]
        else:
            step = [-1, step[1] / math.fabs(step[0])]

    else:
        if step[1] >= 0:
            step = [step[0] / math.fabs(step[1]), 1]
        else:
            step = [step[0] / math.fabs(step[1]), -1]

    blocked = False
    for i in range(pointsNeeded + 1):  # Creates points between graph and solitary point
        for j in range(STEP_DISTANCE):  # Check if there are walls between points
            coordX = round(newPoints[i][0] + step[0] * j)
            coordY = round(newPoints[i][1] + step[1] * j)

            if coordX == a[0] and coordY == a[1]:
                break
            if coordY >= len(img) or coordX >= len(img[0]):
                break
            if img[int(coordY)][int(coordX)][0] < 255:
                blocked = True
            if blocked:
                break

        if blocked:
            break
        if not (coordX == a[0] and coordY == a[1]):
            newPoints.append([newPoints[i][0] + (step[0] * STEP_DISTANCE), newPoints[i][1] + (step[1] * STEP_DISTANCE)])

    if not blocked:
        newPoints.append([a[0], a[1]])
    return newPoints

def findNearestPoint(points, point):
    best = (999999, 999999, 999999)
    for p in points:
        if p == point:
            continue
        dist = math.sqrt((p[0] - point[0]) ** 2 + (p[1] - point[1]) ** 2)
        if dist < best[2]:
            best = (p[0], p[1], dist)
    return (best[0], best[1])

def selectStartGoalPoints(ax, img):
    print('Select a starting point')
    ax.set_xlabel('Select a starting point')
    occupied = True
    while (occupied):
        point = plt.ginput(1, timeout=-1, show_clicks=False, mouse_pop=2)
        start = [round(point[0][0]), round(point[0][1])]
        if (img[int(start[1])][int(start[0])][0] == 255):
            occupied = False
            ax.plot(start[0], start[1], '.r')
        else:
            print('Cannot place a starting point there')
            ax.set_xlabel('Cannot place a starting point there, choose another point')

    print('Select a goal point')
    ax.set_xlabel('Select a goal point')
    occupied = True
    while (occupied):
        point = plt.ginput(1, timeout=-1, show_clicks=False, mouse_pop=2)
        goal = [round(point[0][0]), round(point[0][1])]
        if (img[int(goal[1])][int(goal[0])][0] == 255):
            occupied = False
            ax.plot(goal[0], goal[1], '.b')
        else:
            print('Cannot place a goal point there')
            ax.set_xlabel('Cannot place a goal point there, choose another point')

    plt.draw()
    return start, goal

def visualize(lines,image):
    image =image.copy()
    for i,point in enumerate(lines):
        #print(point)
        start = (int(point[0]+0.5),int(point[1]+0.5))
        if i <len(lines) -1:
            end = (int(lines[i+1][0]+0.5),int(lines[i+1][1]+0.5))
        cv.circle(image,start,2,color=(255,0,0),thickness=-1)
        #cv.circle(image,end,2,color=(255,0,0),thickness=-1)
        cv.line(image,start,end,color=(0,255,0),thickness=1)
    return image

def worker_task():
    while True:
        pack = comm.bcast(None,root=0)
        if pack == False:
            break
        image = pack.image
        start,goal = pack.points
        path = rapidlyExploringRandomTree(image,start,goal,seed = SEED)
        print(path)
        if path != None:
            result = Resultpack(path)
        else:
            result = None
        _ = comm.gather(result,root=0)

def server_task():
    from conf import ws_model,X,V,V_max,goals,REPLAN_V_THRESH
    from local_rvo.RVO import RVO_update, reach, compute_V_des,reach,Tools,distance
    from local_rvo.vis import visualize_traj_dynamic
    print('Loading map... with file \'', MAP_IMG, '\'')
    img = imread(MAP_IMG)
    disp = imread(DISPLAY)
    fig = plt.gcf()
    fig.clf()
    ax = fig.add_subplot(1, 1, 1)
    ax.imshow(img, cmap=cm.Greys_r)
    ax.axis('image')
    plt.draw()
    print('Map is', len(img[0]), 'x', len(img))
    pstart, pgoal = selectStartGoalPoints(ax, img)
    obs_map = img.copy()
    while True:
        sendpack = Workpack(obs_map,pstart,pgoal)
        _  = comm.bcast(sendpack,root=0)
        print('Start Planning')
        results = comm.gather(None,root=0)[1:]
        print('Finished Planning')
        X[0] = Tools.pix2world([pstart])[0]
        tool = Tools(4,conf.THRESH)
        cnt = 0
        rs = []
        for r in results:
            if r is not None:
                rs.append(r)
        optimal_path = min(rs)
        optimal_dist = optimal_path.length
        goals[0] = Tools.pix2world(optimal_path.path)
        desired_time = Tools.toWorldLen(optimal_dist) / (conf.VMAX)
        t = 0
        step = 0.05
        print('RVO start')
        cum_v = [1,1,1,1,1]
        while t*step < desired_time :
            goal = tool.get_goal(X,goals)
            V_des = compute_V_des(X, goal, V_max)
            V = RVO_update(X,V_des,V,ws_model,local=2)
            obs_map = img.copy()
            for i in range(len(X)):
                if X[i][0] <= 8.4 or X[i][0] >= 0.1:
                    X[i][0] += V[i][0]*step
                else:
                    X[i][0] += 0
                if X[i][1] <= 4.4 or X[i][1] >= 0.1: 
                    X[i][1] += V[i][1]*step
                else:
                    X[i][1] = 0
                if i!= 0 and distance(X[i],X[0]) <= 4*conf.THRESH:
                    x,y = Tools.world2pix([[X[i][0],X[i][1]]])[0]
                    obs_map[int(y+0.5)-4:int(y+0.5)+4,int(x+0.5)-4:int(x+0.5)+4,:] = 0
                    #print("Add obstacle:",X[0])
                path_image = cv.cvtColor(visualize(optimal_path.path,obs_map),cv.COLOR_RGB2BGR)
                path_image = cv.resize(path_image,(2*path_image.shape[1],2*path_image.shape[0]),
                             cv.INTER_LINEAR)
            cum_v[t%5] = math.sqrt(V[0][0]**2+V[0][1]**2)
            if sum(cum_v/5) <= REPLAN_V_THRESH:
                break
            if t%2 == 0:
                # === 480 x 640 x 3 Image === 
                rvo_image = visualize_traj_dynamic(ws_model, X, V, goal, time=t*step, name = 'data/out{0}.png'.format(cnt),save=False)
                # === 107 x 202 x 3 Image ===
                canvas = np.zeros_like(rvo_image,dtype=np.uint8)
                canvas[100:314,150:554,:] = path_image
                result = np.hstack([rvo_image,canvas])
                cv.imshow('Visualize path and planning',result)
                cv.imwrite('./data/vis{0}.jpg'.format(cnt),result)
                if cv.waitKey(1) == ord('q'):
                    exit(-1)
                cnt += 1
            t+=1
        else:
            # == Conditions for replanning ==
            if distance(X[0],Tools.pix2world([pgoal])[0]) < conf.THRESH:
                print('Target Reached!')
                return
            pstart = Tools.world2pix([X[0]])[0]


if rank == 0:
    server_task()
    comm.bcast(False,root=0)
    exit(-1)
else:
    worker_task()