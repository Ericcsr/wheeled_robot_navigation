import numpy as np
from matplotlib import pyplot as plt
from matplotlib import cm
from cv2 import imread
import cv2 as cv
import random, sys, math, os.path, time
from mpi4py import MPI
import sys

if len(sys.argv) == 2:
    task = sys.argv[1]
else:
    print("Usage: mpiexec -n <workernumber(10)> python3 bias_rrt.py <task>")

comm = MPI.COMM_WORLD
rank = comm.Get_rank()
print('This is:{0}'.format(rank))
size = comm.Get_size()

MAP_IMG = './maps/'+task+'.bmp'  # Black and white image for a map
DISPLAY = MAP_IMG
#DISPLAY = './maps/'+task+'_orig'+'.bmp' # Comment this line if test narrow planning
MIN_NUM_VERT = 40-5*((rank-1)%5)  # Minimum number of vertex in the graph
MAX_NUM_VERT = 5000  # Maximum number of vertex in the graph
STEP_DISTANCE = 7+2*((rank-1)%5)  # Maximum distance between two vertex
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
    ''' Calculate length of a trajectory
    Args:
        path: double list path
    Return:
    length: length in pix of the path
    '''
    length = 0.0
    for i in range(len(path)-1):
        start = (path[i][0],path[i][1])
        end = (path[i+1][0],path[i+1][1])
        length += math.sqrt((start[0]-end[0])**2+(start[1]-end[1])**2)
    return length

def rapidlyExploringRandomTree(img, start, goal, seed=None):
    '''Mainbody of the algorithm
    Args:
        img: map image
    '''
    ls_time = time.time()
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
    # ======= Biased Sampling prepare =======
    if (rank-1) // 5==0 and rank > 0:
        thresh1 = np.load('./distributions/'+task+'prob.npy')
        rows,cols = thresh1.shape
        xP = thresh1.flatten() / np.sum(thresh1)
        x = np.arange(rows*cols)
    # =======================================
    i = 0
    while (goal not in points) and (len(points) < MAX_NUM_VERT) and i <= 100000:

        while (occupied):
            if phaseTwo and (random.random() > 0.9):
                point = [random.randint(minX, maxX), random.randint(minY, maxY)]
            else:
                if (rank-1) //5 == 1:
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
        i = i + 1

        if len(points) >= MIN_NUM_VERT:
            if not phaseTwo:
                print('Phase Two')
            phaseTwo = True

        if phaseTwo:
            nearest = findNearestPoint(points, goal)
            newPoints = connectPoints(goal, nearest, img)
            addToGraph(graph, newPoints)
            newPoints.pop(0)
            points.extend(newPoints)

    if goal in points and i<= 100000:
        path = searchPath(graph, start, [start])
    else:
        path = None
    return path

def searchPath(graph, point, path):
    '''Helper - Perform Graph Search for the path
    Args:
        graph: the current graph maintained by RRT
        point: point for checking
    Returns:
        finalPath
    '''
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
    '''Helper -  Add a point to graph
    Args:
        graph: current graph maintained by RRT
        point: point for checking
    '''
    if len(newPoints) > 1:  # If there is anything to add to the graph
        for p in range(len(newPoints) - 1):
            nearest = [nearest for nearest in graph if (nearest[0] == [newPoints[p][0], newPoints[p][1]])]
            nearest[0][1].append(newPoints[p + 1])
            graph.append((newPoints[p + 1], []))

def connectPoints(a, b, img):
    '''Helper -  Check weather two nodes are connectable
    Args:
        a: one node
        b: the second node
        img: map
    Returns:
        newpoints: points add in the midway
    '''
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
    '''Helper - Search for nearest point
    Args:
        points: points search space
        point:point which need to find adjacency
    Returns:
        (best[0],best[1]): The coord of the point which is nearest
    '''
    best = (999999, 999999, 999999)
    for p in points:
        if p == point:
            continue
        dist = math.sqrt((p[0] - point[0]) ** 2 + (p[1] - point[1]) ** 2)
        if dist < best[2]:
            best = (p[0], p[1], dist)
    return (best[0], best[1])

def selectStartGoalPoints(ax, img):
    '''Select start and end point on GUI
    Args:
        ax: the handle of the ploter
        img: map
    Returns:
        start: start point
        goal: goal point
    '''
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
    '''plot the path on the image
    Args:
        lines: path
        image: map
    Returns:
        image which has been ploted
    '''
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
    '''Distributed planner
    '''
    pack = comm.bcast(None,root=0)
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
    '''Centralized info server
    '''
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
    start, goal = selectStartGoalPoints(ax, img)
    ls_time = time.time()
    sendpack = Workpack(img,start,goal)
    _  = comm.bcast(sendpack,root=0)
    results = comm.gather(None,root=0)[1:]
    t = time.time() - ls_time
    rs = []
    for r in results:
        if r is not None:
            rs.append(r)
    optimal_path = min(rs).path
    print("Optimal_path:",optimal_path)
    image_result = visualize(optimal_path,disp)
    print('Shape:',image_result.shape)
    plt.imshow(image_result) 
    plt.title("Time Cost:{0}".format(t))
    plt.show()

if rank == 0:
    server_task()
else:
    worker_task()