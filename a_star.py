DEBUG = 2
PLOT = 1
EXECUTE = 0

X = 0
Y = 1

DEBUG_LEVEL = DEBUG

import sys

class Point:
    def __init__(self, x, y):
        self.x      = x
        self.y      = y
        self.cost   = float('inf')
        self.parent = None

import time
import numpy as np
from matplotlib.patches import Rectangle

import map

class AStar:
    def __init__(self, map : RandomMap, startPos : Point, endPos : Point):
        self.startPt   = startPos
        self.endPt     = endPos
        self.map       = map
        self.open_set  = []
        self.close_set = []

    def BaseCost(self, p : Point):
        """Cost from the starting point to the point p
        
        Args: 
            p:  one point in the point list, whose distance to the
                staring point is to be returned
        
        Returns: 
            A floating point number representing the distance
        """
        x_dis = abs(self.startPt.x - p.x)
        y_dis = abs(self.startPt.y - p.y)
        # Distance to start point
        return x_dis + y_dis + (np.sqrt(2) - 2) * min(x_dis, y_dis)

    def HeuristicCost(self, p : Point):
        """ Calculate the Heuristic Cost at a given point

        The Heuristics cost is set to the distance from P(x,y) to the
        ending point. 

        Args:
            p:  one point in the point list

        Returns:
            Heuristic Cost at point p
        """
        x_dis = abs(self.endPt.x - p.x)
        y_dis = abs(self.endPt.y - p.y)
        # Distance to end point
        return x_dis + y_dis + (np.sqrt(2) - 2) * min(x_dis, y_dis)

    def TotalCost(self, p : Point):
        """ Sum of BaseCost(cost from staring point) and HeuristicCost

        Args:
            p:  one point in the point list
        
        Returns:
            Sum of the two costs at point p
        """
        return self.BaseCost(p) + self.HeuristicCost(p)

    def IsValidPoint(self, x : int, y : int):
        """ Whether a point at (x,y) is inside the rectangular map

        Args:
            x:  coordinate x
            y:  coordinate y
        
        Returns:
            Whether the point is inside the map AND whether the point
            belongs to any obstacle
        """
        if x < 0 or y < 0:
            return False
        if x >= self.map.sizeX or y >= self.map.sizeY:
            return False
        return not self.map.IsObstacle(x, y)

    def IsInPointList(self, p : Point, point_list : list):
        for point in point_list:
            if point.x == p.x and point.y == p.y:
                return True
        return False

    def IsInOpenList(self, p : Point):
        return self.IsInPointList(p, self.open_set)

    def IsInCloseList(self, p : Point):
        return self.IsInPointList(p, self.close_set)

    def IsStartPoint(self, p : Point):
        return p.x == self.startPt.x and p.y == self.startPt.y

    def IsEndPoint(self, p : Point):
        return p.x == self.endPt.x and p.y == self.endPt.y

    def SaveImage(self, plt):
        """ Plot the resulting path and save it to a file

        The target file path is under the current directory and named
        as current time (in millisecond) dot png. 

        Args:
            plt:    The ploting lib object
        """
        if DEBUG_LEVEL >= PLOT:
            millis = int(round(time.time() * 1000))
            filename = './' + str(millis) + '.png'
            plt.savefig(filename)

    def ProcessPoint(self, x : int, y : int, parent : Point):
        """ Process a point at position (x,y)

        For the given coordiante, first create a point object, then
        check wether such point is already in the open_set. If not,
        calculate the total cost and push it to the open_set

        Args:
            x: the x coordinate
            y: the y coordinate
        """
        if not self.IsValidPoint(x, y):
            return # Do nothing for invalid point
        p = Point(x, y)
        if self.IsInCloseList(p):
            return # Do nothing for visited point
        if(DEBUG_LEVEL == DEBUG):
            print('Process Point [', p.x, ',', p.y, ']', ', cost: ', p.cost)
        if not self.IsInOpenList(p):
            p.parent = parent
            p.cost   = self.TotalCost(p)
            self.open_set.append(p)

    def SelectPointInOpenList(self):
        """ Find the point with the minimum total cost

        Returns:
            The index of the point in self.open_set with the minimum
            total cost
        """
        index = 0
        selected_index = -1
        min_cost = sys.maxsize
        for p in self.open_set:
            cost = self.TotalCost(p)
            if cost < min_cost:
                min_cost = cost
                selected_index = index
            index += 1
        return selected_index

    def BuildPath(self, p : Point, ax, plt, start_time):
        """ Build path out of the searching result

        Args:
            p:  the end point
            ax: something associated with the drawing
            plt:plotlib object
            start_time: the start time of this execution
        
        Return:
            A list of points, forming a path from start to end
        """
        path = []
        while True:
            path.insert(0, p) # Insert first
            if self.IsStartPoint(p):
                break
            else:
                p = p.parent
        for p in path:
            # rec = Rectangle((p.x, p.y), 1, 1, color='g')
            # ax.add_patch(rec)
            if DEBUG_LEVEL >= PLOT:
                plt.draw()
                self.SaveImage(plt)
        end_time = time.time()
        if DEBUG_LEVEL == DEBUG:
            print('===== Algorithm finish in', int(end_time-start_time), ' seconds')
        return path

    def RunAndSaveImage(self, ax, plt):
        """ The core of the algorithm
        
        First select a point in the open_set. Put the point into the
        close list. For every point around the picked point, process
        it. Continue to do so until no more points in the open_set. 

        Args:
            ax:     I don't know, but something with ploting
            plt:    The plot lib object
        """
        start_time = time.time()

        self.open_set.append(self.startPt)
        while True:
            index = self.SelectPointInOpenList()
            if index < 0:
                if DEBUG_LEVEL == DEBUG:
                    print('No path found, algorithm failed!!!')
                return None
            p = self.open_set[index]
            # rec = Rectangle((p.x, p.y), 1, 1, color='c')
            # ax.add_patch(rec)
            if DEBUG_LEVEL >= PLOT:
                self.SaveImage(plt)

            if self.IsEndPoint(p):
                return self.BuildPath(p, ax, plt, start_time)

            del self.open_set[index]
            self.close_set.append(p)

            # Process all neighbors
            x = p.x
            y = p.y
            self.ProcessPoint(x-1, y+1, p)
            self.ProcessPoint(x-1, y, p)
            self.ProcessPoint(x-1, y-1, p)
            self.ProcessPoint(x, y-1, p)
            self.ProcessPoint(x+1, y-1, p)
            self.ProcessPoint(x+1, y, p)
            self.ProcessPoint(x+1, y+1, p)
            self.ProcessPoint(x, y+1, p)


def aStartPathPlanning(ax, img, start : list, goal : list, seed=None):
    startPos = Point(start[X], start[Y])
    endPos   = Point(goal[X],  goal[Y])
    runner   = AStar(map.RandomMap(img), startPos, endPos)
    return [[point.x, runner.map.sizeY - point.y] for point in runner.RunAndSaveImage()]

