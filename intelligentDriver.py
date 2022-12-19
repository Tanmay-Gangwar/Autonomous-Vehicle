'''
Licensing Information: Please do not distribute or publish solutions to this
project. You are free to use and extend Driverless Car for educational
purposes. The Driverless Car project was developed at Stanford, primarily by
Chris Piech (piech@cs.stanford.edu). It was inspired by the Pacman projects.
'''
import util
import itertools
from turtle import Vec2D
from engine.const import Const
from engine.vector import Vec2d
from engine.model.car.car import Car
from engine.model.layout import Layout
from engine.model.car.junior import Junior
from configparser import InterpolationMissingOptionError
import math
from queue import Queue

# Class: Graph
# -------------
# Utility class
class Graph(object):
    def __init__(self, nodes, edges):
        self.nodes = nodes
        self.edges = edges

# Class: IntelligentDriver
# ---------------------
# An intelligent driver that avoids collisions while visiting the given goal locations (or checkpoints) sequentially. 
class IntelligentDriver(Junior):

    # Funciton: Init
    def __init__(self, layout: Layout):
        self.burnInIterations = 30
        self.layout = layout 
        # self.worldGraph = None
        self.numRows = self.layout.getBeliefRows()
        self.numCols = self.layout.getBeliefCols()
        self.nodes = set()
        self.adj = dict()
        self.worldGraph = self.createWorldGraph()
        self.checkPoints = self.layout.getCheckPoints() # a list of single tile locations corresponding to each checkpoint
        self.transProb = util.loadTransProb()
        self.distances = [self.getDistances(checkPoint) for checkPoint in self.checkPoints]
        self.prevGoal = None
        self.angleThreshold = 90
        self.probabilityThreashold = 0.99

        
    # ONE POSSIBLE WAY OF REPRESENTING THE GRID WORLD. FEEL FREE TO CREATE YOUR OWN REPRESENTATION.
    # Function: Create World Graph
    # ---------------------
    # Using self.layout of IntelligentDriver, create a graph representing the given layout.
    def createWorldGraph(self):
        nodes = []
        edges = []
        # create self.worldGraph using self.layout
        numRows, numCols = self.layout.getBeliefRows(), self.layout.getBeliefCols()

        # NODES #
        ## each tile represents a node
        nodes = [(x, y) for x, y in itertools.product(range(numRows), range(numCols))]
        
        # EDGES #
        ## We create an edge between adjacent nodes (nodes at a distance of 1 tile)
        ## avoid the tiles representing walls or blocks#
        ## YOU MAY WANT DIFFERENT NODE CONNECTIONS FOR YOUR OWN IMPLEMENTATION,
        ## FEEL FREE TO MODIFY THE EDGES ACCORDINGLY.

        ## Get the tiles corresponding to the blocks (or obstacles):
        blocks = self.layout.getBlockData()
        blockTiles = []
        for block in blocks:
            row1, col1, row2, col2 = block[1], block[0], block[3], block[2] 
            # some padding to ensure the AutoCar doesn't crash into the blocks due to its size. (optional)
            row1, col1, row2, col2 = row1-1, col1-1, row2+1, col2+1
            blockWidth = col2-col1 
            blockHeight = row2-row1 

            for i in range(blockHeight):
                for j in range(blockWidth):
                    blockTile = (row1+i, col1+j)
                    blockTiles.append(blockTile)

        ## Remove blockTiles from 'nodes'
        nodes = [x for x in nodes if x not in blockTiles]

        for node in nodes:
            self.nodes.add(node)
            if node not in self.adj: self.adj[node] = set()
            x, y = node[0], node[1]
            adjNodes = [(x, y-1), (x, y+1), (x-1, y), (x+1, y)]
            
            # only keep allowed (within boundary) adjacent nodes
            adjacentNodes = []
            for tile in adjNodes:
                if tile[0]>=0 and tile[1]>=0 and tile[0]<numRows and tile[1]<numCols:
                    if tile not in blockTiles:
                        adjacentNodes.append(tile)

            for tile in adjacentNodes:
                edges.append((node, tile))
                edges.append((tile, node))
                if tile not in self.adj: self.adj[tile] = set()
                self.nodes.add(tile)
                self.adj[node].add(tile)
                self.adj[tile].add(node)
                
        return Graph(nodes, edges)
    
    def getDistances(self, source):
        q = Queue()
        q.put(source)
        dist = dict()
        for node in self.nodes: dist[node] = float('inf')
        dist[source] = 0
        while not q.empty():
            x = q.get()
            for y in self.adj[x]:
                if dist[y] > dist[x] + 1:
                    dist[y] = dist[x] + 1
                    q.put(y)
        return dist


    #######################################################################################
    # Function: Get Next Goal Position
    # ---------------------
    # Given the current belief about where other cars are and a graph of how
    # one can driver around the world, chose the next position.
    #######################################################################################
    def getNextGoalPos(self, beliefOfOtherCars: list, parkedCars:list , chkPtsSoFar: int):
        '''
        Input:
        - beliefOfOtherCars: list of beliefs corresponding to all cars
        - parkedCars: list of booleans representing which cars are parked
        - chkPtsSoFar: the number of checkpoints that have been visited so far 
                       Note that chkPtsSoFar will only be updated when the checkpoints are updated in sequential order!
        
        Output:
        - goalPos: The position of the next tile on the path to the next goal location.
        - moveForward: Unset this to make the AutoCar stop and wait.

        Notes:
        - You can explore some files "layout.py", "model.py", "controller.py", etc.
         to find some methods that might help in your implementation. 
        '''
        # print(self.dir)
        safetyBelief = util.Belief(self.numRows, self.numCols, 1)
        for belief in beliefOfOtherCars:
            temp = util.Belief(self.numRows, self.numCols, 0)
            for key in self.transProb:
                (ri, ci), (rf, cf) = key
                temp.grid[rf][cf] += belief.grid[ri][ci] * self.transProb[key]
            temp.normalize()
            for i in range(self.numRows):
                for j in range(self.numCols):
                    safetyBelief.grid[i][j] *= (1 - temp.grid[i][j])

        currPos = self.getPos() # the current 2D location of the AutoCar (refer util.py to convert it to tile (or grid cell) coordinate)
        # BEGIN_YOUR_CODE 
        x, y = currPos
        r = util.yToRow(y)
        c = util.xToCol(x)
        if (r, c) not in self.adj: return self.prevGoal, True
        # print(self.checkPoints[chkPtsSoFar])
        # print(r, c, self.distances[chkPtsSoFar][(r, c)])

        bestProb = 0
        bestPos = (r, c)
        R = Const.BELIEF_TILE_SIZE
        for theta in range(-180, 180):
            dx = R * math.cos(theta)
            dy = R * math.sin(theta)
            if abs(self.dir.get_angle_between(Vec2D(dx, dy))) <= self.angleThreshold:
                i = util.yToRow(y + dy)
                j = util.xToCol(x + dx)
                if i == r and j == c: continue
                if (i, j) in self.nodes and safetyBelief.grid[i][j] >= bestProb and self.distances[chkPtsSoFar][(i, j)] < self.distances[chkPtsSoFar][(r, c)]:
                    bestProb = safetyBelief.grid[i][j]
                    bestPos = (util.colToX(j), util.rowToY(i))
        if bestProb >= self.probabilityThreashold: 
            self.prevGoal = bestPos
            return bestPos, True
        for theta in range(-180, 180):
            dx = R * math.cos(theta)
            dy = R * math.sin(theta)
            if abs(self.dir.get_angle_between(Vec2D(dx, dy))) <= self.angleThreshold:
                i = util.yToRow(y + dy)
                j = util.xToCol(x + dx)
                if i == r and j == c: continue
                if (i, j) in self.nodes and safetyBelief.grid[i][j] >= bestProb and self.distances[chkPtsSoFar][(i, j)] <= self.distances[chkPtsSoFar][(r, c)]:
                    bestProb = safetyBelief.grid[i][j]
                    bestPos = (util.colToX(j), util.rowToY(i))
        if bestProb >= self.probabilityThreashold: 
            self.prevGoal = bestPos
            return bestPos, True
        for theta in range(-180, 180):
            dx = R * math.cos(theta)
            dy = R * math.sin(theta)
            if abs(self.dir.get_angle_between(Vec2D(dx, dy))) <= self.angleThreshold:
                i = util.yToRow(y + dy)
                j = util.xToCol(x + dx)
                if i == r and j == c: continue
                if (i, j) in self.nodes:
                    bestProb = safetyBelief.grid[i][j]
                    bestPos = (util.colToX(j), util.rowToY(i))
        if bestProb >= self.probabilityThreashold: 
            self.prevGoal = bestPos
            return bestPos, True
        return (0, 0), False

    # DO NOT MODIFY THIS METHOD !
    # Function: Get Autonomous Actions
    # --------------------------------
    def getAutonomousActions(self, beliefOfOtherCars: list, parkedCars: list, chkPtsSoFar: int):
        # Don't start until after your burn in iterations have expired
        if self.burnInIterations > 0:
            self.burnInIterations -= 1
            return[]
       
        goalPos, df = self.getNextGoalPos(beliefOfOtherCars, parkedCars, chkPtsSoFar)
        vectorToGoal = goalPos - self.pos
        wheelAngle = -vectorToGoal.get_angle_between(self.dir)
        driveForward = df
        actions = {
            Car.TURN_WHEEL: wheelAngle
        }
        if driveForward:
            actions[Car.DRIVE_FORWARD] = 1.0
        return actions
    
    