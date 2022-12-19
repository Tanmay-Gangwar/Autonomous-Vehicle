import util 
from util import Belief, pdf 
from engine.const import Const
import math

# Class: Estimator
#----------------------
# Maintain and update a belief distribution over the probability of a car being in a tile.
class Estimator(object):
    def __init__(self, numRows: int, numCols: int):
        self.belief = util.Belief(numRows, numCols) 
        self.transProb = util.loadTransProb() 
        self.numRows = numRows
        self.numCols = numCols
            
    ##################################################################################
    # [ Estimation Problem ]
    # Function: estimate (update the belief about a StdCar based on its observedDist)
    # ----------------------
    # Takes |self.belief| -- an object of class Belief, defined in util.py --
    # and updates it *inplace* based onthe distance observation and your current position.
    #
    # - posX: x location of AutoCar 
    # - posY: y location of AutoCar 
    # - observedDist: current observed distance of the StdCar 
    # - isParked: indicates whether the StdCar is parked or moving. 
    #             If True then the StdCar remains parked at its initial position forever.
    # 
    # Notes:
    # - Carefully understand and make use of the utilities provided in util.py !
    # - Remember that although we have a grid environment but \
    #   the given AutoCar position (posX, posY) is absolute (pixel location in simulator window).
    #   You might need to map these positions to the nearest grid cell. See util.py for relevant methods.
    # - Use util.pdf to get the probability density corresponding to the observedDist.
    # - Note that the probability density need not lie in [0, 1] but that's fine, 
    #   you can use it as probability for this part without harm :)
    # - Do normalize self.belief after updating !!

    ###################################################################################
    def estimate(self, posX: float, posY: float, observedDist: float, isParked: bool) -> None:
        newBelief = util.Belief(self.numRows, self.numCols)
        for i in range(self.numRows):
            for j in range(self.numCols):
                dx = util.colToX(j) - posX
                dy = util.rowToY(i) - posY
                dist = math.sqrt(dx * dx + dy * dy)
                newBelief.grid[i][j] = util.pdf(observedDist, Const.SONAR_STD, dist)
        newBelief.normalize()
        updatedBelief = util.Belief(self.numRows, self.numCols)
        for i in range(self.numRows):
            for j in range(self.numCols):
                updatedBelief.grid[i][j] = 0
        for key in self.transProb:
            ((r, c), (i, j)) = key
            updatedBelief.grid[i][j] += self.belief.grid[r][c] * self.transProb[key]
        updatedBelief.normalize()

        for i in range(self.numRows):
            for j in range(self.numCols):
                if isParked: self.belief.grid[i][j] *= newBelief.grid[i][j]
                else: self.belief.grid[i][j] = updatedBelief.grid[i][j] * newBelief.grid[i][j]
        self.belief.normalize()
  
    def getBelief(self) -> Belief:
        return self.belief

   