import random
import math
from RealWorld.nodesOptimization.Transformations import Transformations
from scipy.optimize import dual_annealing
import numpy as np


class SimulatedAnnealing(object):

    def __init__(self):
        self.optimizationIndexMax = 0
        self.optimizationIndexCurrent = 0

    def getOptimalTheta(self):
        return self.optimalTheta

    def getOptimalShiftX(self):
        return self.optimalShiftX

    def getOptimalShiftY(self):
        return self.optimalShiftY

    def run(self, cart, cartObst, scanDist):
        self.cart = cart
        self.cartObst = cartObst
        self.scanDist = scanDist

        # Initial and final temperature
        T = 1000
        # Temperature at which iteration terminates
        Tmin = 5
        # Decrease in temperature
        alpha = 0.9
        # Number of iterations of annealing before decreasing temperature
        numIterations = 500  # TODO: Savvas in java has 500 iterations

        theta_bounds = (0, 90)
        x_bounds = (0, 2 * self.scanDist)
        y_bounds = (0, 2 * self.scanDist)
        bounds = [theta_bounds, x_bounds, y_bounds]
        minimizer_kwargs = {"args": (self.cart, self.cartObst, self.scanDist), "bounds": bounds}
        ret = dual_annealing(CreateNew, bounds, args=(minimizer_kwargs['args']), maxiter=500)
        self.optimalTheta, self.optimalShiftX, self.optimalShiftY = ret['x']

        print("\n ~~~ Final value of optimization index: ",
              -1 * CreateNew(ret['x'], self.cart, self.cartObst, self.scanDist), " ~~~ ")


def CreateNew(x, cart, cartObst, scanDist):
    randomSol = Transformations()

    randomSol.setTheta(x[0])
    randomSol.setShiftX(x[1])
    randomSol.setShiftY(x[2])
    return -randomSol.rotateAndShift(cart, cartObst, scanDist)


class MyTakeStep:

    def __init__(self, scanDist, stepsize=0.5):
        self.stepsize = stepsize
        self.scan = scanDist

        self.rng = np.random.default_rng()

    def __call__(self, x):
        s = self.stepsize

        x[0] += self.rng.uniform(-45 * s, 45 * s)
        x[1] += self.rng.uniform(-2 * self.scan * s, 2 * self.scan * s)
        x[2] += self.rng.uniform(-2 * self.scan * s, 2 * self.scan * s)

        return x