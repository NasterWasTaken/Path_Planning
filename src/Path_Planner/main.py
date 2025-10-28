import time
import sys
import argparse
import random
import numpy as np
import cProfile
from RealWorld.real_world_parser import real_world
from RealWorld.handleGeo import GridToNED
from RealWorld.handleGeo import Dist
from RealWorld.visualizeNEDPaths import PlotNEDPaths
from RealWorld.handleGeo.GridToNED import GridPathsToNED
from RealWorld.handleGeo import Dist

def run_experiment(optimal_init_pos, number_of_trials, AirSim, AirLearning):
    real_world_parameters = real_world()
    real_world_parameters.geo2cart()
    rows, cols, obstacles_positions = real_world_parameters.get_DARP_params()

    if optimal_init_pos:  # Runs with OPTUNA !!
        optimization = optimize(rows, cols, number_of_trials, False, [], obstacles_positions, False,
                                real_world_parameters.droneNum)

        optimization.optimize()

        FinalPaths = optimization.best_trial.best_case.paths
        initial_positions = optimization.best_trial.darp_instance.initial_positions


        NEDdata = GridPathsToNED(FinalPaths, initial_positions, real_world_parameters.droneNum,
                                 real_world_parameters.subNodes, real_world_parameters.rotate)

        #NEDdata = GridPathsToNED(optimal_init_pos, optimization, real_world_parameters.droneNo, real_world_parameters.subNodes, real_world_parameters.rotate)
        init_posNED = NEDdata.init_posGRIDToNED()

        # WaypointsNED are in the form of WaypointsNED[DroneNo][0]
        WaypointsNED = NEDdata.getWaypointsNED()

    else:  # Runs with random init pos !!

        # Put drones in initial positions (close to physical or random)
        randomInitPos = True

        count = 0
        while True:
            print("\n!!! DARP will run for random initial positions !!! ")
            grid = np.arange(0, rows * cols).reshape(rows, cols)

            DARPgrid = initializeDARPGrid(randomInitPos, rows, cols, real_world_parameters.initial_positions,
                                          real_world_parameters.megaNodes,
                                          real_world_parameters.theta, real_world_parameters.shiftX,
                                          real_world_parameters.shiftY,
                                          real_world_parameters.droneNum)

            positions_DARPgrid = np.where(DARPgrid == 2)

            positions_DARPgrid = np.asarray(positions_DARPgrid).T
            positions = []

            for elem in positions_DARPgrid:
                positions.append(grid[elem[0], elem[1]])

            poly = MultiRobotPathPlanner(rows, cols, real_world_parameters.notEqualPortions, positions,
                                         real_world_parameters.portions, obstacles_positions, False)
            count += 1

            if poly.DARP_success or count > 5:
                break

            print("\n!!! DARP will rerun for random initial positions !!! ")

        if count == 5:
            print("DARP did not manage to find a solution for the given configuration!")
            print("Try to alter one or more of:")
            print("- Scanning distance")
            print("- Number of drones")
            print("Given area")
            exit()
        else:


            FinalPaths = poly.best_case.paths
            initial_positions = poly.darp_instance.initial_positions

            NEDdata = GridPathsToNED(FinalPaths, initial_positions, real_world_parameters.droneNo,
                                     real_world_parameters.subNodes, real_world_parameters.rotate)
            # NEDdata = GridPathsToNED(optimal_init_pos, poly, real_world_parameters.droneNo,
            #                          real_world_parameters.subNodes, real_world_parameters.rotate)

            init_posNED = NEDdata.init_posGRIDToNED()
            WaypointsNED = NEDdata.getWaypointsNED()

    # """ Visualize NED Paths """
    PlotNEDPaths(real_world_parameters.NED_Coords, real_world_parameters.obstNED, real_world_parameters.droneNo,
                 WaypointsNED, init_posNED, optimal_init_pos).plot()