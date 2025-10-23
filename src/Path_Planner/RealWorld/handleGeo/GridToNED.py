import sys
sys.path.append('Optimal_Positions')

class GridPathsToNED:
    def __init__(self, FinalPaths, initial_positions, droneNum, subNodes, rotate):
        self.FinalPaths = FinalPaths
        self.initial_positions = initial_positions
        self.droneNum = droneNum
        self.subNodes = subNodes
        self.rotate = rotate
        self.missionWaypointsNED = [[] for _ in range(self.droneNum)]
        self.reducted_paths = self.delete_intermediate_nodes()

    def delete_intermediate_nodes(self):
        reducted_paths = [[] for _ in range(self.droneNum)]
        for drone in range(len(self.FinalPaths)):
            last_move = ""
            for move in self.FinalPaths[drone]:
                if move[0] == move[2]:
                    current_move = "horizontal"
                elif move[1] == move[3]:
                    current_move = "vertical"
                if last_move != current_move: #type: ignore
                    reducted_paths[drone].append(move)
                last_move = current_move #type: ignore

        return reducted_paths

    def init_posGRIDToNED(self):
        init_pos_NED = []

        for k in range(self.droneNum):
            xInit = 2 * self.initial_positions[k][0]
            yInit = 2 * self.initial_positions[k][1]
            init_pos_NED.append([self.subNodes[xInit][yInit][0], self.subNodes[xInit][yInit][1]])
            initial_positions_NED = self.rotate.rotateBackWaypoints(init_pos_NED)
        return initial_positions_NED #type: ignore

    def getWaypointsNED(self):
        for drone in range(self.droneNum):
            iWaypoints = []
            for j in range(len(self.reducted_paths[drone])):
                x = self.reducted_paths[drone][j][0]
                y = self.reducted_paths[drone][j][1]
                iWaypoints.append([self.subNodes[x][y][0], self.subNodes[x][y][1]])

            x = self.reducted_paths[drone][0][0]
            y = self.reducted_paths[drone][0][1]

            iWaypoints.append([self.subNodes[x][y][0], self.subNodes[x][y][1]])
            WP = self.rotate.rotateBackWaypoints(iWaypoints)
            self.missionWaypointsNED[drone] = WP

        return self.missionWaypointsNED