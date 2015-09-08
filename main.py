#!/usr/bin/env python

__author__ = 'Batylan Nurbekov & Ari Goodman & Doruk Uzunoglu & Miguel Mora'

import sys, re, math, heapq

#Class that implements priority queue.
class PriorityQueue:
    #Initializes priority queue
    def __init__(self):
        self.elements = []

    #Checks if the queue is empty
    def empty(self):
        return len(self.elements) == 0

    #Puts the item with the given priority into the priority queue
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    #Pops the first item off the priority queue
    def get(self):
        return heapq.heappop(self.elements)[1]

class Direction:
    WEST, SOUTH, EAST, NORTH = [x*math.pi/2 for x in range(-2, 2, 1)]

    @staticmethod
    def subtract_angles(angle1, angle2):
        return Direction.normalize_angle(angle2 - angle1)

    #Normalizes angle, so that it only takes values in range [-pi, pi)
    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

class Action:
    FORWARD, BASH, TURN_LEFT, TURN_RIGHT = range(0, 4, 1)


#Class that implements path finding functionality
class PathFinder:
    #Initializes PathFinder class
    def __init__(self, grid):
        self.frontier = PriorityQueue()
        self.parent = {}
        self.cost_so_far = {}
        self.expanded = set()
        self.path = []

        self.start_state = (grid.start, None)

        self.parent[self.start_state] = None
        self.cost_so_far[self.start_state] = 0
        self.frontier.put(self.start_state, 0)
        self.grid = grid

    def getMoves(self, current_state):
        available_actions = []
        current_node = current_state[0]
        current_action = current_state[1]

        cant_move_forward = (current_node[2] == Direction.NORTH and current_node[1] == 0) \
                            or (current_node[2] == Direction.WEST and current_node[0] == 0) \
                            or (current_node[2] == Direction.SOUTH and current_node[1] == len(self.grid.grid)-1) \
                            or (current_node[2] == Direction.EAST and current_node[0] == len(self.grid.grid[0])-1)

        # If can't move forward, check if turns are available
        if(cant_move_forward):
            if(current_action == Action.FORWARD):
                available_actions.append(Action.TURN_LEFT)
                available_actions.append(Action.TURN_RIGHT)

        # Else, all moves are available
        else:
            if current_action != Action.BASH:
                available_actions.append(Action.TURN_LEFT)
                available_actions.append(Action.TURN_RIGHT)
                available_actions.append(Action.BASH)

            available_actions.append(Action.FORWARD)

        return available_actions

    def getPathCost(self, state):
        node = state[0]
        move = state[1]

        node_value = ord(self.grid.grid[node[1]][node[0]]) - ord('0')

        if (node_value > 9):
            node_value = 1

        if(move == Action.TURN_LEFT or move == Action.TURN_RIGHT):
            node_value = int(math.ceil(node_value / 3.0))
        elif(move == Action.BASH):
            node_value = 3
        # else(move == Action.demolish):
        #     node_value = 4

        return node_value

    def getNextNode(self, currentNode, move):
        nextNode= currentNode
        if move == Action.TURN_LEFT:
            currentDir = currentNode[2]
            if(currentDir == Direction.NORTH):
                nextNode = (currentNode[0], currentNode[1], Direction.WEST)
            elif(currentDir == Direction.SOUTH):
                nextNode = (currentNode[0], currentNode[1], Direction.EAST)
            elif(currentDir == Direction.EAST):
                nextNode = (currentNode[0], currentNode[1], Direction.NORTH)
            elif(currentDir == Direction.WEST):
                nextNode = (currentNode[0], currentNode[1], Direction.SOUTH)
            return nextNode
        elif move == Action.TURN_RIGHT:
            currentDir = currentNode[2]
            if(currentDir == Direction.NORTH):
                nextNode = (currentNode[0], currentNode[1], Direction.EAST)
            elif(currentDir == Direction.SOUTH):
                nextNode = (currentNode[0], currentNode[1], Direction.WEST)
            elif(currentDir == Direction.EAST):
                nextNode = (currentNode[0], currentNode[1], Direction.SOUTH)
            elif(currentDir == Direction.WEST):
                nextNode = (currentNode[0], currentNode[1], Direction.NORTH)
            return nextNode
        else:
            currentDir = currentNode[2]
            if(currentDir == Direction.NORTH):
               nextNode = (currentNode[0], currentNode[1]-1, currentDir)
            elif(currentDir == Direction.SOUTH):
                nextNode = (currentNode[0], currentNode[1]+1, currentDir)
            elif(currentDir == Direction.EAST):
                nextNode = (currentNode[0]+1, currentNode[1], currentDir)
            elif(currentDir == Direction.WEST):
               nextNode = (currentNode[0]-1, currentNode[1], currentDir)
            return nextNode

    # Runs an iteration of A star. Returns true if the algorithm is done.
    def runAStarIteration(self):
        if self.frontier.empty():
            raise Exception("Was not able to find a path to the destination!")

        current_state = self.frontier.get()
        current_node = current_state[0]
        current_action = current_state[1]

        #Debug
        print("X: %d, Y: %d, Dir: %f, Action that led to the state: %d, Frontier size: %d"
              %(current_node[0], current_node[1], current_node[2], current_action if current_action != None else -1, len(self.frontier.elements)))

        # #priority queue heap can contain duplicates
        # if current not in self.expanded:
        #     #Add the current node to the list of expanded nodes
        #     self.expanded.add(current)

        #Goal test
        if current_node[0] == self.grid.goal[0] and current_node[1] == self.grid.goal[1]:
            self.goal_state = current_state
            return True

        for move in self.getMoves(current_state):
            next_node = self.getNextNode(current_node, move)
            next_state = (next_node, move)

            move_cost = self.cost_so_far[current_state] + self.getPathCost(next_state)

            if next_state not in self.cost_so_far or move_cost < self.cost_so_far[next_state]:
                self.cost_so_far[next_state] = move_cost

                priority = move_cost + self.grid.getHeuristic(next_node, self.grid.goal)
                self.frontier.put(next_state, priority)

                self.parent[next_state] = current_state

        return False

    #Finds path from goal to destination using internal parent list
    def findPath(self):
        path = []
        current_state = self.goal_state

        while current_state != self.start_state:
            path.append(current_state)
            current_state = self.parent[current_state]

        path.append(current_state)
        path.reverse()

        return path

class Grid:
    def __init__(self, filepath, heuristicNum):
        self.parseGrid(filepath)
        self.heuristicNum = heuristicNum
        return

    #Parses the board into list of lists horizontally (grid)
    def parseGrid(self, filepath):
        file = open(filepath, "r")

        rows = file.readlines()

        self.grid = []
        for row in rows:
            chars = filter(None, re.split('\t|\s|\n|\v|\r', row))
            self.grid.append(chars)
            if 'S' in chars:
                self.start = (chars.index('S'), len(self.grid) - 1, Direction.NORTH)
            if 'G' in chars:
                self.goal = (chars.index('G'), len(self.grid) - 1, None)

        self.printBoard()

    #gets heuristic for the two states
    def getHeuristic(self, start, goal):
        dispatcher = {"1": self.heuristic_1,
                      "2": self.heuristic_2,
                      "3": self.heuristic_3,
                      "4": self.heuristic_4,
                      "5": self.heuristic_5,
                      "6": self.heuristic_6}

        return dispatcher[self.heuristicNum](start, goal)


    #Heuristic functi1`on implementations
    def heuristic_1(self, start, goal):
        return 0

    def heuristic_2(self, start, goal):
        vertical = abs(start[1]-goal[1])
        horizontal = abs(start[0]-goal[0])
        return min(vertical, horizontal)

    def heuristic_3(self, start, goal):
        vertical = abs(start[1]-goal[1])
        horizontal = abs(start[0]-goal[0])
        return max(vertical, horizontal)

    def heuristic_4(self, start, goal):
        vertical = abs(start[1]-goal[1])
        horizontal = abs(start[0]-goal[0])
        return (vertical + horizontal)

    def heuristic_5(self, start, goal):
        vertical = abs(start[1]-goal[1])
        horizontal = abs(start[0]-goal[0])

        angle_to_goal = math.atan2(vertical, horizontal)
        angle_diff = Direction.subtract_angles(start[2], angle_to_goal)

        angle_measure = math.ceil(math.fabs(angle_diff) / (math.pi/2))

        return (vertical + horizontal) + angle_measure

    def heuristic_6(self, start, goal):
        return self.heuristic_5(start, goal) * 3

    #Prints the board
    def printBoard(self):
        for row in self.grid:
            for column in row:
                sys.stdout.write(column + ' ')
            print ""
        print "top left is 0,0"
        print "Start = %d %d %f" % (self.start[0], self.start[1], self.start[2])
        print "Goal = %d %d %f" % (self.goal[0], self.goal[1], self.start[2])

if __name__ == "__main__":
    if (len(sys.argv) < 3):
        raise Exception("The number of arguments should be 2 (filepath, heuristic number).")
    board = Grid(sys.argv[1], sys.argv[2])
    pathFinder = PathFinder(board)
    while not pathFinder.runAStarIteration():
        pass

    print pathFinder.findPath()