from context import map, np
from context import helper

from math import sqrt
from queue import Queue

# Starting from 3rd column and 2nd row and going clockwise
di = [1, 1, 0, -1, -1, -1, 0, 1]
dj = [0, -1, -1, -1, 0, 1, 1, 1]

# 2D grid containing path followed by robot.
path = []
# Initial coordinates of robot
start = ()
# Coordinates of the goal.
goal = ()
# 2D map containing the robot, obstacles and goal.
grid = []
# 2D grid of booleans which tells if a robot has visited a cell or not. 
visited = []
# Checks if the robot reached the goal
reached = False
# For debug
check = []



def bfs():
    '''
    Using breadth first search algorithm to find distance of goal from each 
    point and save it in the 'grid' variable which already has the starting, 
    ending and obstacles coordinates. This will help in finding the shortest 
    point when using the bug3 algorithm. 
    
    Code taken from Aaron Campbell's wavefront algorithm. 
    '''
    global grid
    
    q = Queue()
    finished = False
    goal_x = start[0]
    goal_y = start[1]
    q.put((goal[0], goal[1], 1))
    
    while q.empty() == False or (curr_x == goal_x and curr_y == goal_y):
        el = q.get()
        curr_x = el[0]
        curr_y = el[1]
        count = el[2]
        
        # Left side
        if curr_x != 0:
            # Left Bottom
            if curr_y != 0:
                if grid[curr_x-1][curr_y-1] == 0:
                    grid[curr_x-1][curr_y-1] = count+1
                    q.put( (curr_x-1, curr_y-1, count+1) )
            # Left Top
            if curr_y != max_index-1:
                if grid[curr_x-1][curr_y+1] == 0:
                    grid[curr_x-1][curr_y+1] = count+1
                    q.put( (curr_x-1, curr_y+1, count+1) )

            # Left Middle
            if grid[curr_x-1][curr_y] == 0:
                grid[curr_x-1][curr_y] = count+1
                q.put( (curr_x-1, curr_y, count+1) )

        # Right side
        if curr_x != max_index-1:
            # Right Bottom
            if curr_y != 0:
                if grid[curr_x+1][curr_y-1] == 0:
                    grid[curr_x+1][curr_y-1] = count+1
                    q.put( (curr_x+1, curr_y-1, count+1) )

            # Right Top
            if curr_y != max_index-1:
                if grid[curr_x+1][curr_y+1] == 0:
                    grid[curr_x+1][curr_y+1] = count+1
                    q.put( (curr_x+1, curr_y+1, count+1) )

            # Right Middle
            if grid[curr_x+1][curr_y] == 0:
                grid[curr_x+1][curr_y] = count+1
                q.put( (curr_x+1, curr_y, count+1) )


        # Middle Bottom
        if curr_y != 0:
            if grid[curr_x][curr_y-1] == 0:
                grid[curr_x][curr_y-1] = count+1
                q.put( (curr_x, curr_y-1, count+1) )

        # Middle Top
        if curr_y != max_index-1:
            if grid[curr_x][curr_y+1] == 0:
                grid[curr_x][curr_y+1]
                q.put( (curr_x, curr_y+1, count+1) )

                


def get_neighbors(curr_x, curr_y, grid):
    '''
    Checks the Moore Neighborhood of the robot and returns all valid points the 
    robot can go to.
    
    Parameter
    ---------
    curr_x: Numeric
        x location of the robot.
    curr_y: Numeric
        y location of the robot.
    grid:
        2D map containing the robot, goal and obstacles.
        
    Returns
    -------
    isBoundary: Boolean
        Checks if a neighbor is next to an obstacle.
    neighbors: Lists
        Valid neighbors.
    '''
    neighbors = []
    isBoundary = False
    
    for k in range(8):
        nextI = curr_x + di[k]
        nextJ = curr_y + dj[k]
        if (nextI > -1 and nextI < helper.max_index and nextJ > -1 and nextJ < helper.max_index):
            if grid[nextI][nextJ] == -1:
                isBoundary = True
            else:
                neighbors.append((nextI, nextJ, grid[nextI][nextJ]))
                
    neighbors = sorted(neighbors, key=lambda dist: dist[2])

    return isBoundary, neighbors
    
    

def tangentBug(curr_x, curr_y, prev_i, prev_j):
    '''
    Tangent bug algorithm. The robot goes straight towards the goal. If it hits 
    an obstacle, it follows the boundary which is closer to the goal until it 
    reaches a tangent point, after which it goes straight towards the goal. 
    Uses modified Depth First Algorthm.
    
    Parameters
    ----------
    curr_x: Numeric
        Current x location of the robot.
    curr_y: Numeric
        Current y location of the robot.
    prev_i: Numeric
        Previous x location of the robot.
    prev_j: Numeric
        Previos y location of the robot.
        
    Returns
    -------
    True: Boolean
        Path to goal is found.
    False: Boolean
        No path exists.
    '''
    
    # Global variables
    global path, visited, check, reached
    
    # For debugging
    check.append((curr_x, curr_y))
    
    # Base case
    if reached or helper.reached_goal(grid, curr_x, curr_y):
        reached = True
        return True
        
    # Gets neighbors and checks if the robot is next to an obstacle.
    isBoundary, neighbors = get_neighbors(curr_x, curr_y, grid)
    
    # Next to an obstacle.
    if isBoundary:
        # DFS
        for point in neighbors:
            if not visited[point[0]][point[1]]:
                path[point[0]][point[1]] = 20
                visited[point[0]][point[1]] = True
                found = tangentBug(point[0], point[1], curr_x, curr_y)
                if found:
                    return True
       
    # Not next to an obstacle         
    else:
        # Getting a valid neighbor which is closest to the goal.
        shortestDistance = []
        for point in neighbors:
            if not visited[point[0]][point[1]] and grid[point[0]][point[1]] != -1:
                shortestDistance.append(((point[0], point[1]), helper.get_distance(point[0], point[1], goal)))
        shortestDistance = sorted(shortestDistance, key=lambda dist: dist[1])

        # DFS
        for point in shortestDistance:
            visited[point[0][0]][point[0][1]] = True
            path[point[0][0]][point[0][1]] = 20
            found = tangentBug(point[0][0], point[0][1], curr_x, curr_y)
            if found:
                return True

    return False
    


if __name__ == "__main__":   
    '''
    Main function which initializes the map with the coordinates of the robot, 
    goal and obstacles.
    ''' 
    x_max = int(input("Range on the x axis: "))
    y_max = int(input("Range on the y axis: "))
    n_points = int(input("Points per side: "))

    m = map.Map(x_max, y_max, n_points)

    #Add obstacles
    m.edit()

    grid = m.map

    # starting point
    grid[0][0] = -2
    start = (0, 0)

    #ending point
    grid[n_points-2][n_points-2] = 1
    goal = (n_points-2, n_points-2)
    
    print("Start:", start)
    print("Goal:", goal)
    
    # Initializing globals
    path = np.zeros( (m.pps, m.pps) )
    path[start[0]][start[1]] = 20
    max_index = len(grid)
    helper.max_index = max_index
    visited = [[False for _ in range(max_index)] for _ in range(max_index)]
    visited[start[0]][start[1]] = True
    
    try:
        bfs()
        found_goal = tangentBug(start[0], start[1], start[0], start[1])
        if not found_goal:
            print("Could not find path")
        helper.display(path)
        
    # Debug
    except:
        helper.display(path)
        print("Error")



