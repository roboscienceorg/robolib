import numpy as np
import matplotlib.pyplot as plt

import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from tools.map import Map
from math import sqrt


max_index = -1


def reached_goal(grid, x, y):
    '''
    Checks if the robot reached the goal.
    
    Parameters
    ----------
    grid: 
        2D map containing the robot, goal and obstacles.  
    x: Numeric
        Current x location of the robot.
    y: Numeric
        Current y location of the robot.
        
    Returns
    -------
    True: Boolean
        Robot reached the goal.
    False: Boolean
        Robot did not reached the goal.
    '''
    if (grid[x][y] == 1):
        return True
    return False



def if_boundary(curr_x, curr_y, grid, path):
    '''
    Checks if an obstacle is in the Moore Neighborhood of the point. 
    
    Parameters
    ----------
    curr_x: Numeric
        Current x location of the point.
    curr_y: Numeric
        Current y location of the point.
    grid: 
        2D map containing the robot, goal and obstacles.
    path: 
        2D map containing the path of the robot.
        
    Returns
    -------
    True: Boolean
        Point is next to an obstacle.
    False: Boolean
        Point is not next to an obstacle. 
    '''
    if (grid[curr_x][curr_y] != 0 or path[curr_x][curr_y] != 0):
        return False
    if (curr_x+1 < max_index and grid[curr_x+1][curr_y] == -1):
        return True
    if (curr_x+1 < max_index and curr_y-1 > -1 and grid[curr_x+1][curr_y-1] == -1):
        return True
    if (curr_y-1 > -1 and grid[curr_x][curr_y-1] == -1):
        return True
    if (curr_x-1 > -1 and curr_y-1 > -1 and grid[curr_x-1][curr_y-1] == -1):
        return True
    if (curr_x-1 > -1 and grid[curr_x-1][curr_y] == -1):
        return True
    if (curr_x-1 > -1 and curr_y+1 < max_index and grid[curr_x-1][curr_y+1] == -1):
        return True
    if (curr_y+1 < max_index and grid[curr_x][curr_y+1] == -1):
        return True
    if (curr_x+1 < max_index and curr_y+1 < max_index and grid[curr_x+1][curr_y+1] == -1):
       return True
    return False



def get_distance(curr_x, curr_y, goal):
    '''
    Parameters
    ----------
    curr_x: Numeric
        Current x location of point.
    curr_y: Numeric
        Current y location of point. 
    goal: Numeric
        Coordinates of goal on the grid.
    
    Returns
    -------
    The distance between 2 points.
    '''
    return np.sqrt((curr_x-goal[0])*(curr_x-goal[0]) + (curr_y-goal[1])*(curr_y-goal[1]))



def next_point(curr_x, curr_y, goal, path):
    '''
    Gets the next point the robot should move to when going towards a goal. 
    This basically checks all valid points around the robot and returns the 
    point which is the closest to the goal. It uses the distance formula derived 
    from the Pythagorean Theorem.  
    
    Parameters
    ----------
    curr_x: Numeric
        Current x location of robot.
    curr_y: Numeric
        Current y location of robot. 
    goal: Numeric
        Coordinates of goal on the 2D grid.
    path: 
        2D map containing the path of the robot.
        
    Returns
    -------
    (x,y) coordinates of the point closest to the goal.
    '''
    points = []
    if (curr_x+1 < max_index and path[curr_x+1][curr_y] == 0):
        points.append(((curr_x+1, curr_y), get_distance(curr_x+1, curr_y, goal)))
    if (curr_x+1 < max_index and curr_y-1 > -1 and path[curr_x+1][curr_y-1] == 0):
        points.append(((curr_x+1, curr_y-1), get_distance(curr_x+1, curr_y-1, goal)))
    if (curr_y-1 > -1 and path[curr_x][curr_y-1] == 0):
        points.append(((curr_x, curr_y-1), get_distance(curr_x, curr_y-1, goal)))
    if (curr_x-1 > -1 and curr_y-1 > -1 and path[curr_x-1][curr_y-1] == 0):
        points.append(((curr_x-1, curr_y-1), get_distance(curr_x-1, curr_y-1, goal)))
    if (curr_x-1 > -1 and path[curr_x-1][curr_y] == 0):
        points.append(((curr_x-1, curr_y), get_distance(curr_x-1, curr_y, goal)))
    if (curr_x-1 > -1 and curr_y+1 < max_index and path[curr_x-1][curr_y+1] == 0):
        points.append(((curr_x-1, curr_y+1), get_distance(curr_x-1, curr_y+1, goal)))
    if (curr_y+1 < max_index and path[curr_x][curr_y+1] == 0):
        points.append(((curr_x, curr_y+1), get_distance(curr_x, curr_y+1, goal)))
    if (curr_x+1 < max_index and curr_y+1 < max_index and path[curr_x+1][curr_y+1] == 0):
        points.append(((curr_x+1, curr_y+1), get_distance(curr_x+1, curr_y+1, goal)))
    
    points = sorted(points, key=lambda dist: dist[1])
    curr_x = points[0][0][0]
    curr_y = points[0][0][1]
    
    return curr_x, curr_y



def neighbor(initial_x, initial_y, curr_x, curr_y):
    '''
    Checks if the robot reached the initial point before following the boundary 
    of the obstacle. 
    
    Parameter
    ---------
    initial_x: Numeric
        Starting x location of the loop.
    initial_y: Numeric
        Starting y location of the loop.
    curr_x: Numeric
        Current x location of the point.
    curr_y: Numeric
        Current y location of the point.
    
    Returns
    -------
    True: Boolean
        Robot completed the loop around the obstacle.
    False: Boolean
        Robot did not loop around the obstacle yet.
    '''
    points = []
    if (curr_x+1 < max_index and grid[curr_x+1][curr_y] != -1):
        points.append((curr_x+1, curr_y))
    if (curr_x+1 < max_index and curr_y-1 > -1 and grid[curr_x+1][curr_y-1] != -1):
        points.append((curr_x+1, curr_y-1))
    if (curr_y-1 > -1 and grid[curr_x][curr_y-1] != -1):
        points.append((curr_x, curr_y-1))
    if (curr_x-1 > -1 and curr_y-1 > -1 and grid[curr_x-1][curr_y-1] != -1):
        points.append((curr_x-1, curr_y-1))
    if (curr_x-1 > -1 and grid[curr_x-1][curr_y] != -1):
        points.append((curr_x-1, curr_y))
    if (curr_x-1 > -1 and curr_y+1 < max_index and grid[curr_x-1][curr_y+1] != -1):
        points.append((curr_x-1, curr_y+1))
    if (curr_y+1 < max_index and grid[curr_x][curr_y+1] != -1):
        points.append((curr_x, curr_y+1))
    if (curr_x+1 < max_index and curr_y+1 < max_index and grid[curr_x+1][curr_y+1] != -1):
        points.append((curr_x+1, curr_y+1))
        
    if (initial_x, initial_y) in points:
        return True
    return False
    


def bug1(grid, m, start, goal):
    '''
    Bug 1 algorithm. Robot starts going towards the goal in a straight line. If 
    it hits an obstacle, it follows the boundary and loops around it and then 
    goes to the point which is closest to the goal. Then repeats. 
    
    Parameter
    ---------
    grid: 
        2D map containing the robot, goal and obstacles.
    m: 
        Map object.
    start:
        Starting coordinates of the robot. 
    goal:
        Coordinates of goal.
        
    Returns
    -------
    Path taken by the robot.    
    '''
    path = np.zeros( (m.pps, m.pps) )
    goal_x = goal[0]
    goal_y = goal[1]
    
    curr_x = start[0]
    curr_y = start[1]
    
    prev_x = curr_x
    prev_y = curr_y

    while True:
        initial_x = curr_x
        initial_y = curr_y
        # Goes straight towards the goal until it hits an obstacle. 
        while(grid[curr_x][curr_y] != -1):
            initial_x = curr_x
            initial_y = curr_y
            # Getting closest point to the goal which is in the Moore 
            #  Neighborhood of the point. 
            curr_x, curr_y = next_point(curr_x, curr_y, goal, path)
            path[initial_x][initial_y] = 20
            
            if (reached_goal(grid, curr_x, curr_y)):
                return True, path
        
        curr_x = initial_x
        curr_y = initial_y

        distances = [((curr_x, curr_y), get_distance(curr_x, curr_y, goal))]
        initial_reached = False
        
        # Follows boundary of the obstacle
        while(not initial_reached):
        
            # Checks right, down, left and right
            if curr_x+1 < max_index and if_boundary(curr_x+1, curr_y, grid, path):
                curr_x += 1
            elif curr_y-1 > -1 and if_boundary(curr_x, curr_y-1, grid, path):
                curr_y -= 1
            elif curr_x-1>-1 and if_boundary(curr_x-1, curr_y, grid, path):
                curr_x -= 1
            elif curr_y+1 < max_index and if_boundary(curr_x, curr_y+1, grid, path):
                curr_y += 1
            
            path[curr_x][curr_y] = 20

            # Checks if it reached the initial position of the loop
            if curr_x == prev_x and curr_y == prev_y and neighbor(initial_x, initial_y, curr_x, curr_y):
                initial_reached = True
            elif curr_x == prev_x and curr_y == prev_y:
                return False, path
            if (reached_goal(grid, curr_x, curr_y)):
                return True, path
                
            prev_x = curr_x
            prev_y = curr_y

            distances.append(((curr_x, curr_y), get_distance(curr_x, curr_y, goal)))

        # Finds the closest point to the goal
        distances = sorted(distances, key=lambda dist: dist[1])

        curr_x = distances[0][0][0]
        curr_y = distances[0][0][1]
        
    return True, path



def display(path):
    '''
    Displays the path taken by the robot.  
    
    Parameters
    ----------
    path: 
        2D map containing the path of the robot.
    '''
    grid2 = np.rot90(path)
    plt.imshow(grid2)
    plt.axis('off')
    plt.show()
    

    
if __name__ == "__main__":
    '''
    Main function which initializes the map with the coordinates of the robot, 
    goal and obstacles.
    '''
    x_max = int(input("Range on the x axis: "))
    y_max = int(input("Range on the y axis: "))
    n_points = int(input("Points per side: "))

    m = Map(x_max, y_max, n_points)

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

    max_index = len(grid)
    reached, path = bug1(grid, m, start, goal)
    
    if reached:
        display(path)
    else:
        print("No path found")
