from context import np, plt

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
    Checks if an obstacle is in the Moore Neighborhood (r=1) of the point. 
    
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
    Returns the distance between the current point and the goal.
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



