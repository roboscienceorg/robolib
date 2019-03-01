from context import map, np
from context import helper


max_index = -1


def neighbor(initial_x, initial_y, curr_x, curr_y, grid):
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
    grid: 
        2D map containing robot and obstacles.
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
    


def bug1(grid, path, start, goal):
    '''
    Bug 1 algorithm. Robot starts going towards the goal in a straight line. If 
    it hits an obstacle, it follows the boundary and loops around it and then 
    goes to the point which is closest to the goal. Then repeats. 
    
    Parameter
    ---------
    grid: 
        2D map containing the robot, goal and obstacles.
    path: 
        2d map containing path taken by robot.
    start:
        Starting coordinates of the robot. 
    goal:
        Coordinates of goal.
        
    Returns
    -------
    True or false depending on whether robot found the path, and 
    Path taken by the robot.    
    '''
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
            #  Neighborhood (r=1) of the point. 
            curr_x, curr_y = helper.next_point(curr_x, curr_y, goal, path)
            path[initial_x][initial_y] = 20
            
            if (helper.reached_goal(grid, curr_x, curr_y)):
                return True, path
        
        curr_x = initial_x
        curr_y = initial_y

        distances = [((curr_x, curr_y), helper.get_distance(curr_x, curr_y, goal))]
        initial_reached = False
        
        # Follows boundary of the obstacle
        while(not initial_reached):
        
            # Checks right, down, left and right
            if curr_x+1 < max_index and helper.if_boundary(curr_x+1, curr_y, grid, path):
                curr_x += 1
            elif curr_y-1 > -1 and helper.if_boundary(curr_x, curr_y-1, grid, path):
                curr_y -= 1
            elif curr_x-1>-1 and helper.if_boundary(curr_x-1, curr_y, grid, path):
                curr_x -= 1
            elif curr_y+1 < max_index and helper.if_boundary(curr_x, curr_y+1, grid, path):
                curr_y += 1
            
            path[curr_x][curr_y] = 20

            # Checks if it reached the initial position of the loop
            if curr_x == prev_x and curr_y == prev_y and neighbor(initial_x, initial_y, curr_x, curr_y, grid):
                initial_reached = True
            elif curr_x == prev_x and curr_y == prev_y:
                return False, path
            if (helper.reached_goal(grid, curr_x, curr_y)):
                return True, path
                
            prev_x = curr_x
            prev_y = curr_y

            distances.append(((curr_x, curr_y), helper.get_distance(curr_x, curr_y, goal)))

        # Finds the closest point to the goal
        distances = sorted(distances, key=lambda dist: dist[1])

        curr_x = distances[0][0][0]
        curr_y = distances[0][0][1]
        
    return True, path
    

    
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
    path = np.zeros( (m.pps, m.pps) )

    # starting point
    grid[0][0] = -2
    start = (0, 0)

    #ending point
    grid[n_points-2][n_points-2] = 1
    goal = (n_points-2, n_points-2)
    
    print("Start:", start)
    print("Goal:", goal)

    helper.max_index = len(grid)
    max_index = helper.max_index
    reached, path = bug1(grid, path, start, goal)
    
    if not reached:
        print("No path found")
    helper.display(path)



