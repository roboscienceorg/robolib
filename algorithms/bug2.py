from context import map, np
from context import helper
    
    

def bug2(grid, path, start, goal):
    '''
    Bug 2 algorithm. The robot goes straight towards the goal. It uses the 
    distance formula to find the closest point in it's neighborhood. If it hits 
    and obstacle, it follows the boundary until it finds a point which is on 
    the line of formed by the goal and starting point.  
    
    Parameters
    ----------
    grid: 
        2D map containing the robot, goal and obstacles.
    path: 
        2D map containing the path taken by the robot.
    start:
        Starting coordinates of the robot. 
    goal:
        Coordinates of goal.
        
    Returns
    -------
    Path taken by the robot.
    '''
    
    start_x = start[0]
    start_y = start[1]
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
            curr_x, curr_y = helper.next_point(curr_x, curr_y, goal, path)
            path[initial_x][initial_y] = 20
            
            if (helper.reached_goal(grid, curr_x, curr_y)):
                return True, path
        
        curr_x = initial_x
        curr_y = initial_y

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
            
            if prev_x == curr_x and prev_y == curr_y:
                return False, path
            prev_x = curr_x
            prev_y = curr_y
            
            if (curr_x == initial_x and curr_y == initial_y):
                initial_reached = True
            if (helper.reached_goal(grid, curr_x, curr_y)):
                return True, path
                
            try:
                if (curr_y-start_y)/(curr_x-start_x) == (curr_y-goal_y)/(curr_x-goal_x):
                    break
            except ZeroDivisionError:
                if curr_x-start_x == curr_x-goal_x:
                    continue
                    
    
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
    
    max_index = len(grid)
    helper.max_index = max_index
    reached, path = bug2(grid, path, start, goal)
    
    if not reached:
        print("No path found")
    helper.display(path)



