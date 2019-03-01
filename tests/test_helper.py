import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import algorithms.helper as helper
from math import isclose

# Full grid: Range on x axis: 20, Range on y axis: 20, Points per side: 20 
grid = [[-2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
        [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 0, 0, 0, 0, 0, 0,-1,-1,-1,-1,-1,-1,-1,-1, 0, 0, 0, 0, 0],
        [ 0, 0, 0, 0, 0, 0,-1,-1,-1,-1,-1,-1,-1,-1, 0, 0, 0, 0, 0],
        [ 0, 0, 0, 0, 0, 0,-1,-1,-1,-1,-1,-1,-1,-1, 0, 0, 0, 0, 0],
        [ 0, 0, 0, 0, 0, 0,-1,-1,-1,-1,-1,-1,-1,-1, 0, 0, 0, 0, 0],
        [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]]
        
# Full path
path = [[20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 0,20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
        [ 0, 0,20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 0, 0, 0,20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 0, 0, 0, 0,20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 0, 0, 0, 0, 0,20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 0, 0, 0, 0, 0,20,20,20,20,20,20,20,20,20,20, 0, 0, 0, 0, 0],
        [ 0, 0, 0, 0, 0,20, 0, 0, 0, 0, 0, 0, 0, 0,20, 0, 0, 0, 0, 0],
        [ 0, 0, 0, 0, 0,20, 0, 0, 0, 0, 0, 0, 0, 0,20, 0, 0, 0, 0, 0],
        [ 0, 0, 0, 0, 0,20, 0, 0, 0, 0, 0, 0, 0, 0,20, 0, 0, 0, 0, 0],
        [ 0, 0, 0, 0, 0,20, 0, 0, 0, 0, 0, 0, 0, 0,20, 0, 0, 0, 0, 0],
        [ 0, 0, 0, 0, 0,20,20,20,20,20,20,20,20,20,20, 0, 0, 0, 0, 0],
        [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,20, 0, 0, 0, 0],
        [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,20, 0, 0, 0],
        [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,20, 0, 0],
        [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,20, 0],
        [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,20, 0],
        [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,20, 0],
        [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]



def test_reached_goal():
    assert helper.reached_goal(grid, 18, 18) == True
    assert helper.reached_goal(grid, 0, 0) == False



def test_if_boundary():
    helper.max_index = 19
    temp_path = [[20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 [ 0,20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
                 [ 0, 0,20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 [ 0, 0, 0,20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 [ 0, 0, 0, 0,20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 [ 0, 0, 0, 0, 0,20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 [ 0, 0, 0, 0, 0, 0,20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]
    
    # Cannot go to that point because it has been visited
    assert helper.if_boundary(6, 6, grid, temp_path) == False
    # Point not next to an obstacle
    assert helper.if_boundary(5, 6, grid, temp_path) == False
    # Point is an obstacle
    assert helper.if_boundary(7, 6, grid, temp_path) == False
    # Points next to an obstacle
    assert helper.if_boundary(6, 7, grid, temp_path) == True
    assert helper.if_boundary(6, 5, grid, temp_path) == True
    assert helper.if_boundary(11, 7, grid, temp_path) == True



def test_get_distance():
    goal = (18, 18)
    assert helper.get_distance(18, 18, goal) == 0
    assert isclose(helper.get_distance(0, 0, goal), 25.4558441, rel_tol=1e-6) == True
    assert isclose(helper.get_distance(12, 6, goal), 13.4164078, rel_tol=1e-6) == True
    assert isclose(helper.get_distance(13, 6, goal), 13, rel_tol=1e-6) == True
    assert isclose(helper.get_distance(13, 5, goal), 13.92838827, rel_tol=1e-6) == True
    assert isclose(helper.get_distance(12, 5, goal), 14.31782106, rel_tol=1e-6) == True
    assert isclose(helper.get_distance(11, 5, goal), 14.76482306, rel_tol=1e-6) == True
    assert isclose(helper.get_distance(11, 6, goal), 13.89244398, rel_tol=1e-6) == True
    assert isclose(helper.get_distance(11, 7, goal), 13.03840481, rel_tol=1e-6) == True
    assert isclose(helper.get_distance(12, 7, goal), 12.52996408, rel_tol=1e-6) == True
    assert isclose(helper.get_distance(13, 7, goal), 12.08304597, rel_tol=1e-6) == True
    
    goal = (500, 500)
    assert isclose(helper.get_distance(0, 0, goal), 707.1067811, rel_tol=1e-6) == True


    
def test_next_point():
    goal = (18, 18)
    helper.max_index = 19
    temp_path = [[ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
                 [ 0, 0,20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 [ 0, 0, 0,20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 [ 0, 0, 0, 0, 0,20,20,20,20,20,20,20,20,20,20, 0, 0, 0, 0, 0],
                 [ 0, 0, 0, 0, 0,20, 0, 0, 0, 0, 0, 0, 0, 0,20, 0, 0, 0, 0, 0],
                 [ 0, 0, 0, 0, 0,20, 0, 0, 0, 0, 0, 0, 0, 0,20, 0, 0, 0, 0, 0],
                 [ 0, 0, 0, 0, 0,20, 0, 0, 0, 0, 0, 0, 0, 0,20, 0, 0, 0, 0, 0],
                 [ 0, 0, 0, 0, 0,20, 0, 0, 0, 0, 0, 0, 0, 0,20, 0, 0, 0, 0, 0],
                 [ 0, 0, 0, 0, 0,20,20,20,20,20,20,20,20,20, 0, 0, 0, 0, 0, 0],
                 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,20, 0, 0, 0],
                 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,20, 0, 0],
                 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,20, 0],
                 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]
    
    # Note that some points in the temp_path variable which are supposed to be 
    #   20 are made 0 for illustration. This can be compared to the 'path'
    #   variable at the top of this file. 
    assert helper.next_point(0, 0, goal, temp_path) != (1, 0)
    assert helper.next_point(0, 0, goal, temp_path) == (1, 1)
    assert helper.next_point(4, 4, goal, temp_path) == (5, 5)
    assert helper.next_point(4, 4, goal, temp_path) == (5, 5)
    assert helper.next_point(11, 14, goal, temp_path) == (12, 15)
    assert helper.next_point(15, 18, goal, temp_path) == (16, 18)



