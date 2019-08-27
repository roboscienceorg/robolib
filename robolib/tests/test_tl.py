import numpy as np

from context import tl
from random import random, randint
from nose.tools import raises

MAX_INT = 10000
MIN_INT = 1

# Helper for comparison
def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

def test_tl_instatiation_randomint_success():
    x = randint(MIN_INT, MAX_INT)
    y = randint(MIN_INT, MAX_INT)

    bot = tl.TL(x, y)
    assert isinstance(bot, tl.TL)


@raises(ValueError)   
def test_tl_instatiation_zeros_failure():
    bot = tl.TL(0, 0)


@raises(ValueError)
def test_tl_instatiation_negative_link1_failure():
    x = randint(MIN_INT, MAX_INT) * -1
    y = randint(MIN_INT, MAX_INT)
    
    bot = tl.TL(x, y)


@raises(ValueError)
def test_tl_instatiation_negative_link2_failure():
    x = randint(MIN_INT, MAX_INT) 
    y = randint(MIN_INT, MAX_INT) * -1
    
    bot = tl.TL(x, y)

def test_tl_getters_success():
    x = randint(MIN_INT, MAX_INT)
    y = randint(MIN_INT, MAX_INT)

    bot = tl.TL(x, y)
    assert bot.link_1 == x
    assert bot.link_2 == y
    assert bot.theta_1 == 0.0
    assert bot.theta_2 == 0.0
    assert bot.x == x + y
    assert bot.y == 0.0

def test_TL_IK_bookexample_success():
    bot = tl.TL(15.0, 10.0)

    ang_list = bot.IK( [(10.0, 8.0)] )
    assert ang_list[0][0] > 1.393 and ang_list[0][0] < 1.395
    assert ang_list[0][1] > -2.138 and ang_list[0][1] < -2.136

def test_TL_IK_FK_bookexample_success():
    bot = tl.TL(15.0, 10.0)

    ang_list = bot.IK( [(10.0, 8.0)] )
    pos_list = bot.FK( ang_list )

    assert isclose( pos_list[0][0], 10.0 )
    assert isclose( pos_list[0][1], 8.0 )

