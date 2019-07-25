import numpy as np
import matplotlib.pyplot as plt

from context import sensor
from nose.tools import raises
from random import random, randint

MAX_INT = 10000
MIN_INT = -10000

def test_sensor_instantiation_success_1D():
    x = randint(MIN_INT, MAX_INT) + random()
    y = randint(MIN_INT, MAX_INT) + random()

    s = sensor.Sensor([x], [y])
    assert isinstance(s, sensor.Sensor)

def test_sensor_instantiation_success_2D():
    x = [randint(MIN_INT, MAX_INT) + random()]
    y = [randint(MIN_INT, MAX_INT) + random()]
    z = [randint(MIN_INT, MAX_INT) + random()]

    s = sensor.Sensor([x, y], [[x, y], [y, z]])
    assert isinstance(s, sensor.Sensor)

@raises(ValueError)
def test_sensor_instantiation_failure_mean_size():
    x = [randint(MIN_INT, MAX_INT) + random()]
    y = [randint(MIN_INT, MAX_INT) + random()]
    z = [randint(MIN_INT, MAX_INT) + random()]

    s = sensor.Sensor([ [x], [y] ], [z] )

@raises(ValueError)
def test_sensor_instantiation_failure_mean():
    x = [randint(MIN_INT, MAX_INT) + random()]
    s = sensor.Sensor("not a number", x)

@raises(ValueError)
def test_sensor_instantiation_failure_cov():
    x = [randint(MIN_INT, MAX_INT) + random()]
    s = sensor.Sensor(x, "Not a number")

@raises(ValueError)
def test_sensor_instantiation_failure_mismatched_dimensions():

    s = sensor.Sensor([0.0], [ [0.1, 0.3], [0.3, 0.4]])

# Currently these will be tested only for one dimension

def test_mean_getter():
    x = randint(MIN_INT, MAX_INT) + random()
    y = randint(MIN_INT, MAX_INT) + random()

    s = sensor.Sensor([x], [y])
    assert s.mean == x

def test_mean_setter():
    x = randint(MIN_INT, MAX_INT) + random()
    y = randint(MIN_INT, MAX_INT) + random()
    z = randint(MIN_INT, MAX_INT) + random()

    s = sensor.Sensor([x], [y])
    s.mean = z
    assert s.mean == z

def test_cov_getter():
    x = randint(MIN_INT, MAX_INT) + random()
    y = randint(MIN_INT, MAX_INT) + random()

    s = sensor.Sensor([x], [y])
    assert s.mean == y

def test_cov_setter():
    x = randint(MIN_INT, MAX_INT) + random()
    y = randint(MIN_INT, MAX_INT) + random()
    z = randint(MIN_INT, MAX_INT) + random()

    s = sensor.Sensor([x], [y])
    s.cov = [z]
    assert s.cov == [z]

def test_dimension_getter():
    x = randint(MIN_INT, MAX_INT) + random()
    y = randint(MIN_INT, MAX_INT) + random()

    s = sensor.Sensor([x], [y])
    assert s.dimension == 1
    

# TESTING OF THE SENSE FUNCTION IS NOT CURRENTLY IMPLEMENTED DUE TO THE
# STATISTICAL NATURE OF THE OUTPUTS. THIS MAY BE IMPLEMENTED IN THE FUTURE.
