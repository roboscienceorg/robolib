import numpy as np

class Trig_Error(Exception):
    def __init__(self, message):
        self.message = message

class TriangLasers():
    '''
    This class represents a process of sensor location known and Triangulation.
    This class will be used to give an approximate are in which a item would
    be located given 3 sensor locations and the range at when they each found
    the item.
    '''

    def __init__(self, positions):
        