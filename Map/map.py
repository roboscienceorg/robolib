import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from math import modf
from matplotlib.widgets import PolygonSelector, Button
from matplotlib.path import Path

class Map():
    '''
    This class provides a GUI interface and map object that represents a
    robot's environment. This will be used to graph a robots movements in
    an environment
    '''
    def __init__(self, length=100, width=100, points_per_side=100):
        self._l = length
        self._w = width
        self._pps = points_per_side

        # Get figure objects and adjust for buttons
        self._fig, self._ax = plt.subplots()
        plt.subplots_adjust(bottom=0.2)

        # Build map of points
        points_x = np.tile(np.arange(self._pps), self._pps)
        points_y = np.repeat(np.arange(self._pps), self._pps)
        self._pts = self._ax.scatter(points_x, points_y)
        self._coordinates = self._pts.get_offsets()

        #Make points invisible
        #self._pts.set_facecolors([0, 0, 0, 0])
        self._pts.remove()

        self._poly_select = PolygonSelector(self._ax, self.on_poly_select)

        self._map = np.zeros( (self._pps, self._pps) )

        # Build buttons
        button_location = plt.axes([0.8, 0.05, 0.15, 0.075])
        self._add_object_button = Button(button_location, 'Add Object')
        self._add_object_button.on_clicked(self.click_add_object)

        button_location = plt.axes([0.6, 0.05, 0.18, 0.075])
        self._clear_axes_button = Button(button_location, "Clear Objects")
        self._clear_axes_button.on_clicked(self.click_add_object)

        self._poly_select.set_active(False)

    def click_add_object(self, event):
        self._poly_select.set_active(True)

    def click_clear(self):
        self._ax.clear()

    @property
    def length(self):
        return self._l

    @property
    def width(self):
        return self._w

    def on_poly_select(self, verts):
        #Close and create patch TODO: Replace with Draggable
        verts.append(verts[0])
        path = Path(verts)

        patch = patches.PathPatch(path)
        self._ax.add_patch(patch)

        self._poly_select.set_visible(True)
        self._poly_select.update()
        self._poly_select._xs, self._poly_select._ys = [0], [0]
        self._poly_select._polygon_completed = False
        self._poly_select.update()
        self._poly_select.set_active(False)

if __name__ == "__main__":
    x = Map()
    plt.show()
