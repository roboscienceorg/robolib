import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from math import modf, floor
from matplotlib.widgets import PolygonSelector, Button
from matplotlib.path import Path

class Map():
    """
    This class provides a GUI interface and map object that represents a
    robot's environment. This will be used to graph a robots movements in
    an environment
    """

    def __init__(self, height=100, width=100, points_per_side=1000):
        """
        This function creates and initializes the Map with the given parameters.

        Parameters
        ----------
        height:  Integer
            The height of the map
        width:  Integer
            The width of the map
        points_per_side:      Integer
            The number of points on the map grid per side (x and y)

        Returns
        -------
        Map object with the given parameters after the user has edited.
        """

        self._h = height
        self._w = width
        self._pps = points_per_side
        self._objects = []

        # Get figure objects and adjust for buttons
        self.setup()

        # Build map of points
        points_x = np.tile(np.linspace(0, self._w, self._pps), self._pps)
        points_y = np.repeat(np.linspace(0, self._h, self._pps), self._pps)
        self._pts = self._ax.scatter(points_x, points_y)
        self._coordinates = self._pts.get_offsets()

        #Make points invisible
        #self._pts.set_facecolors([0, 0, 0, 0])
        self._pts.remove()

        self._map = np.zeros( (self._pps - 1, self._pps - 1) )


    def edit(self):
        try:
            plt.close(self._fig)
        except:
            print("Exception")
            pass

        self.setup()
        self._fig.show()

    def click_add_object(self, event):
        """
        Callback for the 'Add Object' button
        """
        self._poly_select.set_active(True)

    def click_clear(self, event):
        """
        Callback for the 'Clear Objects' button
        """
        for item in self._objects:
            item.remove()

        self._objects.clear()
        self._map = np.zeros( (self._pps - 1, self._pps - 1) )

    def click_finished(self, event):
        """
        Callback for the 'Finished' button
        """
        plt.close(self._fig)

    @property
    def height(self):
        """
        The height of the map (Max y value)
        """
        return self._h

    @property
    def width(self):
        """
        The width of the map (Max x value)
        """
        return self._w

    @property
    def map(self):
        """
        The map defined by the object. -1 indicates an obstacle.
        """
        return self._map

    @map.setter
    def map(self, new_map):
        """
        This is a setter for the internal map object. This allows the user
        to manually edit a copy of the map and set it back in the object.

        """

        self._map = new_map

    def setup(self):
        # Get figure objects and adjust for buttons
        self._fig = plt.figure()
        self._ax = self._fig.add_subplot(1, 1, 1)
        self._ax.set_title("Map Editor")
        self._fig.canvas.set_window_title('RoboLib')
        self._ax.set_xlim( [0, self._w] )
        self._ax.set_ylim( [0, self._h] )
        plt.subplots_adjust(bottom=0.2)

        # Build buttons
        button_location = plt.axes([0.8, 0.05, 0.15, 0.075])
        self._add_object_button = Button(button_location, 'Add Object')
        self._add_object_button.on_clicked(self.click_add_object)

        button_location = plt.axes([0.6, 0.05, 0.18, 0.075])
        self._clear_axes_button = Button(button_location, "Clear Objects")
        self._clear_axes_button.on_clicked(self.click_clear)

        button_location = plt.axes([0.4, 0.05, 0.15, 0.075])
        self._finished_button = Button(button_location, 'Finished')
        self._finished_button.on_clicked(self.click_finished)

        # Clear from previous axis
        for patch in self._objects:
            patch.remove()

        for patch in self._objects:
            self._ax.add_patch(patch)

        self._poly_select = PolygonSelector(self._ax, self.on_poly_select)
        self._poly_select.set_active(False)

    def on_poly_select(self, verts):
        """
        This function is called by the polygon select when a polygon
        is closed by the user. It adds a patch to the figure and updates
        the grid map.
        """
        #Close and create patch TODO: Replace with Draggable
        verts.append(verts[0])
        path = Path(verts)

        patch = patches.PathPatch(path)
        self._ax.add_patch(patch)
        self._objects.append(patch)

        # Find points which the polygon contains
        self._intersections = np.nonzero( path.contains_points(self._coordinates) )[0]

        # Update the grid
        for index in self._intersections:
            x = index % self._pps
            y = floor(index / self._pps)

            if x != self._w and y != self._h:
                self._map[x, y] = -1
            if x != 0 and y != 0:
                self._map[x-1, y-1] = -1
            if x != self._w and y != 0:
                self._map[x, y-1] = -1
            if x != 0 and y != self._h:
                self._map[x-1, y] = -1

        # Necessary Matplotlib internal functions on the PolygonSelector
        self._poly_select._clean_event(self._poly_select._prev_event)
        self._poly_select.set_visible(True)
        self._poly_select._xs, self._poly_select._ys = [0], [0]
        self._poly_select._polygon_completed = False
        self._poly_select.update()
        self._poly_select.set_active(False)

if __name__ == "__main__":
    x = Map(5, 5, 6)
