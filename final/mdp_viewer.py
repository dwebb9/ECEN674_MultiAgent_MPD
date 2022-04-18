"""
mavsim_python: waypoint viewer (for chapter 11)
    - Beard & McLain, PUP, 2012
    - Update history:
        4/15/2019 - BGM
"""
import sys
sys.path.append("..")
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from chap2.draw_mav import DrawMav
from chap10.draw_path import DrawPath
from chap11.draw_waypoints import DrawWaypoints
from final.draw_grid import DrawGrid


class WaypointViewer:
    def __init__(self, map):
        self.scale = 4000
        # initialize Qt gui application and window
        self.app = pg.QtGui.QApplication([])  # initialize QT
        self.window = gl.GLViewWidget()  # initialize the view object
        self.window.setWindowTitle('World Viewer')
        self.window.setGeometry(0, 0, 1500, 1500)  # args: upper_left_x, upper_right_y, width, height
        grid = gl.GLGridItem() # make a grid to represent the ground
        grid.scale(self.scale/20, self.scale/20, self.scale/20) # set the size of the grid (distance between each line)
        self.window.addItem(grid) # add grid to viewer
        self.window.setCameraPosition(distance=self.scale, elevation=50, azimuth=-90)
        self.window.setBackgroundColor('k')  # set background color to black
        self.window.show()  # display configured window
        self.window.raise_()  # bring window to the front
        self.plot_initialized = False  # has the mav been plotted yet?
        self.mav_plot = []
        self.mav_plot2 = []
        self.mav_plot3 = []

        self.path_plot = []
        self.path_plot2 = []
        self.path_plot3 = []

        self.waypoint_plot = []
        self.waypoint_plot2 = []
        self.grid = DrawGrid(map, self.window)
        self.waypoint_plot3 = []

    def update(self, state, state2, state3, path, path2, path3, waypoints, waypoints2, waypoints3):
        blue = np.array([[30, 144, 255, 255]])/255.
        red = np.array([[1., 0., 0., 1]])
        # initialize the drawing the first time update() is called
        if not self.plot_initialized:
            self.mav_plot = DrawMav(state, self.window, 0)
            self.mav_plot2 = DrawMav(state2, self.window, 1)
            self.mav_plot3 = DrawMav(state3, self.window, 2)

            self.waypoint_plot = DrawWaypoints(waypoints, path.orbit_radius, blue, self.window)
            self.waypoint_plot2 = DrawWaypoints(waypoints2, path2.orbit_radius, blue, self.window)
            self.waypoint_plot3 = DrawWaypoints(waypoints3, path3.orbit_radius, blue, self.window)

            self.path_plot = DrawPath(path, red, self.window)
            self.path_plot2 = DrawPath(path2, red, self.window)
            self.path_plot3 = DrawPath(path3, red, self.window)

            self.plot_initialized = True
        # else update drawing on all other calls to update()
        else:
            self.mav_plot.update(state)
            self.mav_plot2.update(state2)
            self.mav_plot3.update(state3)

            if waypoints.flag_waypoints_changed:
                self.waypoint_plot.update(waypoints)
                waypoints.flag_waypoints_changed = False

            if waypoints2.flag_waypoints_changed:
                self.waypoint_plot2.update(waypoints2)
                waypoints2.flag_waypoints_changed = False

            if waypoints3.flag_waypoints_changed:
                self.waypoint_plot3.update(waypoints3)
                waypoints3.flag_waypoints_changed = False

            
            if not path.plot_updated:  # only plot path when it changes
                self.path_plot.update(path, red)
                path.plot_updated = True

            if not path2.plot_updated:  # only plot path when it changes
                self.path_plot2.update(path2, red)
                path2.plot_updated = True

            if not path3.plot_updated:  # only plot path when it changes
                self.path_plot3.update(path3, red)
                path3.plot_updated = True
        # redraw
        self.app.processEvents()