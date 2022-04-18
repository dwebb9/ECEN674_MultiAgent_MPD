"""
mavsim_python: drawing tools
    - Beard & McLain, PUP, 2012
    - Update history:
        4/15/2019 - BGM
"""
import numpy as np
import pyqtgraph.opengl as gl
from tools.rotations import Euler2Rotation


class DrawMav:
    def __init__(self, state, window, color):
        """
        Draw the MAV.

        The input to this function is a (message) class with properties that define the state.
        The following properties are assumed:
            state.north  # north position
            state.east  # east position
            state.altitude   # altitude
            state.phi  # roll angle
            state.theta  # pitch angle
            state.psi  # yaw angle
        """
        # get points that define the non-rotated, non-translated mav and the mesh colors
        self.color = color
        self.mav_points, self.mav_meshColors = self.get_points()

        mav_position = np.array([[state.north], [state.east], [-state.altitude]])  # NED coordinates
        # attitude of mav as a rotation matrix R from body to inertial
        R = Euler2Rotation(state.phi, state.theta, state.psi)
        # rotate and translate points defining mav
        rotated_points = self.rotate_points(self.mav_points, R)
        translated_points = self.translate_points(rotated_points, mav_position)
        # convert North-East Down to East-North-Up for rendering
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

        translated_points = R @ translated_points
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = self.points_to_mesh(translated_points)
        self.mav_body = gl.GLMeshItem(vertexes=mesh,  # defines the triangular mesh (Nx3x3)
                                      vertexColors=self.mav_meshColors,  # defines mesh colors (Nx1)
                                      drawEdges=True,  # draw edges between mesh elements
                                      smooth=False,  # speeds up rendering
                                      computeNormals=False)  # speeds up rendering
        window.addItem(self.mav_body)  # add body to plot

    def update(self, state):
        mav_position = np.array([[state.north], [state.east], [-state.altitude]])  # NED coordinates
        # attitude of mav as a rotation matrix R from body to inertial
        R = Euler2Rotation(state.phi, state.theta, state.psi)
        # rotate and translate points defining mav
        rotated_points = self.rotate_points(self.mav_points, R)
        translated_points = self.translate_points(rotated_points, mav_position)
        # convert North-East Down to East-North-Up for rendering
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

        translated_points = R @ translated_points
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = self.points_to_mesh(translated_points)
        # draw MAV by resetting mesh using rotated and translated points
        self.mav_body.setMeshData(vertexes=mesh, vertexColors=self.mav_meshColors)

    def rotate_points(self, points, R):
        "Rotate points by the rotation matrix R"
        rotated_points = R @ points
        return rotated_points

    def translate_points(self, points, translation):
        "Translate points by the vector translation"
        translated_points = points + np.dot(translation, np.ones([1, points.shape[1]]))
        return translated_points

    def get_points(self):
        """"
            Points that define the mav, and the colors of the triangular mesh
            Define the points on the aircraft following diagram in Figure C.3
        """
        # define MAV body parameters
        unit_length = 0.25
        fuse_h = unit_length
        fuse_w = unit_length
        fuse_l1 = unit_length * 2
        fuse_l2 = unit_length
        fuse_l3 = unit_length * 4
        wing_l = unit_length
        wing_w = unit_length * 6
        tail_h = unit_length
        tail_l = unit_length
        tail_w = unit_length * 2

        # points are in NED coordinates
        # define the points on the aircraft following diagram Fig 2.14
        points = np.array([[fuse_l1, 0, fuse_h*0.25],  # point 1 [0]
                           [fuse_l2, fuse_w*0.5, -fuse_h*0.5],  # point 2 [1]
                           [fuse_l2, -fuse_w*0.5, -fuse_h*0.5],  # point 3 [2]
                           [fuse_l2, -fuse_w*0.5, fuse_h*0.5],  # point 4 [3]
                           [fuse_l2, fuse_w*0.5, fuse_h*0.5],  # point 5 [4]
                           [-fuse_l3, 0, 0],  # point 6 [5]
                           [0, wing_w*0.5, 0],  # point 7 [6]
                           [-wing_l, wing_w*0.5, 0],  # point 8 [7]
                           [-wing_l, -wing_w*0.5, 0],  # point 9 [8]
                           [0, -wing_w*0.5, 0],  # point 10 [9]
                           [-fuse_l3 + tail_l, tail_w*0.5, 0],  # point 11 [10]
                           [-fuse_l3, tail_w*0.5, 0],  # point 12 [11]
                           [-fuse_l3, -tail_w*0.5, 0],  # point 13 [12]
                           [-fuse_l3 + tail_l, -tail_w*0.5, 0],  # point 14 [13]
                           [-fuse_l3 + tail_l, 0, 0],  # point 15 [14]
                           [-fuse_l3, 0, -tail_h]  # point 16 [15]
                           ]).T

        # points = np.array([[0,0,0],[1,0,0],[0,0,1]]).T

        # scale points for better rendering
        scale = 50
        points = scale * points

        #   define the colors for each face of triangular mesh
        red = np.array([1., 0., 0., 1])
        green = np.array([0., 1., 0., 1])
        blue = np.array([0., 0., 1., 1])
        yellow = np.array([1., 1., 0., 1])
        meshColors = np.empty((13, 3, 4), dtype=np.float32)
        
        if self.color == 0:
            meshColors[0] = blue
            meshColors[1] = blue
            meshColors[2] = blue
            meshColors[3] = blue
            meshColors[4] = blue
            meshColors[5] = blue
            meshColors[6] = blue
            meshColors[7] = blue
            meshColors[8] = red
            meshColors[9] = red
            meshColors[10] = red
            meshColors[11] = red
            meshColors[12] = red
        elif self.color == 1:
            meshColors[0] = green
            meshColors[1] = green
            meshColors[2] = green
            meshColors[3] = green
            meshColors[4] = green
            meshColors[5] = green
            meshColors[6] = green
            meshColors[7] = green
            meshColors[8] = red
            meshColors[9] = red
            meshColors[10] = red
            meshColors[11] = red
            meshColors[12] = red
        else:
            meshColors[0] = red
            meshColors[1] = red
            meshColors[2] = red
            meshColors[3] = red
            meshColors[4] = red
            meshColors[5] = red
            meshColors[6] = red
            meshColors[7] = red
            meshColors[8] = yellow
            meshColors[9] = yellow
            meshColors[10] = yellow
            meshColors[11] = yellow
            meshColors[12] = yellow




        return points, meshColors

    def points_to_mesh(self, points):
        """"
        Converts points to triangular mesh
        Each mesh face is defined by three 3D points
          (a rectangle requires two triangular mesh faces)
        """
        points = points.T
        mesh = np.array([
                        [points[0], points[1], points[2]], #front top
                        [points[0], points[3], points[4]], #front bottom
                        [points[0], points[1], points[4]], #front right
                        [points[0], points[3], points[2]], #front left
                        [points[1], points[2], points[5]], #main top
                        [points[3], points[4], points[5]], #main bottom
                        [points[1], points[4], points[5]], #main right
                        [points[2], points[3], points[5]], #main left
                        [points[6], points[7], points[8]], #wing1
                        [points[6], points[8], points[9]], #wing2
                        [points[10], points[11], points[12]], #rect tail1
                        [points[10], points[12], points[13]], #rect tail1
                        [points[5], points[14], points[15]] #tri tail
                        ]) 

        # mesh = np.array([[points[0], points[1], points[2]]])

        return mesh
