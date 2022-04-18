import numpy as np
import pyqtgraph.opengl as gl


class DrawGrid:
    def __init__(self, map, window):
        blue = np.array([0., 0., 1., 1])
        red = np.array([[1., 0., 0., 1]])
        green = np.array([0., 1., 0., 1])

        line_1_points = np.array([[0,0,100], [1800,0,100], [1800,1800,100], [0,1800,100], [0,0,100]])
        line_2_points = np.array([[0,300,100], [1800,300,100], [1800,1200,100], [0,1200,100], [0,300,100]])
        line_3_points = np.array([[0,600,100], [1800,600,100], [1800,900,100], [0,900,100], [0,600,100]])
        line_4_points = np.array([[300,0,100], [1200,0,100], [1200,1800,100], [300,1800,100], [300,0,100]])
        line_5_points = np.array([[600,0,100], [900,0,100], [900,1800,100], [600,1800,100], [600,0,100]])
        line_6_points = np.array([[0,0,100], [1500,0,100], [1500,1500,100], [0,1500,100], [0,0,100]])
        line_7_points = np.array([[1500,1500,100], [1500,1800,100], [1800,1800,100], [1800,1500,100], [1500,1500,100]])

        line_color = np.tile(blue, (line_1_points.shape[0], 1))


        grid_object1 = gl.GLLinePlotItem(pos=line_1_points,
                                                      color=line_color,
                                                      width=2,
                                                      antialias=True,
                                                      mode='line_strip')
        grid_object2 = gl.GLLinePlotItem(pos=line_2_points,
                                                      color=line_color,
                                                      width=2,
                                                      antialias=True,
                                                      mode='line_strip')
        grid_object3 = gl.GLLinePlotItem(pos=line_3_points,
                                                      color=line_color,
                                                      width=2,
                                                      antialias=True,
                                                      mode='line_strip')
        grid_object4 = gl.GLLinePlotItem(pos=line_4_points,
                                                      color=line_color,
                                                      width=2,
                                                      antialias=True,
                                                      mode='line_strip')
        grid_object5 = gl.GLLinePlotItem(pos=line_5_points,
                                                      color=line_color,
                                                      width=2,
                                                      antialias=True,
                                                      mode='line_strip')
        grid_object6 = gl.GLLinePlotItem(pos=line_6_points,
                                                      color=line_color,
                                                      width=2,
                                                      antialias=True,
                                                      mode='line_strip')
        grid_object7 = gl.GLLinePlotItem(pos=line_7_points,
                                                      color=line_color,
                                                      width=2,
                                                      antialias=True,
                                                      mode='line_strip')
        window.addItem(grid_object1)
        window.addItem(grid_object2)
        window.addItem(grid_object3)
        window.addItem(grid_object4)
        window.addItem(grid_object5)
        window.addItem(grid_object6)
        window.addItem(grid_object7)

        # draw goal and nf zones
        goal = np.array([[0,0,0], [0,0,0], [0,0,0]])
        goal_itt = 0
        nf = np.array([[0,0,0], [0,0,0], [0,0,0]])
        nf_itt = 0
        for i in range(0, 6):
            for j in range(0, 6):
                if map[i][j] == 100:
                    goal[goal_itt] = [150+j*300, 150+i*300, 100]
                    goal_itt += 1
                if map[i][j] == -100:
                    nf[nf_itt] = [150+j*300, 150+i*300, 100]
                    nf_itt += 1
        
        goal1 = goal[0]
        goal2 = goal[1]
        goal3 = goal[2]
        goal_points = np.array([    
                                [goal1[0]-150, goal1[1]-150, goal1[2]], 
                                [goal1[0]+150, goal1[1]+150, goal1[2]], 
                                [goal1[0]-150, goal1[1]+150, goal1[2]], 
                                [goal1[0]+150, goal1[1]-150, goal1[2]], 
                                [goal2[0]-150, goal2[1]-150, goal2[2]], 
                                [goal2[0]+150, goal2[1]+150, goal2[2]], 
                                [goal2[0]-150, goal2[1]+150, goal2[2]], 
                                [goal2[0]+150, goal2[1]-150, goal2[2]],
                                [goal3[0]-150, goal3[1]-150, goal3[2]], 
                                [goal3[0]+150, goal3[1]+150, goal3[2]], 
                                [goal3[0]-150, goal3[1]+150, goal3[2]], 
                                [goal3[0]+150, goal3[1]-150, goal3[2]] 
                                ])

        goalMesh = np.array([
                            [goal_points[0], goal_points[2], goal_points[1]],
                            [goal_points[0], goal_points[3], goal_points[1]],
                            [goal_points[4], goal_points[6], goal_points[5]],
                            [goal_points[4], goal_points[7], goal_points[5]],
                            [goal_points[8], goal_points[10], goal_points[9]],
                            [goal_points[8], goal_points[11], goal_points[9]]
                            ])

        meshColors = np.empty((6, 3, 4), dtype=np.float32)
        meshColors[0] = green
        meshColors[1] = green
        meshColors[2] = green
        meshColors[3] = green
        meshColors[4] = green
        meshColors[5] = green
        goal_object = gl.GLMeshItem(vertexes=goalMesh,  # defines the triangular mesh (Nx3x3)
                                      vertexColors=meshColors,  # defines mesh colors (Nx1)
                                      drawEdges=True,  # draw edges between mesh elements
                                      smooth=False,  # speeds up rendering
                                      computeNormals=False)  # speeds up rendering

        window.addItem(goal_object)

        nf1 = nf[0]
        nf2 = nf[1]
        nf3 = nf[2]
        nf_points = np.array([    
                                [nf1[0]-150, nf1[1]-150, nf1[2]], 
                                [nf1[0]+150, nf1[1]+150, nf1[2]], 
                                [nf1[0]-150, nf1[1]+150, nf1[2]], 
                                [nf1[0]+150, nf1[1]-150, nf1[2]], 
                                [nf2[0]-150, nf2[1]-150, nf2[2]], 
                                [nf2[0]+150, nf2[1]+150, nf2[2]], 
                                [nf2[0]-150, nf2[1]+150, nf2[2]], 
                                [nf2[0]+150, nf2[1]-150, nf2[2]],
                                [nf3[0]-150, nf3[1]-150, nf3[2]], 
                                [nf3[0]+150, nf3[1]+150, nf3[2]], 
                                [nf3[0]-150, nf3[1]+150, nf3[2]], 
                                [nf3[0]+150, nf3[1]-150, nf3[2]] 
                                ])

        nf_Mesh = np.array([
                            [nf_points[0], nf_points[2], nf_points[1]],
                            [nf_points[0], nf_points[3], nf_points[1]],
                            [nf_points[4], nf_points[6], nf_points[5]],
                            [nf_points[4], nf_points[7], nf_points[5]],
                            [nf_points[8], nf_points[10], nf_points[9]],
                            [nf_points[8], nf_points[11], nf_points[9]]
                            ])

        meshColors = np.empty((6, 3, 4), dtype=np.float32)
        meshColors[0] = red
        meshColors[1] = red
        meshColors[2] = red
        meshColors[3] = red
        meshColors[4] = red
        meshColors[5] = red
        nf_object = gl.GLMeshItem(vertexes=nf_Mesh,  # defines the triangular mesh (Nx3x3)
                                      vertexColors=meshColors,  # defines mesh colors (Nx1)
                                      drawEdges=True,  # draw edges between mesh elements
                                      smooth=False,  # speeds up rendering
                                      computeNormals=False)  # speeds up rendering

        window.addItem(nf_object)

    def update(self, waypoints):
        if waypoints.type=='straight_line' or waypoints.type=='fillet':
            points = self.straight_waypoint_points(waypoints)
        elif waypoints.type=='dubins':
            points = self.dubins_points(waypoints, self.radius, 0.1)
        self.waypoint_plot_object.setData(pos=points)

   