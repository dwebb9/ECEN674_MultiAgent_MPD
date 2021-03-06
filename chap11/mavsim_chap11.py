"""
mavsim_python
    - Chapter 11 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        3/26/2019 - RWB
        2/27/2020 - RWB
"""
import sys
sys.path.append('..')
import numpy as np
import copy
import parameters.simulation_parameters as SIM
import parameters.planner_parameters as PLAN

from chap3.data_viewer import DataViewer
from chap4.wind_simulation import WindSimulation
from chap6.autopilot import Autopilot
from chap7.mav_dynamics import MavDynamics
from chap8.observer import Observer
from chap10.path_follower import PathFollower
from chap11.path_manager import PathManager
from chap11.waypoint_viewer import WaypointViewer

# initialize the visualization
VIDEO = False  # True==write video, False==don't write video
waypoint_view = WaypointViewer()  # initialize the viewer
data_view = DataViewer()  # initialize view of data plots
if VIDEO is True:
    from chap2.video_writer import VideoWriter
    video = VideoWriter(video_name="chap11_video.avi",
                        bounding_box=(0, 0, 1000, 1000),
                        output_rate=SIM.ts_video)

# initialize elements of the architecture
wind = WindSimulation(SIM.ts_simulation)
mav = MavDynamics(SIM.ts_simulation)
autopilot = Autopilot(SIM.ts_simulation)
initial_state = copy.deepcopy(mav.true_state)
observer = Observer(SIM.ts_simulation, initial_state)
path_follower = PathFollower()
path_manager = PathManager()

mav2 = MavDynamics(SIM.ts_simulation)
autopilot2 = Autopilot(SIM.ts_simulation)
path_follower2 = PathFollower()
path_manager2 = PathManager()


mav3 = MavDynamics(SIM.ts_simulation)
autopilot3 = Autopilot(SIM.ts_simulation)
path_follower3 = PathFollower()
path_manager3 = PathManager()


# waypoint definition
from message_types.msg_waypoints import MsgWaypoints
waypoints = MsgWaypoints()
waypoints2 = MsgWaypoints()
waypoints3 = MsgWaypoints()

waypoints.type = 'fillet'
waypoints2.type = 'dubins'
waypoints3.type = 'straight_line'


Va = PLAN.Va0
waypoints.add(np.array([[0, 0, -150]]).T, Va, np.radians(0), np.inf, 0, 0)
waypoints.add(np.array([[1000, 0, -150]]).T, Va, np.radians(45), np.inf, 0, 0)
waypoints.add(np.array([[0, 1000, -150]]).T, Va, np.radians(45), np.inf, 0, 0)
waypoints.add(np.array([[1000, 1000, -150]]).T, Va, np.radians(-135), np.inf, 0, 0)

waypoints2.add(np.array([[0, 0, -100]]).T, Va, np.radians(0), np.inf, 0, 0)
waypoints2.add(np.array([[1000, 0, -100]]).T, Va, np.radians(45), np.inf, 0, 0)
waypoints2.add(np.array([[0, 1000, -100]]).T, Va, np.radians(45), np.inf, 0, 0)
waypoints2.add(np.array([[1000, 1000, -100]]).T, Va, np.radians(-135), np.inf, 0, 0)

waypoints3.add(np.array([[0, 0, -50]]).T, Va, np.radians(0), np.inf, 0, 0)
waypoints3.add(np.array([[1000, 0, -50]]).T, Va, np.radians(45), np.inf, 0, 0)
waypoints3.add(np.array([[0, 1000, -50]]).T, Va, np.radians(45), np.inf, 0, 0)
waypoints3.add(np.array([[1000, 1000, -50]]).T, Va, np.radians(-135), np.inf, 0, 0)


# initialize the simulation time
sim_time = SIM.start_time
plot_timer = 0

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    # -------observer-------------
    measurements = mav.sensors()  # get sensor measurements
    estimated_state = observer.update(measurements)  # estimate states from measurements

    # -------path manager-------------
    path = path_manager.update(waypoints, PLAN.R_min, mav.true_state)
    path2 = path_manager2.update(waypoints2, PLAN.R_min, mav2.true_state)
    path3 = path_manager3.update(waypoints3, PLAN.R_min, mav3.true_state)

    # -------path follower-------------
    # autopilot_commands = path_follower.update(path, estimated_state)
    autopilot_commands = path_follower.update(path, mav.true_state)  # for debugging
    autopilot_commands2 = path_follower2.update(path2, mav2.true_state)
    autopilot_commands3 = path_follower3.update(path3, mav3.true_state)

    # -------autopilot-------------
    estimated_state = mav.true_state  # uses true states in the control
    delta, commanded_state = autopilot.update(autopilot_commands, mav.true_state)
    delta2, commanded_state2 = autopilot2.update(autopilot_commands2, mav2.true_state)
    delta3, commanded_state3 = autopilot3.update(autopilot_commands3, mav3.true_state)

    # -------physical system-------------
    current_wind = wind.update()  # get the new wind vector
    mav.update(delta, current_wind)  # propagate the MAV dynamics
    mav2.update(delta2, current_wind)
    mav3.update(delta3, current_wind)

    # -------update viewer-------------
    if plot_timer > SIM.ts_plotting:
        waypoint_view.update(mav.true_state, mav2.true_state, mav3.true_state, path, path2, path3, waypoints, waypoints2, waypoints3)  # plot path and MAV
        data_view.update(mav.true_state,  # true states
                         estimated_state,  # estimated states
                         commanded_state,  # commanded states
                         delta,  # input to aircraft
                         SIM.ts_plotting)
        plot_timer = 0
    plot_timer += SIM.ts_simulation

    if VIDEO is True:
        video.update(sim_time)

    # -------increment time-------------
    sim_time += SIM.ts_simulation

if VIDEO is True:
    video.close()




