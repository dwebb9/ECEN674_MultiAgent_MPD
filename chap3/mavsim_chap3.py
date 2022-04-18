"""
mavsimPy
    - Chapter 3 assignment for Beard & McLain, PUP, 2012
    - Update history:  
        12/18/2018 - RWB
        1/14/2019 - RWB
"""
import sys
sys.path.append('..')
import numpy as np
import parameters.simulation_parameters as SIM


from chap2.mav_viewer import MavViewer
from chap3.data_viewer import DataViewer
from chap3.mav_dynamics import MavDynamics
from message_types.msg_delta import MsgDelta

# initialize the visualization
VIDEO = False  # True==write video, False==don't write video
mav_view = MavViewer()  # initialize the mav viewer
data_view = DataViewer()  # initialize view of data plots
if VIDEO is True:
    from chap2.video_writer import VideoWriter
    video = VideoWriter(video_name="chap3_video.avi",
                        bounding_box=(0, 0, 1000, 1000),
                        output_rate=SIM.ts_video)

# initialize elements of the architecture
mav = MavDynamics(SIM.ts_simulation)
delta = MsgDelta()

mav2 = MavDynamics(SIM.ts_simulation)
delta2 = MsgDelta()

mav3 = MavDynamics(SIM.ts_simulation)
delta3 = MsgDelta()

# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    # -------vary forces and moments to check dynamics-------------
    fx = 0
    fy = 0  # 10
    fz = 0  # 10
    Mx = 0.1  # 0.1
    My = 0  # 0.1
    Mz = 0  # 0.1
    forces_moments = np.array([[10, fy, fz, 0, My, Mz]]).T

    forces_moments2 = np.array([[fx, 10, fz, 0, My, Mz]]).T

    forces_moments3 = np.array([[fx, fy, 10, 0, My, Mz]]).T

    # -------physical system-------------
    mav.update(forces_moments)  # propagate the MAV dynamics
    mav2.update(forces_moments2)  # propagate the MAV dynamics
    mav3.update(forces_moments3)

    # -------update viewer-------------
    mav_view.update(mav.true_state, mav2.true_state, mav3.true_state)  # plot body of MAV

    data_view.update(mav.true_state,  # true states
                     mav.true_state,  # estimated states
                     mav.true_state,  # commanded states
                     delta,  # inputs to the aircraft
                     SIM.ts_simulation)
    if VIDEO is True:
        video.update(sim_time)

    # -------increment time-------------
    sim_time += SIM.ts_simulation

if VIDEO is True:
    video.close()




