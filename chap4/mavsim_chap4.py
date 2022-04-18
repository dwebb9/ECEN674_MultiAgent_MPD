"""
mavsimPy
    - Chapter 4 assignment for Beard & McLain, PUP, 2012
    - Update history:  
        12/27/2018 - RWB
        1/17/2019 - RWB
"""
import sys
sys.path.append('..')
import numpy as np
import parameters.simulation_parameters as SIM

from chap2.mav_viewer import MavViewer
from chap3.data_viewer import DataViewer
from chap4.mav_dynamics import MavDynamics
from chap4.wind_simulation import WindSimulation
from message_types.msg_delta import MsgDelta

# initialize the visualization
VIDEO = False  # True==write video, False==don't write video
mav_view = MavViewer()  # initialize the mav viewer
data_view = DataViewer()  # initialize view of data plots
if VIDEO is True:
    from chap2.video_writer import VideoWriter
    video = VideoWriter(video_name="chap4_video.avi",
                        bounding_box=(0, 0, 1000, 1000),
                        output_rate=SIM.ts_video)

# initialize elements of the architecture
wind = WindSimulation(SIM.ts_simulation)
mav = MavDynamics(SIM.ts_simulation)
delta = MsgDelta()

mav2 = MavDynamics(SIM.ts_simulation)
delta2 = MsgDelta()

mav3 = MavDynamics(SIM.ts_simulation)
delta3 = MsgDelta()

# initialize the simulation time
sim_time = SIM.start_time
plot_time = sim_time

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    # -------set control surfaces-------------
    delta.elevator = -0.1248
    delta.aileron = 0.001836
    delta.rudder = -0.0003026
    delta.throttle = 0.6768

    delta2.elevator = -0.1248
    delta2.aileron = 0.00
    delta2.rudder = -0.0003026
    delta2.throttle = 0.6768

    delta3.elevator = -0.1248
    delta3.aileron = 0.001836*2
    delta3.rudder = -0.0003026
    delta3.throttle = 0.6768
    # transpose to make it a column vector

    # -------physical system-------------
    current_wind = wind.update()  # get the new wind vector
    mav.update(delta, current_wind)  # propagate the MAV dynamics
    mav2.update(delta2, current_wind)
    mav3.update(delta3, current_wind)

    # -------update viewer-------------
    if sim_time-plot_time > SIM.ts_plotting:
        mav_view.update(mav.true_state, mav2.true_state, mav3.true_state)  # plot body of MAV
        plot_time = sim_time
    data_view.update(mav.true_state,  # true states
                     mav.true_state,  # estimated states
                     mav.true_state,  # commanded states
                     delta,  # inputs to aircraft
                     SIM.ts_simulation)
    if VIDEO is True:
        video.update(sim_time)

    # -------increment time-------------
    sim_time += SIM.ts_simulation

if VIDEO is True:
    video.close()




