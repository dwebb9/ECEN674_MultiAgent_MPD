"""
mavsim_python
    - Chapter 6 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        2/5/2019 - RWB
        2/24/2020 - RWB
"""
import sys
sys.path.append('..')
import numpy as np
import parameters.simulation_parameters as SIM

from chap2.mav_viewer import MavViewer
from chap3.data_viewer import DataViewer
from chap4.mav_dynamics import MavDynamics
from chap4.wind_simulation import WindSimulation
from chap6.autopilot import Autopilot
#from chap6.autopilot_tecs import Autopilot
from tools.signals import Signals

# initialize the visualization
VIDEO = False  # True==write video, False==don't write video
mav_view = MavViewer()  # initialize the mav viewer
data_view = DataViewer()  # initialize view of data plots
if VIDEO is True:
    from chap2.video_writer import VideoWriter
    video = VideoWriter(video_name="chap6_video.avi",
                        bounding_box=(0, 0, 1000, 1000),
                        output_rate=SIM.ts_video)

# initialize elements of the architecture
wind = WindSimulation(SIM.ts_simulation)
mav = MavDynamics(SIM.ts_simulation)
autopilot = Autopilot(SIM.ts_simulation)

mav2 = MavDynamics(SIM.ts_simulation)
autopilot2 = Autopilot(SIM.ts_simulation)

mav3 = MavDynamics(SIM.ts_simulation)
autopilot3 = Autopilot(SIM.ts_simulation)

# autopilot commands
from message_types.msg_autopilot import MsgAutopilot
commands = MsgAutopilot()
commands2 = MsgAutopilot()
commands3 = MsgAutopilot()

Va_command = Signals(dc_offset=25.0,
                     amplitude=3.0,
                     start_time=2.0,
                     frequency=0.01)
altitude_command = Signals(dc_offset=100.0,
                           amplitude=50.0,
                           start_time=0.0,
                           frequency=0.02)
course_command = Signals(dc_offset=np.radians(180),
                         amplitude=np.radians(60),
                         start_time=5.0,
                         frequency=0.015)

altitude_command2 = Signals(dc_offset=40.0,
                           amplitude=50.0,
                           start_time=0.0,
                           frequency=0.02)
                           
altitude_command3 = Signals(dc_offset=140.0,
                           amplitude=50.0,
                           start_time=0.0,
                           frequency=0.02)

# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:

    # -------autopilot commands-------------
    commands.airspeed_command = Va_command.square(sim_time)
    commands.course_command = course_command.square(sim_time)
    commands.altitude_command = altitude_command.trapezoid(sim_time)

    commands2.airspeed_command = commands.airspeed_command
    commands2.course_command = commands.course_command
    commands2.altitude_command = altitude_command2.trapezoid(sim_time)

    commands3.airspeed_command = commands.airspeed_command
    commands3.course_command = commands.course_command
    commands3.altitude_command = altitude_command3.trapezoid(sim_time)

    # -------autopilot-------------
    estimated_state = mav.true_state  # uses true states in the control
    delta, commanded_state = autopilot.update(commands, mav.true_state)
    delta2, commanded_state2 = autopilot2.update(commands2, mav2.true_state)
    delta3, commanded_state3 = autopilot3.update(commands3, mav3.true_state)

    # -------physical system-------------
    current_wind = wind.update()  # get the new wind vector
    mav.update(delta, current_wind)  # propagate the MAV dynamics
    mav2.update(delta2, current_wind)
    mav3.update(delta3, current_wind)
    
    # -------update viewer-------------
    mav_view.update(mav.true_state, mav2.true_state, mav3.true_state)  # plot body of MAV
    data_view.update(mav.true_state,  # true states
                     estimated_state,  # estimated states
                     commanded_state,  # commanded states
                     delta,  # input to aircraft
                     SIM.ts_simulation)
    if VIDEO is True:
        video.update(sim_time)

    # -------increment time-------------
    sim_time += SIM.ts_simulation

if VIDEO is True:
    video.close()




