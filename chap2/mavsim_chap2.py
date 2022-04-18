"""
mavSimPy 
    - Chapter 2 assignment for Beard & McLain, PUP, 2012
    - Update history:  
        1/10/2019 - RWB
"""
import sys
sys.path.append('..')

# import viewers and video writer
from chap2.mav_viewer import MavViewer

# import parameters
import parameters.simulation_parameters as SIM
# import message types
from message_types.msg_state import MsgState

# initialize messages
state = MsgState()  # instantiate state message
state2 = MsgState()  # instantiate state message
state3 = MsgState()  # instantiate state message

state2.north = 10
state2.east = 10

state2.north = 20
state2.east = 20

# initialize viewers and video
VIDEO = False  # True==write video, False==don't write video
mav_view = MavViewer()
if VIDEO is True:
    from chap2.video_writer import VideoWriter
    video = VideoWriter(video_name="chap2_video.avi",
                        bounding_box=(0, 0, 1000, 1000),
                        output_rate=SIM.ts_video)

# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
while sim_time < SIM.end_time:
    # -------vary states to check viewer-------------
    if sim_time < SIM.end_time/6:
        state.north += 10*SIM.ts_simulation
    elif sim_time < 2*SIM.end_time/6:
        state.east += 10*SIM.ts_simulation
    elif sim_time < 3*SIM.end_time/6:
        state.altitude += 10*SIM.ts_simulation
    elif sim_time < 4*SIM.end_time/6:
        state.psi += 0.1*SIM.ts_simulation
    elif sim_time < 5*SIM.end_time/6:
        state.theta += 0.1*SIM.ts_simulation
    else:
        state.phi += 0.1*SIM.ts_simulation


    if sim_time < SIM.end_time/6:
        state2.north += 20*SIM.ts_simulation
    elif sim_time < 2*SIM.end_time/6:
        state2.east += 20*SIM.ts_simulation
    elif sim_time < 6*SIM.end_time/6:
        state2.altitude += 10*SIM.ts_simulation
    elif sim_time < 8*SIM.end_time/6:
        state2.psi += 0.1*SIM.ts_simulation
    elif sim_time < 10*SIM.end_time/6:
        state2.theta += 0.2*SIM.ts_simulation
    else:
        state2.phi += 0.2*SIM.ts_simulation


    if sim_time < SIM.end_time/6:
        state3.north += 5*SIM.ts_simulation
    elif sim_time < 1*SIM.end_time/6:
        state3.east += 20*SIM.ts_simulation
    elif sim_time < 1.5*SIM.end_time/6:
        state3.altitude += 10*SIM.ts_simulation
    elif sim_time < 2*SIM.end_time/6:
        state3.psi += 0.05*SIM.ts_simulation
    elif sim_time < 10*SIM.end_time/6:
        state3.theta += 0.05*SIM.ts_simulation
    else:
        state3.phi += 0.05*SIM.ts_simulation


    # -------update viewer and video-------------
    mav_view.update(state, state2, state3)
    if VIDEO is True:
        video.update(sim_time)

    # -------increment time-------------
    sim_time += SIM.ts_simulation

print("Press Ctrl-Q to exit...")
if VIDEO is True:
    video.close()



