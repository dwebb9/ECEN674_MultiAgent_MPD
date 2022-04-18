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
from final.mdp_viewer import WaypointViewer

#MPD path calculation ----------------------------------------------------------------------------------------------
goal = 100
nf = -100

origonal_map = map = np.array([[-1, -1 ,-1, -1, -1, -1], [-1, -1, nf, -1, nf, -1], [-1, goal, -1, -1, -1, -1], [-1, -1, goal, -1, -1, -1], [-1, -1, -1, goal, -1, -1], [-1,-1,-1,nf,-1,-1]])
map = np.array([[-1, -1 ,-1, -1, -1, -1], [-1, -1, nf, -1, nf, -1], [-1, goal, -1, -1, -1, -1], [-1, -1, goal, -1, -1, -1], [-1, -1, -1, goal, -1, -1], [-1,-1,-1,nf,-1,-1]])
gamma = 0.1
movementprob = 0.9
utilites = np.array([[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0]])
utilites_calc = np.array([[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0]])

initial_state = [0,0]
init2 = [0,5]
init3 = [2,5]

print("agent 1 init: ", initial_state)
print("agent 2 init: ", initial_state)
print("agent 3 init: ", initial_state)


path = [initial_state]
path2 = [init2]
path3 = [init3]
max_state = 5
min_state = 0

num_calls = 0

def utility(state):
    global utilites, num_calls
    num_calls += 1

    if state[0] < min_state or state[0] > max_state or state[1] < min_state or state[1] > max_state:
        return -1000

    if map[state[0]][state[1]] == goal:
        utilites[state[0]][state[1]] = map[state[0]][state[1]]
        utilites_calc[state[0]][state[1]] = -1
        return utilites[state[0]][state[1]]  
    
    if not utilites_calc[state[0]][state[1]] == 0:
        return -1000  

    utilites_calc[state[0]][state[1]] = -1
    up = utility([state[0]-1, state[1]])
    right = utility([state[0], state[1]+1])
    down = utility([state[0]+1, state[1]])
    left = utility([state[0], state[1]-1])

    u = map[state[0]][state[1]] + gamma*max(up, down, left, right)
    utilites[state[0]][state[1]] = u 
    
    return u

def pathCalc(map, initial_state, path):
    state = initial_state
    while not map[state[0]][state[1]] == goal:
        if state[0]-1 >= min_state:
            up = utilites[state[0]-1][state[1]]
        else:
            up = -1000
        if state[0]+1 <= max_state:
            down = utilites[state[0]+1][state[1]]
        else:
            down = -1000
        if state[1]-1 >= min_state:
            left = utilites[state[0]][state[1]-1]
        else:
            left = -1000
        if state[1]+1 <= max_state:
            right = utilites[state[0]][state[1]+1]
        else:
            right = -1000

        choice = max(up, down, right, left)

        if right == choice:
            state = [state[0], state[1]+1]
        if left == choice:
            state = [state[0], state[1]-1]
        if up == choice:
            state = [state[0]-1, state[1]]
        if down == choice:
            state = [state[0]+1, state[1]]

        path.append(state)
            

utility(initial_state)
print("utility map: \n", utilites)
pathCalc(map, initial_state, path)

print("map: \n", map)
print("path1: ", path)

rmgoal = path[len(path)-1]
map[rmgoal[0]][rmgoal[1]] = -1

utilites = np.array([[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0]])
utilites_calc = np.array([[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0]])

utility(init2)
pathCalc(map, init2, path2)

print("path2: ", path2)
rmgoal = path2[len(path2)-1]
map[rmgoal[0]][rmgoal[1]] = -1

utilites = np.array([[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0]])
utilites_calc = np.array([[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0]])

utility(init3)
pathCalc(map, init3, path3)
print("path3: ", path3)

#Run Simulation using given path---------------------------------------------------------------------------------

# initialize the visualization
VIDEO = False  # True==write video, False==don't write video
waypoint_view = WaypointViewer(origonal_map)  # initialize the viewer
data_view = DataViewer()  # initialize view of data plots
if VIDEO is True:
    from chap2.video_writer import VideoWriter
    video = VideoWriter(video_name="chap11_video.avi",
                        bounding_box=(0, 0, 1000, 1000),
                        output_rate=SIM.ts_video)

# initialize elements of the architecture
wind = WindSimulation(SIM.ts_simulation)
mav = MavDynamics(SIM.ts_simulation)
initial_state = copy.deepcopy(mav.true_state)
offset = 300


path_follower = PathFollower()
path_manager = PathManager()
autopilot = Autopilot(SIM.ts_simulation)
observer = Observer(SIM.ts_simulation, initial_state)

mav2 = MavDynamics(SIM.ts_simulation, initN=(init2[0]*300+150), initE=(init2[1]*300+150+offset))
autopilot2 = Autopilot(SIM.ts_simulation)
path_follower2 = PathFollower()
path_manager2 = PathManager()
observer2 = Observer(SIM.ts_simulation, initial_state)

mav3 = MavDynamics(SIM.ts_simulation, initN=(init3[0]*300+150), initE=(init3[1]*300+150), alpha=np.pi/2)
autopilot3 = Autopilot(SIM.ts_simulation)
path_follower3 = PathFollower()
path_manager3 = PathManager()
observer3 = Observer(SIM.ts_simulation, initial_state)


# waypoint definition
from message_types.msg_waypoints import MsgWaypoints
waypoints = MsgWaypoints()
waypoints2 = MsgWaypoints()
waypoints3 = MsgWaypoints()

waypoints.type = 'fillet'
waypoints2.type = 'fillet'
waypoints3.type = 'fillet'


Va = PLAN.Va0

# add way points from MPD

for node in path:
    waypoints.add(np.array([[150 + node[0]*300, 150 + node[1]*300, -100]]).T, Va, np.radians(0), np.inf, 0, 0)

for node in path2:
    waypoints2.add(np.array([[150 + node[0]*300, 150 + node[1]*300, -100]]).T, Va, np.radians(0), np.inf, 0, 0)

for node in path3:
    waypoints3.add(np.array([[150 + node[0]*300, 150 + node[1]*300, -100]]).T, Va, np.radians(0), np.inf, 0, 0)


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
