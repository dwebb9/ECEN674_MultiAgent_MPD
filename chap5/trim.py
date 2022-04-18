"""
compute_trim 
    - Chapter 5 assignment for Beard & McLain, PUP, 2012
    - Update history:  
        12/29/2018 - RWB
"""
import sys
sys.path.append('..')
import numpy as np

from scipy.optimize import minimize
import parameters.aerosonde_parameters as MAV
from tools.rotations import Euler2Quaternion
from message_types.msg_delta import MsgDelta

def compute_trim(mav, Va, gamma):
    # define initial state and input
    e0 = Euler2Quaternion(0., gamma, 0.)
    state0 = np.array([[0],  # pn
                   [0.0],  # pe
                   [MAV.down0],  # pd
                   [Va],  # u
                   [0.0],  # v
                   [0.0],  # w
                   [e0.item(0)],  # e0
                   [e0.item(1)],  # e1
                   [e0.item(2)],  # e2
                   [e0.item(3)],  # e3
                   [0.0],  # p
                   [0.0],  # q
                   [0.0]   # r
                   ])
    # delta0 = np.array([[0,0,0,0]]).T #MsgDelta()
    delta0 = MsgDelta()

    delta0.elevator = -0.2
    delta0.aileron = 0.0
    delta0.rudder = -0.0
    delta0.throttle = 0.4


    x0 = np.concatenate((state0, delta0.to_array()), axis=0)
    # define equality constraints
    cons = ({'type': 'eq',
             'fun': lambda x: np.array([
                                x[3]**2 + x[4]**2 + x[5]**2 - Va**2,  # magnitude of velocity vector is Va
                                x[4],  # v=0, force side velocity to be zero
                                x[6]**2 + x[7]**2 + x[8]**2 + x[9]**2 - 1.,  # force quaternion to be unit length
                                x[7],  # e1=0  - forcing e1=e3=0 ensures zero roll and zero yaw in trim
                                x[9],  # e3=0
                                x[10],  # p=0  - angular rates should all be zero
                                x[11],  # q=0
                                x[12],  # r=0
                                ]),
             'jac': lambda x: np.array([
                                [0., 0., 0., 2*x[3], 2*x[4], 2*x[5], 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 2*x[6], 2*x[7], 2*x[8], 2*x[9], 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0.],
                                ])
             })
    # solve the minimization problem to find the trim states and inputs

    res = minimize(trim_objective_fun, x0, method='SLSQP', args=(mav, Va, gamma),
                   constraints=cons, options={'ftol': 1e-10, 'disp': True})
    # extract trim state and input and return
    trim_state = np.array([res.x[0:13]]).T
    trim_input = MsgDelta(elevator=res.x.item(13),
                          aileron=res.x.item(14),
                          rudder=res.x.item(15),
                          throttle=res.x.item(16))
    trim_input.print()
    print('trim_state-', trim_state.T)
    return trim_state, trim_input



############ SOURCE OF PROBLEM!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

def trim_objective_fun(x, mav, Va, gamma): 
    # objective function to be minimized
    state = np.array([[x[0]],  # pn
                   [x[1]],  # pe
                   [x[2]],  # pd
                   [x[3]],  # u
                   [x[4]],  # v
                   [x[5]],  # w
                   [x[6]],  # e0
                   [x[7]],  # e1
                   [x[8]],  # e2
                   [x[9]],  # e3
                   [x[10]],  # p
                   [x[11]],  # q
                   [x[12]]   # r
                   ])
    delta = MsgDelta(elevator=x.item(13),
    aileron=x.item(14),
    rudder=x.item(15),
    throttle=x.item(16))
    desired_trim_state_dot = np.array([[0., 0., -Va*np.sin(gamma), 0.,
                                        0., 0., 0., 0., 0., 0., 0., 0.,
                                        0.]]).T
    mav._state = state
    mav._update_velocity_data()
    forces_moments = mav._forces_moments(delta)
    # print("forces from trim: ", forces_moments)
    f = mav._derivatives(state, forces_moments)
    tmp = desired_trim_state_dot - f
    J = np.linalg.norm(tmp[2:13]**2.0)
    return J
