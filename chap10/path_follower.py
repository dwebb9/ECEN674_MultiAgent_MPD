import numpy as np
from math import sin, cos
import sys

sys.path.append('..')
from message_types.msg_autopilot import MsgAutopilot
from tools.wrap import wrap
import parameters.aerosonde_parameters as MAV


class PathFollower:
    def __init__(self):
        self.chi_inf = np.pi/4  # approach angle for large distance from straight-line path
        self.k_path =  0.07  # proportional gain for straight-line path following
        self.k_orbit =  10  # proportional gain for orbit following
        self.gravity = 9.8
        self.autopilot_commands = MsgAutopilot()  # message sent to autopilot

    def update(self, path, state):
        if path.type == 'line':
            self._follow_straight_line(path, state)
        elif path.type == 'orbit':
            self._follow_orbit(path, state)
        return self.autopilot_commands

    def _follow_straight_line(self, path, state):
        # course command
        self.autopilot_commands.airspeed_command = MAV.Va0
        chi_q = np.arctan2(path.line_direction.item(1), path.line_direction.item(0))
        chi_q = wrap(chi_q, state.chi)

        deltaN = state.north - path.line_origin.item(0)
        deltaE = state.east - path.line_origin.item(1)
        epy = -np.sin(chi_q)*deltaN + np.cos(chi_q)*deltaE

        self.autopilot_commands.course_command = chi_q - self.chi_inf*2.0*np.arctan(self.k_path*epy)/np.pi

        # altitude command
        rd = path.line_origin.item(2)
        ep = np.array([[state.north, state.east, state.altitude]]) - path.line_origin
        epi = np.array([ep.item(0), ep.item(1), ep.item(2)])
        ki = np.array([0.0, 0.0, -1])
        line = np.array([path.line_direction.item(0), path.line_direction.item(1), path.line_direction.item(2)])
        n = np.cross(ki, line)/np.linalg.norm(np.cross(ki, line))
        si = epi - (np.dot(epi, n)*n)
        qn = path.line_direction.item(0)
        qe = path.line_direction.item(1)
        qd = path.line_direction.item(2)

        sn = si[0]
        se = si[1]

        self.autopilot_commands.altitude_command = -rd - np.sqrt(sn**2 + se**2)*(qd/np.sqrt(qn**2 + qe**2))
        # feedforward roll angle for straight line is zero
        self.autopilot_commands.phi_feedforward = 0.0

    def _follow_orbit(self, path, state):

        if path.orbit_direction == 'CW':
            direction = 1.0
        else:
            direction = -1.0
        # airspeed command
        self.autopilot_commands.airspeed_command = MAV.Va0
        # distance from orbit center
        d = np.sqrt((state.north - path.orbit_center.item(0))**2 + (state.east - path.orbit_center.item(1))**2)
        # print("d: ", d)
        # compute wrapped version of angular position on orbit
        varphi_temp = np.arctan2((state.east - path.orbit_center.item(1)), (state.north - path.orbit_center.item(0)))
        varphi = wrap(varphi_temp, state.chi)
        # compute normalized orbit error
        orbit_error = (d-path.orbit_radius)/path.orbit_radius
        # course command
        self.autopilot_commands.course_command = varphi + direction*(np.pi/2.0 + np.arctan(self.k_orbit*orbit_error))
        # altitude command
        self.autopilot_commands.altitude_command = -path.orbit_center.item(2)
        # roll feedforward command
        if orbit_error < 10:
            self.autopilot_commands.phi_feedforward = direction*np.arctan2((state.Vg**2),(self.gravity*path.orbit_radius*np.cos(state.chi - state.psi)))
        else:
            self.autopilot_commands.phi_feedforward = 0.0



