# dubins_parameters
#   - Dubins parameters that define path between two configurations
#
# mavsim_matlab 
#     - Beard & McLain, PUP, 2012
#     - Update history:  
#         3/26/2019 - RWB
#         4/2/2020 - RWB

import numpy as np
import sys
sys.path.append('..')


class DubinsParameters:
    def __init__(self, ps=9999*np.ones((3,1)), chis=9999,
                 pe=9999*np.ones((3,1)), chie=9999, R=9999):
        if R == 9999:
            L = R
            cs = ps
            lams = R
            ce = ps
            lame = R
            w1 = ps
            q1 = ps
            w2 = ps
            w3 = ps
            q3 = ps
        else:
            L, cs, lams, ce, lame, w1, q1, w2, w3, q3 \
                = compute_parameters(ps, chis, pe, chie, R)
        self.p_s = ps
        self.chi_s = chis
        self.p_e = pe
        self.chi_e = chie
        self.radius = R
        self.length = L
        self.center_s = cs
        self.dir_s = lams
        self.center_e = ce
        self.dir_e = lame
        self.r1 = w1
        self.n1 = q1
        self.r2 = w2
        self.r3 = w3
        self.n3 = q3

    def update(self, ps, chis, pe, chie, R):
         L, cs, lams, ce, lame, w1, q1, w2, w3, q3 \
            = compute_parameters(ps, chis, pe, chie, R)
         self.p_s = ps
         self.chi_s = chis
         self.p_e = pe
         self.chi_e = chie
         self.radius = R
         self.length = L
         self.center_s = cs
         self.dir_s = lams
         self.center_e = ce
         self.dir_e = lame
        #  self.r1 = np.transpose(w1)
        #  self.n1 = np.transpose(q1)
        #  self.r2 = np.transpose(w2)

         self.r1 = np.array([[w1[0]], [w1[1]], [w1[2]]])
         self.n1 = np.array([[q1[0]], [q1[1]], [q1[2]]])
         self.r2 = np.array([[w2[0]], [w2[1]], [w2[2]]])

         self.r3 = w3
         self.n3 = q3


def compute_parameters(ps, chis, pe, chie, R):
    ell = np.linalg.norm(ps-pe)
    if ell < 2 * R:
        print('Error in Dubins Parameters: The distance between nodes must be larger than 2R.')
    else:
        # compute start and end circles
        ps_temp = np.array([ps.item(0), ps.item(1), ps.item(2)])
        pe_temp = np.array([pe.item(0), pe.item(1), pe.item(2)])


        crs = ps_temp + R*rotz(np.pi/2)@np.transpose(np.array([np.cos(chis), np.sin(chis), 0]))
        cls = ps_temp + R*rotz(-np.pi/2)@np.transpose(np.array([np.cos(chis), np.sin(chis), 0]))
        cre = pe_temp + R*rotz(np.pi/2)@np.transpose(np.array([np.cos(chie), np.sin(chie), 0]))
        cle = pe_temp + R*rotz(-np.pi/2)@np.transpose(np.array([np.cos(chie), np.sin(chie), 0]))

        # compute L1
        theta = np.arctan2((cre.item(1) - crs.item(1)), (cre.item(0) - crs.item(0)))
        R1Ang = mod(2*np.pi + mod(theta - np.pi/2) - mod(chis - np.pi/2))
        R2Ang = mod(2*np.pi + mod(chie - np.pi/2) - mod(theta - np.pi/2))
        L1 = np.linalg.norm(crs - cre) + R*R1Ang + R*R2Ang
        # compute L2

        ell = np.linalg.norm(crs - cle)
        theta = np.arctan2((cle.item(1) - crs.item(1)), (cle.item(0) - crs.item(0)))
        theta2 = theta - np.pi/2 + np.arcsin(2*R/ell)

        R1Ang = mod(2*np.pi + mod(theta2) - mod(chis - np.pi/2))
        R2Ang = mod(2*np.pi + mod(theta2 + np.pi) - mod(chie + np.pi/2))
        if not np.isreal(theta2):
            L2 = np.sqrt(ell**2 - 4*R**2)
        else:
            L2 = np.sqrt(ell**2 - 4*R**2) + R*R1Ang + R*R2Ang

        # compute L3
        ell = np.linalg.norm(cls - cre)
        theta = np.arctan2((cre.item(1) - cls.item(1)), (cre.item(0) - cls.item(0)))
        theta2 = np.arccos(2*R/ell)

        R1Ang = mod(2*np.pi + mod(chis + np.pi/2) - mod(theta + theta2))
        R2Ang = mod(2*np.pi + mod(chie - np.pi/2) - mod(theta + theta2 - np.pi))
        if not np.isreal(theta2):
            L3 = np.sqrt(ell**2 - 4*R**2)
        else:
            L3 = np.sqrt(ell**2 - 4*R**2) + R*R1Ang + R*R2Ang
        # compute L4
        theta = np.arctan2((cle.item(1) - cls.item(1)), (cle.item(0) - cls.item(0)))

        R1Ang = mod(2*np.pi + mod(chis + np.pi/2) - mod(theta  + np.pi/2))
        R2Ang = mod(2*np.pi + mod(theta + np.pi/2) - mod(chie + np.pi/2))

        L4 = np.linalg.norm(cls - cle) + R*R1Ang + R*R2Ang
        # L is the minimum distance
        L = np.min([L1, L2, L3, L4])
        idx = np.argmin([L1, L2, L3, L4])
        e1 = np.array([1,0,0])
        if idx == 0:
            cs = crs
            lams = 1
            ce = cre
            lame = 1
            q1 = (ce - cs)/np.linalg.norm((ce - cs))
            w1 = cs + R*rotz(-np.pi/2)@q1
            w2 = ce + R*rotz(-np.pi/2)@q1
        elif idx == 1:
            cs = crs
            lams = 1
            ce = cle
            lame = -1

            ell = np.linalg.norm(ce - cs)
            theta = np.arctan2((ce.item(1) - cs.item(1)), (ce.item(0) - cs.item(0)))
            theta2 = theta - np.pi/2 + np.arcsin(2*R/ell)

            q1 = rotz(theta2 + np.pi/2)@e1
            w1 = cs + R*rotz(theta2)@e1
            w2 = ce + R*rotz(theta2 + np.pi)@e1
        elif idx == 2:
            cs = cls
            lams = -1   
            ce = cre
            lame = 1    

            ell = np.linalg.norm(ce - cs)
            theta = np.arctan2((ce.item(1) - cs.item(1)), (ce.item(0) - cs.item(0)))
            theta2 = np.arccos(2*R/ell)

            q1 = rotz(theta + theta2 - np.pi/2)@e1
            w1 = cs + R*rotz(theta + theta2)@e1
            w2 = ce + R*rotz(theta + theta2 - np.pi)@e1
        elif idx == 3:
            cs = cls
            lams = -1   
            ce = cle
            lame = -1
            q1 = (ce - cs)/np.linalg.norm((ce - cs))
            w1 = cs + R*rotz(np.pi/2)@q1
            w2 = ce + R*rotz(np.pi/2)@q1
        w3 = pe
        q3 = rotz(chie)@e1

        return L, cs, lams, ce, lame, w1, q1, w2, w3, q3


def rotz(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                    [np.sin(theta), np.cos(theta), 0],
                    [0, 0, 1]])


def mod(x):
    while x < 0:
        x += 2*np.pi
    while x > 2*np.pi:
        x -= 2*np.pi
    return x

