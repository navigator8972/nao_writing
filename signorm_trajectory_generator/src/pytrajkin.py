
"""
Module of supporting kinematic trajectory encoding
See Plamondon, A kinematic theory of rapid human movements, Biological Cybernetics, 1995
    Plamondon et al, Recent Developments in the study of rapid human movements
    O'Reilly and Plamondon, Development of a Sigma-Lognormal representation for on-line signatures
"""
import numpy as np
import copy

import pytrajkin_rxzero as pyrzx

class TrajKinMdl:
    """
    class definition for kinematic trajectory model
    Support type of component type:
    siglognormal
    """
    def __init__(self, mdl_type='siglognormal', mdl_parms=None):
        self.mdl_type_ = mdl_type
        self.mdl_parms_ = mdl_parms
        self.x0 = 0.0
        self.y0 = 0.0
        #not sure if the interval should be fixed, let it be now although t_idx is exact time, confusing, uh?
        self.dt = 0.01

    def eval(self, t_idx, parms=None):
        """
        evaluate model with given time indexes
        """
        if parms is None:
            parms = self.mdl_parms_

        res_pos = None
        res_vel = None
        if self.mdl_type_ == 'siglognormal':
            """
            for siglognormal model, each parm should be a tuple: (D, t0, mu, sigma, theta_s, theta_e)
            """
            res_pos, res_vel = pyrzx.rxzero_traj_eval(parms, t_idx, self.x0, self.y0)
        else:
            print 'Invalid or unsupported model type'

        return res_pos, res_vel

    def train(self, pos_traj):
        #extract parameters from given position trajectory
        #pos_traj:      an array of 2D position coordinates
        #get a series of t idx
        # vel_profile = self.get_vel_profile(pos_traj)
        # reg_pnts = vel_profile_registration(vel_profile)
        #<hyin/Feb-09-2015> use rxzero, though a not complete version...
        parms, reg_pnts = pyrzx.rxzero_train(pos_traj, getregpnts=True)
        self.mdl_parms_ = parms
        return parms, reg_pnts

    def get_vel_profile(self, stroke):
        """
        get the velocity profile for a stroke
        input is an array of position coordinates
        """
        vel = np.diff(stroke, axis=0)
        vel_prf = np.sum(vel**2, axis=-1)**(1./2)
        return vel_prf
