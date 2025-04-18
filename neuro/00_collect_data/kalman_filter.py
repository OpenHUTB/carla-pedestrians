#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2021 Ashwin Rajesh
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import numpy as np

class kalman_filter:
    # State : A column vector with [x_pos, y_pos, yaw, x_vel, y_vel]    
    def __init__(self, init_state, init_time, accel_var, yaw_var, meas_var):
        self.state          = np.asarray(init_state).reshape(5,1)
        self.prev_time      = init_time
        self.covar          = np.zeros((5,5))

        self.Q              = np.diag([accel_var, accel_var, yaw_var])
        self.R              = np.diag([meas_var, meas_var])

        self.states = []
        self.covars = []

    # Input : A column vector with [x_accel, y_accel, yaw_vel]
    def update(self, inp, time):
        
        inp = np.asarray(inp).reshape(3,1)
        
        dt = time - self.prev_time
        
        # Transition matrix :
        #
        # | 1   0   0   dt  0  |
        # | 0   1   0   0   dt | 
        # | 0   0   1   0   0  |
        # | 0   0   0   1   0  |
        # | 0   0   0   0   1  |
        #
        A = np.asarray([\
            [1, 0, 0, dt,0], \
            [0, 1, 0, 0, dt],\
            [0, 0, 1, 0, 0], \
            [0, 0, 0, 1, 0], \
            [0, 0, 0, 0, 1]  \
            ])
        
        # Input influence matrix
        # 
        # | 0   0   0  |
        # | 0   0   0  |
        # | 0   0   dt |
        # | dt  0   0  |
        # | 0   dt  0  |
        #
        B = np.asarray([\
            [0, 0, 0], \
            [0, 0, 0], \
            [0, 0, dt],\
            [dt,0, 0], \
            [0, dt,0], \
            ])

        # L = np.asarray([\
        #     [0, 0, 0,], \
        #     [0, 0, 0,], \
        #     [0, 0, 1,], \
        #     [1, 0, 0],  \
        #     [0, 1, 0],  \
        #     ])
        
        yaw      = self.state[2][0]
        accel_xl = inp[0][0]
        accel_yl = inp[1][0]
        accel_xg = accel_xl * np.cos(yaw) - accel_yl * np.sin(yaw)
        accel_yg = accel_xl * np.sin(yaw) + accel_yl * np.cos(yaw)

        dxvel_dyaw = -dt * (inp[0][0] * np.sin(self.state[2][0]) + inp[1][0] * np.cos(self.state[2][0]))
        dyvel_dyaw =  dt * (inp[0][0] * np.cos(self.state[2][0]) - inp[1][0] * np.sin(self.state[2][0]))

        dxvel_din1 =  dt * np.cos(self.state[2][0])
        dxvel_din2 = -dt * np.sin(self.state[2][0])
        dyvel_din1 =  dt * np.sin(self.state[2][0])
        dyvel_din2 =  dt * np.cos(self.state[2][0])

        g_inp = np.asarray([accel_xg, accel_yg, inp[2][0]]).reshape(3,1)
        # State updation with input
        self.state = A.dot(self.state) + B.dot(g_inp)
        #self.state = np.asarray([x_new, y_new, yaw_new, xvel_new, yvel_new]).reshape(5,1) 
        
        if(self.state[2][0] > np.pi):
            self.state[2][0] = self.state[2][0] - 2 * np.pi
        elif(self.state[2][0] < -np.pi):
            self.state[2][0] = self.state[2][0] + 2 * np.pi

        # x_new    = self.state[0][0] + dt * self.state[3][0]
        # y_new    = self.state[1][0] + dt * self.state[4][0]
        # yaw_new  = self.state[2][0] + dt * inp[2][0]
        # xvel_new = self.state[3][0] + dt * (inp[0][0] * np.cos(self.state[2][0]) - inp[1][0] * np.sin(self.state[2][0]))
        # yvel_new = self.state[4][0] + dt * (inp[0][0] * np.sin(self.state[2][0]) + inp[1][0] * np.cos(self.state[2][0]))
 
        A = np.asarray([\
            [1, 0, 0,           dt,0], \
            [0, 1, 0,           0, dt],\
            [0, 0, 1,           0, 0], \
            [0, 0, dxvel_dyaw,  1, 0], \
            [0, 0, dyvel_dyaw,  0, 1]  \
            ])

        B = np.asarray([\
            [0,             0,          0], \
            [0,             0,          0], \
            [0,             0,          dt],\
            [dxvel_din1,    dxvel_din2, 0], \
            [dyvel_din1,    dyvel_din2, 0], \
            ])
        
        # Covariance update
        self.covar = A.dot(self.covar.dot(A.T)) + B.dot(self.Q.dot(B.T))
    
        # Append to trajectory
        self.states.append([self.state, time, 0])
        self.covars.append([self.covar, time, 0])

        # Update previous time
        self.prev_time = time

    def measure(self, measurement, time):
        # How to find expected measurement from state?
        H = np.asarray([\
                        [1, 0, 0, 0, 0], \
                        [0, 1, 0, 0, 0], \
                        ])

        measurement = np.asarray(measurement).reshape(2,1)

        # Error of measurement from expected measurement
        V = measurement - H.dot(self.state)

        S = H.dot(self.covar.dot(H.T)) + self.R

        K = self.covar.dot(H.T.dot(np.linalg.inv(S)))

        self.state = self.state + K.dot(V)

        self.covar = self.covar - K.dot(S.dot(K.T))
        
        # Append to trajectory
        self.states.append([self.state, time, 1])
        self.covars.append([self.covar, time, 1])

    # Return position
    def get_pos(self):
        return (self.states[len(self.states)-1])
