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


class imu_integrate:
    # State : A column vector with [x_pos, y_pos, yaw, x_vel, y_vel]    
    def __init__(self, init_state, init_time):
        self.state  = init_state
        self.prev_time   = init_time

        self.states = []

    # Input : A column vector with [x_accel, y_accel, yaw_vel]
    def update(self, inp, time):
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
        
        yaw      = self.state[2]
        accel_xl = inp[0]
        accel_yl = inp[1]
        accel_xg = accel_xl * np.cos(yaw) - accel_yl * np.sin(yaw)
        accel_yg = accel_xl * np.sin(yaw) + accel_yl * np.cos(yaw)

        inp[0]  = accel_xg
        inp[1]  = accel_yg

        # State updation with input
        self.state = A.dot(self.state) + B.dot(inp)

        if(self.state[2] > np.pi):
            self.state[2] = self.state[2] - 2 * np.pi
        elif(self.state[2] < -np.pi):
            self.state[2] = self.state[2] + 2 * np.pi

        # Append to states
        self.states.append([self.state, time])

        # Update previous time
        self.prev_time = time

    # Return position
    def get_pos(self):
        return (self.states[len(self.states)-1])
