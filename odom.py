#!/usr/bin/env python3

import math, random
#import numpy as np

#from functools import partial #allows more arguments to a callback

import time
import sys

class Odom():
    def __init__(self):
        now_msec = time.ticks_ms()
        self.prev_msec = now_msec
        
        self.bot_rad = 0
        self.botx = 0.0
        self.boty = 0.0
        self.dist = 0.0
        
        self.counts_per_meter = -1706.0
        self.gyro_bias_dps = -0.3
        self.prev_enc_left = 0
        self.prev_enc_right = 0
    
    def update_odom(self, enc_left, enc_right, yaw_rate_deg):
        
        t2 = time.ticks_ms()
        t1 = self.prev_msec
        self.prev_msec = t2
        dt = (t2 - t1) * 1.e-3

        dleft = enc_left - self.prev_enc_left
        dright = enc_right - self.prev_enc_right
        self.prev_enc_left = enc_left
        self.prev_enc_right = enc_right
        
        dmeters = (dleft + dright) / 2.0 / self.counts_per_meter
        self.dist += dmeters
        dtheta_rad = (yaw_rate_deg - self.gyro_bias_dps) * math.pi/180.0 * dt

        #update bot position
        self.bot_rad = self.bot_rad + dtheta_rad
        dx = dmeters*math.cos(self.bot_rad)
        dy = dmeters*math.sin(self.bot_rad)
        self.botx = self.botx + dx
        self.boty = self.boty + dy
