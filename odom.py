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
        self.track_width = 3.4 * 0.0254  # 3.4 inches wheel-to-wheel distance, converted to meters
        self.prev_enc_left = 0
        self.prev_enc_right = 0
    
    def update_odom(self, enc_left, enc_right, yaw_rate_deg):
        """
        Update odometry using encoder-based differential drive kinematics.
        
        For differential drive:
        - Distance: dmeters = (dLeft + dRight) / 2 / counts_per_meter
        - Heading: dTheta = (dRight - dLeft) / track_width
        - Position: integrate distance at current heading
        """
        
        dleft = enc_left - self.prev_enc_left
        dright = enc_right - self.prev_enc_right
        self.prev_enc_left = enc_left
        self.prev_enc_right = enc_right
        
        # Distance traveled using encoder deltas (same formula as before)
        dmeters = (dleft + dright) / 2.0 / self.counts_per_meter
        self.dist += dmeters
        
        # Change in heading from differential motion (encoder-based, no gyro)
        # Convert encoder deltas to meters, then compute rotation
        dleft_m = dleft / self.counts_per_meter
        dright_m = dright / self.counts_per_meter
        dtheta_rad = (dright_m - dleft_m) / self.track_width

        # Update heading
        self.bot_rad = self.bot_rad + dtheta_rad
        
        # Update position based on average distance and current heading
        dx = dmeters * math.cos(self.bot_rad)
        dy = dmeters * math.sin(self.bot_rad)
        self.botx = self.botx + dx
        self.boty = self.boty + dy
