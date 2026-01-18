import numpy as np
import matplotlib.pyplot as plt
from math import asin

import socket
import time

def clamp(x,a,b):
    if(x > b):
        return b
    if(x < a):
        return a
    return x

def clamp3(x,a,b):
    return np.array([clamp(x[0],a,b),clamp(x[1],a,b),clamp(x[2],a,b)])

def clamp_sum(x,max_sum):
    real_sum = np.sum(np.abs(x))
    if(real_sum > max_sum):
        return x * (max_sum/real_sum)
    return x


class HighpassFilter():
    def __init__(self,f):
        self.first_frame = 1
        self.T = 1/(2*np.pi*f)

    def do_filter(self,x,dt):
        if(self.first_frame):
            self.first_frame = 0
            self.x_prev = x
            self.y = x * 0.0
        alpha = self.T/(self.T+dt)
        self.y = alpha * (self.y +  x - self.x_prev)
        self.x_prev = x
        return self.y

class LowpassFilter():
    def __init__(self,f):
        self.first_frame = 1
        self.T = 1/(2*np.pi*f)

    def do_filter(self,x,dt):
        if(self.first_frame):
            self.first_frame = 0
            self.y_prev = x
        alpha = dt/(dt+self.T)
        self.y_prev = self.y_prev + alpha * (x - self.y_prev)
        return self.y_prev


class Butterworth2ndOrder:
    def __init__(self, lowpass=1, cutoff_freq = 1.0):
        self.type = lowpass
        self.f_c = cutoff_freq
        self.prev_dt = 0

        # History buffers (using arrays to support 3D vectors like [x, y, z])
        self.x1 = self.x2 = 0
        self.y1 = self.y2 = 0

        # Coefficients
        self.b0 = self.b1 = self.b2 = 0
        self.a1 = self.a2 = 0

    def _update_coeffs(self, dt):
        # Sampling frequency
        fs = 1.0 / dt

        # Standard Bilinear Transform equations
        # Pre-warp frequency to ensure the digital filter matches the analog intent
        pre_warped_omega = 2 * fs * np.tan(np.pi * self.f_c / fs)

        # Intermediate variables for Butterworth (Q = 0.707)
        K = np.pi * self.f_c * dt
        K2 = K * K
        norm = 1 / (1 + np.sqrt(2) * K + K2)

        if self.type == 1: # Lowpass
            self.b0 = K2 * norm
            self.b1 = 2 * self.b0
            self.b2 = self.b0
            self.a1 = 2 * (K2 - 1) * norm
            self.a2 = (1 - np.sqrt(2) * K + K2) * norm
        else: # Highpass
            self.b0 = norm
            self.b1 = -2 * self.b0
            self.b2 = self.b0
            self.a1 = 2 * (K2 - 1) * norm
            self.a2 = (1 - np.sqrt(2) * K + K2) * norm

    def do_filter(self, x, dt):
        # Only recompute if dt changes (with a small epsilon for float precision)
        if abs(dt - self.prev_dt) > 1e-6:
            self._update_coeffs(dt)
            self.prev_dt = dt

        # Difference equation: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
        y = (self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2
             - self.a1 * self.y1 - self.a2 * self.y2)

        # Update history
        self.x2, self.x1 = self.x1, x
        self.y2, self.y1 = self.y1, y

        return y

class Integrator():
    def __init__(self,bleeding_lambda = 0.15):
        self.first_frame = 1
        self.bleeding_lambda = bleeding_lambda

    def do_filter(self,x,dt):
        if(self.first_frame):
            self.first_frame = 0
            self.acc = x*0.0
        self.acc *= np.exp(-self.bleeding_lambda * dt) #Bleeding
        self.acc += x*dt
        return self.acc

class Derivator():
    def __init__(self):
        self.first_frame = 1

    def do_filter (self,x,dt):
        if(self.first_frame):
            self.first_frame = 0
            self.x_last = x
            self.first_frame = 0
        value = (x-self.x_last)/dt
        self.x_last = x
        return value

class FilteredDerivator:
    def __init__(self, cutoff_freq):
        self.f_c = cutoff_freq
        self.first_frame = True
        self.x_prev = 0
        self.y_prev = 0

    def do_filter(self, x, dt):
        if self.first_frame:
            self.x_prev = x
            self.y_prev = x * 0.0 # Initialize with same shape as x
            self.first_frame = False
            return self.y_prev

        # Calculate the raw derivative
        raw_derivative = (x - self.x_prev) / dt

        # Low-pass filter the result (First-order)
        # alpha = dt / (dt + T), where T = 1 / (2 * pi * f_c)
        T = 1.0 / (2.0 * np.pi * self.f_c)
        alpha = dt / (dt + T)

        y = self.y_prev + alpha * (raw_derivative - self.y_prev)

        # Update history
        self.x_prev = x
        self.y_prev = y

        return y

class TiltCoordinate():
    def __init__(self):
        self.roll_scale = 1.0
        self.pitch_scale = 1.0
    def do_filter(self,x,dt):
        return np.array([asin(clamp(self.roll_scale*x[1],-1,1)),asin(clamp(-self.pitch_scale*x[0],-1,1)),0])

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
def sendTelemetry(name, value):
    now = time.time() * 1000
    msg = name+":"+str(now)+":"+str(value)+"|g"
    sock.sendto(msg.encode(), ("127.0.0.1",47269))

class ClassicalWashout():
    def __init__(self):
        self.angle_limit = np.deg2rad(5.5) #Hard limit, the sum of output pitch and roll cannot go above this threshold
        self.pos_limit = 0.25 #Hard limit, the sum of all output x y and z cannot go above this threshold

        self.angle_vel_limit = np.deg2rad(6) #Output Platform angular velocity cannot exceed this
        self.pos_vel_limit = 0.15 #Output Platform linear velocity cannot exceed this

        self.pos_acc_scale = 0.05 # The scale of the linear acceleration input
        self.tilt_scale = 0.25 # Tilt is already scaled according to gravity
        self.ang_vel_scale = 1 # The scale of the angular velocity input

        self.translation_hp_cut_freq = 0.3 # Translation lane highpass frequency
        self.rotation_hp_cut_freq = 0.8 # Rotation lane highpass frequency
        self.tilt_lp_cut_freq = 1.0 # Tilt lane lowpass frequency

        self.gravity = 9.81

        #Intiliatisation of classical washout building blocks
        # self.translation_hp = HighpassFilter(self.translation_hp_cut_freq)
        # self.rotation_hp = HighpassFilter(self.rotation_hp_cut_freq)
        # self.tilt_lp = LowpassFilter(self.tilt_lp_cut_freq)

        self.translation_hp = Butterworth2ndOrder(lowpass=0, cutoff_freq=self.translation_hp_cut_freq)
        self.rotation_hp = Butterworth2ndOrder(lowpass=0, cutoff_freq=self.rotation_hp_cut_freq)
        self.tilt_lp = Butterworth2ndOrder(lowpass=1, cutoff_freq=self.tilt_lp_cut_freq)

        self.translation_integrator1 = Integrator(bleeding_lambda=0.5)
        self.translation_integrator2 = Integrator(bleeding_lambda=0.3)
        self.rotation_integrator = Integrator()
        self.tilt_coordinate = TiltCoordinate()

    def do_filter(self,pos_acc,ang_vel,ang,dt):

        # sendTelemetry("Input Acceleration X",pos_acc[0])
        # sendTelemetry("Input Acceleration Y",pos_acc[1])
        # sendTelemetry("Input Acceleration Z",pos_acc[2])

        # sendTelemetry("Input Angle Velocity Roll",ang_vel[0])
        # sendTelemetry("Input Angle Velocity Pitch",ang_vel[1])
        # sendTelemetry("Input Angle Velocity Yaw",ang_vel[2])

        # Translation Lane
        translation_lane = pos_acc * self.pos_acc_scale
        tilt_lane = pos_acc.copy()

        translation_lane = self.translation_hp.do_filter(translation_lane,dt)
        # sendTelemetry("Filtered Acceleration X",translation_lane[0])
        # sendTelemetry("Filtered Acceleration Y",translation_lane[1])
        # sendTelemetry("Filtered Acceleration Z",translation_lane[2])

        translation_lane = self.translation_integrator1.do_filter(translation_lane,dt)
        translation_lane = clamp3(translation_lane,-self.pos_vel_limit,self.pos_vel_limit) # Velocity limit
        translation_lane = self.translation_integrator2.do_filter(translation_lane,dt)

        res_pos = translation_lane

        # Tilt-coordination Lane
        gravity_vec = self.gravity*np.array([np.sin(-ang[1]), -np.sin(ang[0])*np.cos(-ang[1]), np.cos(ang[0])*np.cos(-ang[1])])
        tilt_lane = tilt_lane + gravity_vec.astype(np.float64)

        tilt_lane = self.tilt_lp.do_filter(tilt_lane,dt)

        tilt_lane = self.tilt_scale * tilt_lane/self.gravity

        tilt_lane = (self.tilt_coordinate.do_filter(tilt_lane,dt))

        # Rotation Lane
        rotation_lane = ang_vel * self.ang_vel_scale

        rotation_lane = self.rotation_hp.do_filter(rotation_lane,dt)
        rotation_lane = clamp3(rotation_lane,-self.angle_vel_limit,self.angle_vel_limit) # Angular velocity limit

        rotation_lane = self.rotation_integrator.do_filter(rotation_lane,dt)

        res_angle = rotation_lane + tilt_lane
        res_angle[2] = 0 # Ignore Yaw

        res_pos = clamp_sum(res_pos,self.pos_limit) #Hard Limit
        res_angle = clamp_sum(res_angle,self.angle_limit) #Hard Limit

        # sendTelemetry("Output Position X",res_pos[0])
        # sendTelemetry("Output Position Y",res_pos[1])
        # sendTelemetry("Output Position Z",res_pos[2])

        # sendTelemetry("Output Angle Roll",res_angle[0])
        # sendTelemetry("Output Angle Pitch",res_angle[1])
        # sendTelemetry("Output Angle Yaw",res_angle[2])

        return (res_pos,res_angle)