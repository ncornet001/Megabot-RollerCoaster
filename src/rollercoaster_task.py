import time
import sys
import numpy as np
import pinocchio as pin
import placo
from placo_utils.visualization import robot_viz, frame_viz, point_viz, robot_frame_viz, line_viz, arrow_viz, get_viewer
from placo_utils.tf import tf
import math
import megabot
from threading import Thread, Lock
import threading

from geo import X,Y,Z
from task import Task
from rollercoaster.src.washout_algorithm import ClassicalWashout, Derivator, LowpassFilter, Butterworth2ndOrder, FilteredDerivator
from rollercoaster.src.simulated_telemetry import SimulatedReceiver

import time
import os

import socket
import re
import json

teleplotAddr = ("127.0.0.1",47269)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# --- Main Telemetry Receiver ---
class EpicRollerCoastersTelemetry():
    def __init__(self, ip="0.0.0.0", port=7701):
        self.ip = ip
        self.port = port
        self.last_received_udp_packet = ""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.ip, self.port))
        print(f"[INFO] Listening for UDP packets on {self.ip}:{self.port}")

    def start(self):
        thread = threading.Thread(target=self.receive_data, daemon=True)
        thread.start()
        print("[INFO] UDP receive thread started.")
        # Keep main thread alive
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n[INFO] Telemetry listener stopped.")

    def receive_data(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(1024)
                text = data.decode("utf-8")
                self.last_received_udp_packet = text

                speed = self.get_number("S", text)
                vibration_intensity = self.get_number("V", text)
                vibration_frequency = self.get_number("F", text)
                yaw = self.get_number("Y", text)
                pitch = self.get_number("P", text)
                roll = self.get_number("R", text)
                heave = self.get_number("H", text)
                sway = self.get_number("W", text)
                surge = self.get_number("U", text)
                yaw_vel = self.get_number("A", text)
                pitch_vel = self.get_number("I", text)
                roll_vel = self.get_number("O", text)

                # os.system('cls' if os.name == 'nt' else 'clear')
                print(f">> Speed: {speed:.3f}, Yaw: {yaw:.3f}, Pitch: {pitch:.3f}, Roll: {roll:.3f}, "
                      f"Heave: {heave:.3f}, Sway: {sway:.3f}, Surge: {surge:.3f}, "
                      f"YawVelocity: {yaw_vel:.3f}, PitchVelocity: {pitch_vel:.3f}, RollVelocity: {roll_vel:.3f}, "
                      f"VibrationIntensity: {vibration_intensity:.3f}, VibrationFrequency: {vibration_frequency:.3f}")
                # sendTelemetry("Speed",speed)

                # sendTelemetry("Yaw",yaw)
                # sendTelemetry("Pitch",pitch)
                # sendTelemetry("Roll",roll)

                # sendTelemetry("Heave",heave)
                # sendTelemetry("Sway",sway)
                # sendTelemetry("Surge",surge)

            except Exception as e:
                print(f"[UDPReceive] Error: {e}")

    def get_number(self, prefix, data):
        number_rx = r"-*\d{3}\.\d{3}"
        data_block_rx = rf"{prefix}\[{number_rx}\]"
        match = re.search(data_block_rx, data)
        if match:
            num_match = re.search(number_rx, match.group())
            if num_match:
                return float(num_match.group())
        return 0.0

    def get_int(self, prefix, data):
        int_rx = r"-*\d{2}"
        data_block_rx = rf"{prefix}\[{int_rx}\]"
        match = re.search(data_block_rx, data)
        if match:
            num_match = re.search(int_rx, match.group())
            if num_match:
                return int(num_match.group())
        return 0

def sendTelemetry(name, value):
    now = time.time() * 1000
    msg = name+":"+str(now)+":"+str(value)+"|g"
    sock.sendto(msg.encode(), teleplotAddr)

def pose_to_T(position, rpy):
    R = pin.rpy.rpyToMatrix(rpy)

    # Homogeneous transformation matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = position

    return T

class RollerCoasterWashout(Task):
    def __init__(self,com,robot,robotLock, simulated_telemetry_active = False, ip="0.0.0.0", port=7701) -> None:
        self.com = com
        self.robot = robot
        self.robotLock = robotLock
        self.simulated_telemetry_active = simulated_telemetry_active
        super().__init__(0.005)
        self.solver = megabot.make_solver(self.robot)
        T_world_base=self.robot.get_T_world_frame("base")
        self.base_height=T_world_base[2,3]
        self.height_dir=1
        self.pos_origin = T_world_base[:,3][:3]
        self.base_task = self.solver.add_position_task("base",self.pos_origin)
        self.base_task.configure("base", "soft", 1.0)
        self.base_orientation_task = self.solver.add_orientation_task("base",T_world_base[:3,:3])
        self.base_orientation_task.configure("base", "soft", 100.0)
        for l in [1,2,3,4]:
            current_position=self.robot.get_T_world_frame(f"leg_{l}")[:,3][:3]
            task = self.solver.add_position_task(f"leg_{l}",current_position)
            task.configure(f"leg_{l}", "hard", 1.0) # keep legs at their position
        self.state="start"
        self.initial_rotation=T_world_base[:3,:3].copy()
        print("initial rotation is ",self.initial_rotation)
        self.magnitude=math.pi/360.0*20.0

        self.ip = ip
        self.port = port
        self.last_received_udp_packet = ""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.ip, self.port))
        self.sock.settimeout(.25)
        print(f"[INFO] Listening for UDP packets on {self.ip}:{self.port}")


        self.start_time=time.time()

        self.wagon_acc = np.array([.0,.0,.0])
        self.wagon_T = np.zeros((4,4))

        self.input_angles = [[0,0,0]]
        self.output_angles = [[0,0,0]]
        self.timestamps = [0]

        self.speed = 0
        self.vibration_intensity = 0
        self.vibration_frequency = 0
        self.yaw = 0
        self.pitch = 0
        self.roll = 0
        self.heave = 0
        self.sway = 0
        self.surge = 0
        self.yaw_vel = 0
        self.pitch_vel = 0
        self.roll_vel = 0

        self.delta_time = 0.1
        self.pos_derivator = FilteredDerivator(cutoff_freq=1)
        self.vel_derivator = FilteredDerivator(cutoff_freq=1)
        self.pos_lp = Butterworth2ndOrder(lowpass=1, cutoff_freq=1)
        self.gravity = 9.81
        self.cw = ClassicalWashout()
        self.angles_derivator = Derivator()
        self.angles_lp = LowpassFilter(1)
        self.angles_vel_lp = LowpassFilter(3)

        self.last_delta_time = time.time()

    def onVis(self,mc):
        arrow_viz("WagonAcc",np.array([.0,.0,.0]),self.wagon_acc,0x669911,0.01)
        frame_viz("WagonFrame",self.wagon_T,1.0,2.0)

    def leaving(self):
        print("Stopping")

    def dtTick(self):
        current_time = time.time()
        self.delta_time = current_time-self.last_delta_time
        self.last_delta_time = current_time
        base_position=self.robot.get_T_world_frame("base")[:,3][:3].copy()

        if self.state=="start":
            # set start rotation to : (0,self.magnitude,0)
            t=time.time()-self.start_time
            print("Roller coaster start ",t)
            self.base_orientation_task.R_world_frame=pin.rpy.rpyToMatrix(0,0,0)
            if t>2.0:
                print("running")
                self.state="running"
                self.start_time=time.time()

        elif self.state=="end": # back to zero
            t=time.time()-self.start_time
            self.base_orientation_task.R_world_frame=pin.rpy.rpyToMatrix(self.last[0]*(2.0-t)/2.0,
                                                                         self.last[1]*(2.0-t)/2.0,0)@self.initial_rotation
            if  t>2.0:
                self.com.send_stop()
                return True
        else: # running
            print("roller coaster:run")
            t=time.time()-self.start_time
            if(self.simulated_telemetry_active):
                self.simulate_receive_data()
            else:
                self.receive_data()
            target = self.process_data(self.delta_time)
            self.base_task.target_world = target[0] + self.pos_origin
            self.base_orientation_task.R_world_frame=pin.rpy.rpyToMatrix(target[1][0], target[1][1], 0)@self.initial_rotation

            out_pos = self.pos_lp.do_filter(np.array([base_position[X],base_position[Y],base_position[Z]]), self.delta_time)

            sendTelemetry("Real position X",out_pos[0])
            sendTelemetry("Real position Y",out_pos[1])
            sendTelemetry("Real position Z",out_pos[2])

            out_pos_vel = self.pos_derivator.do_filter(out_pos, self.delta_time)

            sendTelemetry("Real speed X",out_pos_vel[0])
            sendTelemetry("Real speed Y",out_pos_vel[1])
            sendTelemetry("Real speed Z",out_pos_vel[2])

            out_pos_acc = self.vel_derivator.do_filter(out_pos_vel, self.delta_time)

            sendTelemetry("Real Acceleration X",out_pos_acc[0])
            sendTelemetry("Real Acceleration Y",out_pos_acc[1])
            sendTelemetry("Real Acceleration Z",out_pos_acc[2])

            if t>3000:
                print("end")
                self.state="end"
                self.last=(target[1][0], target[1][1], 0)
                self.start_time=time.time()
        self.solver.solve(True)
        #sendTelemetry("Megabot Angle Roll",res_angle[0])
        #sendTelemetry("Megabot Angle Pitch",res_angle[1])
        print("error:",self.base_orientation_task.error_norm())
        print("\n\n")
        self.robotLock.acquire()
        self.robot.update_kinematics()
        self.robotLock.release()
        for l in [1,2,3,4]:
            for c in [1,2,3]:
                self.com.send_move(l,c,self.robot.get_joint(f"l{l}_c{c}"))
        return False

    sr = SimulatedReceiver('./rollercoaster/assets/rocky_mountains-telemetry.json',1765984761.0+90)
    def simulate_receive_data(self):
        t=time.time()-self.start_time
        print("Gathering data from log at " + str(t))
        self.speed = self.sr.interpolate(t,'Speed')
        self.vibration_intensity = self.sr.interpolate(t,'VibrationIntensity')

        self.yaw = self.sr.interpolate(t,'Yaw')
        self.pitch = self.sr.interpolate(t,'Pitch')
        self.roll = self.sr.interpolate(t,'Roll')

        self.heave = self.sr.interpolate(t,'Heave')
        self.sway = self.sr.interpolate(t,'Sway')
        self.surge = self.sr.interpolate(t,'Surge')

        self.yaw_vel = self.sr.interpolate(t,'YawVelocity')
        self.pitch_vel = self.sr.interpolate(t,'PitchVelocity')
        self.roll_vel = self.sr.interpolate(t,'RollVelocity')

        #sendTelemetry("Speed",self.speed)

        sendTelemetry("YawVel",self.yaw_vel)
        sendTelemetry("PitchVel",self.pitch_vel)
        sendTelemetry("RollVel",self.roll_vel)

        #sendTelemetry("PitchVel",self.pitch_vel)

        #sendTelemetry("Heave",self.heave)
        #sendTelemetry("Sway",self.sway)
        #sendTelemetry("Surge",self.surge)

    def receive_data(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            text = data.decode("utf-8")
            self.last_received_udp_packet = text

            self.speed = self.get_number("S", text)
            self.vibration_intensity = self.get_number("V", text)
            self.vibration_frequency = self.get_number("F", text)
            self.yaw = self.get_number("Y", text)
            self.pitch = self.get_number("P", text)
            self.roll = self.get_number("R", text)
            self.heave = self.get_number("H", text)
            self.sway = self.get_number("W", text)
            self.surge = self.get_number("U", text)
            self.yaw_vel = self.get_number("A", text)
            self.pitch_vel = self.get_number("I", text)
            self.roll_vel = self.get_number("O", text)

            # os.system('cls' if os.name == 'nt' else 'clear')
            print(f">> Speed: {self.speed:.3f}, Yaw: {self.yaw:.3f}, Pitch: {self.pitch:.3f}, Roll: {self.roll:.3f}, "
                    f"Heave: {self.heave:.3f}, Sway: {self.sway:.3f}, Surge: {self.surge:.3f}, "
                    # f"YawVelocity: {yaw_vel:.3f}, PitchVelocity: {pitch_vel:.3f}, RollVelocity: {roll_vel:.3f}, "
                    # f"VibrationIntensity: {vibration_intensity:.3f}, VibrationFrequency: {vibration_frequency:.3f}"
                    )


            #sendTelemetry("Heave",self.heave)
            #sendTelemetry("Sway",self.sway)
            #sendTelemetry("Surge",self.surge)

        except Exception as e:
            print(f"[UDPReceive] Error: {e}")

    def process_data(self,dt):
        raw_acceleration = np.array([self.heave,self.sway,self.surge])
        raw_angles = np.deg2rad(np.array([self.roll,self.pitch,self.yaw]))
        raw_angles_velocities = np.array([self.roll_vel,self.pitch_vel,self.yaw_vel])

        self.wagon_acc = raw_acceleration
        self.wagon_T = pose_to_T(np.array([.0,.0,.0]),np.array([raw_angles[0],raw_angles[1],0]))

        target = self.cw.do_filter(raw_acceleration,raw_angles_velocities, raw_angles,dt) # Classical Washout processing
        return target

    def get_number(self, prefix, data):
        number_rx = r"-*\d{3}\.\d{3}"
        data_block_rx = rf"{prefix}\[{number_rx}\]"
        match = re.search(data_block_rx, data)
        if match:
            num_match = re.search(number_rx, match.group())
            if num_match:
                return float(num_match.group())
        return 0.0

    def get_int(self, prefix, data):
        int_rx = r"-*\d{2}"
        data_block_rx = rf"{prefix}\[{int_rx}\]"
        match = re.search(data_block_rx, data)
        if match:
            num_match = re.search(int_rx, match.group())
            if num_match:
                return int(num_match.group())
        return 0


class RollerCoasterLinear(Task):
    def __init__(self,com,robot,robotLock, simulated_telemetry_active = False,ip="0.0.0.0", port=7701) -> None:
        self.com = com
        self.robot = robot
        self.robotLock = robotLock
        self.simulated_telemetry_active = simulated_telemetry_active
        super().__init__(0.005)
        self.solver = megabot.make_solver(self.robot)
        T_world_base=self.robot.get_T_world_frame("base")
        self.base_height=T_world_base[2,3]
        self.height_dir=1
        self.base_task = self.solver.add_position_task("base",T_world_base[:,3][:3])
        self.base_task.configure("base", "hard", 1.0)
        self.base_orientation_task = self.solver.add_orientation_task("base",T_world_base[:3,:3])
        self.base_orientation_task.configure("base", "soft", 100.0)
        for l in [1,2,3,4]:
            current_position=self.robot.get_T_world_frame(f"leg_{l}")[:,3][:3]
            task = self.solver.add_position_task(f"leg_{l}",current_position)
            task.configure(f"leg_{l}", "hard", 1.0) # keep legs at their position
        self.state="start"
        self.initial_rotation=T_world_base[:3,:3].copy()
        print("initial rotation is ",self.initial_rotation)
        self.magnitude=math.pi/180.0*5.0

        self.ip = ip
        self.port = port
        self.last_received_udp_packet = ""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.ip, self.port))
        self.sock.settimeout(.25)
        print(f"[INFO] Listening for UDP packets on {self.ip}:{self.port}")

        # self.load_telemetry()
        self.roll=0.0
        self.pitch=0.0

        self.sr = SimulatedReceiver('./rollercoaster/assets/rocky_mountains-telemetry.json',1765984761.0)

        self.start_time=time.time()
        self.freq_time=time.time()

    def dtTick(self):
        base_position=self.robot.get_T_world_frame("base")[:,3][:3].copy()

        if self.state=="start":
            # set start rotation to : (0,self.magnitude,0)
            t=time.time()-self.start_time
            print("Roller coaster start ",t)
            self.base_orientation_task.R_world_frame=pin.rpy.rpyToMatrix(0,0,0)
            if t>1.0:
                print("running")
                self.state="running"
                self.start_time=time.time()
            #print("danse:start : ",base_position[X])
            # move slowly base to startup position
            #base_position[X]+=self.dt/4.0
            #self.base_task.target_world=base_position
            #if abs(base_position[X]-0.1)<0.02:
            #    print("running")
        elif self.state=="end": # back to zero
            t=time.time()-self.start_time
            self.base_orientation_task.R_world_frame=pin.rpy.rpyToMatrix(self.last[0]*(2.0-t)/2.0,
                                                                         self.last[1]*(2.0-t)/2.0,0)@self.initial_rotation
            if  t>2.0:
                self.com.send_stop()
                return True
        else: # running
            # print("roller coaster:run")
            t=time.time()-self.start_time

            if(self.simulated_telemetry_active):
                self.simulate_receive_data()
            else:
                self.receive_data()

            # self.simulate_receive_data()
            # self.get_telemetry_at_time((time.time()-self.start_run_time)*1000.0)
            # print(f"t={t}ms -> Roll={telemetry['roll']:.2f}, Pitch={telemetry['pitch']:.2f}")
            # self.roll = telemetry['roll']
            # self.pitch = telemetry['pitch']

            if self.roll<0:
                self.roll = max(self.roll/8, -1)*self.magnitude
            else:
                self.roll = min(self.roll/8,  1)*self.magnitude

            if self.pitch<0:
                self.pitch = max(self.pitch/20, -1)*self.magnitude
            else:
                self.pitch = min(self.pitch/20,  1)*self.magnitude

            if(abs(self.pitch)+abs(self.roll)>self.magnitude):
                self.pitch = self.pitch/((abs(self.pitch)+abs(self.roll))/self.magnitude)
                self.roll = self.roll/((abs(self.pitch)+abs(self.roll))/self.magnitude)

            # sendTelemetry("Game Yaw",self.yaw)
            sendTelemetry("Meg target pitch",self.pitch*180/np.pi)
            sendTelemetry("Meg target roll",self.roll*180/np.pi)

            # sendTelemetry("Game Heave",self.heave)
            # sendTelemetry("Game Sway",self.sway)
            # sendTelemetry("Game Surge",self.surge)

            self.base_orientation_task.R_world_frame=pin.rpy.rpyToMatrix(self.roll, self.pitch, 0)@self.initial_rotation
            if t>3000:
                print("end")
                self.state="end"
                self.last=(self.roll, self.pitch, 0)
                self.start_time=time.time()

        self.solver.solve(True)
        print("error:",self.base_orientation_task.error_norm())
        print("\n")
        # sendTelemetry("Placo error",self.base_orientation_task.error_norm())
        self.robotLock.acquire()
        self.robot.update_kinematics()
        # frame_viz("target", self.base_orientation_task.R_world_frame)
        self.robotLock.release()

        rpy = pin.rpy.matrixToRpy(self.base_orientation_task.R_world_frame @ self.initial_rotation.T)
        sendTelemetry("Meg actual pitch",rpy[1])
        sendTelemetry("Meg actual roll",rpy[0])
        sendTelemetry("Meg actual yaw",rpy[2])

        if(self.base_orientation_task.error_norm()>0.02):
            old_error = self.base_orientation_task.error_norm()
            self.solver.solve(True)
            print("error:",self.base_orientation_task.error_norm())
            print("\n")
            # sendTelemetry("Placo error",self.base_orientation_task.error_norm())
            self.robotLock.acquire()
            self.robot.update_kinematics()
            # frame_viz("target", self.base_orientation_task.R_world_frame)
            self.robotLock.release()
            sendTelemetry("Placo error improvement",old_error - self.base_orientation_task.error_norm())

        sendTelemetry("Placo error",self.base_orientation_task.error_norm())

        for l in [1,2,3,4]:
            for c in [1,2,3]:
                self.com.send_move(l,c,self.robot.get_joint(f"l{l}_c{c}"))

        sendTelemetry("Prog freq",1/(time.time()-self.freq_time))
        self.freq_time=time.time()
        return False

    def receive_data(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            text = data.decode("utf-8")
            self.last_received_udp_packet = text

            self.speed = self.get_number("S", text)
            self.vibration_intensity = self.get_number("V", text)
            self.vibration_frequency = self.get_number("F", text)
            self.yaw = self.get_number("Y", text)
            self.pitch = self.get_number("P", text)
            self.roll = self.get_number("R", text)
            self.heave = self.get_number("H", text)
            self.sway = self.get_number("W", text)
            self.surge = self.get_number("U", text)
            self.yaw_vel = self.get_number("A", text)
            self.pitch_vel = self.get_number("I", text)
            self.roll_vel = self.get_number("O", text)

            # os.system('cls' if os.name == 'nt' else 'clear')
            # print(f">> Speed: {self.speed:.3f}, Yaw: {self.yaw:.3f}, Pitch: {self.pitch:.3f}, Roll: {self.roll:.3f}, "
                    # f"Heave: {self.heave:.3f}, Sway: {self.sway:.3f}, Surge: {self.surge:.3f}, "
                    # f"YawVelocity: {yaw_vel:.3f}, PitchVelocity: {pitch_vel:.3f}, RollVelocity: {roll_vel:.3f}, "
                    # f"VibrationIntensity: {vibration_intensity:.3f}, VibrationFrequency: {vibration_frequency:.3f}"
                    # )
            sendTelemetry("Game Speed",self.speed)

            sendTelemetry("Game Yaw",self.yaw)
            sendTelemetry("Game Pitch",self.pitch)
            sendTelemetry("Game Roll",self.roll)

            sendTelemetry("Game Heave",self.heave)
            sendTelemetry("Game Sway",self.sway)
            sendTelemetry("Game Surge",self.surge)

        except Exception as e:
            print(f"[UDPReceive] Error: {e}")

    def simulate_receive_data(self):
        t=time.time()-self.start_time
        print("Gathering data from log at " + str(t))
        self.speed = self.sr.interpolate(t,'Speed')
        self.vibration_intensity = self.sr.interpolate(t,'VibrationIntensity')

        self.yaw = self.sr.interpolate(t,'Yaw')
        self.pitch = self.sr.interpolate(t,'Pitch')
        self.roll = self.sr.interpolate(t,'Roll')

        self.heave = self.sr.interpolate(t,'Heave')
        self.sway = self.sr.interpolate(t,'Sway')
        self.surge = self.sr.interpolate(t,'Surge')

        self.yaw_vel = self.sr.interpolate(t,'YawVelocity')
        self.pitch_vel = self.sr.interpolate(t,'PitchVelocity')
        self.roll_vel = self.sr.interpolate(t,'RollVelocity')


    def get_number(self, prefix, data):
        number_rx = r"-*\d{3}\.\d{3}"
        data_block_rx = rf"{prefix}\[{number_rx}\]"
        match = re.search(data_block_rx, data)
        if match:
            num_match = re.search(number_rx, match.group())
            if num_match:
                return float(num_match.group())
        return 0.0

    def get_int(self, prefix, data):
        int_rx = r"-*\d{2}"
        data_block_rx = rf"{prefix}\[{int_rx}\]"
        match = re.search(data_block_rx, data)
        if match:
            num_match = re.search(int_rx, match.group())
            if num_match:
                return int(num_match.group())
        return 0