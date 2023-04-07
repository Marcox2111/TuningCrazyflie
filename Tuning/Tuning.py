# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2016 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

import logging
import time

from math import isnan
import numpy as np
import pickle
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from swarm_cf.sensor.qualisys import QualisysSensor
from swarm_cf.agent import CrazyflieRealAgent
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
import threading
import matplotlib.pyplot as plt
import tkinter as tk
import pickle
from tkinter import ttk

t_values=[]
x_values = []
y_values = []
z_values = []
yaw_values=[]
fig, axs = plt.subplots(3, 1, sharex=True)
time_start=float(time.time())
# URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7C7')
URI = 'radio://0/80/2M/E7E7E7E7C7'
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


def send_command(tab_index):
    xkp = xkp_text.get()
    yawkp = yawkp_text.get()
    xkd = xkd_text.get()
    ykp = ykp_text.get()
    yawki = yawki_text.get()
    yawkd = yawkd_text.get()
    zkp = zkp_text.get()
    ykd = ykd_text.get()
    zkd = zkd_text.get()
    vxkp = vxkp_text.get()
    vyawkp = vyawkp_text.get()
    vxkd = vxkd_text.get()
    vykp = vykp_text.get()
    vyawki = vyawki_text.get()
    vyawkd = vyawkd_text.get()
    vzkp = vzkp_text.get()
    vykd = vykd_text.get()
    vzkd = vzkd_text.get()

    try:
        agent.scf.cf.param.set_value('posCtlPid.xKp', float(xkp))
    except: 
        pass
    try:
        agent.scf.cf.param.set_value('pid_attitude.yaw_kp', float(yawkp))
    except: 
        pass
    try:
        agent.scf.cf.param.set_value('posCtlPid.xKd', float(xkd))
    except: 
        pass
    try:
        agent.scf.cf.param.set_value('posCtlPid.yKp', float(ykp))
    except: 
        pass
    try:
        agent.scf.cf.param.set_value('pid_attitude.yaw_ki', float(yawki))
    except: 
        pass
    try:
        agent.scf.cf.param.set_value('pid_attitude.yaw_kd', float(yawkd))
    except: 
        pass
    try:
        agent.scf.cf.param.set_value('posCtlPid.zKp', float(zkp))
    except: 
        pass
    try:
        agent.scf.cf.param.set_value('posCtlPid.yKd', float(ykd))
    except: 
        pass
    try:
        agent.scf.cf.param.set_value('posCtlPid.zKd', float(zkd))
    except: 
        pass
    try:
        agent.scf.cf.param.set_value('posCtlPid.yKp', float(ykp))
    except: 
        pass
    try:
        agent.scf.cf.param.set_value('pid_attitude.yaw_ki', float(yawki))
    except: 
        pass
    try:
        agent.scf.cf.param.set_value('pid_attitude.yaw_kd', float(yawkd))
    except: 
        pass
    try:
        agent.scf.cf.param.set_value('posCtlPid.zKp', float(zkp))
    except: 
        pass
    try:
        agent.scf.cf.param.set_value('posCtlPid.yKd', float(ykd))
    except: 
        pass
    try:
        agent.scf.cf.param.set_value('posCtlPid.zKd', float(zkd))
    except: 
        pass
    try:
        agent.scf.cf.param.set_value('velCtlPid.vxKp', float(vxkp))
    except: 
        pass
    try:
        agent.scf.cf.param.set_value('pid_rate.yaw_kp', float(vyawkp))
    except: 
        pass
    try:
        agent.scf.cf.param.set_value('velCtlPid.vxKd', float(vxkd))
    except: 
        pass
    try:
        agent.scf.cf.param.set_value('velCtlPid.vyKp', float(vykp))
    except: 
        pass
    try:
        agent.scf.cf.param.set_value('pid_rate.yaw_ki', float(vyawki))
    except: 
        pass
    try:
        agent.scf.cf.param.set_value('pid_rate.yaw_kd', float(vyawkd))
    except: 
        pass
    try:
        agent.scf.cf.param.set_value('velCtlPid.vzKp', float(vzkp))
    except: 
        pass
    try:
        agent.scf.cf.param.set_value('velCtlPid.vyKd', float(vykd))
    except: 
        pass
    try:
        agent.scf.cf.param.set_value('velCtlPid.vzKd', float(vzkd))
    except: 
        pass
    try:
        agent.scf.cf.param.set_value('velCtlPid.vyKp', float(vykp))
    except: 
        pass
    try:
        agent.scf.cf.param.set_value('pid_rate.yaw_ki', float(vyawki))
    except: 
        pass
    try:
        agent.scf.cf.param.set_value('pid_rate.yaw_kd', float(vyawkd))
    except: 
        pass
    try:
        agent.scf.cf.param.set_value('velCtlPid.vzKp', float(vzkp))
    except: 
        pass
    try:
        agent.scf.cf.param.set_value('velCtlPid.vyKd', float(vykd))
    except: 
        pass
    try:
        agent.scf.cf.param.set_value('velCtlPid.vzKd', float(vzkd))
    except: 
        pass

    with open("Tuning.pkl", "wb") as f:
        pickle.dump((xkp, yawkp, xkd, ykp, yawki, yawkd, zkp, ykd, zkd, vxkp, vyawkp, vxkd, vykp, vyawki, vyawkd, vzkp, vykd, vzkd), f)
        

def start_gui():
    root = tk.Tk()
    root.title("My GUI")
    notebook = ttk.Notebook(root)
    tab1 = ttk.Frame(notebook)
    tab2 = ttk.Frame(notebook)

    with open("Tuning.pkl", "rb") as f:
        value_xkp, value_yawkp, value_xkd, value_ykp, value_yawki, value_yawkd, value_zkp, value_ykd, value_zkd, value_vxkp, value_vyawkp, value_vxkd, value_vykp, value_vyawki, value_vyawkd, value_vzkp, value_vykd, value_vzkd= pickle.load(f)
    
    global xkp_text, yawkp_text, xkd_text, ykp_text, yawki_text, ykd_text, zkp_text,zkd_text, yawkd_text, vxkp_text, vyawkp_text, vxkd_text, vykp_text, vyawki_text, vykd_text, vzkp_text, vyawkd_text, vzkd_text 

    xkp_text = tk.StringVar(value=str(value_xkp))
    yawkp_text = tk.StringVar(value=str(value_yawkp))
    xkd_text = tk.StringVar(value=str(value_xkd))
    ykp_text = tk.StringVar(value=str(value_ykp))
    yawki_text = tk.StringVar(value=str(value_yawki))
    yawkd_text = tk.StringVar(value=str(value_yawkd))
    zkp_text = tk.StringVar(value=str(value_zkp))
    ykd_text = tk.StringVar(value=str(value_ykd))
    zkd_text = tk.StringVar(value=str(value_zkd)) 
    vxkp_text = tk.StringVar(value=str(value_vxkp))
    vyawkp_text = tk.StringVar(value=str(value_vyawkp))
    vxkd_text = tk.StringVar(value=str(value_vxkd))
    vykp_text = tk.StringVar(value=str(value_vykp))
    vyawki_text = tk.StringVar(value=str(value_vyawki))
    vyawkd_text = tk.StringVar(value=str(value_vyawkd))
    vzkp_text = tk.StringVar(value=str(value_vzkp))
    vykd_text = tk.StringVar(value=str(value_vykd))
    vzkd_text = tk.StringVar(value=str(value_vzkd)) 
    # Labels and Entries for Tab 1
    labels_and_vars1 = [

        (xkp_text, "XKP:", value_xkp),
        (yawkp_text, "YAWKP:", value_yawkp),
        (xkd_text, "XKD:", value_xkd),
        (ykp_text, "YKP:", value_ykp),
        (yawki_text, "YAWKI:", value_yawki),
        (yawkd_text, "YAWKD:", value_yawkd),
        (zkd_text, "ZKP:", value_zkp),
        (zkp_text, "YKD:", value_ykd),
        (ykd_text, "ZKD:", value_zkd)
    ]
    for i, (variable, label, value) in enumerate(labels_and_vars1):
        tk.Label(tab1, text=label).grid(row=i // 3, column=(i%3) * 2,sticky="E")
        tk.Entry(tab1, textvariable=variable).grid(row=i // 3, column=(i%3) *2+ 1,sticky="E")


    button1 = tk.Button(tab1, text="Send command", command=lambda: send_command(0))
    button1.grid(row=3, column=2)
    
    # Labels and Entries for Tab 2
    labels_and_vars2 = [
        (vxkp_text, "VXKP:", value_vxkp),
        (vyawkp_text, "VYAWKP:", value_vyawkp),
        (vxkd_text, "VXKD:", value_vxkd),
        (vykp_text, "VYKP:", value_vykp),
        (vyawki_text, "VYAWKI:", value_vyawki),
        (vyawkd_text,"VYAWKD:", value_vyawkd),
        (vzkd_text, "VZKP:", value_vzkp),
        (vzkp_text, "VYKD:", value_vykd),
        (vykd_text, "VZKD:", value_vzkd)
    ]
    for i, (variable, label, value) in enumerate(labels_and_vars2):
        tk.Label(tab2, text=label).grid(row=i // 3, column=(i % 3) * 2,sticky="E")
        tk.Entry(tab2, textvariable=variable).grid(row=i // 3, column=(i%3)*2 + 1,sticky="E")

    button2 = tk.Button(tab2, text="Send command", command=lambda: send_command(1))
    button2.grid(row=3, column=2)
    notebook.add(tab1, text="Tab 1")
    notebook.add(tab2, text="Tab 2")
    notebook.pack(expand=True, fill="both")
    root.mainloop()
 

def feed_pose_awesome(agent):
    quaternion = np.zeros(4)
    # estimated = agent.sensors.estimated_state.sense()
    measured = agent.sensors.groundtruth.sense()
    yaw = -measured['yaw']
    quaternion[2] = np.sin(yaw/2)
    quaternion[3] = np.cos(yaw/2)
    if not isnan(quaternion[0]) and \
    not isnan(quaternion[1]) and \
    not isnan(quaternion[2]) and \
    not isnan(quaternion[3]):
        agent.scf.cf.extpos.send_extpose(measured['x'], measured['y'], measured['z'], quaternion[0], quaternion[1], quaternion[2], quaternion[3])

        time_now=float(time.time())-time_start
        t_values.append(time_now)
        x_values.append(float(measured['x']))
        y_values.append(float(measured['y']))
        z_values.append(float(measured['z']))


              
def plot():
     # create the figure and subplots
    # plt.plot(time,x_values,color="blue")
    axs[0].plot(t_values,x_values,color="blue")
    axs[1].plot(t_values,y_values,color="blue")
    axs[2].plot(t_values,z_values,color="blue")
    plt.xlim(0,30)
    if t_values[-1] >= 30:
         plt.xlim(t_values[-1]-30,t_values[-1])
    plt.pause(0.3)

def setup_agent(agent):
	# add lighthouse sensor
	sensors = {
		# 'groundtruth': LHSensorCrazyflie(
		#     agent,
		#     get_calib_filename(),
		#     _lighthouse_print_status_bool=PRINT_BOOL
		# ),

		'groundtruth': QualisysSensor(agent, rigid_body_id="uav31"),
	}
	# sensors['estimated_state'].initialize(agent)
	agent.add_sensors(sensors)


def connection_backgroud(agent):
    # Initialize the low-level drivers
    # setup_agent(agent)
    sensors = {
		'groundtruth': QualisysSensor(agent, rigid_body_id="blimp2"),
	}
    agent.add_sensors(sensors)
    agent.takeoff()
    agent.scf.cf.param.set_value('quadSysId.timetostart', '300')
    agent.scf.cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    agent.scf.cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(0.1)

    # while True:
    for _ in range(100000):
        feed_pose_awesome(agent)
        agent.scf.cf.commander.send_position_setpoint(0, 0, 1, 0)
        
    print("hey")
    # for i in range(4000):
    #     feed_pose_awesome(agent,i)
    #     agent.scf.cf.commander.send_position_setpoint(1, 0, 1, 0)
    # print("ciao")
    agent.land()


    # with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
    #     cf = scf.cf

    #     cf.param.set_value('kalman.resetEstimation', '1')
    #     time.sleep(0.1)
    #     cf.param.set_value('kalman.resetEstimation', '0')
    #     time.sleep(2)

    #     turning_time = 5
    #     forward_time = 10
    #     size = 2

    #     print("takeoff")
    #     while True:
    #         for y in range(100):
    #             cf.commander.send_position_setpoint(0, 0, 1, 0)
    #             time.sleep(0.1)
        # while True:
        #     print("straight")
        #     for y in range(forward_time*10):
        #         cf.commander.send_position_setpoint(size, 0, 1, 0)
        #         time.sleep(0.1)

        #     print("turn")
        #     for y in range(turning_time*10):
        #         cf.commander.send_position_setpoint(size, 0, 1, -90)
        #         time.sleep(0.1)

        #     print("continue")
        #     for y in range(forward_time*10):
        #         cf.commander.send_position_setpoint(size, -size, 1, -90)
        #         time.sleep(0.1)

        #     print("turn")
        #     for y in range(turning_time*10):
        #         cf.commander.send_position_setpoint(size, -size, 1, -180)
        #         time.sleep(0.1)

        #     print("continue")
        #     for y in range(forward_time*10):
        #         cf.commander.send_position_setpoint(0, -size, 1, -180)
        #         time.sleep(0.1)

        #     print("turn")
        #     for y in range(turning_time*10):
        #         cf.commander.send_position_setpoint(0, -size, 1, -270)
        #         time.sleep(0.1)

        #     print("continue")
        #     for y in range(forward_time*10):
        #         cf.commander.send_position_setpoint(0, 0, 1, -270)
        #         time.sleep(0.1)

        #     print("turn")
        #     for y in range(turning_time*10):
        #         cf.commander.send_position_setpoint(0, 0, 1, 0)
        #         time.sleep(0.1)


if __name__ == '__main__':
    cflib.crtp.init_drivers()    
    agent = CrazyflieRealAgent(uri=URI, cf_height=1, max_velocity=5, max_rate_yaw=10)
    gui_thread = threading.Thread(target=start_gui)
    gui_thread.start()
    # connection_backgroud(agent)
    # bg_thread = threading.Thread(target=connection_backgroud(agent))
    # plot_thread = threading.Thread(target=plot)
    # bg_thread.start()
    # plot_thread.start()()
