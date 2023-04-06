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
import sys
from PyQt5 import QtWidgets, QtCore


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
    xkp = xkp_text.text()
    yawkp = yawkp_text.text()
    xkd = xkd_text.text()
    ykp = ykp_text.text()
    yawki = yawki_text.text()
    yawkd = yawkd_text.text()
    zkp = zkp_text.text()
    ykd = ykd_text.text()
    zkd = zkd_text.text()
    vxkp = vxkp_text.text()
    vyawkp = vyawkp_text.text()
    vxkd = vxkd_text.text()
    vykp = vykp_text.text()
    vyawki = vyawki_text.text()
    vyawkd = vyawkd_text.text()
    vzkp = vzkp_text.text()
    vykd = vykd_text.text()
    vzkd = vzkd_text.text()

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
        
class MyWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("My GUI")

        self.notebook = QtWidgets.QTabWidget()
        self.tab1 = QtWidgets.QWidget()
        self.tab2 = QtWidgets.QWidget()

        with open("Tuning.pkl", "rb") as f:
            value_xkp, value_yawkp, value_xkd, value_ykp, value_yawki, value_yawkd, value_zkp, value_ykd, value_zkd, value_vxkp, value_vyawkp, value_vxkd, value_vykp, value_vyawki, value_vyawkd, value_vzkp, value_vykd, value_vzkd = pickle.load(f)

        # create the labels
        xkp_lab = QtWidgets.QLabel("XKP:", self.tab1)
        yawkp_lab = QtWidgets.QLabel("YAWKP:", self.tab1)
        xkd_lab = QtWidgets.QLabel("XKD:", self.tab1)
        ykp_lab = QtWidgets.QLabel("YKP:", self.tab1)
        yawki_lab = QtWidgets.QLabel("YAWKI:", self.tab1)
        yawkd_lab = QtWidgets.QLabel("YAWKD:", self.tab1)
        zkp_lab = QtWidgets.QLabel("ZKP:", self.tab1)
        ykd_lab = QtWidgets.QLabel("ykd:", self.tab1)
        zkd_lab = QtWidgets.QLabel("ZKD:", self.tab1)
        vxkp_lab = QtWidgets.QLabel("VXKP:", self.tab2)
        vyawkp_lab = QtWidgets.QLabel("VYAWKP:", self.tab2)
        vxkd_lab = QtWidgets.QLabel("VXKD:", self.tab2)
        vykp_lab = QtWidgets.QLabel("VYKP:", self.tab2)
        vyawki_lab = QtWidgets.QLabel("VYAWKI:", self.tab2)
        vyawkd_lab = QtWidgets.QLabel("VYAWKD:", self.tab2)
        vzkp_lab = QtWidgets.QLabel("VZKP:", self.tab2)
        vykd_lab = QtWidgets.QLabel("Vykd:", self.tab2)
        vzkd_lab = QtWidgets.QLabel("VZKD:", self.tab2)

        # create the text boxes
        global xkp_text, yawkp_text, xkd_text, ykp_text, yawki_text, yawkd_text, zkp_text, ykd_text,zkd_text, vxkp_text, vyawkp_text, vxkd_text, vykp_text, vyawki_text, vyawkd_text, vzkp_text, vykd_text, vzkd_text    
        xkp_text = QtWidgets.QLineEdit(str(value_xkp), self.tab1)
        yawkp_text = QtWidgets.QLineEdit(str(value_yawkp), self.tab1)
        xkd_text = QtWidgets.QLineEdit(str(value_xkd), self.tab1)
        ykp_text = QtWidgets.QLineEdit(str(value_ykp), self.tab1)
        yawki_text = QtWidgets.QLineEdit(str(value_yawki), self.tab1)
        yawkd_text = QtWidgets.QLineEdit(str(value_yawkd), self.tab1)
        zkp_text = QtWidgets.QLineEdit(str(value_zkp), self.tab1)
        ykd_text = QtWidgets.QLineEdit(str(value_ykd), self.tab1)
        zkd_text = QtWidgets.QLineEdit(str(value_zkd), self.tab1)
        vxkp_text = QtWidgets.QLineEdit(str(value_vxkp), self.tab2)
        vyawkp_text = QtWidgets.QLineEdit(str(value_vyawkp), self.tab2)
        vxkd_text = QtWidgets.QLineEdit(str(value_vxkd), self.tab2)
        vykp_text = QtWidgets.QLineEdit(str(value_vykp), self.tab2)
        vyawki_text = QtWidgets.QLineEdit(str(value_vyawki), self.tab2)
        vyawkd_text = QtWidgets.QLineEdit(str(value_vyawkd), self.tab2)
        vzkp_text = QtWidgets.QLineEdit(str(value_vzkp), self.tab2)
        vykd_text = QtWidgets.QLineEdit(str(value_vykd), self.tab2)
        vzkd_text = QtWidgets.QLineEdit(str(value_vzkd), self.tab2)


        # create the button
        button1 = QtWidgets.QPushButton("Send command", self.tab1)
        button1.clicked.connect(lambda: send_command(0))
        button2 = QtWidgets.QPushButton("Send command", self.tab2)
        button2.clicked.connect(lambda: send_command(1))

        # position the labels and text boxes
        grid_layout1 = QtWidgets.QGridLayout(self.tab1)
        grid_layout1.addWidget(xkp_lab, 0, 0)
        grid_layout1.addWidget(xkp_text, 0, 1)
        grid_layout1.addWidget(xkd_lab, 0, 2)
        grid_layout1.addWidget(xkd_text, 0, 3)
        grid_layout1.addWidget(yawkp_lab, 0, 4)
        grid_layout1.addWidget(yawkp_text, 0, 5)
        grid_layout1.addWidget(ykp_lab, 1, 0)
        grid_layout1.addWidget(ykp_text, 1, 1)
        grid_layout1.addWidget(ykd_lab, 1, 2)
        grid_layout1.addWidget(ykd_text, 1, 3)
        grid_layout1.addWidget(yawkd_lab, 1, 4)
        grid_layout1.addWidget(yawkd_text, 1, 5)
        grid_layout1.addWidget(zkp_lab, 2, 0)
        grid_layout1.addWidget(zkp_text, 2, 1)
        grid_layout1.addWidget(zkd_lab, 2, 2)
        grid_layout1.addWidget(zkd_text, 2, 3)
        grid_layout1.addWidget(yawki_lab, 2, 4)
        grid_layout1.addWidget(yawki_text, 2, 5)
        grid_layout1.addWidget(button1, 3, 2)

        # position the labels and text boxes
        grid_layout2 = QtWidgets.QGridLayout(self.tab2)
        grid_layout2.addWidget(vxkp_lab, 0, 0)
        grid_layout2.addWidget(vxkp_text, 0, 1)
        grid_layout2.addWidget(vxkd_lab, 0, 2)
        grid_layout2.addWidget(vxkd_text, 0, 3)
        grid_layout2.addWidget(vyawkp_lab, 0, 4)
        grid_layout2.addWidget(vyawkp_text, 0, 5)
        grid_layout2.addWidget(vykp_lab, 1, 0)
        grid_layout2.addWidget(vykp_text, 1, 1)
        grid_layout2.addWidget(vykd_lab, 1, 2)
        grid_layout2.addWidget(vykd_text, 1, 3)
        grid_layout2.addWidget(vyawkd_lab, 1, 4)
        grid_layout2.addWidget(vyawkd_text, 1, 5)
        grid_layout2.addWidget(vzkp_lab, 2, 0)
        grid_layout2.addWidget(vzkp_text, 2, 1)
        grid_layout2.addWidget(vzkd_lab, 2, 2)
        grid_layout2.addWidget(vzkd_text, 2, 3)
        grid_layout2.addWidget(vyawki_lab, 2, 4)
        grid_layout2.addWidget(vyawki_text, 2, 5)
        grid_layout2.addWidget(button2, 3, 2)

        self.notebook.addTab(self.tab1, "Tab 1")
        self.notebook.addTab(self.tab2, "Tab 2")

        # position the button
        vbox = QtWidgets.QVBoxLayout(self)
        vbox.addWidget(self.notebook)

def feed_pose_awesome(agent,i):
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
        
        # plt.figure(2)
        # plot(measured['y'])
        # plt.figure(3)
        # plot(measured['z'])
              
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
		'groundtruth': QualisysSensor(agent, rigid_body_id="blimp2"),
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
    time_start=float(time.time())

    # while True:
    for i in range(100000):
        feed_pose_awesome(agent,i)
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
    app = QtWidgets.QApplication(sys.argv)
    window = MyWindow()
    window.show()
    agent = CrazyflieRealAgent(uri=URI, cf_height=1, max_velocity=5, max_rate_yaw=10)
    QtCore.QTimer.singleShot(1000, connection_backgroud(agent))
    # bg_thread = threading.Thread(target=connection_backgroud(agent))
    # plot_thread = threading.Thread(target=plot)
    # bg_thread.start()
    # plot_thread.start()
    sys.exit(app.exec_())
    
