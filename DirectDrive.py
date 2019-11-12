# -----------------------------------------------------------
# This Software is for creating a Direct Drive ForceFeedback
# Steering Wheel for Driving Simulations.
#
# It connects with an ODrive Robotics Board (https://odriverobotics.com/)
# and uses it to send torque commands to a Servo-Motor.
# By interfacing with different games, live ForceFeedback data is acquired.
#
# Feature Overview:
#     - intuitive graphical user interface
#
#     Simulation of physical effects of real Steering Wheel:
#         - friction (constant resistance against movement of the wheel)
#         - damping (proportional resistance against movement of the wheel)
#         - inertia (proportional resistance against acceleration of the wheel)
#
#         - resistance against further wheel movement when reaching bump-to-bump limits
#
#
#     Profile Management:
#         - all settings can be saved in a profile
#         - profiles can be renamed, or deleted
#         - profiles will be saved in 'profiles.xml'
#         - profiles will be loaded when starting the software
#
#
#     Data Reconstruction (!!!DISABLED IN CURRENT VERSION!!!):
#         - dynamic realtime temporal upsampling of incoming force-feedback
#           data to target sample rate of motor controller
#         - optional temporal smoothing of reconstructed data
#
#
# This Software was developed and tested in Python 3.5.3
#
# Packages used in this software:
#     - NumPy
#     - PyQt5
#     - PyQtGraph
#     - PYXinput
#     - ODrive (https://docs.odriverobotics.com/#downloading-and-installing-tools)
#
#
# (C) 2019 Jan Balke, Beckum, Germany
# Released under GNU Public License (GPL)
# email JanBalke94@gmail.com
#
# -----------------------------------------------------------
import os
import sys
import time
import numpy as np
import multiprocessing
from multiprocessing import Value
from MotorController import MotorController
from GameInterfacer import GameInterfacer
from VirtualController import VirtualController
from PyQt5.QtWidgets import QApplication, QPushButton, QSlider, QWidget, QLabel, QCheckBox, QLineEdit, QComboBox
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from Profile import Profile
import xml.etree.ElementTree as ET
from pyqtgraph import PlotWidget
import pyqtgraph as pg

#class LiveGuiUpdater(QThread):
#    ticked = pyqtSignal()
#    def run(self):
#        while True:
#            time.sleep(1/60)
#            self.ticked.emit()

class DirectDriveWindow(QWidget):
    virtual_controller = None                                           # will hold an instance of the VirtualController-class
    motor_controller = None                                             # will hold an instance of the MotorController-class
    game_interfacer = None                                              # will hold an instance of the GameInterfacer-class


    # Variables that will be shared between multiple processes
    raw_force_signal_global = Value('f', 0.0)                           # raw force-feedback value coming from the game
    game_status_playing_global = Value('b', False)                      # game status (playing/not-playing)
    motor_pause_mode = Value('b', False)                                # if the motor should be idle when game is paused

    data_reconstruction = Value('b', False)                             # if the data-reconstruction feature should be used
    data_smoothing_level = Value('i', 0)                                # if the temproal smoothing feature should be used

    friction_global = Value('f', 0.0)                                   # strength of the friction effect
    damping_global = Value('f', 0.0)                                    # strength of the damping effect
    inertia_global = Value('f', 0.0)                                    # strength of the inertia effect

    encoder_pos_global = Value('f', 0.0)                                # holds the wheel position in encoder counts
    wheel_position_deg_global = Value('f', 0.0)                         # holds the wheel position in degrees
    lock_to_lock_global = Value('f', 900.0)                             # holds the digital limit of the wheel (used by the virtual controller)
    bump_to_bump_global = Value('f', 900.0)                             # holds the physical limit of the wheel (used by the motor controller)

    motor_current_global = Value('f', 0.0)                              # holds the current(Ampere) that is applied to the motor at any moment

    actual_max_force_global = Value('f', 0.0)                           # sets the actual maximum torque(Nm) the motor should be limited to
    invert_force_global = Value('b', False)                             # if the raw force-feedback value should be inverted

    motor_connected_global = Value('b', False)                          # motor status (connected/not-connected)
    motor_controller_due_for_restart_global = Value('b', False)         # is set to true when connection to odrive board is lost

    motor_control_rate = Value('f', 0.0)                                # target refresh rate for the motor-control process

    profiles = []                                                       # holds all loaded and newly created profiles
    selected_profile_index = 0                                          # holds the index of the currently selected profile

    graph_sample_rate = 60                                              # update-rate(Hz) for the motor-current graph
    graph_history_time_sec = 2                                          # time-window of the motor-current graph



    def __init__(self):
        super().__init__()
        self.setGeometry(100, 100, 1700, 575)
        self.setWindowTitle('DIY Direct Drive Wheel')

        #self.updater = LiveGuiUpdater()
        #self.updater.ticked.connect(self.LiveGuiTick)
        #self.updater.start()

        self.timer = QTimer()
        self.timer.timeout.connect(self.graph_update)
        self.timer.setInterval(1000 / self.graph_sample_rate)
        self.timer.start()

        self.gui_timer = QTimer()
        self.gui_timer.timeout.connect(self.LiveGuiTick)
        self.gui_timer.setInterval(1000 / 60)
        self.gui_timer.start()

        self.virtual_controller = VirtualController()
        self.virtual_controller.set_frequency(180)
        self.virtual_controller.steering_value_deg = self.wheel_position_deg_global
        self.virtual_controller.lock_to_lock = self.lock_to_lock_global
        self.virtual_controller.start()

        self.game_interfacer = GameInterfacer()
        self.game_interfacer.set_frequency(180)
        self.game_interfacer.wheel_force = self.raw_force_signal_global
        self.game_interfacer.invert_force = self.invert_force_global
        self.game_interfacer.game_status_playing = self.game_status_playing_global

        self.motor_controller = MotorController()
        self.motor_controller.set_frequency(360)
        self.motor_controller.signal_in = self.raw_force_signal_global
        self.motor_controller.game_status_playing = self.game_status_playing_global
        self.motor_controller.motor_pause_mode = self.motor_pause_mode
        self.motor_controller.data_reconstruction = self.data_reconstruction
        self.motor_controller.data_smoothing_level = self.data_smoothing_level
        self.motor_controller.friction = self.friction_global
        self.motor_controller.damping = self.damping_global
        self.motor_controller.inertia = self.inertia_global
        self.motor_controller.encoder_pos = self.encoder_pos_global
        self.motor_controller.wheel_pos_deg = self.wheel_position_deg_global
        self.motor_controller.bump_to_bump = self.bump_to_bump_global
        self.motor_controller.motor_current = self.motor_current_global
        self.motor_controller.actual_max_force = self.actual_max_force_global
        self.motor_controller.motor_connected = self.motor_connected_global
        self.motor_controller.due_for_restart = self.motor_controller_due_for_restart_global
        self.motor_controller.achieved_freq = self.motor_control_rate
        self.motor_controller.start()

        self.lblMotorRate = QLabel('motor control rate:', self)
        self.lblMotorRate.setFont(QFont("Times", 24, QFont.Normal))
        self.lblMotorRate.move(675,25)

        self.lblMotorRateHz = QLabel('0.00 Hz', self)
        self.lblMotorRateHz.setFont(QFont("Times", 24, QFont.Normal))
        self.lblMotorRateHz.adjustSize()
        self.lblMotorRateHz.move(1125-self.lblMotorRateHz.width(), 25)

        self.force_graph = PlotWidget(self)
        self.force_graph.move(675, 75)
        self.force_graph.resize(1000, 475)
        self.force_graph.setYRange(-15.0, 15.0)
        self.force_graph.setBackground((255, 255, 255))
        self.force_graph.showGrid(x=False, y=True)
        self.force_graph.addLegend()

        pen_ffb = pg.mkPen(color=(0, 0, 255))
        self.ffb_curve = self.force_graph.getPlotItem().plot(pen=pen_ffb, name='Game FFB (Nm)')
        pen_current = pg.mkPen(color=(0, 255, 0))
        self.current_curve = self.force_graph.getPlotItem().plot(pen=pen_current, name='Motor Torque (Nm)')
        self.ffb_history = [0]*self.graph_sample_rate*self.graph_history_time_sec
        self.current_history = [0]*self.graph_sample_rate*self.graph_history_time_sec

        self.load_profiles_from_file()
        self.build_gui()
        self.select_profile(0)



    # updates the graph showing the raw ffb and the motor torque
    def graph_update(self):
        # adding the latest raw game-ffb value(scaled to the actual torque value) to a list storing a history of this value
        self.ffb_history.append(self.raw_force_signal_global.value/self.actual_max_force_global.value)
        if len(self.ffb_history) >= self.graph_history_time_sec * self.graph_sample_rate:
            self.ffb_history.pop(0)
        self.ffb_curve.setData(self.ffb_history)

        # adding the latest final motor torque value to a list storing a history of this value
        self.current_history.append(self.motor_current_global.value/(self.actual_max_force_global.value*2.75))
        if len(self.current_history) >= self.graph_history_time_sec*self.graph_sample_rate:
            self.current_history.pop(0)
        self.current_curve.setData(self.current_history)

        # displaying the actually achieved refresh rate of the motor-control process
        self.lblMotorRateHz.setText(str(int(self.motor_control_rate.value*100)/100) + ' Hz')
        if self.motor_control_rate.value >= 360.0:
            self.lblMotorRateHz.setStyleSheet('QLabel { color: rgb(0,155,0); }')
        else:
            self.lblMotorRateHz.setStyleSheet('QLabel { color: rgb(155,0,0); }')

    # initializes and configures all the GUI-elements
    def build_gui(self):
        # WHEEL POSITION DATA AND SETTINGS

        # current bar
        self.lblCurrent = QLabel(self)
        self.lblCurrent.move(25+128, 25)
        self.lblCurrent.resize(0, 25)
        self.lblCurrent.setAutoFillBackground(True)
        self.lblCurrent.setStyleSheet('QLabel { background-color: rgb(0,255,0); }')

        self.lblCurrentDivider = QLabel(self)
        self.lblCurrentDivider.move(25 + 128 - 1, 20)
        self.lblCurrentDivider.resize(2, 35)
        self.lblCurrentDivider.setAutoFillBackground(True)
        self.lblCurrentDivider.setStyleSheet('QLabel { background-color: rgb(0,0,0); }')

        # wheel image
        self.lblWheelImage = QLabel(self)
        self.wheel_image = QImage('wheel_image.png')
        self.wheel_image_discon = QImage('wheel_image_disconnect.png')
        self.wheel_pixmap = QPixmap.fromImage(self.wheel_image)
        self.lblWheelImage.setPixmap(self.wheel_pixmap)
        self.lblWheelImage.resize(self.wheel_pixmap.width(), self.wheel_pixmap.height())
        self.lblWheelImage.move(25,75)

        # motor status label
        self.lblMotorStatus_1 = QLabel('Motor Status:', self)
        self.lblMotorStatus_1.move(15, 60)
        self.lblMotorStatus_2 = QLabel('Not Connected', self)
        self.lblMotorStatus_2.move(15, 75)
        self.lblMotorStatus_2.setStyleSheet('QLabel { color: rgb(155,0,0); }')


        # wheel position
        self.lblWheelPos = QLabel('0.0°', self)
        self.lblWheelPos.move(25+150+25, 75+256+10)
        self.lblWheelPos.resize(200, 50)
        self.lblWheelPos.setFont(QFont("Times", 24, QFont.Normal))

        # center button
        center_pos = [25, 75+256+22, 150, 30]
        self.btnCenter = QPushButton('center Wheel', self)
        self.btnCenter.move(center_pos[0], center_pos[1])
        self.btnCenter.resize(center_pos[2], center_pos[3])
        self.btnCenter.clicked.connect(self.center_wheel)

        # lock-to-lock slider
        self.locklock_pos = [25, 425, 250, 0]
        self.lblLock = QLabel('Lock to Lock:', self)
        self.lblLock.move(self.locklock_pos[0], self.locklock_pos[1])
        self.sliLock = QSlider(Qt.Horizontal, self)
        self.sliLock.setTickPosition(QSlider.TicksAbove)
        self.sliLock.setTickInterval(10)
        self.sliLock.setMinimum(10)
        self.sliLock.setMaximum(120)
        self.sliLock.setValue(90)
        self.sliLock.setSingleStep(1)
        self.sliLock.move(self.locklock_pos[0], self.locklock_pos[1] + 20)
        self.sliLock.resize(self.locklock_pos[2], 25)
        self.sliLock.valueChanged.connect(self.change_lock)
        self.lblLockVal = QLabel('900°', self)
        self.lblLockVal.adjustSize()
        self.lblLockVal.move(self.locklock_pos[0] + self.locklock_pos[2] - self.lblLockVal.width(), self.locklock_pos[1])

        # bump_to_bump slider
        self.bumpbump_pos = [25, 500, 250, 0]
        self.lblBump = QLabel('Bump to Bump:', self)
        self.lblBump.move(self.bumpbump_pos[0], self.bumpbump_pos[1])
        self.sliBump = QSlider(Qt.Horizontal, self)
        self.sliBump.setTickPosition(QSlider.TicksAbove)
        self.sliBump.setTickInterval(10)
        self.sliBump.setMinimum(10)
        self.sliBump.setMaximum(120)
        self.sliBump.setValue(90)
        self.sliBump.setSingleStep(1)
        self.sliBump.move(self.bumpbump_pos[0], self.bumpbump_pos[1] + 20)
        self.sliBump.resize(self.bumpbump_pos[2], 25)
        self.sliBump.valueChanged.connect(self.change_bump)
        self.lblBumpVal = QLabel('900°', self)
        self.lblBumpVal.adjustSize()
        self.lblBumpVal.move(self.bumpbump_pos[0] + self.bumpbump_pos[2] - self.lblBumpVal.width(), self.bumpbump_pos[1])





        # WHEEL FORCE AND BEHAVIOR SETTINGS

        # game select combobox
        gameselect_pos = [325, 25, 250, 25]
        self.cbxGame = QComboBox(self)
        self.cbxGame.move(gameselect_pos[0], gameselect_pos[1])
        self.cbxGame.resize(gameselect_pos[2], gameselect_pos[3])
        for game in self.game_interfacer.games_available:
            self.cbxGame.addItem(game)
        self.cbxGame.activated.connect(self.select_game)

        # invert force checkbox
        invert_pos = [325, 55]
        self.cbInvert = QCheckBox('Invert Force Feedback', self)
        self.cbInvert.move(invert_pos[0], invert_pos[1])
        self.cbInvert.stateChanged.connect(self.toggle_invert)

        # motor pause mode checkbox
        motorpause_pos = [325, 85]
        self.cbMotorPause = QCheckBox('pause motor when game is paused', self)
        self.cbMotorPause.move(motorpause_pos[0], motorpause_pos[1])
        self.cbMotorPause.stateChanged.connect(self.toggle_motor_pause_mode)
        self.cbMotorPause.toggle()

        # maximum force slider
        self.max_force_pos = [325,125,250,0]
        self.lblMaxForce = QLabel('Maximum Force in Nm:', self)
        self.lblMaxForce.move(self.max_force_pos[0], self.max_force_pos[1])
        self.sliMaxForce = QSlider(Qt.Horizontal, self)
        self.sliMaxForce.setTickPosition(QSlider.TicksAbove)
        self.sliMaxForce.setTickInterval(10)
        self.sliMaxForce.setMinimum(1)
        self.sliMaxForce.setMaximum(150)
        self.sliMaxForce.setValue(0)
        self.sliMaxForce.setSingleStep(1)
        self.sliMaxForce.move(self.max_force_pos[0], self.max_force_pos[1]+20)
        self.sliMaxForce.resize(self.max_force_pos[2], 25)
        self.sliMaxForce.valueChanged.connect(self.change_max_force)
        self.lblMaxForceVal = QLabel('0.0 Nm - (0.0 Amp)', self)
        self.lblMaxForceVal.adjustSize()
        self.lblMaxForceVal.move(self.max_force_pos[0] + self.max_force_pos[2] - self.lblMaxForceVal.width(), self.max_force_pos[1])

        # reconstruction checkbox
        reconst_pos = [325, 175+3]
        self.cbReconst = QCheckBox('Use Data Reconstruction', self)
        self.cbReconst.move(reconst_pos[0], reconst_pos[1])
        self.cbReconst.stateChanged.connect(self.toggle_reconstruction)
        self.cbReconst.setEnabled(False)

        # smoothing level combobox
        smoothing_pos = [475, 175, 100, 25]
        self.cbxSmoothing = QComboBox(self)
        self.cbxSmoothing.move(smoothing_pos[0], smoothing_pos[1])
        self.cbxSmoothing.resize(smoothing_pos[2], smoothing_pos[3])
        self.cbxSmoothing.addItem('no Smoothing')
        self.cbxSmoothing.addItem('1')
        self.cbxSmoothing.addItem('2')
        self.cbxSmoothing.addItem('3')
        self.cbxSmoothing.addItem('4')
        self.cbxSmoothing.addItem('5')
        self.cbxSmoothing.addItem('6')
        self.cbxSmoothing.addItem('7')
        self.cbxSmoothing.addItem('8')
        self.cbxSmoothing.addItem('9')
        self.cbxSmoothing.activated.connect(self.change_smoothing)
        self.cbxSmoothing.setEnabled(False)

        # friction slider
        self.friction_pos = [325, 215, 250, 0]
        self.lblFriction = QLabel('Friction:', self)
        self.lblFriction.move(self.friction_pos[0], self.friction_pos[1])
        self.sliFriction = QSlider(Qt.Horizontal, self)
        self.sliFriction.setTickPosition(QSlider.TicksAbove)
        self.sliFriction.setTickInterval(10)
        self.sliFriction.setMinimum(0)
        self.sliFriction.setMaximum(100)
        self.sliFriction.setValue(0)
        self.sliFriction.setSingleStep(1)
        self.sliFriction.move(self.friction_pos[0], self.friction_pos[1] + 20)
        self.sliFriction.resize(self.friction_pos[2], 25)
        self.sliFriction.valueChanged.connect(self.change_friction)
        self.lblFrictionVal = QLabel('0 %', self)
        self.lblFrictionVal.adjustSize()
        self.lblFrictionVal.move(self.friction_pos[0] + self.friction_pos[2] - self.lblFrictionVal.width(), self.friction_pos[1])

        # damping slider
        self.damping_pos = [325, 275, 250, 0]
        self.lblDamping = QLabel('Damping:', self)
        self.lblDamping.move(self.damping_pos[0], self.damping_pos[1])
        self.sliDamping = QSlider(Qt.Horizontal, self)
        self.sliDamping.setTickPosition(QSlider.TicksAbove)
        self.sliDamping.setTickInterval(10)
        self.sliDamping.setMinimum(0)
        self.sliDamping.setMaximum(100)
        self.sliDamping.setValue(0)
        self.sliDamping.setSingleStep(1)
        self.sliDamping.move(self.damping_pos[0], self.damping_pos[1] + 20)
        self.sliDamping.resize(self.damping_pos[2], 25)
        self.sliDamping.valueChanged.connect(self.change_damping)
        self.lblDampingVal = QLabel('0 %', self)
        self.lblDampingVal.adjustSize()
        self.lblDampingVal.move(self.damping_pos[0] + self.damping_pos[2] - self.lblDampingVal.width(), self.damping_pos[1])

        # inertia slider
        self.inertia_pos = [325, 335, 250, 0]
        self.lblInertia = QLabel('Inertia:', self)
        self.lblInertia.move(self.inertia_pos[0], self.inertia_pos[1])
        self.sliInertia = QSlider(Qt.Horizontal, self)
        self.sliInertia.setTickPosition(QSlider.TicksAbove)
        self.sliInertia.setTickInterval(10)
        self.sliInertia.setMinimum(0)
        self.sliInertia.setMaximum(100)
        self.sliInertia.setValue(0)
        self.sliInertia.setSingleStep(1)
        self.sliInertia.move(self.inertia_pos[0], self.inertia_pos[1] + 20)
        self.sliInertia.resize(self.inertia_pos[2], 25)
        self.sliInertia.valueChanged.connect(self.change_inertia)
        self.lblInertiaVal = QLabel('0 %', self)
        self.lblInertiaVal.adjustSize()
        self.lblInertiaVal.move(self.inertia_pos[0] + self.inertia_pos[2] - self.lblInertiaVal.width(),
                                self.inertia_pos[1])

        # profile select combobox
        profileselect_pos = [325, 430, 250, 25]
        self.cbxProfiles = QComboBox(self)
        self.cbxProfiles.move(profileselect_pos[0], profileselect_pos[1])
        self.cbxProfiles.resize(profileselect_pos[2], profileselect_pos[3])
        for profile in self.profiles:
            self.cbxProfiles.addItem(profile.name)
        self.cbxProfiles.activated.connect(self.select_profile)

        # create profile button
        profilecreate_pos = [325, 460, 250, 25]
        self.btnCreate = QPushButton('create new profile', self)
        self.btnCreate.move(profilecreate_pos[0], profilecreate_pos[1])
        self.btnCreate.resize(profilecreate_pos[2], profilecreate_pos[3])
        self.btnCreate.clicked.connect(self.create_profile)

        # rename profile button
        profilerename_pos = [325, 490, 250, 25]
        self.btnRename = QPushButton('rename profile', self)
        self.btnRename.move(profilerename_pos[0], profilerename_pos[1])
        self.btnRename.resize(profilerename_pos[2], profilerename_pos[3])
        self.btnRename.clicked.connect(self.rename_profile)

        # cancel rename button
        cancelrename_pos = [325+200, 490, 50, 25]
        self.btnCancelRename = QPushButton('cancel', self)
        self.btnCancelRename.move(cancelrename_pos[0], cancelrename_pos[1])
        self.btnCancelRename.resize(cancelrename_pos[2], cancelrename_pos[3])
        self.btnCancelRename.clicked.connect(self.cancel_rename)
        self.btnCancelRename.setVisible(False)

        # rename textbox
        self.txtRename = QLineEdit(self)
        self.txtRename.move(profilerename_pos[0], profilerename_pos[1])
        self.txtRename.resize(profilerename_pos[2]-35-55, profilerename_pos[3])
        self.txtRename.setVisible(False)
        self.txtRename.textChanged.connect(self.check_name_validity)

        # save profile button
        profilesave_pos = [325, 520, 120, 25]
        self.btnSave = QPushButton('save profile', self)
        self.btnSave.move(profilesave_pos[0], profilesave_pos[1])
        self.btnSave.resize(profilesave_pos[2], profilesave_pos[3])
        self.btnSave.clicked.connect(self.save_current_profile_internally)

        # delete profile button
        profiledelete_pos = [325+130, 520, 120, 25]
        self.btnDelete = QPushButton('delete profile', self)
        self.btnDelete.move(profiledelete_pos[0], profiledelete_pos[1])
        self.btnDelete.resize(profiledelete_pos[2], profiledelete_pos[3])
        self.btnDelete.clicked.connect(self.delete_current_profile)

    # updates the GUI-elements that indicate the status of the motor (connected/not-connected)
    def LiveGuiTick(self):
        self.lblWheelPos.setText(str(int(self.wheel_position_deg_global.value*10)/10)+'°')

        self.lblCurrent.resize(np.abs(self.motor_current_global.value/(self.actual_max_force_global.value*2.75)*128), 25)
        if self.motor_current_global.value < 0.0:
            self.lblCurrent.move(25+128 - np.abs(self.motor_current_global.value/(self.actual_max_force_global.value*2.75)*128), 25)

        tf = QTransform()
        tf.rotate(self.wheel_position_deg_global.value)

        if self.motor_connected_global.value:
            self.lblMotorStatus_2.setText('Connected')
            self.lblMotorStatus_2.setStyleSheet('QLabel { color: rgb(0,155,0); }')
            img_rot = self.wheel_image.transformed(tf)
            self.btnCenter.setEnabled(True)
            self.lblWheelPos.setStyleSheet('QLabel { color: rgb(0,0,0); }')
        else:
            self.lblMotorStatus_2.setText('Not Connected')
            self.lblMotorStatus_2.setStyleSheet('QLabel { color: rgb(155,0,0); }')
            img_rot = self.wheel_image_discon.transformed(tf)
            self.btnCenter.setEnabled(False)
            self.lblWheelPos.setStyleSheet('QLabel { color: rgb(155,155,155); }')

        diagonal_overlength = (np.sqrt(np.square(256) * 2) - 256) / 2
        shift = np.abs(np.sin(np.deg2rad(self.wheel_position_deg_global.value * 2)) * diagonal_overlength)
        crop_rect = QRect(shift, shift, 256, 256)
        self.wheel_pixmap = QPixmap.fromImage(img_rot).copy(crop_rect)
        self.lblWheelImage.setPixmap(self.wheel_pixmap)

        if self.motor_controller_due_for_restart_global.value:
            self.motor_controller_due_for_restart_global.value = False
            print('restarting motor...')
            try:
                self.motor_controller.motor_control_process.terminate()
            except:
                pass
            self.motor_controller.start()

    # helper function to get the index of a profile with a certain name from the profile-list
    def get_profile_index(self, name):
        for i in range(len(self.profiles)):
            if self.profiles[i].name == name:
                return i



    # WHEEL POSITION SETTING FUNCTIONS
    def center_wheel(self):
        print('centering wheel')
        self.motor_controller.set_encoder_offset()

    def change_lock(self):
        val = int(self.sender().value()*10)
        self.lblLockVal.setText(str(val) + '°')
        self.lock_to_lock_global.value = val
        self.lblLockVal.adjustSize()
        self.lblLockVal.move(self.locklock_pos[0] + self.locklock_pos[2] - self.lblLockVal.width(),
                             self.locklock_pos[1])

    def change_bump(self):
        val = int(self.sender().value()*10)
        self.lblBumpVal.setText(str(val) + '°')
        self.bump_to_bump_global.value = val
        self.lblBumpVal.adjustSize()
        self.lblBumpVal.move(self.bumpbump_pos[0] + self.bumpbump_pos[2] - self.lblBumpVal.width(),
                             self.bumpbump_pos[1])



    # WHEEL FORCE SETTING FUNCTIONS
    def select_game(self, item_index):
        print('selecting', item_index)
        self.game_interfacer.terminate()
        time.sleep(1)
        self.game_interfacer.start(item_index)

    def toggle_invert(self, state):
        if state == Qt.Checked:
            self.invert_force_global.value = True
        else:
            self.invert_force_global.value = False

    def toggle_motor_pause_mode(self, state):
        if state == Qt.Checked:
            self.motor_pause_mode.value = True
        else:
            self.motor_pause_mode.value = False

    def change_max_force(self):
        val = self.sender().value()/10
        self.lblMaxForceVal.setText(str(val)+' Nm - ('+str(int(val*2.75*100)/100)+' Amp)')
        self.actual_max_force_global.value = val
        self.lblMaxForceVal.adjustSize()
        self.lblMaxForceVal.move(self.max_force_pos[0] + self.max_force_pos[2] - self.lblMaxForceVal.width(),
                                 self.max_force_pos[1])
        self.force_graph.setYRange(-val, val)

    def change_friction(self):
        val = self.sender().value()/100
        self.lblFrictionVal.setText(str(int(val*100))+' %')
        self.friction_global.value = val
        self.lblFrictionVal.adjustSize()
        self.lblFrictionVal.move(self.friction_pos[0] + self.friction_pos[2] - self.lblFrictionVal.width(),
                                 self.friction_pos[1])

    def change_damping(self):
        val = self.sender().value()/100
        self.lblDampingVal.setText(str(int(val*100))+' %')
        self.damping_global.value = val
        self.lblDampingVal.adjustSize()
        self.lblDampingVal.move(self.damping_pos[0] + self.damping_pos[2] - self.lblDampingVal.width(),
                                self.damping_pos[1])

    def change_inertia(self):
        val = self.sender().value()/100
        self.lblInertiaVal.setText(str(int(val*100))+' %')
        self.inertia_global.value = val
        self.lblInertiaVal.adjustSize()
        self.lblInertiaVal.move(self.inertia_pos[0] + self.inertia_pos[2] - self.lblInertiaVal.width(),
                                self.inertia_pos[1])

    def toggle_reconstruction(self, state):
        if state == Qt.Checked:
            self.data_reconstruction.value = True
        else:
            self.data_reconstruction.value = False

    def change_smoothing(self, item_index):
        self.data_smoothing_level.value = item_index

    def select_profile(self, selected_profile):
        self.sliLock.setValue(int(self.profiles[selected_profile].lock_to_lock/10))
        self.sliBump.setValue(int(self.profiles[selected_profile].bump_to_bump/10))
        self.cbInvert.setChecked(bool(self.profiles[selected_profile].invert_force))
        self.sliMaxForce.setValue(int(self.profiles[selected_profile].max_force*10))
        self.cbReconst.setChecked(bool(self.profiles[selected_profile].use_reconstruction))
        self.cbxSmoothing.setCurrentIndex(int(self.profiles[selected_profile].smoothing_level))
        self.sliFriction.setValue(int(self.profiles[selected_profile].friction*100))
        self.sliDamping.setValue(int(self.profiles[selected_profile].damping*100))
        self.sliInertia.setValue(int(self.profiles[selected_profile].inertia*100))

    def save_current_profile_internally(self):
        current_index = self.cbxProfiles.currentIndex()
        self.profiles[current_index].lock_to_lock = self.lock_to_lock_global.value
        self.profiles[current_index].bump_to_bump = self.bump_to_bump_global.value
        self.profiles[current_index].invert_force = self.invert_force_global.value
        self.profiles[current_index].max_force = self.actual_max_force_global.value
        self.profiles[current_index].use_reconstruction = self.data_reconstruction.value
        self.profiles[current_index].smoothing_level = self.data_smoothing_level.value
        self.profiles[current_index].friction = self.friction_global.value
        self.profiles[current_index].damping = self.damping_global.value
        self.profiles[current_index].inertia = self.inertia_global.value
        self.save_profiles_to_file()

    def save_profiles_to_file(self):
        root = ET.Element('profiles')
        for profile in self.profiles:
            prof_elem = ET.SubElement(root, 'profile', name=profile.name)

            ET.SubElement(prof_elem, 'lock_to_lock').text = str(profile.lock_to_lock)
            ET.SubElement(prof_elem, 'bump_to_bump').text = str(profile.bump_to_bump)

            ET.SubElement(prof_elem, 'invert_force').text = str(bool(profile.invert_force))

            ET.SubElement(prof_elem, 'max_force').text = str(profile.max_force)

            ET.SubElement(prof_elem, 'use_reconstruction').text = str(bool(profile.use_reconstruction))
            ET.SubElement(prof_elem, 'smoothing_level').text = str(profile.smoothing_level)

            ET.SubElement(prof_elem, 'friction').text = str(profile.friction)
            ET.SubElement(prof_elem, 'damping').text = str(profile.damping)
            ET.SubElement(prof_elem, 'inertia').text = str(profile.inertia)

        tree = ET.ElementTree(root)
        tree.write("profiles.xml")

    def load_profiles_from_file(self):
        self.profiles.clear()
        tree = ET.parse('profiles.xml')
        root = tree.getroot()
        for prof in root:
            profile = Profile(prof.attrib['name'])
            profile.lock_to_lock = float(prof[0].text)
            profile.bump_to_bump = float(prof[1].text)
            profile.invert_force = bool(prof[2].text == 'True')
            profile.max_force = float(prof[3].text)
            profile.use_reconstruction = bool(prof[4].text == 'True')
            profile.smoothing_level = int(prof[5].text)
            profile.friction = float(prof[6].text)
            profile.damping = float(prof[7].text)
            profile.inertia = float(prof[8].text)
            self.profiles.append(profile)

    def delete_current_profile(self):
        current_index = self.cbxProfiles.currentIndex()
        print('deleting profile at index:', current_index)
        self.profiles.pop(current_index)
        self.cbxProfiles.clear()
        for profile in self.profiles:
            self.cbxProfiles.addItem(profile.name)
        self.cbxProfiles.setCurrentIndex(0)
        self.select_profile(0)
        self.save_current_profile_internally()
        if len(self.profiles) == 1:
            self.btnDelete.setEnabled(False)

    def create_profile(self):
        new_profile = Profile('profile #'+str(len(self.profiles)+1))
        current_index = self.cbxProfiles.currentIndex()

        new_profile.lock_to_lock = self.profiles[current_index].lock_to_lock
        new_profile.bump_to_bump = self.profiles[current_index].bump_to_bump
        new_profile.invert_force = self.profiles[current_index].invert_force
        new_profile.max_force = self.profiles[current_index].max_force
        new_profile.use_reconstruction = self.profiles[current_index].use_reconstruction
        new_profile.smoothing_level = self.profiles[current_index].smoothing_level
        new_profile.friction = self.profiles[current_index].friction
        new_profile.damping = self.profiles[current_index].damping
        new_profile.inertia = self.profiles[current_index].inertia
        self.profiles.append(new_profile)

        self.cbxProfiles.clear()
        for profile in self.profiles:
            self.cbxProfiles.addItem(profile.name)
        self.cbxProfiles.setCurrentIndex(len(self.profiles)-1)
        self.select_profile(len(self.profiles)-1)
        self.save_current_profile_internally()
        if len(self.profiles) > 1:
            self.btnDelete.setEnabled(True)

    def rename_profile(self):
        rename_pos = [325, 490, 250, 25]
        if self.sender().text() == 'rename profile':
            self.sender().setText('OK')
            self.sender().move(rename_pos[0]+220-55, rename_pos[1])
            self.sender().resize(rename_pos[2]-220, rename_pos[3])
            self.cbxProfiles.setEnabled(False)
            self.btnCreate.setEnabled(False)
            self.btnSave.setEnabled(False)
            self.btnDelete.setEnabled(False)
            self.btnCancelRename.setVisible(True)
            self.txtRename.setVisible(True)
            current_index = self.cbxProfiles.currentIndex()
            self.txtRename.setText(self.profiles[current_index].name)
        else:
            self.sender().setText('rename profile')
            self.sender().move(rename_pos[0], rename_pos[1])
            self.sender().resize(rename_pos[2], rename_pos[3])
            self.cbxProfiles.setEnabled(True)
            self.btnCreate.setEnabled(True)
            self.btnSave.setEnabled(True)
            self.btnDelete.setEnabled(True)
            self.btnCancelRename.setVisible(False)
            self.txtRename.setVisible(False)
            current_index = self.cbxProfiles.currentIndex()
            self.profiles[current_index].name = self.txtRename.text()

            self.cbxProfiles.clear()
            for profile in self.profiles:
                self.cbxProfiles.addItem(profile.name)
            self.cbxProfiles.setCurrentIndex(current_index)
            self.select_profile(current_index)
            self.save_current_profile_internally()

    def cancel_rename(self):
        rename_pos = [325, 490, 250, 25]
        self.btnRename.setText('rename profile')
        self.btnRename.move(rename_pos[0], rename_pos[1])
        self.btnRename.resize(rename_pos[2], rename_pos[3])
        self.cbxProfiles.setEnabled(True)
        self.btnCreate.setEnabled(True)
        self.btnSave.setEnabled(True)
        self.btnDelete.setEnabled(True)
        self.btnCancelRename.setVisible(False)
        self.txtRename.setVisible(False)

    def check_name_validity(self, text):
        if text == '':
            self.btnRename.setEnabled(False)
        else:
            self.btnRename.setEnabled(True)





if __name__ == '__main__':
    if sys.platform.startswith('win'):
        # On Windows calling this function is necessary.
        multiprocessing.freeze_support()

    print('executing main...')
    print('sys.argv:', sys.argv)
    # launching the application and showing the gui
    app = QApplication(sys.argv)
    dd = DirectDriveWindow()
    dd.show()
    app.exec()

    dd.motor_controller.terminate()
    dd.game_interfacer.terminate()
    dd.virtual_controller.terminate()

    try:
        dd.motor_controller.motor_control_process.join()
        dd.game_interfacer.game_interface_process.join()
        dd.virtual_controller.send_input_process.join()
    except:
        print('some processes were already killed')