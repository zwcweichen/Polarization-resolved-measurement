# import module 
import clr
import os
import System
from System import Decimal
import pythoncom

# Thorlabs imports
clr.AddReference(r"C:\\Program Files\\Thorlabs\\Kinesis\\Thorlabs.MotionControl.DeviceManagerCLI.dll")
clr.AddReference(r"C:\\Program Files\\Thorlabs\\Kinesis\\Thorlabs.MotionControl.GenericMotorCLI.dll")
clr.AddReference(r"C:\\Program Files\\Thorlabs\\Kinesis\\Thorlabs.MotionControl.TCube.DCServoCLI.dll")
from Thorlabs.MotionControl.DeviceManagerCLI import *
from Thorlabs.MotionControl.GenericMotorCLI import *
from Thorlabs.MotionControl.TCube.DCServoCLI import *

import sys
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QGridLayout,
                           QHBoxLayout, QGroupBox, QLabel, QPushButton, 
                           QLineEdit, QMessageBox, QTabWidget, QToolBar,
                           QFileDialog, QCheckBox, QSizePolicy)
from PyQt6.QtGui import QIcon, QActionGroup, QAction, QFont
from PyQt6.QtCore import Qt, QTimer, QSize
from PyQt6.QtCore import*
import pyqtgraph as pg
import numpy as np
import time
import ctypes as ct
from ctypes import byref


# stylesheet
class StyleSheet:
    @staticmethod
    def get_style(dpi_scale):

        base_padding = int(4 * dpi_scale)
        base_margin = int(4 * dpi_scale)
        base_font_size = int(10 * dpi_scale)
        base_radius = int(10 * dpi_scale)
        
        return f"""
            QMainWindow {{
                background-color: #2b2b2b;
                color: #ffffff;
            }}
            QTabWidget::pane {{
                border: 1px solid #444444;
                background-color: #2b2b2b;
            }}
            QTabBar::tab {{
                background-color: #353535;
                color: #ffffff;
                padding: {base_padding}px {base_padding * 2}px;
                border: none;
                margin-right: {base_margin}px;
                font-size: {base_font_size}px;
            }}
            QTabBar::tab:selected {{
                background-color: #0078d4;
            }}
            QTabBar::tab:hover {{
                background-color: #404040;
            }}
            QPushButton {{
                background-color: #0078d4;
                color: white;
                border: none;
                padding: {base_padding}px {base_padding * 2}px;
                border-radius: {base_radius}px;
                font-size: {base_font_size}px;
                min-height: {base_padding * 3}px;
            }}
            QPushButton:hover {{
                background-color: #1084d8;
            }}
            QPushButton:pressed {{
                background-color: #006cbd;
            }}
            QPushButton:disabled {{
                background-color: #555555;
                color: #888888;
            }}
            QLabel {{
                color: #ffffff;
                font-size: {base_font_size}px;
            }}
            QGroupBox {{
                border: 2px solid #444444;
                border-radius: {base_radius}px;
                margin-top: {base_padding * 2}px;
                padding-top: {base_padding * 2}px;
                color: #ffffff;
                font-size: {base_font_size + 2}px;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: {base_padding}px;
                padding: 0 {base_padding}px;
            }}
            QLineEdit {{
                background-color: #353535;
                border: 1px solid #444444;
                color: #ffffff;
                padding: {base_padding}px;
                border-radius: {base_radius}px;
                min-height: {base_padding * 2.5}px;
                font-size: {base_font_size}px;
            }}
            QLineEdit:focus {{
                border: 1px solid #0078d4;
            }}
            QCheckBox {{
                color: #ffffff;
                font-size: {base_font_size}px;
            }}
            QCheckBox::indicator {{
                width: {base_padding * 2}px;
                height: {base_padding * 2}px;
                border-radius: {base_radius/2}px;
            }}
            QCheckBox::indicator:unchecked {{
                background-color: #353535;
                border: 1px solid #444444;
            }}
            QCheckBox::indicator:checked {{
                background-color: #0078d4;
                border: 1px solid #0078d4;
            }}
            QMessageBox {{
                background-color: #2b2b2b;
            }}
            QMessageBox QLabel {{
                color: #ffffff;
                font-size: {base_font_size}px;
            }}
            QStatusBar {{
                background-color: #2b2b2b;
                color: #ffffff;
            }}
            QStatusBar QLabel {{
                padding: {base_padding/2}px;
                margin: {base_margin}px;
            }}
        """
# instrument picoharp300
class PicoHarp300:
    def __init__(self):
        # load dll: picoharp libary
        if os.name == 'nt':
            self.phlib = ct.WinDLL("phlib64.dll")
            print("PicoHarp 64-bit DLL loaded")
        else:
            self.phlib = ct.CDLL("phlib.so")
            print("PicoHarp DLL loaded")
        
        self.dev_idx = 0
        self.serial = ct.create_string_buffer(b"", 8)
        self.error = ct.c_int(0)
        
    def initialize(self):
        # connect picoharp
        self.error = self.phlib.PH_OpenDevice(ct.c_int(self.dev_idx), self.serial)
        if self.error < 0:
            raise Exception(f"Cannot open PicoHarp device: {self.error}")
        # initialize picoharp
        self.error = self.phlib.PH_Initialize(ct.c_int(self.dev_idx), ct.c_int(0))
        if self.error < 0:
            raise Exception(f"Cannot initialize PicoHarp: {self.error}")
        
        print(f"PicoHarp connected successfully, serial: {self.serial.value.decode()}")
        return True

    def get_count_rate(self, channel):
        # load channel, get count rate
        count_rate = ct.c_int(0)
        self.error = self.phlib.PH_GetCountRate(
            ct.c_int(self.dev_idx),
            ct.c_int(channel),
            byref(count_rate)
        )
        if self.error < 0:
            raise Exception(f"Cannot read count rate, error: {self.error}")
        return count_rate.value

    def close(self):
        self.error = self.phlib.PH_CloseDevice(ct.c_int(self.dev_idx))
        if self.error < 0:
            raise Exception(f"Error closing PicoHarp: {self.error}")
# instrument thorlabs kinesismotor
class KinesisMotor:
    def __init__(self, SerialNum=None, polling=100, verbose=False):
        self.verbose = verbose
        self.Connected = False
        self.device = None
        self.polling = polling
        self.lims = None
        
        # check serial number
        if SerialNum is not None:
            if isinstance(SerialNum, int):
                SerialNum = str(SerialNum)
            if self.verbose: 
                print(f"Motor Serial Number: {SerialNum}")
            self.SerialNum = str(SerialNum)
            self.initialize()
        else:
            raise Exception('Please provide a motor serial number')

    def initialize(self):
        try:
            device_list_result = DeviceManagerCLI.BuildDeviceList()
            # read serial number load device
            self.device = TCubeDCServo.CreateTCubeDCServo(self.SerialNum)
            # connect motor
            self.device.Connect(self.SerialNum)
            # wait time for initialized
            self.device.WaitForSettingsInitialized(5000)
            self.motorSettings = self.device.LoadMotorConfiguration(
                self.SerialNum,
                DeviceConfiguration.DeviceSettingsUseOptionType.UseFileSettings
            )

            self.device.StartPolling(self.polling)
            self.device.EnableDevice()
            time.sleep(0.5)  # 等待設備啟動
            
            self.getVelParams()
            self.getLimits()

            if self.verbose:
                deviceInfo = self.device.GetDeviceInfo()
                print(f'Motor Connected: {deviceInfo.Name}, SN: {deviceInfo.SerialNumber}')
                
            self.Connected = True
            return True

        except Exception as e:
            self.Connected = False
            raise Exception(f"Motor initialization error: {str(e)}")

    def disconnect(self):
        if self.device:
            try:
                self.device.StopPolling()
                self.device.ShutDown()
                time.sleep(0.5)
                self.Connected = False
                print("Motor disconnected successfully")
            except Exception as e:
                print(f"Error during motor disconnection: {str(e)}")

    def getVelParams(self):
        try:
            self.velPars = self.device.GetVelocityParams()
            self.accel = Decimal.ToDouble(self.velPars.get_Acceleration())
            self.min_vel = Decimal.ToDouble(self.velPars.get_MinVelocity())
            self.max_vel = Decimal.ToDouble(self.velPars.get_MaxVelocity())

            if self.verbose:
                print(f'Acceleration: {self.accel} mm/s^2')
                print(f'Min Velocity: {self.min_vel} mm/s')
                print(f'Max Velocity: {self.max_vel} mm/s')

        except Exception as e:
            raise Exception(f"Error getting velocity parameters: {str(e)}")

    def getLimits(self):
        try:
            self.lims = self.device.AdvancedMotorLimits
            self.max_accel_lim = Decimal.ToDouble(self.lims.get_AccelerationMaximum())
            self.max_vel_lim = Decimal.ToDouble(self.lims.get_VelocityMaximum())
            self.max_travel_lim = Decimal.ToDouble(self.lims.get_LengthMaximum())
            self.min_travel_lim = Decimal.ToDouble(self.lims.get_LengthMinimum())
            
            if self.verbose:
                print(f'Acceleration limit: {self.max_accel_lim} mm/s^2')
                print(f'Max velocity limit: {self.max_vel_lim} mm/s')
                print(f'Travel limits: {self.min_travel_lim} to {self.max_travel_lim} mm')
            
        except Exception as e:
            raise Exception(f"Error getting motor limits: {str(e)}")

    def moveTo(self, position, time_out=60000):
        try:
            if not self.Connected:
                raise Exception("Motor not connected")
                
            if position < 0 or position > 180:  
                raise ValueError("Position must be between 0 and 180 degrees")
                
            if self.verbose:
                print(f"Moving to position: {position}°")
                
            self.device.MoveTo(Decimal(position), time_out)
            
        except Exception as e:
            raise Exception(f"Error moving to position: {str(e)}")

    def getPos(self):
        try:
            if not self.Connected:
                raise Exception("Motor not connected")
            return Decimal.ToDouble(self.device.Position)
        except Exception as e:
            raise Exception(f"Error getting position: {str(e)}")       

# simulation device for testing design
class SimulatedDevices:
    class PicoHarp:
        def __init__(self):
            self.connected = False
            self.count_base = 1000  
            self.noise_level = 100  
            self.angle_dependency = True  
            
        def initialize(self):
            self.connected = True
            print("Simulated PicoHarp initialized")
            return True
            
        def get_count_rate(self, channel=1, current_angle=None):
            if not self.connected:
                raise Exception("Device not connected")
            count = self.count_base + np.random.normal(0, self.noise_level)
            if self.angle_dependency and current_angle is not None:
                angle_rad = np.radians(current_angle)
                count *= np.cos(angle_rad) ** 2
                
            return max(0, count) 
            
        def close(self):
            self.connected = False
            print("Simulated PicoHarp disconnected")
            
    class KinesisMotor:
        def __init__(self, serial=None):
            self.connected = False
            self.serial = serial
            self.position = 0.0  
            self.moving = False
            self.speed = 10.0  
            
        def initialize(self):
            self.connected = True
            print(f"Simulated Motor initialized with serial: {self.serial}")
            return True
            
        def disconnect(self):
            self.connected = False
            print("Simulated Motor disconnected")
            
        def moveTo(self, position, time_out=60000):
            if not self.connected:
                raise Exception("Device not connected")
                
            distance = abs(position - self.position)
            move_time = distance / self.speed
            time.sleep(min(move_time, 0.5))  
            
            self.position = position
            print(f"Simulated move to position: {position:.2f}°")
            
        def getPos(self):
            if not self.connected:
                raise Exception("Device not connected")
            return self.position

class MeasurementGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        try:
            pythoncom.CoInitialize()
        except:
            pass
        
        # get the screen information ----------
        screen = QApplication.primaryScreen()
        self.dpi = screen.logicalDotsPerInch()
        self.dpi_scale = self.dpi / 96.0  
        self.screen_width = screen.size().width() if screen else 1920
        self.screen_height = screen.size().height() if screen else 1080
        self.window_width = int(self.screen_width * 0.75)
        self.window_height = int(self.screen_height * 0.75)
        # ----------
        
        # set window size ----------
        self.setGeometry(
            (self.screen_width - self.window_width) // 2,
            (self.screen_height - self.window_height) // 2,
            self.window_width,
            self.window_height
        )
        min_width = int(800 * self.dpi_scale)
        min_height = int(600 * self.dpi_scale)
        self.setMinimumSize(min_width, min_height)
        # ----------
        
        # set stylesheet ----------
        self.setStyleSheet(StyleSheet.get_style(self.dpi_scale))
        # ----------
        
        # set font ----------
        self.base_font_size = max(9, int(self.dpi_scale * 4))
        self.title_font_size = int(self.base_font_size * 1.2)    
        self.base_font = QFont()
        self.base_font.setPointSize(self.base_font_size)
        self.title_font = QFont()
        self.title_font.setPointSize(self.title_font_size)
        self.title_font.setBold(True)
        # ----------

        # set space ----------
        self.base_spacing = int(1 * self.dpi_scale)
        self.large_spacing = int(2 * self.dpi_scale)
        # ----------

        # initialize device ----------
        self.simulation_mode = True
        self.picoharp = None
        self.motor = None
        self.is_measuring = False
        self.is_monitoring = False
        # ---------        
        

        self.initUI()        
        
    def initUI(self):
        self.setWindowTitle('SHG Polarization Measurement')
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setSpacing(self.base_spacing)
        main_layout.setContentsMargins(
            self.base_spacing,
            self.base_spacing,
            self.base_spacing,
            self.base_spacing
        )
        
        self.createModeToolbar()
        
        # create tabs ----------
        self.tabs = QTabWidget()
        self.tabs.setFont(self.base_font)
        main_layout.addWidget(self.tabs)
        self.connection_tab = self.createConnectionTab()
        self.monitor_tab = self.createMonitorTab()
        self.measurement_tab = self.createMeasurementTab()
        
        self.tabs.addTab(self.connection_tab, "Device Connections")
        self.tabs.addTab(self.monitor_tab, "Count Monitor")
        self.tabs.addTab(self.measurement_tab, "SHG Measurement")
        # ----------
        
        # status bar ----------
        self.statusBar().setFont(self.base_font)
        self.statusBar().showMessage('Ready')
        # ----------
    
    # Working mode ----------     
    '''
    . select working mode
    '''
    
    def createModeToolbar(self):
        toolbar = QToolBar("Mode Selection")
        toolbar.setMovable(False)
        toolbar.setFont(self.base_font)
        icon_size = int(24 * self.dpi_scale)
        toolbar.setIconSize(QSize(icon_size, icon_size))
        
        mode_group = QActionGroup(self)
        self.sim_mode_action = QAction('Simulation Mode', self)
        self.sim_mode_action.setCheckable(True)
        self.sim_mode_action.setChecked(True)
        self.real_mode_action = QAction('Real Hardware Mode', self)
        self.real_mode_action.setCheckable(True)
        
        mode_group.addAction(self.sim_mode_action)
        mode_group.addAction(self.real_mode_action)
        toolbar.addAction(self.sim_mode_action)
        toolbar.addAction(self.real_mode_action)
        toolbar.addSeparator()
        
        self.apply_mode_btn = QPushButton("Apply Mode Change")
        self.apply_mode_btn.setFont(self.base_font)
        self.apply_mode_btn.setEnabled(False)
        btn_height = int(32 * self.dpi_scale)
        self.apply_mode_btn.setMinimumHeight(btn_height)
        toolbar.addWidget(self.apply_mode_btn)
        
        # change mode ----------
        self.sim_mode_action.triggered.connect(lambda: self.prepareModeChange(True))
        self.real_mode_action.triggered.connect(lambda: self.prepareModeChange(False))
        self.apply_mode_btn.clicked.connect(self.applyModeChange)
        # ----------
        
        self.addToolBar(Qt.ToolBarArea.TopToolBarArea, toolbar)
     
    def applyModeChange(self):
        if not hasattr(self, 'pending_mode'):
            return
            
        if self.is_measuring:
            self.showCustomMessageBox(
                QMessageBox.Icon.Warning,
                "Warning",
                "Please stop current measurement before changing mode."
            )
            return
            
        reply = self.showCustomMessageBox(
            QMessageBox.Icon.Question,
            'Confirm Mode Change',
            f"Switch to {'simulation' if self.pending_mode else 'real hardware'} mode?\n"
            "This will disconnect all devices.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
            
        if reply == QMessageBox.StandardButton.Yes:
            self.disconnectDevices()
            self.simulation_mode = self.pending_mode
            self.updateModeStatus()
            self.resetConnectionStatus()
            self.apply_mode_btn.setEnabled(False)
            self.apply_mode_btn.setStyleSheet("")
            self.updateDeviceInfo()        
        
    def updateDeviceInfo(self):
        info_text = "Connected Devices:\n"
        
        if self.simulation_mode:
            info_text += "Running in Simulation Mode\n"
            
        if self.picoharp:
            info_text += "\nPicoHarp 300:"
            info_text += "\n - Status: Connected"
            if not self.simulation_mode:
                info_text += f"\n - Serial: {self.picoharp.serial.value.decode()}"
                
        if self.motor:
            info_text += "\n\nRotation Stage:"
            info_text += "\n - Status: Connected"
            if not self.simulation_mode:
                info_text += f"\n - Serial: {self.motor.SerialNum}"
                
        if not self.picoharp and not self.motor:
            info_text = "No devices connected"
            
        self.device_info_label.setText(info_text)        
    # ----------
    
   
    # connection tab ----------
    '''
    . connect instrument: motor, picoharp
    . disconnect instrument
    '''
    
    def createConnectionTab(self):
        tab = QWidget()
        layout = QVBoxLayout()
        layout.setSpacing(self.large_spacing)
        layout.setContentsMargins(
            self.large_spacing,
            self.large_spacing,
            self.large_spacing,
            self.large_spacing
        )
        
        # picoharp group
        picoharp_group = QGroupBox("PicoHarp 300 Connection")
        picoharp_group.setFont(self.title_font)
        picoharp_layout = QVBoxLayout()
        picoharp_layout.setSpacing(self.base_spacing)
        self.picoharp_status = QLabel("Status: Disconnected")
        self.picoharp_connect_btn = QPushButton("Connect PicoHarp")
        self.picoharp_disconnect_btn = QPushButton("Disconnect PicoHarp")
        self.picoharp_disconnect_btn.setEnabled(False)
        for widget in [self.picoharp_status, self.picoharp_connect_btn, 
                    self.picoharp_disconnect_btn]:
            widget.setFont(self.base_font)
            if isinstance(widget, QPushButton):
                widget.setMinimumHeight(int(32 * self.dpi_scale))
        picoharp_layout.addWidget(self.picoharp_status)
        picoharp_layout.addWidget(self.picoharp_connect_btn)
        picoharp_layout.addWidget(self.picoharp_disconnect_btn)
        picoharp_group.setLayout(picoharp_layout)
        
        # motor group
        motor_group = QGroupBox("Half-wave Plate Rotator Connection")
        motor_group.setFont(self.title_font)
        motor_layout = QVBoxLayout()
        motor_layout.setSpacing(self.base_spacing)
        
        self.motor_status = QLabel("Status: Disconnected")
        self.motor_serial = QLineEdit()
        self.motor_serial.setPlaceholderText("Enter motor serial number")
        self.motor_connect_btn = QPushButton("Connect Rotator")
        self.motor_disconnect_btn = QPushButton("Disconnect Rotator")
        self.motor_disconnect_btn.setEnabled(False)
        

        serial_info = QLabel("Serial number can be found on the device or in Kinesis software")
        serial_info.setStyleSheet(f"color: #888888; font-size: {self.base_font_size-1}px;")
        for widget in [self.motor_status, self.motor_serial, self.motor_connect_btn, 
                    self.motor_disconnect_btn, serial_info]:
            widget.setFont(self.base_font)
            if isinstance(widget, (QPushButton, QLineEdit)):
                widget.setMinimumHeight(int(32 * self.dpi_scale))
        motor_layout.addWidget(QLabel("Serial Number:"))
        motor_layout.addWidget(self.motor_serial)
        motor_layout.addWidget(serial_info)
        motor_layout.addWidget(self.motor_status)
        motor_layout.addWidget(self.motor_connect_btn)
        motor_layout.addWidget(self.motor_disconnect_btn)
        motor_group.setLayout(motor_layout)
        
        # device information group
        info_group = QGroupBox("Device Information")
        info_group.setFont(self.title_font)
        info_layout = QVBoxLayout()
        info_layout.setSpacing(self.base_spacing)
        
        self.device_info_label = QLabel("No devices connected")
        self.device_info_label.setFont(self.base_font)
        self.device_info_label.setWordWrap(True)
        info_layout.addWidget(self.device_info_label)
        info_group.setLayout(info_layout)
        layout.addWidget(picoharp_group)
        layout.addWidget(motor_group)
        layout.addWidget(info_group)
        layout.addStretch()
        # UI connect function
        self.picoharp_connect_btn.clicked.connect(self.connectPicoHarp)
        self.picoharp_disconnect_btn.clicked.connect(self.disconnectPicoHarp)
        self.motor_connect_btn.clicked.connect(self.connectMotor)
        self.motor_disconnect_btn.clicked.connect(self.disconnectMotor)
        
        tab.setLayout(layout)
        return tab     

    def connectMotor(self):
        try:
            serial = self.motor_serial.text()
            if not serial and not self.simulation_mode:
                QMessageBox.warning(self, "Warning", "Please enter motor serial number")
                return    
            if self.simulation_mode:
                self.motor = SimulatedDevices.KinesisMotor(serial)
                success = self.motor.initialize()
            else:
                self.motor = KinesisMotor(serial)
                success = True    
            if success:
                self.motor_status.setText("Status: Connected")
                self.motor_connect_btn.setEnabled(False)
                self.motor_disconnect_btn.setEnabled(True)
                self.updateDeviceInfo()
                # after connect motor, move to initial position
                self.homeMotor()
                QMessageBox.information(
                    self, 
                    "Success", 
                    f"Motor connected successfully in {'simulation' if self.simulation_mode else 'hardware'} mode!"
                )
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to connect motor: {str(e)}")

    def disconnectMotor(self):
        if self.motor:
            try:
                self.motor.disconnect()
                self.motor = None
                self.motor_status.setText("Status: Disconnected")
                self.motor_connect_btn.setEnabled(True)
                self.motor_disconnect_btn.setEnabled(False)
                self.updateDeviceInfo()
                
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Error disconnecting motor: {str(e)}")   

    def connectPicoHarp(self):
        try:
            if self.simulation_mode:
                self.picoharp = SimulatedDevices.PicoHarp()
            else:
                self.picoharp = PicoHarp300()
                
            success = self.picoharp.initialize()                
            if success:
                self.picoharp_status.setText("Status: Connected")
                self.picoharp_connect_btn.setEnabled(False)
                self.picoharp_disconnect_btn.setEnabled(True)
                self.updateDeviceInfo()
                QMessageBox.information(
                    self, 
                    "Success", 
                    f"PicoHarp connected successfully in {'simulation' if self.simulation_mode else 'hardware'} mode!"
                )
                
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to connect PicoHarp: {str(e)}")

    def disconnectPicoHarp(self):
        if self.picoharp:
            try:
                self.picoharp.close()
                self.picoharp = None
                self.picoharp_status.setText("Status: Disconnected")
                self.picoharp_connect_btn.setEnabled(True)
                self.picoharp_disconnect_btn.setEnabled(False)
                self.updateDeviceInfo()         
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Error disconnecting PicoHarp: {str(e)}")  
    # ----------


    # monitor count rate tab ----------
    '''
    . monitor count rate to optimize the optical path
    '''
    def createMonitorTab(self):
        tab = QWidget()
        layout = QVBoxLayout()
        layout.setSpacing(self.large_spacing)
        layout.setContentsMargins(
            self.large_spacing,
            self.large_spacing,
            self.large_spacing,
            self.large_spacing
        )
        
        # monitor group
        monitor_group = QGroupBox("Count Rate Monitor")
        monitor_group.setFont(self.title_font)
        monitor_layout = QVBoxLayout()
        monitor_layout.setSpacing(self.large_spacing)
        
        # monitor window
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('default')
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.setLabel('left', 'Counts/sec')
        self.plot_widget.setLabel('bottom', 'Time (s)')
        self.plot_widget.setMinimumHeight(int(200 * self.dpi_scale))
        self.curve = self.plot_widget.plot(pen='y')
        label_style = {'color': '#ffffff', 'font-size': f'{self.base_font_size}pt'}
        self.plot_widget.setLabel('left', 'Counts/sec', **label_style)
        self.plot_widget.setLabel('bottom', 'Time (s)', **label_style)
        self.count_display = QLabel("Current Count: 0 cps")
        self.count_display.setFont(self.title_font)
        self.count_display.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        # monitor button
        control_layout = QHBoxLayout()
        control_layout.setSpacing(self.base_spacing)
        self.start_monitor_btn = QPushButton("Start Monitor")
        self.stop_monitor_btn = QPushButton("Stop Monitor")
        self.stop_monitor_btn.setEnabled(False)
        
        # 設置按鈕字體和大小
        for btn in [self.start_monitor_btn, self.stop_monitor_btn]:
            btn.setMinimumHeight(int(32 * self.dpi_scale))
            control_layout.addWidget(btn)   
        apd_warning = QLabel("Please ensure APD is powered on before monitoring")
        apd_warning.setFont(self.base_font)
        apd_warning.setStyleSheet(f"color: #FFA500; font-style: italic;")
        apd_warning.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        
        monitor_layout.addWidget(self.plot_widget)
        monitor_layout.addWidget(self.count_display)
        monitor_layout.addLayout(control_layout)
        monitor_layout.addWidget(apd_warning)
        
        monitor_group.setLayout(monitor_layout)
        layout.addWidget(monitor_group)
        
        # UI connect funtion   
        self.start_monitor_btn.clicked.connect(self.startMonitoring)
        self.stop_monitor_btn.clicked.connect(self.stopMonitoring)
        
        tab.setLayout(layout)
        return tab

    def startMonitoring(self):
        if not self.picoharp:
            QMessageBox.warning(self, "Warning", "Please connect PicoHarp first")
            return
            
        try:
            # initial collect count rate
            self.monitoring_data = {'time': [], 'counts': []}
            self.start_monitor_time = time.time()
            self.is_monitoring = True
            # update UI status
            self.start_monitor_btn.setEnabled(False)
            self.stop_monitor_btn.setEnabled(True)
            # update monitor
            self.monitor_timer = QTimer()
            self.monitor_timer.timeout.connect(self.updateMonitor)
            # data update rate 100ms
            self.monitor_timer.start(100)  
            self.statusBar().showMessage("Monitoring started")
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to start monitoring: {str(e)}")
            self.stopMonitoring()

    def updateMonitor(self):
        try:
            current_time = time.time() - self.start_monitor_time
            count = self.picoharp.get_count_rate(channel=1)
            # update data
            self.monitoring_data['time'].append(current_time)
            self.monitoring_data['counts'].append(count)
            
            if len(self.monitoring_data['time']) > 100:
                self.monitoring_data['time'] = self.monitoring_data['time'][-100:]
                self.monitoring_data['counts'] = self.monitoring_data['counts'][-100:]
            # update window
            self.curve.setData(
                self.monitoring_data['time'],
                self.monitoring_data['counts']
            )
            self.count_display.setText(f"Current Count: {count:.0f} cps")
            if count == 0:
                self.statusBar().showMessage("Warning: Zero count detected! Check APD.", 3000)
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Monitoring error: {str(e)}")
            self.stopMonitoring()

    def stopMonitoring(self):
        if hasattr(self, 'monitor_timer'):
            self.monitor_timer.stop()
        self.is_monitoring = False
        self.start_monitor_btn.setEnabled(True)
        self.stop_monitor_btn.setEnabled(False)
        self.statusBar().showMessage("Monitoring stopped")
    # ----------
    
    
    # Polarization measurement ----------
    '''
    . move to specific angle and home
    . continuous angle variation
    . save data
    '''
    def createMeasurementTab(self):
        tab = QWidget()
        layout = QHBoxLayout()
        layout.setSpacing(self.large_spacing)
        layout.setContentsMargins(
            self.large_spacing,
            self.large_spacing,
            self.large_spacing,
            self.large_spacing
        )
        
        # motor control group
        control_panel = QGroupBox("Measurement Controls")
        control_panel.setFont(self.title_font)
        control_layout = QVBoxLayout()
        control_layout.setSpacing(self.large_spacing)
        hwp_group = QGroupBox("Rotator Control")
        hwp_group.setFont(self.title_font)
        hwp_layout = QGridLayout()
        hwp_layout.setVerticalSpacing(self.base_spacing)
        hwp_layout.setHorizontalSpacing(self.base_spacing)
        angle_label = QLabel("Rotator Angle (°):")
        self.angle_input = QLineEdit()
        self.angle_input.setPlaceholderText("")
        angle_label.setFont(self.base_font)
        self.angle_input.setFont(self.base_font)
        self.angle_input.setMinimumHeight(int(32 * self.dpi_scale))
        self.move_to_btn = QPushButton("Move To")
        self.home_btn = QPushButton("Home")
        
        for btn in [self.move_to_btn, self.home_btn]:
            btn.setFont(self.base_font)
            btn.setMinimumHeight(int(32 * self.dpi_scale))
        self.current_rotator_label = QLabel("Current motor angle: -")
        
        for label in [self.current_rotator_label]:
            label.setFont(self.base_font)
        hwp_layout.addWidget(angle_label, 0, 0)
        hwp_layout.addWidget(self.angle_input, 0, 1)
        hwp_layout.addWidget(self.move_to_btn, 1, 0)
        hwp_layout.addWidget(self.home_btn, 1, 1)
        hwp_layout.addWidget(self.current_rotator_label, 2, 0, 1, 2)
        
        hwp_group.setLayout(hwp_layout)

        # continuous measurement setup group
        params_group = QGroupBox(" Continuous HWP Measurement Parameters")
        params_group.setFont(self.title_font)
        params_layout = QGridLayout()
        params_layout.setVerticalSpacing(self.base_spacing)
        params_layout.setHorizontalSpacing(self.base_spacing)
        self.start_angle = QLineEdit("0")
        self.end_angle = QLineEdit("180")
        self.step_size = QLineEdit("5")
        self.integration_time = QLineEdit("1.0")
    
        params = [
            ("Start motor Angle (°):", self.start_angle),
            ("End motor Angle (°):", self.end_angle),
            ("Step Size (°):", self.step_size),
            ("Integration Time (s):", self.integration_time)
        ]
        
        for i, (label_text, input_widget) in enumerate(params):
            label = QLabel(label_text)
            label.setFont(self.base_font)
            input_widget.setFont(self.base_font)
            input_widget.setMinimumHeight(int(32 * self.dpi_scale))
            params_layout.addWidget(label, i, 0)
            params_layout.addWidget(input_widget, i, 1)

        note = QLabel("Note: HWP rotates 0-180°, corresponding to 0-360° polarization")
        note.setFont(self.base_font)
        note.setStyleSheet(f"color: #888888;")
        note.setWordWrap(True)
        params_layout.addWidget(note, len(params), 0, 1, 2)
        
        params_group.setLayout(params_layout)
        
        # continuous measurement control group
        control_group = QGroupBox("Continuous HWP Measurement Control")
        control_group.setFont(self.title_font)
        measurement_buttons_layout = QVBoxLayout()
        measurement_buttons_layout.setSpacing(self.base_spacing)
        
        self.start_measurement_btn = QPushButton("Start Measurement")
        self.pause_measurement_btn = QPushButton("Pause")
        self.stop_measurement_btn = QPushButton("Stop")
        
        self.pause_measurement_btn.setEnabled(False)
        self.stop_measurement_btn.setEnabled(False)
        for btn in [self.start_measurement_btn, self.pause_measurement_btn, self.stop_measurement_btn]:
            btn.setFont(self.base_font)
            btn.setMinimumHeight(int(40 * self.dpi_scale))
            measurement_buttons_layout.addWidget(btn)
        self.progress_label = QLabel("Progress: 0/0")
        self.progress_label.setFont(self.base_font)
        self.progress_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        measurement_buttons_layout.addWidget(self.progress_label)
        
        control_group.setLayout(measurement_buttons_layout)
        
        # save data group
        save_group = QGroupBox("Data Saving")
        save_group.setFont(self.title_font)
        save_layout = QVBoxLayout()
        save_layout.setSpacing(self.base_spacing) 
        self.auto_save_checkbox = QCheckBox("Auto Save")
        self.save_path_input = QLineEdit()
        self.save_path_input.setPlaceholderText("Enter save path")
        self.browse_btn = QPushButton("Browse")
        self.auto_save_checkbox.setFont(self.base_font)
        self.save_path_input.setFont(self.base_font)
        self.browse_btn.setFont(self.base_font)
        self.save_path_input.setMinimumHeight(int(32 * self.dpi_scale))
        self.browse_btn.setMinimumHeight(int(32 * self.dpi_scale))
        save_layout.addWidget(self.auto_save_checkbox)
        save_layout.addWidget(self.save_path_input)
        save_layout.addWidget(self.browse_btn)        
        save_group.setLayout(save_layout)

        for group in [hwp_group, params_group, control_group, save_group]:
            control_layout.addWidget(group)
        
        control_layout.addStretch()
        control_panel.setLayout(control_layout)
        
        # polarization group
        display_panel = QGroupBox("Measurement Data")
        display_panel.setFont(self.title_font)
        display_layout = QVBoxLayout()
        display_layout.setSpacing(self.large_spacing)
        self.polar_plot = pg.PlotWidget()
        self.polar_plot.setBackground('default')
        self.polar_plot.setAspectLocked()
        min_plot_size = int(400 * self.dpi_scale)
        self.polar_plot.setMinimumSize(min_plot_size, min_plot_size)
        self.setupPolarPlot()
        
        # update measurement data
        counts_layout = QHBoxLayout()
        self.current_count_label = QLabel("Current Count: -")
        self.max_count_label = QLabel("Max Count: -")
        
        for label in [self.current_count_label, self.max_count_label]:
            label.setFont(self.base_font)
            counts_layout.addWidget(label)
        

        plot_note = QLabel("Plot shows polarization angle (0-360°)")
        plot_note.setFont(self.base_font)
        plot_note.setStyleSheet("color: #888888;")
        plot_note.setAlignment(Qt.AlignmentFlag.AlignCenter)
        

        display_layout.addWidget(self.polar_plot)
        display_layout.addLayout(counts_layout)
        display_layout.addWidget(plot_note)
        display_panel.setLayout(display_layout)
        layout.addWidget(control_panel, stretch=2)
        layout.addWidget(display_panel, stretch=3)
        
        # UI connect function
        self.move_to_btn.clicked.connect(self.moveToAngle)
        self.home_btn.clicked.connect(self.homeMotor)
        self.start_measurement_btn.clicked.connect(self.startMeasurement)
        self.pause_measurement_btn.clicked.connect(self.pauseMeasurement)
        self.stop_measurement_btn.clicked.connect(self.stopMeasurement)
        self.browse_btn.clicked.connect(self.browseSavePath)
        
        tab.setLayout(layout)
        return tab
                    
    def moveToAngle(self):
        try:
            angle = float(self.angle_input.text())
            
            if self.motor:
                self.motor.moveTo(angle)
                self.current_rotator_label.setText(f"Current motor angel: {angle:.1f}°")
                self.statusBar().showMessage(f"Moved to motor angle: {angle:.1f}°")
            else:
                raise Exception("Motor not connected")
                
        except ValueError as e:
            QMessageBox.warning(self, "Invalid Input", str(e))
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to move motor: {str(e)}")

    def homeMotor(self):
        try:
            if self.motor:
                self.motor.moveTo(0)
                self.current_rotator_label.setText("Current motor: 0.0°")
                self.statusBar().showMessage("Motor homed")
            else:
                raise Exception("Motor not connected")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to home motor: {str(e)}")

    def startMeasurement(self):
        try:
            if not all([self.motor, self.picoharp]):
                raise Exception("Please connect both devices first")
            
            start = float(self.start_angle.text())
            end = float(self.end_angle.text())
            step = float(self.step_size.text())
            self.integration_time_val = float(self.integration_time.text())
            
            # only for continuous hwp measurement
            if not (0 <= start <= 180 and 0 <= end <= 180):
                raise ValueError("Half-wave plate angles must be between 0 and 180")
            if step <= 0:
                raise ValueError("Step size must be positive")
            if self.integration_time_val <= 0:
                raise ValueError("Integration time must be positive")
                
            
            self.rotator_angles = np.arange(start, end + step, step)
            self.polarization_angles = 2 * self.rotator_angles
            
            # initialize data
            self.measurement_data = []
            self.current_angle_index = 0
            self.is_measuring = True
            self.is_paused = False
            
            # update UI
            self.updateMeasurementUIState(True)
            total_points = len(self.rotator_angles)
            self.progress_label.setText(f"Progress: 0/{total_points}")
            

            self.measureNextPoint()
            
        except ValueError as e:
            QMessageBox.warning(self, "Invalid Parameters", str(e))
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to start measurement: {str(e)}")

    def measureNextPoint(self):
        if not self.is_measuring or self.is_paused:
            return
            
        if self.current_angle_index >= len(self.rotator_angles):
            self.finishMeasurement()
            return
            
        try:
            rotator_angle = self.rotator_angles[self.current_angle_index]
            self.motor.moveTo(rotator_angle)
            
            pol_angle = self.polarization_angles[self.current_angle_index]
            self.current_rotator_label.setText(f"Current rotator: {rotator_angle:.1f}°")

            total_points = len(self.rotator_angles)
            self.progress_label.setText(
                f"Progress: {self.current_angle_index + 1}/{total_points}"
            )
            

            QTimer.singleShot(
                int(self.integration_time_val * 1000), 
                self.collectMeasurementData
            )
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Measurement error: {str(e)}")
            self.stopMeasurement()

    def pauseMeasurement(self):
        self.is_paused = not self.is_paused
        self.pause_measurement_btn.setText("Resume" if self.is_paused else "Pause")
        if not self.is_paused:
            self.measureNextPoint()
        status = "paused" if self.is_paused else "resumed"
        self.statusBar().showMessage(f"Measurement {status}")

    def stopMeasurement(self):
        self.is_measuring = False
        self.finishMeasurement()
        self.statusBar().showMessage("Measurement stopped")

    def finishMeasurement(self):
        self.is_measuring = False
        self.updateMeasurementUIState(False)
        if self.measurement_data and not self.auto_save_checkbox.isChecked():
            reply = QMessageBox.question(
                self, 
                "Measurement Complete", 
                "Would you like to save the measurement data?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.Yes
            )
            
            if reply == QMessageBox.StandardButton.Yes:
                self.saveData()
        
    def setupPolarPlot(self):
        self.polar_plot.clear()
        self.polar_plot.getAxis('bottom').setStyle(tickTextOffset=int(self.dpi_scale * 10))
        self.polar_plot.getAxis('left').setStyle(tickTextOffset=int(self.dpi_scale * 10))
        self.polar_plot.setXRange(-1000, 1000)
        self.polar_plot.setYRange(-1000, 1000)
        self.updatePolarGrids(1000)

    def updatePolarGrids(self, max_count):
        self.polar_plot.clear()
        step = max_count / 4
        step = float(f"{step:.1e}")

        text_size = max(8, int(self.base_font_size * 0.8))
        text_font = QFont()
        text_font.setPointSize(text_size)
    
        for r in np.arange(step, max_count + step, step):
            theta = np.linspace(0, 2*np.pi, 100)
            x = r * np.cos(theta)
            y = r * np.sin(theta)
            self.polar_plot.plot(x, y, pen=(100,100,100))
            
            text = pg.TextItem(
                text=f'{r:.0f}',
                color=(200,200,200),
                anchor=(0.5, 0.5)
            )
            text.setFont(text_font)
            text.setPos(r * 1.02, 0)
            self.polar_plot.addItem(text)
        
        for theta in np.linspace(0, 2*np.pi, 12, endpoint=False):
            x = [0, max_count * np.cos(theta)]
            y = [0, max_count * np.sin(theta)]
            self.polar_plot.plot(x, y, pen=(100,100,100))
            
            angle_deg = np.degrees(theta)
            label_radius = max_count * 1.15
            text = pg.TextItem(
                text=f'{angle_deg:.0f}°',
                color=(200,200,200),
                anchor=(0.5, 0.5)
            )
            text.setFont(text_font)
            text.setPos(
                label_radius * np.cos(theta),
                label_radius * np.sin(theta)
            )
            self.polar_plot.addItem(text)

    def updatePolarPlot(self):
        if not hasattr(self, 'measurement_data') or not self.measurement_data:
            return
        angles = np.array([d[0] for d in self.measurement_data])
        counts = np.array([d[1] for d in self.measurement_data])
        max_count = max(counts.max() if len(counts) > 0 else 1000, 1000)
        # update polar grid
        self.updatePolarGrids(max_count * 1.2)
        
        plot_range = max_count * 1.3
        self.polar_plot.setXRange(-plot_range, plot_range)
        self.polar_plot.setYRange(-plot_range, plot_range)
        
        if len(angles) > 0:
            theta = np.radians(angles)
            x = counts * np.cos(theta)
            y = counts * np.sin(theta)
            
            point_size = max(6, int(self.dpi_scale * 6))
            line_width = max(1, int(self.dpi_scale * 1.5))
            
            self.polar_plot.plot(
                x, y,
                pen=pg.mkPen('r', width=line_width),
                symbol='o',
                symbolBrush='r',
                symbolPen='w',
                symbolSize=point_size
            )
            self.max_count_label.setText(f"Max Count: {max_count:.0f} cps")

    def resizeEvent(self, event):
        super().resizeEvent(event)
        width = event.size().width()
        height = event.size().height()
        self.updateFontSizes(width, height)
        if hasattr(self, 'polar_plot'):
            self.setupPolarPlot()
            self.updatePolarPlot()

    def updateFontSizes(self, width, height):
        new_base_size = max(9, int(min(width, height) / 100))
        
        if new_base_size != self.base_font_size:
            self.base_font_size = new_base_size
            self.title_font_size = int(new_base_size * 1.2)
            self.base_font.setPointSize(self.base_font_size)
            self.title_font.setPointSize(self.title_font_size)
            self.updateAllFonts(self)

    def updateAllFonts(self, widget):
        if isinstance(widget, QLabel):
            widget.setFont(self.base_font)
        elif isinstance(widget, QGroupBox):
            widget.setFont(self.title_font)
        elif isinstance(widget, (QPushButton, QLineEdit, QCheckBox)):
            widget.setFont(self.base_font)
            widget.setMinimumHeight(int(32 * self.dpi_scale))
        for child in widget.findChildren(QWidget):
            self.updateAllFonts(child)

    def browseSavePath(self):
        file_path, selected_filter = QFileDialog.getSaveFileName(
            self,
            "Save Measurement Data",
            "",
            "Text Files (*.txt);;CSV Files (*.csv);;All Files (*)"
        )
        
        if not file_path:
            return
            
        if selected_filter == "Text Files (*.txt)" and not file_path.endswith('.txt'):
            file_path += '.txt'
        elif selected_filter == "CSV Files (*.csv)" and not file_path.endswith('.csv'):
            file_path += '.csv'
            
        self.save_path_input.setText(file_path)

    def updateMeasurementUIState(self, measuring):
        self.start_measurement_btn.setEnabled(not measuring)
        self.pause_measurement_btn.setEnabled(measuring)
        self.stop_measurement_btn.setEnabled(measuring)
        self.move_to_btn.setEnabled(not measuring)
        self.home_btn.setEnabled(not measuring)

        for widget in [self.start_angle, self.end_angle, 
                    self.step_size, self.integration_time]:
            widget.setEnabled(not measuring)
        self.pause_measurement_btn.setText("Pause" if not self.is_paused else "Resume")

    def collectMeasurementData(self):
        try:
            if not self.is_measuring or self.is_paused:
                return
                
            pol_angle = self.polarization_angles[self.current_angle_index]
            count = self.picoharp.get_count_rate(channel=1)
            
            self.measurement_data.append((pol_angle, count))
            self.current_count_label.setText(f"Current Count: {count:.0f} cps")

            if count == 0:
                self.statusBar().showMessage(
                    "Warning: Zero count detected! Check if APD is on.", 
                    3000
                )
            self.updatePolarPlot()
            

            if self.auto_save_checkbox.isChecked():
                self.autoSaveData()
            
            self.current_angle_index += 1
            self.measureNextPoint()
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Data collection error: {str(e)}")
            self.stopMeasurement()

    def saveData(self):
        file_path, selected_filter = QFileDialog.getSaveFileName(
            self,
            "Save Measurement Data",
            "",
            "Text Files (*.txt);;CSV Files (*.csv);;All Files (*)"
        )
        
        if not file_path:
            return
            
        try:
            if selected_filter == "Text Files (*.txt)" and not file_path.endswith('.txt'):
                file_path += '.txt'
            elif selected_filter == "CSV Files (*.csv)" and not file_path.endswith('.csv'):
                file_path += '.csv'
                
            self._saveDataToFile(file_path)
            self.statusBar().showMessage(f"Data saved to {file_path}")
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save data: {str(e)}")

    def autoSaveData(self):
        if not self.auto_save_checkbox.isChecked():
            return
            
        try:
            save_path = self.save_path_input.text()
            if not save_path:
                return
                
            self._saveDataToFile(save_path)
            
        except Exception as e:
            self.statusBar().showMessage(f"Auto save failed: {str(e)}")

    def _saveDataToFile(self, file_path):
        is_txt = file_path.lower().endswith('.txt')
        separator = '\t' if is_txt else ','
        
        with open(file_path, 'w') as f:
            header = f"HWP Angle (degrees){separator}Polarization Angle (degrees){separator}Counts{separator}Normalized Counts\n"
            f.write(header)
            
            if self.measurement_data:
                pol_angles = np.array([d[0] for d in self.measurement_data])
                counts = np.array([d[1] for d in self.measurement_data])
                rotator_angles = pol_angles / 2
                
                max_count = counts.max()
                norm_counts = counts / max_count if max_count > 0 else np.zeros_like(counts)

                for hwp, pol, count, norm in zip(rotator_angles, pol_angles, counts, norm_counts):
                    if is_txt:
                        line = f"{hwp:8.1f}{separator}{pol:8.1f}{separator}{count:10.0f}{separator}{norm:10.6f}\n"
                    else:
                        line = f"{hwp:.1f}{separator}{pol:.1f}{separator}{count}{separator}{norm:.6f}\n"
                    f.write(line)
    
                if is_txt:
                    f.write("\n=== Measurement Summary ===\n")
                    f.write(f"Total Points: {len(counts)}\n")
                    f.write(f"Maximum Count: {max_count:.0f}\n")
                    f.write(f"Average Count: {counts.mean():.0f}\n")
                    f.write(f"Measurement Time: {self.integration_time_val:.1f} s per point\n")

    def closeEvent(self, event):
        if self.is_measuring:
            reply = QMessageBox.question(
                self, 
                "Exit Confirmation",
                "A measurement is in progress. Are you sure you want to quit?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No
            )
            
            if reply == QMessageBox.StandardButton.Yes:
                self.stopMeasurement()
                self.disconnectDevices()
                event.accept()
            else:
                event.ignore()
        else:
            self.disconnectDevices()
            event.accept()

    def disconnectDevices(self):
        self.disconnectPicoHarp()
        self.disconnectMotor()

    def createMessageBoxStyle(self):
        base_padding = int(6 * self.dpi_scale)
        base_radius = int(3 * self.dpi_scale)
        return f"""
            QMessageBox {{
                background-color: #2b2b2b;
                color: #ffffff;
            }}
            QMessageBox QLabel {{
                color: #ffffff;
                font-size: {self.base_font_size}px;
            }}
            QMessageBox QPushButton {{
                background-color: #0078d4;
                color: white;
                border: none;
                padding: {base_padding}px {base_padding * 2}px;
                border-radius: {base_radius}px;
                min-width: {self.base_font_size * 5}px;
                font-size: {self.base_font_size}px;
            }}
            QMessageBox QPushButton:hover {{
                background-color: #1084d8;
            }}
            QMessageBox QPushButton:pressed {{
                background-color: #006cbd;
            }}
        """

    def showCustomMessageBox(self, icon, title, text, buttons=QMessageBox.StandardButton.Ok):
        msg_box = QMessageBox(self)
        msg_box.setIcon(icon)
        msg_box.setWindowTitle(title)
        msg_box.setText(text)
        msg_box.setStandardButtons(buttons)
        msg_box.setStyleSheet(self.createMessageBoxStyle())
        return msg_box.exec()

def main():
    app = QApplication(sys.argv)
    app.setAttribute(Qt.ApplicationAttribute.AA_Use96Dpi)    
    # Scale factors for different pixel densities
    if hasattr(Qt, 'AA_EnableHighDpiScaling'):
        QApplication.setAttribute(Qt.ApplicationAttribute.AA_EnableHighDpiScaling, True)
    if hasattr(Qt, 'AA_UseHighDpiPixmaps'):
        QApplication.setAttribute(Qt.ApplicationAttribute.AA_UseHighDpiPixmaps, True)
        

    app.setStyle('Fusion')

    pg.setConfigOptions(
        antialias=True,
        background='k',
        foreground='w' 
    )

    window = MeasurementGUI()
    window.show()
    
    sys.exit(app.exec())

if __name__ == '__main__':
    main()

















        