#!/usr/bin/env python

# Generic driver for the Neato XV-11 Robot Vacuum
# Copyright (c) 2010 University at Albany. All right reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the University at Albany nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
neato_driver.py is a generic driver for the Neato XV-11 Robotic Vacuum.
ROS Bindings can be found in the neato_node package.
"""

__author__ = "ferguson@cs.albany.edu (Michael Ferguson)"

import serial
import time
import traceback
import numpy as np
import matplotlib.pyplot as plt


BASE_WIDTH = 248    # millimeters
MAX_SPEED = 300     # millimeters/second


#DEFAULT_DEV_PATH = "/dev/ttyACM0"
#DEFAULT_DEV_PATH = "/dev/serial/by-id/usb-Linux_2.6.33.7_with_fsl-usb2-udc_Neato_Robotics_USB_v2-if00"
DEFAULT_DEV_PATH = "/dev/serial/by-id/usb-Acme_Corporation_CDC_Serial_Peripheral_xxxx-xxxx-xxxx-if00"



xv11_analog_sensors = [ "WallSensorInMM",
                "BatteryVoltageInmV",
                "LeftDropInMM",
                "RightDropInMM",
                "RightMagSensor",
                "LeftMagSensor",
                "XTemp0InC",
                "XTemp1InC",
                "VacuumCurrentInmA",
                "ChargeVoltInmV",
                "NotConnected1",
                "BatteryTemp1InC",
                "NotConnected2",
                "CurrentInmA",
                "NotConnected3",
                "BatteryTemp0InC" ]

xv11_digital_sensors = [ "SNSR_DC_JACK_CONNECT",
                "SNSR_DUSTBIN_IS_IN",
                "SNSR_LEFT_WHEEL_EXTENDED",
                "SNSR_RIGHT_WHEEL_EXTENDED",
                "LSIDEBIT",
                "LFRONTBIT",
                "RSIDEBIT",
                "RFRONTBIT" ]

xv11_motor_info = [ "Brush_MaxPWM",
                "Brush_PWM",
                "Brush_mVolts",
                "Brush_Encoder",
                "Brush_RPM",
                "Vacuum_MaxPWM",
                "Vacuum_PWM",
                "Vacuum_CurrentInMA",
                "Vacuum_Encoder",
                "Vacuum_RPM",
                "LeftWheel_MaxPWM",
                "LeftWheel_PWM",
                "LeftWheel_mVolts",
                "LeftWheel_Encoder",
                "LeftWheel_PositionInMM",
                "LeftWheel_RPM",
                "RightWheel_MaxPWM",
                "RightWheel_PWM",
                "RightWheel_mVolts",
                "RightWheel_Encoder",
                "RightWheel_PositionInMM",
                "RightWheel_RPM",
                "Laser_MaxPWM",
                "Laser_PWM",
                "Laser_mVolts",
                "Laser_Encoder",
                "Laser_RPM",
                "Charger_MaxPWM",
                "Charger_PWM",
                "Charger_mAH" ]

xv11_charger_info = [ "FuelPercent",
                "BatteryOverTemp",
                "ChargingActive",
                "ChargingEnabled",
                "ConfidentOnFuel",
                "OnReservedFuel",
                "EmptyFuel",
                "BatteryFailure",
                "ExtPwrPresent",
                "ThermistorPresent[0]",
                "ThermistorPresent[1]",
                "BattTempCAvg[0]",
                "BattTempCAvg[1]",
                "VBattV",
                "VExtV",
                "Charger_mAH",
                "MaxPWM" ]


LI_ION_VOLT = [4.0, 3.6, 3.5, 3.4, 3.3, 3.2, 3.1, 3.0, 2.9]
LI_ION_SOC  = [100,  90,  80,  70,  50,  30,  20,  10,   5] 
             




def interpolate_vector(data, factor):
    n = len(data)
    # X interpolation points. For factor=4, it is [0, 0.25, 0.5, 0.75, 1, 1.25, 1.5, ...]
    x = np.linspace(0, n - 1, (n - 1) * factor + 1)
    # Alternatively:
    # x = np.arange((n - 1) * factor + 1) / factor
    # X data points: [0, 1, 2, ...]
    xp = np.arange(n)
    # Interpolate
    return np.interp(x, xp, np.asarray(data))


class xv11():

    def __init__(self, port="/dev/ttyUSB0"):
        self.portName = port
        self.nextMeasureTime = 0
        self.loopsPerSec = 0
        self.loopCounter = 0
        self.LDSstate = ""
        self.BrushState = ""
        self.VacuumState = 0
        self.TestModeState = ""
        self.charging = False
        self.chargerConnected = True
        self.chargingCompleted = False        
        self.batteryLow = False
        self.connected = False
        self.nextBatTime = 0
        self.nextBumperTime = 0
        self.nextLogTime = 0                     
        self.batCellCount = 4
        self.LI_ION_VOLT = interpolate_vector(LI_ION_VOLT, 20)
        self.LI_ION_SOC  = interpolate_vector(LI_ION_SOC, 20)
        #self.LI_ION_VOLT = np.flip(interpolate_vector(LI_ION_VOLT, 10))
        #self.LI_ION_SOC  = np.flip(interpolate_vector(LI_ION_SOC, 10))
        #print(self.LI_ION_VOLT)
        #print(self.LI_ION_SOC)
        #plt.plot(self.LI_ION_VOLT, self.LI_ION_SOC)
        #plt.show()
        self.start()

    def start(self):
        try:
            print('xv11: opening port', self.portName)
            time.sleep(1.0)        
            self.port = serial.Serial(self.portName,115200,timeout=0.005)
        except:
            print('error opening serial', self.portName)
            self.port = None
            return
        print('xv1: port opened', self.portName)
        #self.port.flushInput()                                
        # Storage for motor and sensor information
        self.state = {"LeftWheel_PositionInMM": 0, "RightWheel_PositionInMM": 0}
        self.stop_state = True
        # turn things on
        self.setTestMode("on")
        #self.setLDS("on")
        time.sleep(0.5)

    def close(self):
        print('xv11 closing serial', self.portName)
        if self.port is None: return        
        self.port.close()
        self.port = None
        print('xv11 closed serial', self.portName)
        

    def exit(self):
        print('xv11 exiting...')
        if self.port is None: return
        self.setLDS("off")
        if self.port is None: return        
        self.setTestMode("off")
        self.close()
        

    def readline(self):
        if self.port is None:
            print('readline: port not open')
            time.sleep(2.0)
            self.start()
            return ""
        line = ""        
        timeout = time.time() + 0.2
        try:                    
            #self.port.flushInput()                    
            while True:
                if self.port.inWaiting() > 0:
                    line = self.port.readline()
                    try:
                        line = str(line.decode().strip())        
                        #print('readline', line)                    
                        return line
                    except:        
                        line = ""
                if time.time() > timeout:
                    print('readline: timeout')
                    return line
                time.sleep(0.001)

        except OSError:
            print('readline inWaiting failed')
            self.close()
            return ""
        
    
    def writeline(self, line):
        if self.port is None:
            print('writeline: port not open')
            return
        try:
            self.port.flushInput()                                    
            #print('writeline', line)
            self.port.write( (line + "\n").encode() )
            time.sleep(0.001)            
        except:
            print('writeline failed')
            self.close()

    def setTestMode(self, value):
        if self.TestModeState == value: return
        self.TestModeState = value
        print('********* setTestMode', value)
        """ Turn test mode on/off. """
        self.writeline("testmode " + value)

    def setLDS(self, value):
        if self.LDSstate == value: return
        self.LDSstate = value
        print('setLDS', value)
        self.writeline("setldsrotation " + value )

    def setFuelGauge(self, percent):
        print('setFuelGauge', percent)
        self.writeline("SetFuelGauge " + str(int(percent)) )

    def playSound(self, soundId):
        self.writeline("PlaySound " + str(int(soundId)) )

    def playSoundStop(self):
        self.writeline("PlaySound Stop")

    def requestScan(self):
        """ Ask neato for an array of scan reads. """
        self.writeline( "getldsscan" )

    def getScanRanges(self):
        """ Read values of a scan -- call requestScan first! """
        # print('getScanRanges')
        ranges = list()
        angle = 0
        tryCount = 0
        while True:
            line = self.readline()
            if line.split(",")[0] == "AngleInDegrees":
                break
            tryCount += 1
            if tryCount > 10:
                print('getScanRanges: no answer')
                return ranges
        # print('data start')
        while angle < 360:
            try:
                vals = self.readline()
            except:
                print('getScanRanges: loop error')
                pass
            vals = vals.split(",")
            #print angle, vals
            try:
                a = int(vals[0])
                r = int(vals[1])
                ranges.append(r/1000.0)
            except:
                ranges.append(0)
            angle += 1
        #print('getScanRanges', ranges)
        return ranges

    def setMotors(self, l, r, s):        
        #print('setMotors')
        """ Set motors, distance left & right + speed """
        #This is a work-around for a bug in the Neato API. The bug is that the
        #robot won't stop instantly if a 0-velocity command is sent - the robot
        #could continue moving for up to a second. To work around this bug, the
        #first time a 0-velocity is sent in, a velocity of 1,1,1 is sent. Then, 
        #the zero is sent. This effectively causes the robot to stop instantly.
        if (int(l) == 0 and int(r) == 0 and int(s) == 0):
            if (not self.stop_state):
                self.stop_state = True
                l = 1
                r = 1
                s = 1
        else:
            self.stop_state = False
        # left motor distance (mm), right motor distance (mm), speed (mm/s)
        self.writeline( "setmotor "+str(int(l))+" "+str(int(r))+" "+str(int(s)) )

    def getMotors(self):
        #print('getMotors')
        """ Update values for motors in the self.state dictionary.
            Returns current left, right encoder values. """
        #print('write')
        self.writeline( "getmotors" )
        tryCount = 0
        headers = None
        valIdx = 1
        while True:
            line = self.readline()
            headers = line.split(",") 
            if headers[0] == "Parameter":
                valIdx = headers.index('Value')
                break
            tryCount += 1
            if tryCount > 10:
                print('getMotors: no answer')
                return [0,0]
        while True:
            try:
                values = self.readline().split(",")
                if len(values) < 2: break
                #print(values)
                self.state[values[0]] = float(values[valIdx])
            except Exception :
                traceback.print_exc()
                break
        return [self.state["LeftWheel_PositionInMM"],self.state["RightWheel_PositionInMM"]]


    def getAnalogSensors(self):
        """ Update values for analog sensors in the self.state dictionary. """
        #print('getAnalogSensors begin')
        self.writeline( "getanalogsensors" )
        tryCount = 0
        headers = None
        valIdx = 1
        while True:
            line = self.readline()
            headers = line.split(",")
            if headers[0] == "SensorName":
                valIdx = headers.index('Value')
                break
            tryCount += 1
            if tryCount > 10:
                print('getAnalogSensor: no answer')
                return False
        while True:
            try:
                values = self.readline().split(",")
                if len(values) < 2: 
                    #print('getAnalogSensors end')
                    return True
                #print(values)
                self.state[values[0]] = float(values[valIdx])
            except Exception:
                traceback.print_exc()
                return False                 
        

    def getDigitalSensors(self):
        """ Update values for digital sensors in the self.state dictionary. """
        self.writeline( "getdigitalsensors" )
        headers = None
        valIdx = 1        
        tryCount = 0
        while True:
            line = self.readline()
            headers = line.split(",")
            headers = [s.strip() for s in headers]
            #print('headers', headers)
            if headers[0] == "Digital Sensor Name":
                valIdx = headers.index('Value')
                break
            tryCount += 1
            if tryCount > 10:
                print('getDigitalSensors: no answer')
                return False 
        while True:
            try:
                values = self.readline().split(",")
                if len(values) < 2: return True
                #print(values)
                self.state[values[0]] = float(values[valIdx])
            except Exception:
                traceback.print_exc()
                return False

            

    def getCharger(self):
        """ Update values for charger/battery related info in self.state dictionary. """
        self.writeline( "getcharger" )
        headers = None
        valIdx = 1        
        tryCount = 0
        while True:
            line = self.readline()
            headers = line.split(",") 
            if headers[0] == "Label":
                valIdx = headers.index('Value')
                break
            tryCount += 1
            if tryCount > 10:
                print('getCharger: no answer')
                return False         
        while True:
            try:            
                values = self.readline().split(",")
                if len(values) < 2: return True
                #print(values)
                self.state[values[0]] = float(values[valIdx])
            except Exception:
                traceback.print_exc()
                return False

    def setBacklight(self, value):
        if value > 0:
            self.writeline( "setled backlighton" )
        else:
            self.writeline( "setled backlightoff" )

    def setVacuum(self, value):
        if self.VacuumState == value: return
        self.VacuumState = value
        print('setVacuum', value)        
        if value > 0:
            self.writeline( "SetMotor VacuumOn") 
        else:
            self.writeline ("SetMotor VacuumOff" )
        
    def setBrush(self, value):
        if self.BrushState == value: return
        self.BrushState = value
        print('setBrush', value)        
        self.writeline( "SetMotor Brush RPM " + str(int(value)))     
        #if value > 0:
        #    self.writeline( "SetMotor Brush Enable") 
        #    self.writeline( "SetMotor Brush RPM " + str(int(value))) 
        #else:
        #    self.writeline( "SetMotor Brush Disable") 


    def setConfig(self, liIon4Cell):
        # 1  NIMH_12CELL
        # 2  NIMH_10CELL
        # 3  LIION_4CELL
        # 4  LIION_4CELL_SMART
        if liIon4Cell:
            self.writeline( "SetConfig BatteryType 3")     
        else:           
            self.writeline( "SetConfig BatteryType 1")     


    def setNewBatteryInserted(self):
        self.writeline("TestMode on")
        self.writeline("NewBattery")     
                        

    #SetLED - Sets the specified LED to on,off,blink, or dim. (TestMode Only)
    #BacklightOn - LCD Backlight On  (mutually exclusive of BacklightOff)
    #BacklightOff - LCD Backlight Off (mutually exclusive of BacklightOn)
    #ButtonAmber - Start Button Amber (mutually exclusive of other Button options)
    #ButtonGreen - Start Button Green (mutually exclusive of other Button options)
    #LEDRed - Start Red LED (mutually exclusive of other Button options)
    #LEDGreen - Start Green LED (mutually exclusive of other Button options)
    #ButtonAmberDim - Start Button Amber Dim (mutually exclusive of other Button options)
    #ButtonGreenDim - Start Button Green Dim (mutually exclusive of other Button options)
    #ButtonOff - Start Button Off


    def getBatteryVoltage(self):
        if 'BatteryVoltageInmV' in self.state:
            return round(self.state['BatteryVoltageInmV'] / 1000.0, 2)
        if 'BatteryVoltage' in self.state:
            return round(self.state['BatteryVoltage'] / 1000.0, 2)
        return 0
    
    def getChargeVoltage(self):
        if 'ChargeVoltInmV' in self.state:
            return round(self.state['ChargeVoltInmV'] / 1000.0, 2)
        if 'VExtV' in self.state:
            return round(self.state['VExtV'], 2)
        return 0

    def getFuelPercent(self):
        if 'FuelPercent' in self.state:
            return self.state['FuelPercent']
        return 0

    def getDropSensors(self):
        if 'LeftDropInMM' in self.state:
            return [ self.state['LeftDropInMM'], self.state['RightDropInMM'] ] 
        if  'DropSensorLeft' in self.state:
            return [ self.state['DropSensorLeft'], self.state['DropSensorRight'] ]
        return [0, 0]

    def getBumperSensors(self):
        if 'LFRONTBIT' in self.state:         
            return  [ self.state['LFRONTBIT'], self.state['LSIDEBIT'], self.state['RSIDEBIT'], self.state['RFRONTBIT'] ]
        return [0, 0, 0, 0]                    



    def run(self):
        if time.time() > self.nextMeasureTime:
            self.loopsPerSec = self.loopCounter
            self.loopCounter = 0            
            self.nextMeasureTime = time.time() + 1.0            

        if time.time() > self.nextBatTime:
            self.nextBatTime = time.time() + 2.0            
            if self.getAnalogSensors():
                self.connected = True                    
                self.batteryLow = self.getBatteryVoltage() < 13.0
                self.chargerConnected =  self.getChargeVoltage() > 10.0
                self.chargingCompleted = self.getBatteryVoltage() > 14.5 
            else:
                self.connected = False                

            if not self.connected:
                return
            #print('+++', self.connected, self.chargerConnected)
            #if self.connected and self.chargerConnected: 

            if self.getCharger():
                #print('ChargingActive', self.state['ChargingActive'])
                self.charging = (int(self.state['ChargingActive']) == 1)                                        
            if self.chargerConnected or self.batteryLow:
                #self.setBrush(0)
                #self.setVacuum(0)
                #self.setLDS("off")
                pass
            if self.chargerConnected:
                #self.setTestMode("off")
                pass   

        if time.time() > self.nextBumperTime:
            self.nextBumperTime = time.time() + 1.0
            self.getDigitalSensors()

        if time.time() > self.nextLogTime:
            self.nextLogTime = time.time() + 3.0
            self.log()
        self.loopCounter += 1


    def log(self):
        print('lps', self.loopsPerSec, 'conn', self.connected, 'chgConn', self.chargerConnected,
               'chg', self.charging, 'batLow', self.batteryLow, 'chgCompl', self.chargingCompleted, 
               'fuelPerc', self.getFuelPercent(), 'batV', self.getBatteryVoltage(), round(self.getBatteryVoltage()/self.batCellCount, 2) , 
               'chgV', self.getChargeVoltage(), round(self.getChargeVoltage()/self.batCellCount, 2) )

        print('bumper', self.getBumperSensors(), 'drop', self.getDropSensors())
                


    def liIonCellFuelGauge(self, cellVoltage):
        idx = 0
        for level in self.LI_ION_VOLT:
            #print('cellVoltage', cellVoltage, 'level', level)
            if cellVoltage > level:
                return round(self.LI_ION_SOC[idx]) 
            idx += 1
        return 0


def calibrateBattery():
    robot = xv11(DEFAULT_DEV_PATH)
    nextCalTime = 0
    loadActive = True
    try:
        while True:
            time.sleep(0.001)        
            robot.run()                       
            #robot.setVacuum(1)            
            cellVoltage = round(robot.getBatteryVoltage() / robot.batCellCount, 2)
            if time.time() > nextCalTime:
                nextCalTime = time.time() + 10.0
                gauge = robot.liIonCellFuelGauge(cellVoltage)
                print('cellVoltage', cellVoltage, 'liIonCellFuelGauge', gauge)
                if gauge > 0:
                    robot.setFuelGauge(gauge)                
    except Exception :
        traceback.print_exc()
    finally:
        robot.exit()


def plotScan(ranges):
    plt.clf()    
    #fig = plt.figure(dpi=200)
    #ax = fig.add_subplot(projection='polar')
    angles = range(0, 360)
    #print(angles)
    angles = [angle / 180.0 * 3.1415 for angle in angles]    
    plt.polar(angles, ranges, marker='o', linestyle='')
    ax = plt.gca()
    ax.set_ylim(0,1)    
    ax.grid(True)
    plt.pause(1.0)    
    #plt.show()
        


def testRobotFeatures():
    robot = xv11(DEFAULT_DEV_PATH)
    try:   
        nextMotorTime = 0
        driveForward = False
        
        while True: 
            time.sleep(0.001)    
            robot.run()
 
            robot.setTestMode("on")
            robot.setLDS("on")
            robot.requestScan()            
            ranges = robot.getScanRanges()            
            print(time.time(), 'ranges', len(ranges))        
 
            if not robot.connected:
                continue

            plotScan(ranges)

            if robot.chargerConnected:
                continue
            if robot.batteryLow:
                continue    

            continue 
            res = robot.getMotors()
            # print('motors: ', res, '')
            if driveForward:                
                robot.setMotors(50, 50, 50)  
                robot.setVacuum(1) 
                robot.setBrush(300)       
            else:
                robot.setMotors(-50, -50, 50)
                robot.setVacuum(0)
                robot.setBrush(0)
            if time.time() > nextMotorTime:
                nextMotorTime = time.time() + 10.0
                driveForward = not driveForward
                if driveForward:
                    robot.playSound(3)
                #time.sleep(1.0)
            if robot.getBumperSensors()[0] > 0:
                nextMotorTime = time.time() + 10.0
                driveForward = False

    except Exception:
        traceback.print_exc()
    finally:        
        robot.exit()


if __name__ == "__main__":
    testRobotFeatures()
    #calibrateBattery()





