
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil

import spidev
import sys
import time
import argparse 
import json

import smbus
import serial

# Connecting telemetry module
try:
    ser = serial.Serial('/dev/ttyUSB0', 57600) #cambiar por purto serial de pixhawk
    #ser.open()
    print "Conected to:", ser.name
except Exception, e:
    print "Serial is not avalible..", e

# Connecting Arduino module
try:
    arduino = serial.Serial('/dev/ttyUSB1',baudrate=9600)
    #arduino.open()
    print "Conected to:", arduino.name
except Exception, e:
    print "Serial is not avalible..", e


# Connecting Mavproxy
try:
    global vehicle
    parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
    parser.add_argument('--connect', default='127.0.0.1:14550',help="vehicle connection target. Default '127.0.0.1:14550'")
    args = parser.parse_args()
    print "\nConnecting to vehicle on: %s" % args.connect
    vehicle = connect(args.connect, wait_ready=True)
    vehicle = connect('127.0.0.1:14550', wait_ready=True)
except Exception, e:
   print "Error connecting to Mavproxy", e

# BGN: GENERAL FUNCTIONS

# Reading Arduino Data
# Arduino's code need a 'B' letter to start reading and sending data sensors
def readDataArduino():
    try:
        text=''
        # sending B letter to the Arduino to check the sensors
        arduino.write(b'B')
        time.sleep(0.1)
        # getting Ardiunos data
        while arduino.inWaiting() > 0 :
            text += arduino.read(1)
        # creating tuple with data readed
        data = text.split(',')
        text = ''
        return data
    except Exception, e:
            print "Error reading Arduino's data:", e

def writeNumber(value):
    bus.write_byte(address, int(value))
    return -1

def readNumber():
    number = bus.read_byte(address)
    return number

def set_arm(arm):
    if arm:
        print "Arming motors"
        vehicle.armed = True
    else:
        print "Disarming motors"
        vehicle.armed = False

# SET Mode

def set_mode(mode):
    if mode == 1:
        vehicle.mode = VehicleMode("STABILIZE")

    if mode == 2:
        vehicle.mode = VehicleMode("ALT_HOLD")

    if mode == 3:
        vehicle.mode = VehicleMode("LOITER")

    if mode == 4:
        vehicle.mode = VehicleMode("RTL")

    if mode == 5:
        vehicle.mode = VehicleMode("AUTO")

    print "Change mode: " + str(mode)

def takeoff(t):
    vehicle.commands.takeoff(t)
    print "takeoff at " + str(t)

# END: GENERAL FUNCTIONS


# MAIN LOOP
def main():
    try:
        while True:
            ## Vehicle data ##

            lat = vehicle.location.global_frame.lat
            lon = vehicle.location.global_frame.lon
            alt = vehicle.location.global_frame.alt

            arm = str(vehicle.armed)
            mode = str(vehicle.mode.name)
            batt = str(vehicle.battery.level)

            vel = str(vehicle.velocity)
            ground_speed = str(vehicle.groundspeed)
            air_speed = str(vehicle.airspeed)
            
            dataArduino = readDataArduino()
            
            print "Mode: %s" % mode
            print "Latitud: " + str(lat)
            print "Longitud: " + str(lon)
            print "Attitude: " + str(alt)
            
            print "Sensor 1:" , dataArduino[0]
            print "Sensor 2:" , dataArduino[1]
            print "Sensor 3:" , dataArduino[2]

            data = "[]"
            
            data = '[' + str(arm) + ',' + str(mode) + ',' + str(batt) + ',' + str(lat) + ',' + str(lon) + ',' + str(alt) + str(dataArduino[0]) + str(dataArduino[1]) + str(dataArduino[2]) +']'

            ser.write(str(data) + '\n')
            #time.sleep(0.1)

            callback = ''

            if ser.inWaiting() > 1:
                print "Reading data from ...", ser.name
                callback = ser.readline()
                command = callback.split(':')
                print "%s:%s" % (str(command[0]), str(command[1]))
                if str(command[0]) == 'mode':
                    set_mode(int(command[1]))
                ser.flush()

    except KeyboardInterrupt:
        ser.close()
        arduino.close()
        ser.close()

if __name__ == '__main__':
    print "==Inicio=="
    main()