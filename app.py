
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil

import spidev
import sys
import time
import argparse 
import json

import smbus
import serial
"""
import Adafruit_DHT

arduinoData = []

bus = smbus.SMBus(1)

# Open SPI bus
spi = spidev.SpiDev()
spi.open(0,0)

# I2C Constants
# ADDR = 0x60
baro_addr = 0x60
CTRL_REG1 = 0x26
PT_DATA_CFG = 0x13

# Connecting barometer/temp sensor
who_am_i = bus.read_byte_data(baro_addr, 0x0C)
# print hex(who_am_i)
if who_am_i != 0xc4:
    print "Device not active."
    exit(1)

# Set oversample rate to 128
setting = bus.read_byte_data(baro_addr, CTRL_REG1)
newSetting = setting | 0x38
bus.write_byte_data(baro_addr, CTRL_REG1, newSetting)

# Enable event flags
bus.write_byte_data(baro_addr, PT_DATA_CFG, 0x07)

# Humidity sensor supported.
sensor_args = { 
                '11': Adafruit_DHT.DHT11,
                '22': Adafruit_DHT.DHT22,
                '2302': Adafruit_DHT.AM2302
              }
"""
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
   print "Error to Connect", e

# BGN: GENERAL FUNCTIONS

def readArduinoData():
    print "Function to read Arduino's data ...", arduino.is_open
    try:
        arduino.write(b'B')
        if arduino.inWaiting() > 1:
            print "Reading data from ...", arduino.name
            callback = arduino.readline()
            command = callback.split(',')
            print command
            arduino.flush()
        print "Arduino is open:", arduino.is_open
        if arduino.is_open:
            incomingData = arduino.readline()
            print incomingData, "Data from Arduino"
            # serparar y guardar en vector = arduinoData
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
"""
# Function to read SPI data from MCP3008 chip
# Channel must be an integer 0-7
def ReadChannel(channel):
  adc = spi.xfer2([1,(8+channel)<<4,0])
  data = ((adc[1]&3) << 8) + adc[2]
  return data
 
# Function to convert data to voltage level,
# rounded to specified number of decimal places.
def ConvertVolts(data,places):
  volts = (data * 3.3) / float(1023)
  volts = round(volts,places)
  return volts
  
"""

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
            
            readArduinoData()

            print "Mode: %s" % mode
            print "Latitud: " + str(lat)
            print "Longitud: " + str(lon)
            print "Attitude: " + str(alt)

            data = "[]"

            # writeNumber(2)
            # time.sleep(1)
            # number = readNumber()

            # print("RPI: Hello computer I give you this data ", data)
            # ser.write(str(data) + '\n')

            ## Read barometer/temp sensor ##
            # Toggle One Shot
            """
            setting = bus.read_byte_data(baro_addr, CTRL_REG1)
            if (setting & 0x02) == 0:
                bus.write_byte_data(baro_addr, CTRL_REG1, (setting | 0x02))

            # Read sensor data
            print "Waiting for barometer data..."
            status = bus.read_byte_data(baro_addr,0x00)
            while (status & 0x08) == 0:
                #print bin(status)
                status = bus.read_byte_data(baro_addr,0x00)
                #time.sleep(0.5)

            print "Reading barometer sensor data..."
            p_data = bus.read_i2c_block_data(baro_addr,0x01,3)
            t_data = bus.read_i2c_block_data(baro_addr,0x04,2)
            status = bus.read_byte_data(baro_addr,0x00)
            print "Baro status: "+bin(status)

            p_msb = p_data[0]
            p_csb = p_data[1]
            p_lsb = p_data[2]
            t_msb = t_data[0]
            t_lsb = t_data[1]

            pressure = (p_msb << 10) | (p_csb << 2) | (p_lsb >> 6)
            p_decimal = ((p_lsb & 0x30) >> 4)/4.0

            celsius = t_msb + (t_lsb >> 4)/16.0
            fahrenheit = (celsius * 9)/5 + 32

            print "Pressure and Temperature at "+time.strftime('%m/%d/%Y %H:%M:%S%z')
            print str(pressure+p_decimal)+" Pa"
            print str(celsius)+" C"

            ## Read CO analog sensor, reading channel 0 ##
            co_level = ReadChannel(0)

            ## Read humidity and temp ##
            print "Reading humidity sensor data..."

            humidity, temperature = Adafruit_DHT.read_retry(sensor_args['22'], 4)

            if humidity is not None and temperature is not None:
                print 'Temp={0:0.1f}*  Humidity={1:0.1f}%'.format(temperature, humidity)
            else:
                humidity = 0
                temperature = 0
                print 'Failed to get reading. Try again!'
            """

            ## Sending data ##

            #data = '[' + str(arm) + ',' + str(mode) + ',' + str(batt) + ',' + str(lat) + ',' + str(lon) + ',' + str(alt) + ',' + str(pressure+p_decimal) + ',' + str(celsius) + ',' + str(humidity) + ',' + str(temperature) + ',' + str(ground_speed) + ',' + str(co_level) + ']'
            
            data = '[' + str(arm) + ',' + str(mode) + ',' + str(batt) + ',' + str(lat) + ',' + str(lon) + ',' + str(alt) + ']'

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
        #GPIO.cleanup()
        ser.close()
        arduino.close()
        ser.close()


if __name__ == '__main__':
    print "==Inicio=="
    main()
