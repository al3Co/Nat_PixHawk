import serial

try:
    pixHawk = serial.Serial('/dev/ttyACM0',baudrate=9600, timeout = 3.0)  # declare port
    arduino = serial.Serial('/dev/ttyUSB0',baudrate=9600, timeout = 3.0)  # declare port
    a = True
except:
    print "Serial is not avalible."
    a = False

while True:
    if arduino.is_open:
        print arduino.readline()

arduino.close()
