import serial

arduino = serial.Serial('/dev/ttyACM0',baudrate=9600, timeout = 3.0)  # declare port
while True:
    print arduino.readline()

arduino.close()
