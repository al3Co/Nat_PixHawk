import serial   # serial port communication
import time     # time for sleep

arduino = serial.Serial('/dev/ttyUSB1',baudrate=9600, timeout = 3.0)  # Arduino's port and transmition speed

incomingData=''

while True:
    arduino.write(b'B') # singal to consult arduino's data
    time.sleep(0.1)     # waiting for response
    while arduino.inWaiting() > 0 :
        incomingData += arduino.read(1)    # getting data sended by the arduino
    data = incomingData.split(',')
    print data
    incomingData = ''
    time.sleep(2)

arduino.close()
