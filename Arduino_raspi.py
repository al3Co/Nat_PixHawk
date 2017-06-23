import serial#comunicacion puerto serie
import time #modulo time para tiempo de respuesta entre tarjetas

arduino = serial.Serial('/dev/ttyACM0',baudrate=9600, timeout = 3.0)  #puerto de entrada arduino y velocidad de transmision


texto=''

while True:
    arduino.write(b'B')
    time.sleep(0.1)
    while arduino.inWaiting() > 0 :
        texto += arduino.read(1) #es suma compuesta
    print texto
    texto = ''
    time.sleep(2)

arduino.close()
