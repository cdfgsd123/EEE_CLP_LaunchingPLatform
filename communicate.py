#communicate

import serial

port = "COM5"
baudrate = 115200
receivedString = ""

LeftString =  "60 0B FF FF FF FF"
RightString = "60 0B 00 00 00 00"
stopString =  "60 0B 80 80 80 80"

ser = serial.Serial(port,baudrate)
ser.open()

while(1):
  receivedString = ser.readline()

  try:
    print(receivedString)
  except:
    pass

def sendTest():
  ser.write(stopString)
