#communicate

import serial

com = "COM5"
baudrate = 115200
serialString = ""

ser = serial.Serial(com,baudrate)
ser.open()

while(1):
  serialString = ser.readline()

  try:
    print(serialString.decode("HEX"))
  except:
    pass
