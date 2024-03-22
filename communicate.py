# communicate
import serial
import time

receivedString = ""
pos = 0


leftMaxSpeed = 0xFF
rightMaxSpeed = 0x00



RightString = [0x60, 0x0B, 0xFF, 0xFF, 0xFF, 0xFF]
LeftString = [0x60, 0x0B, 0x00, 0x00, 0x00, 0x00]

backwardString = [0x60, 0x0B, 0xFF, 0xFF, 0x00, 0x00]
forwardString = [0x60, 0x0B, 0x00, 0x00, 0xFF, 0xFF]

stopString = [0x60, 0x0B, 0x80, 0x80, 0x80, 0x80]

ser = serial.Serial(port="COM5", baudrate=115200, timeout=0.01)

# ser.write(serial.to_bytes(stopString))

def turnLeft():
    ser.write(serial.to_bytes(LeftString))

def turnRight():
    ser.write(serial.to_bytes(RightString))

def stop():
    ser.write(serial.to_bytes(stopString))

def goForward():
    ser.write(serial.to_bytes(forwardString))

def goBack():
    ser.write(serial.to_bytes(backwardString))

def showOutput():
    receivedString = ser.readline()
    if len(receivedString) >= 25:
        pos = int(int(receivedString[25]) / 255 * 360)  # current degree
        print(receivedString)

        print("Degree = ", pos)
        targetPos = updatePos(command)
        process = True

        pos, process = dstop(command, pos, targetPos)

        print()


def dstop(command, pos, targetPos):
    speed = "fast"
    if targetPos == pos:
        stop()
        return pos, False
    if abs(targetPos - pos) < 5:
        speed = "slow"
    if speed == "slow":
        if targetPos > pos:
            turnLeftSlow()
        elif targetPos < pos:
            turnRightSlow()
    else:
        if targetPos > pos:
            turnLeft()
        elif targetPos < pos:
            turnRight()
    return pos, True


def updatePos(command):
    # if command == "right":
    #     pos += 12  # right turn 12 degree
    # elif command == "left":
    #     pos -= 12  # left turn 12 degree
    # if pos >= 360:x
    #     pos -= 360
    # elif pos < 0:
    #     pos += 360
    targetPos = 0
    return int(targetPos)

goForward()
time.sleep(5)
stop()
time.sleep(3)
command = "right"
while 1:

    turnRight()
    time.sleep(1)

    stop()
    time.sleep(3)
