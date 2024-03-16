# communicate
import serial

receivedString = ""
pos = 0

LeftString = [0x60, 0x0B, 0xFF, 0xFF, 0xFF, 0xFF]
RightString = [0x60, 0x0B, 0x00, 0x00, 0x00, 0x00]

LeftStringSlow = [0x60, 0x0B, 0xC0, 0xC0, 0xC0, 0xC0]
RightStringSlow = [0x60, 0x0B, 0x40, 0x40, 0x40, 0x40]

stopString = [0x60, 0x0B, 0x80, 0x80, 0x80, 0x80]

ser = serial.Serial(port="COM5", baudrate=115200, timeout=0.5)


# ser.write(serial.to_bytes(stopString))


def turnLeft():
    ser.write(serial.to_bytes(LeftString))


def turnRight():
    ser.write(serial.to_bytes(RightString))


def turnLeftSlow():
    ser.write(serial.to_bytes(LeftStringSlow))


def turnRightSlow():
    ser.write(serial.to_bytes(RightStringSlow))


def stop():
    ser.write(serial.to_bytes(stopString))


def showOutput():
    receivedString = ser.readline()
    if len(receivedString) >= 25:
        pos = int(receivedString[25]) / 255 * 360  # current degree
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
    # if pos >= 360:
    #     pos -= 360
    # elif pos < 0:
    #     pos += 360
    targetPos = 0
    return int(targetPos)


command = "right"
while 1:
    showOutput()
