# control
import threading

import serial


class ServoControl:
    def __init__(self, targetPos=None, calibrate=0):
        self.calibrate = calibrate
        self.receivedString = ""
        self.pos = 0  # current position in degree
        if targetPos is not None:
            self.targetPos = targetPos  # desired position
        self.command = None  # control command
        self.controls = ["a", "d", "s", "w", "calibrate"]
        # acceptable angle error
        self.lowThreshold = 5  # acceptable angle error
        self.highThreshold = 20  # for calcuating speed of rotation
        self.thresholdRange = self.highThreshold - self.lowThreshold
        self.angleDivision = 360 // 30
        # speed of servo (self-defined)
        self.maxSpeed = 0xB0  # larger than 0x80 and assume
        self.minSpeed = 0x90  # larger than 0x80
        self.speedRange = self.maxSpeed - self.minSpeed

        # 6 byte motion control signal to ESP32
        self.LeftString = [0x60, 0x0B, 0xFF, 0xFF, 0xFF, 0xFF]
        self.RightString = [0x60, 0x0B, 0x00, 0x00, 0x00, 0x00]

        self.backwardString = [0x60, 0x0B, 0x90, 0x90, 0x70, 0x70]
        self.forwardString = [0x60, 0x0B, 0x70, 0x70, 0x90, 0x90]

        self.stopString = [0x60, 0x0B, 0x80, 0x80, 0x80, 0x80]

        # serial connection
        self.ser = serial.Serial(port="COM6", baudrate=115200, timeout=0.5)

        # initial the position
        self.initPos = False
        calibrateCount = 0
        while calibrateCount < 3:
            receivedString = self.ser.readline()
            if len(receivedString) >= 25:
                # current pos
                tempPos = int(int(receivedString[25]) / 255 * 360)
                tempPos -= self.calibrate
                if tempPos >= 360:
                    tempPos -= 360
                elif tempPos < 0:
                    tempPos += 360
                self.pos = tempPos
                print(receivedString)
                calibrateCount += 1
                print("Degree = ", self.pos)
        self.initPos = True
        if targetPos is None:
            self.targetPos = self.pos
        print("finish init")

    #  setter and getter
    def setCommand(self, input):
        self.command = input

    def setTargetPos(self, input):
        self.targetPos = int(input)
        self.updateTargetPos()

    def getTargetPos(self):
        return self.targetPos

    def getPos(self):
        return self.pos

    def getSlotNum(self):
        slot = self.pos // self.angleDivision
        return int(slot)

    def setTargetPosBySlotNum(self, slotNum):
        self.targetPos = slotNum * self.angleDivision + self.angleDivision // 2

    # 4 directions motion and stop
    def turnLeft(self, speed):
        self.LeftString[2] = speed  # servo 1
        self.LeftString[4] = speed  # servo 3

        self.LeftString[3] = speed  # servo 2
        self.LeftString[5] = speed  # servo 4
        self.ser.write(serial.to_bytes(self.LeftString))

    def turnRight(self, speed):
        self.RightString[2] = 0xFF - speed  # servo 1
        self.RightString[4] = 0xFF - speed  # servo 3

        self.RightString[3] = 0xFF - speed  # servo 2
        self.RightString[5] = 0xFF - speed  # servo 4
        self.ser.write(serial.to_bytes(self.RightString))

    def stop(self):
        self.ser.write(serial.to_bytes(self.stopString))

    def goForward(self):
        self.ser.write(serial.to_bytes(self.forwardString))

    def goBack(self):
        self.ser.write(serial.to_bytes(self.backwardString))

    # show output and update logic
    def showOutput(self, command=None):
        self.command = command
        receivedString = self.ser.readline()
        if len(receivedString) >= 25:
            tempPos = int(int(receivedString[25]) / 255 * 360)  # current pos
            tempPos -= self.calibrate
            if tempPos >= 360:
                tempPos -= 360
            elif tempPos < 0:
                tempPos += 360
            self.pos = tempPos
            print(receivedString)

            print("Degree = ", self.pos)
            print("targetPos = ", self.targetPos)

            self.updateTargetPos()

            stopped = self.dstop()
            print("stopped: ", stopped)
            print("getSpeedDisplay: ", self.getSpeedDisplay())
            print()
        return int(self.targetPos)

    # determine move or stop
    def dstop(self):
        # print("pos:", pos, " targetPos: ", targetPos)
        tempDist = abs(self.targetPos - self.pos)
        if tempDist >= 180:
            tempDist = 360 - tempDist

        if tempDist <= self.lowThreshold:
            self.stop()
            return True

        speed = self.getSpeed()
        print("speed: ", speed)
        if self.targetPos <= 180 and self.pos > 180:
            self.turnLeft(speed)
        elif self.targetPos > 180 and self.pos <= 180:
            self.turnRight(speed)
        elif self.targetPos > self.pos:
            self.turnLeft(speed)
        else:
            self.turnRight(speed)
        return False

    # update target position
    def updateTargetPos(self):
        # + or minus depend on the putting angle
        if self.command == "d":
            self.targetPos -= 12  # right turn 12 degree
        elif self.command == "a":
            self.targetPos += 12  # left turn 12 degree

        if self.targetPos >= 360:
            self.targetPos -= 360
        elif self.targetPos < 0:
            self.targetPos += 360

    # return a value between 0 to 1 to represent servo speed(1: max speed)
    def getSpeedDisplay(self):
        distance = abs(self.targetPos - self.pos)
        if distance >= 180:
            distance = 360 - distance

        if distance >= self.highThreshold:
            distance = self.highThreshold

        if distance < self.lowThreshold:
            return 0
        else:
            fraction = (distance - self.lowThreshold) / self.thresholdRange
            return fraction

    # Servo speed for motion
    def getSpeed(self):
        fraction = self.getSpeedDisplay()
        if fraction == 0:
            return 0
        else:
            speed = int(self.speedRange * fraction + self.minSpeed)
            return speed

    # calibrate the position of
    # set the current position as 0 degree, target position reset
    def calibrateAngle(self):
        # initial the position
        prevPos, tempPos, tempAngleDiff = 0
        self.stop()

        self.initPos = False
        calibrateCount = 0
        while calibrateCount < 5 or tempAngleDiff > 3:
            receivedString = self.ser.readline()
            if len(receivedString) >= 25:
                prevPos = tempPos
                # current pos
                tempPos = int(int(receivedString[25]) / 255 * 360)
                if tempPos >= 360:
                    tempPos -= 360
                elif tempPos < 0:
                    tempPos += 360
                print("tempPos = ", tempPos)
                calibrateCount += 1

                if calibrateCount >= 5:
                    tempAngleDiff = abs(prevPos - tempPos)
                    if tempAngleDiff > 180:
                        tempAngleDiff = 360 - tempAngleDiff

        self.initPos = True
        self.calibrate = tempPos
        self.pos = 0

    # main function
    # if command is given, we need "self.coomand = None"
    # to clear the command
    def main(self):
        while not self.initPos:
            pass
        while True:
            print(self.command)
            if self.command is None:
                self.showOutput()
            elif self.command == "calibrate":
                self.calibrateAngle()
                self.command = None
            elif self.command == "x":
                return
            else:
                self.showOutput(self.command)
                self.command = None


if __name__ == "__main__":
    servo = ServoControl()
    job = threading.Thread(target=servo.main)
    job.daemon = True
    job.start()

    while True:
        user_input = input("Please enter desired degree: ")
        if user_input == "x":
            servo.stop()
            servo.command = "x"
            break
        if user_input in servo.controls:  # input: a, s, d, w, control
            servo.command = user_input
        else:
            try:
                servo.setTargetPos(int(user_input))
            except ValueError:
                print("Enter integer")
