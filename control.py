# communicate
import threading

import serial


class ServoControl:
    def __init__(self, targetPos):
        self.receivedString = ""
        self.pos = 0  # current position in degree
        self.targetPos = targetPos  # desired position
        self.command = None  # control command
        self.directions = ["a", "d", "s", "w"]
        # acceptable angle error
        self.lowThreshold = 5  # acceptable angle error
        self.highThreshold = 20  # for calcuating speed of rotation
        self.thresholdRange = self.highThreshold - self.lowThreshold
        self.angleDivision = 360 // 30
        # speed of servo (self-defined)
        self.maxSpeed = 0xCC
        self.minSpeed = 0x90
        self.speedRange = self.maxSpeed - self.minSpeed

        # 6 byte motion control signal to ESP32
        self.RightString = [0x60, 0x0B, 0xFF, 0xFF, 0xFF, 0xFF]
        self.LeftString = [0x60, 0x0B, 0x00, 0x00, 0x00, 0x00]

        self.backwardString = [0x60, 0x0B, 0x90, 0x90, 0x70, 0x70]
        self.forwardString = [0x60, 0x0B, 0x70, 0x70, 0x90, 0x90]

        self.stopString = [0x60, 0x0B, 0x80, 0x80, 0x80, 0x80]

        # serial connection
        self.ser = serial.Serial(port="COM5", baudrate=115200, timeout=0.1)

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
        self.LeftString[2] = self.maxSpeed - speed  # servo 1
        self.LeftString[4] = self.maxSpeed - speed  # servo 3

        self.LeftString[3] = self.maxSpeed - speed  # servo 2
        self.LeftString[5] = self.maxSpeed - speed  # servo 4
        self.ser.write(serial.to_bytes(self.LeftString))

    def turnRight(self, speed):
        self.RightString[2] = speed  # servo 1
        self.RightString[4] = speed  # servo 3

        self.RightString[3] = speed  # servo 2
        self.RightString[5] = speed  # servo 4
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
            self.pos = int(int(receivedString[25]) / 255 * 360)  # current pos
            print(receivedString)

            print("Degree = ", self.pos)
            print("targetPos = ", self.targetPos)

            self.updateTargetPos()

            stopped = self.dstop(command, self.pos, self.targetPos)
            print("stopped: ", stopped)
            print()
        return int(self.targetPos)

    # determine move or stop
    def dstop(self):
        # print("pos:", pos, " targetPos: ", targetPos)
        if abs(self.targetPos - self.pos) <= self.lowThreshold:
            self.stop()
            return True

        speed = self.getSpeed()

        temp = self.targetPos - self.pos
        if temp > 0 and abs(temp) < 180:
            self.turnLeft(speed)
        elif temp > 0 and abs(temp) >= 180:
            self.turnRight(speed)
        elif temp < 0 and abs(temp) < 180:
            self.turnRight(speed)
        else:
            self.turnLeft(speed)
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

        if distance >= self.highThreshold:
            distance = self.highThreshold

        if distance < self.highThreshold:
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

    # main function
    def main(self):
        while True:
            print(self.command)
            if self.command is None:
                self.showOutput()
            elif self.command == "x":
                return
            else:
                self.showOutput(self.command)
                self.command = None


if __name__ == "__main__":
    servo = ServoControl(0)
    job = threading.Thread(target=servo.main)
    job.start()

    while True:
        user_input = input("Please enter desired degree: ")
        if user_input == "x":
            servo.stop()
            servo.command = "x"
            break

        if user_input in servo.directions:  # key: a, s, d, w
            servo.command = user_input
        else:
            try:
                servo.setTargetPos(int(user_input) // 360)
            except ValueError:
                print("Enter integer")
