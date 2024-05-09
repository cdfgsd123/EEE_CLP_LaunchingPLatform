# communicate
import threading
import time

import serial


class ServoControl:
    def __init__(self, targetPos):
        self.receivedString = ""
        self.pos = 0
        self.targetPos = targetPos
        self.command = None

        self.maxSpeed = 0xCC
        self.minSpeed = 0x90

        self.RightString = [0x60, 0x0B, 0xFF, 0xFF, 0xFF, 0xFF]
        self.LeftString = [0x60, 0x0B, 0x00, 0x00, 0x00, 0x00]

        self.RightStringMax = [0x60, 0x0B, 0xFF, 0xFF, 0xFF, 0xFF]
        self.LeftStringMax = [0x60, 0x0B, 0x00, 0x00, 0x00, 0x00]
        self.stopString = [0x60, 0x0B, 0x80, 0x80, 0x80, 0x80]

        self.ser = serial.Serial(port="COM6", baudrate=115200, timeout=0.01)
        self.position = 0
        self.directions = ["a", "d", "s", "w"]

    def setTargetPos(self, input):
        self.targetPos = input

    def turnLeft(self, speed):
        # assume move servo 1 and 3
        self.LeftString[2] = self.maxSpeed - speed  # servo 1
        self.LeftString[4] = self.maxSpeed - speed  # servo 3
        self.ser.write(serial.to_bytes(self.LeftString))

    def turnRight(self, speed):
        # assume move servo 1 and 3
        self.RightString[2] = speed  # servo 1
        self.RightString[4] = speed  # servo 3
        self.ser.write(serial.to_bytes(self.RightString))

    def stop(self):
        self.ser.write(serial.to_bytes(self.stopString))

    def showOutput(self, targetPos, command=None):

        receivedString = self.ser.readline()
        if len(receivedString) >= 25:
            self.pos = int(int(receivedString[25]) / 255 * 360)  # current pos
            # print(receivedString)

            # print("Degree = ", pos)
            self.updatePos(command, targetPos)
            process = True

            self.pos, process = self.dstop(command, self.pos, self.targetPos)

            # print()
        return int(targetPos)

    def dstop(self, command, pos, targetPos):
        # print("pos:", pos, " targetPos: ", targetPos)
        if abs(targetPos - pos) <= 5:
            self.stop()
            return pos, False
        # asssume max_dispalcement 20
        distance = abs(targetPos - pos)
        # print("displacement", displacement)
        if distance >= 20:
            distance = 20
        range
        speed = int(
            (self.maxSpeed - self.minSpeed) * (distance / 20) + self.minSpeed,
        )
        # print("speed", speed)

        temp = targetPos - pos
        if temp > 0 and abs(temp) < 180:
            self.turnLeft(speed)
        elif temp > 0 and abs(temp) >= 180:
            self.turnRight(speed)
        elif temp < 0 and abs(temp) < 180:
            self.turnRight(speed)
        else:
            self.turnLeft(speed)
        return pos, True

    def updatePos(self, command, targetPos):
        # + or minus depend on the putting angle
        if command == "d":
            targetPos -= 12  # right turn 12 degree
        elif command == "a":
            targetPos += 12  # left turn 12 degree

        if targetPos >= 360:
            targetPos -= 360
        elif targetPos < 0:
            targetPos += 360
        self.setTargetPos(int(targetPos))

    def main(self):
        while self.start:
            if self.command is None:
                self.showOutput(self.targetPos)
            else:
                self.showOutput(self.targetPos, self.command)
                self.command = None


if __name__ == "__main__":
    servo = ServoControl(None)
    job = threading.Thread(target=servo.main)
    job.start()
    while True:
        user_input = input("Please enter desired degree: ")
        if user_input == "exit":
            servo.stop()
            break
        if user_input in servo.directions:  # key: a, s, d, w
            servo.command = user_input
        else:
            try:
                servo.setTargetPos(int(user_input))
            except ValueError:
                print("Enter integer")
