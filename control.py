# control with wireless communication
# function declaration
# https://docs.google.com/spreadsheets/d/1Wc7fULlIymNZdzb6pMLg2qQt56mEpI93gTLk9oNqMaI/edit#gid=0
import socket
import threading


class ServoControl:
    def __init__(self, targetPos=None, calibrate=0):
        self.calibrateFileName = "calibrateFile.txt"
        self.calibrate = calibrate
        self.receivedString = ""
        self.pos = 0  # current position in degree
        if targetPos is not None:
            self.targetPos = targetPos  # desired position
        self.command = None  # control command
        self.controls = ["a", "d", "s", "w", "calibrate", "q"]
        # acceptable angle error
        self.lowThreshold = 5  # acceptable angle error
        self.highThreshold = 20  # for calcuating speed of rotation
        self.thresholdRange = self.highThreshold - self.lowThreshold
        self.angleDivision = 360 // 30
        # speed of servo (self-defined)
        self.maxSpeed = 0xC0  # larger than 0x80 and assume
        self.minSpeed = 0x90  # larger than 0x80
        self.speedRange = self.maxSpeed - self.minSpeed

        # 6 byte motion control signal to ESP32
        self.LeftString = [0x60, 0x0B, 0xFF, 0xFF, 0xFF, 0xFF]
        self.RightString = [0x60, 0x0B, 0x00, 0x00, 0x00, 0x00]
        # current pos [0x60, 0x0B, 0x00, 0x00, 0x00, 0x00]

        self.backwardString = [0x60, 0x0B, 0xA0, 0x60, 0x60, 0xA0]
        self.forwardString = [0x60, 0x0B, 0x60, 0xA0, 0xA0, 0x60]
        self.continueString = [0x60, 0x0A, 0x80, 0x80, 0x80, 0x80]
        self.stopString = [0x60, 0x0B, 0x80, 0x80, 0x80, 0x80]

        self.port = 8088

        # Prompt for IP address at the start
        self.ip = "192.168.10.8"

        # Read the calibrate parameter for text file
        readInteger = self.readCalibraeFromFile(self.calibrateFileName)
        if readInteger is not None:
            print(f"Integer read from the file: {readInteger}")
            self.calibrate = readInteger
        else:  # if fail, store 0 as calibrating parameter
            print("Failed to read integer from the file. Writing a new file.")
            self.writeCalibrateToFile(0, self.calibrateFileName)
            readInteger = self.readCalibraeFromFile(self.calibrateFileName)
            if readInteger is not None:
                print(f"Integer read from the new file: {readInteger}")
            self.calibarte = 0
        # initial the position
        self.initPos = False
        initCount = 0
        while initCount < 3:
            success, _ = self.readImuAndControl(
                self.ip, self.port, self.continueString, isCalibrating=False
            )
            if success:
                initCount += 1

        self.initPos = True
        if targetPos is None:
            self.targetPos = self.pos
        print("finish init")

    def writeCalibrateToFile(self, integer, filename):
        with open(filename, "w") as file:
            file.write(str(integer))
        print(f"Integer {integer} has been written to {filename}")

    def readCalibraeFromFile(self, filename):
        try:
            with open(filename, "r") as file:
                return int(file.read())
        except FileNotFoundError:
            print(f"File {filename} not found. Creating a new file.")
            return None
        except ValueError:
            print(f"File {filename} does not contain a valid integer.")
            return None

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
        degree = self.pos % 360
        if (360 - self.angleDivision // 2) <= degree <= 359 or 0 <= degree < (
            self.angleDivision // 2
        ):
            return 1
        else:
            return int((degree - self.angleDivision // 2) // 12) + 2

    def setTargetPosBySlotNum(self, slotNum):
        if slotNum == 1:
            self.targetPos = 0
        else:
            self.targetPos = (slotNum - 2) * 12 + 12
        return self.targetPos

    # 4 directions motion and stop of 3/6 configuration
    def turnLeft(self, speed):
        self.LeftString[2] = speed  # servo 1, clockwise
        self.LeftString[3] = speed  # servo 2, clockwise

        self.LeftString[4] = 0xFF - speed  # servo 3, anti-clockwise
        self.LeftString[5] = 0xFF - speed  # servo 4, anti-clockwise
        self.readImuAndControl(
            self.ip,
            self.port,
            self.LeftString,
            isCalibrating=False,
        )
        return self.pos

    def turnRight(self, speed):
        self.RightString[2] = 0xFF - speed  # servo 1, anti-clockwise
        self.RightString[3] = 0xFF - speed  # servo 2, anti-clockwise

        self.RightString[4] = speed  # servo 3, clockwise
        self.RightString[5] = speed  # servo 4, clockwise
        self.readImuAndControl(
            self.ip,
            self.port,
            self.RightString,
            isCalibrating=False,
        )
        return self.pos

    def stop(self):
        self.readImuAndControl(
            self.ip,
            self.port,
            self.stopString,
            isCalibrating=False,
        )
        return self.pos

    def goForward(self):
        self.readImuAndControl(
            self.ip,
            self.port,
            self.forwardString,
            isCalibrating=False,
        )

    def goBack(self):
        self.readImuAndControl(
            self.ip,
            self.port,
            self.backwardString,
            isCalibrating=False,
        )

    def continuePrev(self):
        self.readImuAndControl(
            self.ip,
            self.port,
            self.continueString,
            isCalibrating=False,
        )

    # when isCalibrating == True, it means calibrating angle.
    def readImuAndControl(self, ip, port, hex_array, isCalibrating=False):
        print("try1")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        message = bytes(hex_array)
        try:
            sock.sendto(message, (ip, port))
            response, server = sock.recvfrom(26)
            response_array = [f"{byte:02X}" for byte in response]
            print("trying")

            if len(response_array) >= 26:
                print("receive imu pos successfully")
                value = int(response_array[25], 16)  # Convert hex str to int
                tempPos = int((value / 255) * 360)
                if isCalibrating is False:
                    tempPos -= self.calibrate
                if tempPos >= 360:
                    tempPos -= 360
                elif tempPos < 0:
                    tempPos += 360
                if isCalibrating is False:
                    self.pos = tempPos
                return True, tempPos
            else:
                return False, 0
        except socket.error as e:
            print(f"Socket error: {e}")
            return False, 0
        finally:
            sock.close()

    # show information
    def printDetails(self):
        print("Degree = ", self.pos)
        print("targetPos = ", self.targetPos)
        print("getSpeedDisplay: ", self.getSpeedDisplay())
        print()

    # show output and update logic
    def process(self, command=None) -> int:
        self.command = command
        success, _ = self.readImuAndControl(
            self.ip,
            self.port,
            self.continueString,
            isCalibrating=False,
        )
        if success:
            self.updateTargetPos()
            stopped = self.dstop()
            print("stopped:", stopped)
            self.printDetails()
        return int(self.pos)

    # determine move or stop
    def dstop(self) -> bool:
        # print("pos:", pos, " targetPos: ", targetPos)
        if self.command == "w":
            self.goForward()
            return False
        if self.command == "s":
            self.goBack()
            return False
        tempDist = abs(self.targetPos - self.pos)
        print("tempDist", tempDist)
        if tempDist >= 180:
            tempDist = 360 - tempDist

        if tempDist <= self.lowThreshold:
            self.stop()
            return True

        speed = self.getSpeed()
        print("speed: ", speed)
        diff = self.targetPos - self.pos
        if diff < 0 and abs(diff) > 180:
            self.turnRight(speed)
        elif diff < 0 and abs(diff) <= 180:
            self.turnLeft(speed)
        elif diff > 0 and abs(diff) > 180:
            self.turnLeft(speed)
        else:
            self.turnRight(speed)
        # if self.targetPos <= 180 and self.pos > 180:
        #     self.turnRight(speed)
        # elif self.targetPos > 180 and self.pos <= 180:
        #     self.turnLeft(speed)
        # elif self.targetPos > self.pos:
        #     self.turnRight(speed)
        # else:
        #     self.turnLeft(speed)
        return False

    # update target position
    def updateTargetPos(self):
        # + or minus depend on the putting angle
        if self.command == "d":
            self.targetPos += 12  # right turn 12 degree
        elif self.command == "a":
            self.targetPos -= 12  # left turn 12 degree

        if self.targetPos >= 360:
            self.targetPos -= 360
        elif self.targetPos < 0:
            self.targetPos += 360

    # return a value between 0 to 1 to represent servo speed(1: max speed)
    def getSpeedDisplay(self) -> float:
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
    def getSpeed(self) -> int:
        fraction = self.getSpeedDisplay()
        if fraction == 0:
            return 0
        else:
            speed = int(self.speedRange * fraction + self.minSpeed)
            return speed

    # calibrate the position of
    # set the current position as 0 degree, target position reset

    # calibration only last until the program is ended
    def calibrateAngle(self):
        # initial the position
        prevPos, tempPos, tempAngleDiff = 0, 0, 0
        self.stop()
        self.initPos = False
        calibrateCount = 0
        # tempAngleDiff <= 3: the platform approximately stopped
        while calibrateCount < 5 or tempAngleDiff > 3:
            success, tempPos = self.readImuAndControl(
                self.ip, self.port, self.continueString, isCalibrating=True
            )
            if success:
                calibrateCount += 1
                if calibrateCount >= 5:
                    tempAngleDiff = abs(prevPos - tempPos)
                    if tempAngleDiff > 180:
                        tempAngleDiff = 360 - tempAngleDiff
                    if tempAngleDiff <= 3:
                        self.initPos = True
                        self.calibrate = tempPos
                        # store calibrate parameter to file
                        self.writeCalibrateToFile(
                            tempPos,
                            self.calibrateFileName,
                        )
                        self.pos = 0
                        return self.pos
                prevPos = tempPos
        return self.pos

    # main function
    # if command is given, we need "self.coomand = None"
    # to clear the command
    def main(self):
        while not self.initPos:
            pass
        while True:
            print(self.command)
            if self.command is None:
                _ = self.process()  # out:current pos with offset

            elif self.command == "calibrate":
                _ = self.calibrateAngle()  # out:current pos with offset
                self.command = None
            elif self.command == "x":
                return
            elif self.command == "q":
                # stop the platform from moving forward or backward only
                self.stop()
                self.command = None
            elif self.command == "w" or "s":
                _ = self.process(self.command)  # out:current pos with offset
            else:
                # input: a, s
                _ = self.process(self.command)  # out:current pos with offset
                self.command = None


if __name__ == "__main__":
    servo = ServoControl()
    job = threading.Thread(target=servo.main)
    job.daemon = True
    job.start()

    while True:
        user_input = input("Please enter desired degree: ")
        if user_input == "x":  # end the program
            _ = servo.stop()  # out:current pos with offset
            servo.command = "x"
            break
        if user_input in servo.controls:  # input: a, s, d, w, calibrate, q
            servo.command = user_input
        else:
            try:
                servo.setTargetPos(int(user_input))
            except ValueError:
                print("Enter integer")
