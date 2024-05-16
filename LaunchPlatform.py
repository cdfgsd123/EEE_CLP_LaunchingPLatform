import sys
from PyQt6.QtWidgets import QApplication,QWidget
from PyQt6 import uic
from functools import partial
import os
from LaunchPlatform_ViewModel import ViewModel


class LaunchPlatformApp(QWidget):
    def __init__(self):
        super().__init__()
        self.ui : QWidget = uic.loadUi("LaunchPlatform.ui",self)
        # Keep all the logic in viewModel,
        # Seperate UI and logic
        # Google MVVM for more information
        self.viewModel = ViewModel()
        self.initButton()
        self.initDial()

    def initLineEdit(self):
        self.ui.Position.setText(self.dial.value)

    def initButton(self):
        self.btn_enter.clicked.connect(self.GoToPosition)
        self.NextBTN.clicked.connect(partial(self.rotate_dial, 1))
        self.PreviousBTN.clicked.connect(partial(self.rotate_dial, -1))

    def initDial(self):
        # Connect the button click signal to the dial rotation slot
        self.dial.valueChanged.connect(self.sliderMoved)

    def rotate_dial(self, change):
        # Increment dial by 12 degrees, wrap around if exceeds 360
        trueAngle = (self.dial.value() + (12 * change)) % 360
        self.ui.Position.setText(str(trueAngle))
        self.dial.setValue(trueAngle)

    def GoToPosition(self):
        # Go to the desired position at self.ui.Position.text()
        try:
            value = int(self.ui.Position.text())
            trueAngle = value % 360
            print(trueAngle)
            self.dial.setValue(trueAngle)
            # Send Commanf to rotate the launchPlatform
            self.viewModel.rotateLaunchPlatFormToAngle(trueAngle)
        except ValueError:
            print("Please Enter an integer")


    def sliderMoved(self):
        print("Dial value = %i" % (self.dial.value()))




def main():
    # app = QApplication(sys.argv)
    # ex = ExampleApp()
    app = QApplication(sys.argv)
    window = LaunchPlatformApp()
    window.show()
    if app.exec():
        os._exit(0)


if __name__ == '__main__':
    main()