import sys
import serial
import queue as Queue
import numpy as np
from PyQt5.QtWidgets import QMainWindow, QApplication, QTableWidgetItem
from PyQt5 import QtGui, QtCore, uic
from mainWindow import Ui_MainWindow
import enumSerialPorts


NUM_PULSETRAINS = 100


class PulseTrain():
    def __init__(self, ch0Mode=3, ch1Mode=3, frequency_hz=100, duration_sec=0.5):
        self.ch0Mode = ch0Mode
        self.ch1Mode = ch1Mode
        self.frequency_hz = frequency_hz
        self.duration_sec = duration_sec
        self.phases = np.zeros((10,3), dtype='int32')
        #self.phases = (np.random.rand(10,3)*100).astype(int) #, dtype='int32')


class SerialThread(QtCore.QThread):
    def __init__(self, portName):
        QtCore.QThread.__init__(self)
        self.portName = portName
        self.txq = Queue.Queue()
        self.running = False

    # In serial thread:
    def run(self):
        self.running = True
        self.con = serial.Serial(self.portName, 256000, timeout=0.1)
        self.con.flushInput()
        print(f"# Opening {self.portName}")

        while self.running:
            if not self.txq.empty():
                self.con.write(str.encode(self.txq.get())) # send string     
            s = self.con.read(self.con.in_waiting or 1)
            if s:
                print(s.decode().replace("\r",""), end='')

    def send(self, s):
        self.txq.put(s)


class AppWindow(QMainWindow):
    text_update = QtCore.pyqtSignal(str)
    pulseTrains = []

    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.serialThread = SerialThread("")

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.updateSerialPorts)
        self.timer.start(1000)

        self.ui.connectButton.clicked.connect(self.connectSerial)
        self.ui.disconnectButton.clicked.connect(self.disconnectSerial)
        self.ui.startButton.clicked.connect(self.startPulseTrain)
        self.ui.stopButton.clicked.connect(self.stopPulseTrain)

        self.updateButtonStates(False)

        self.ui.in0TriggerSpinBox.valueChanged.connect(self.setIn0Trigger) # update display
        self.ui.in1TriggerSpinBox.valueChanged.connect(self.setIn1Trigger) # update display

        self.ui.pulseTrainSpinBox.valueChanged.connect(self.updatePulseTrainSettings) # update display
        self.ui.phases.cellChanged.connect(self.updateInternalPulseTrains) # update display
        self.ui.ch0Mode.currentIndexChanged.connect(self.updateInternalPulseTrains) # update display
        self.ui.ch1Mode.currentIndexChanged.connect(self.updateInternalPulseTrains) # update display
        self.ui.frequency_hz.valueChanged.connect(self.updateInternalPulseTrains) # update display
        self.ui.duration_sec.valueChanged.connect(self.updateInternalPulseTrains) # update display
        self.ignoreChanges = False

        self.pulseTrains = [PulseTrain() for i in range(NUM_PULSETRAINS)]
        self.updatePulseTrainSettings()

        self.text_update.connect(self.appendText)
        sys.stdout = self
        self.show()

    def updateSerialPorts(self):
        self.ui.portName.clear()
        self.ui.portName.addItems(enumSerialPorts.enumSerialPorts())

    def setIn0Trigger(self):
        self.serialThread.send(f"R0,{self.ui.in0TriggerSpinBox.value()}\n")

    def setIn1Trigger(self):
        self.serialThread.send(f"R1,{self.ui.in1TriggerSpinBox.value()}\n")

    def updatePulseTrainSettings(self):
        i = self.ui.pulseTrainSpinBox.value()
        pt = self.pulseTrains[i]
        self.ignoreChanges = True
        self.ui.ch0Mode.setCurrentIndex(pt.ch0Mode)
        self.ui.ch1Mode.setCurrentIndex(pt.ch1Mode)
        self.ui.frequency_hz.setValue(pt.frequency_hz)
        self.ui.duration_sec.setValue(pt.duration_sec) 
        for ix, iy in np.ndindex(pt.phases.shape):
            self.ui.phases.setItem(ix, iy, QTableWidgetItem(str(pt.phases[ix, iy])))
        self.ignoreChanges = False

    def updateInternalPulseTrains(self):
        if not self.ignoreChanges:
            i = self.ui.pulseTrainSpinBox.value()
            self.pulseTrains[i].ch0Mode = self.ui.ch0Mode.currentIndex()
            self.pulseTrains[i].ch1Mode = self.ui.ch1Mode.currentIndex()
            self.pulseTrains[i].frequency_hz = self.ui.frequency_hz.value()
            self.pulseTrains[i].duration_sec = self.ui.duration_sec.value()

            for ix, iy in np.ndindex(self.pulseTrains[i].phases.shape):
                self.pulseTrains[i].phases[ix, iy] = int(self.ui.phases.item(ix, iy).text())

    def updateButtonStates(self, b):
            self.ui.connectButton.setEnabled(not b)
            self.ui.disconnectButton.setEnabled(b)
            self.ui.startButton.setEnabled(b)
            self.ui.stopButton.setEnabled(b)
            self.ui.in0TriggerSpinBox.setEnabled(b)
            self.ui.in1TriggerSpinBox.setEnabled(b)

    def connectSerial(self):
        try:
            print(f"# Connecting to port {self.ui.portName.currentText()}")
            self.serialThread = SerialThread(self.ui.portName.currentText())
            self.serialThread.start()
            self.updateButtonStates(True)
        except :
            print("# Failed to open port!")

    def disconnectSerial(self):
        print("# Disconnecting!")
        self.serialThread.running = False
        self.serialThread.wait()
        self.updateButtonStates(False)
        
    def startPulseTrain(self):
        pt = self.pulseTrains[self.ui.pulseTrainSpinBox.value()]

        buf = "S0,%d,%d,%d,%d;" % (pt.ch0Mode, pt.ch1Mode, int(1e6/pt.frequency_hz), int(1e6*pt.duration_sec))
        for i in range(pt.phases.shape[0]):
            if (pt.phases[i, 2] > 0):
                buf += "%d,%d,%d;" % (pt.phases[i,0], pt.phases[i,1], pt.phases[i,2])
        print(f"-> {buf}\n-> T0\n")
        self.serialThread.send(buf + "\nT0\n")
        
    def stopPulseTrain(self):
        print("# ending pulse train!")
        self.serialThread.send("T-1\n")

    def write(self, s):                      # Handle sys.stdout.write: update display
        self.text_update.emit(s)             # Send signal to synchronise call with main thread
 
    def flush(self):                            # Handle sys.stdout.flush: do nothing
        pass
 
    def appendText(self, text):                # Text display update handler
        cur = self.ui.serialOutputBrowser.textCursor()
        cur.movePosition(QtGui.QTextCursor.End) # Move cursor to end of text
        s = str(text)
        while s:
            head,sep,s = s.partition("\n")      # Split line at LF
            cur.insertText(head)                # Insert text at cursor
            if sep:                             # New line if LF
                cur.insertBlock()
        self.ui.serialOutputBrowser.setTextCursor(cur)         # Update visible cursor


if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = AppWindow()
    w.show()
    sys.exit(app.exec_())