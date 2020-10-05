
import sys
import serial
import queue as Queue
from PyQt5.QtWidgets import QMainWindow, QApplication
from PyQt5 import QtGui, QtCore, uic
from mainWindow import Ui_MainWindow
import enumSerialPorts


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
    
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        
        self.serialThread = SerialThread("")
        
        self.ui.portName.addItems(enumSerialPorts.enumSerialPorts())
        self.ui.connectButton.clicked.connect(self.connectSerial)
        self.ui.disconnectButton.clicked.connect(self.disconnectSerial)
        self.ui.startButton.clicked.connect(self.startPulseTrain)
        self.ui.stopButton.clicked.connect(self.stopPulseTrain)
        
        self.updateButtonStates(False)
        #self.ui.phase.currentIndexChanged.connect(lambda : print("shit")) # update display
        #self.ui.phaseDuration.valueChanged.connect(lambda : print("dur changed")) # save params
        #self.ui.ch0Amp.valueChanged.connect(lambda : print("ch0 changed")) # save params
        #self.ui.ch1Amp.valueChanged.connect(lambda : print("ch1 changed")) # save params
        
        self.ui.in0TriggerSpinBox.valueChanged.connect(self.setIn0Trigger) # update display
        self.ui.in1TriggerSpinBox.valueChanged.connect(self.setIn1Trigger) # update display

        self.text_update.connect(self.appendText)
        sys.stdout = self    
        self.show()
    
    def setIn0Trigger(self):
        self.serialThread.send(f"R0,{self.ui.in0TriggerSpinBox.value()}\n")

    def setIn1Trigger(self):
        self.serialThread.send(f"R1,{self.ui.in1TriggerSpinBox.value()}\n")

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
        buf = "S0,%d,%d,%d,%d;" % (self.ui.ch0Mode.currentIndex(), self.ui.ch1Mode.currentIndex(), \
                                   int(1e6/self.ui.frequency.value()), int(1e6*self.ui.duration.value()))
        
        for i in range(self.ui.phases.rowCount()):
            amp0 = int(self.ui.phases.item(i,0).text())
            amp1 = int(self.ui.phases.item(i,1).text())
            phaseDuration = int(self.ui.phases.item(i,2).text())
            if (phaseDuration > 0):
                buf += "%d,%d,%d;" % (amp0, amp1, phaseDuration)
        buf += "\nT0\n"
        print(f"-> {buf}")
        self.serialThread.send(buf)
        
    def stopPulseTrain(self):
        print("# ending pulse train!")
        
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
         

app = QApplication(sys.argv)
w = AppWindow()
w.show()
sys.exit(app.exec_())