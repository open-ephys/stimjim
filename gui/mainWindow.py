# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mainWindow.ui'
#
# Created by: PyQt5 UI code generator 5.9.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1100, 316)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MainWindow.sizePolicy().hasHeightForWidth())
        MainWindow.setSizePolicy(sizePolicy)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.formLayout_2 = QtWidgets.QFormLayout()
        self.formLayout_2.setFieldGrowthPolicy(QtWidgets.QFormLayout.ExpandingFieldsGrow)
        self.formLayout_2.setSpacing(6)
        self.formLayout_2.setObjectName("formLayout_2")
        self.label_9 = QtWidgets.QLabel(self.centralwidget)
        self.label_9.setObjectName("label_9")
        self.formLayout_2.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.label_9)
        self.portName = QtWidgets.QComboBox(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.portName.sizePolicy().hasHeightForWidth())
        self.portName.setSizePolicy(sizePolicy)
        self.portName.setMinimumSize(QtCore.QSize(100, 0))
        self.portName.setObjectName("portName")
        self.formLayout_2.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.portName)
        self.connectButton = QtWidgets.QPushButton(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.connectButton.sizePolicy().hasHeightForWidth())
        self.connectButton.setSizePolicy(sizePolicy)
        self.connectButton.setMinimumSize(QtCore.QSize(100, 0))
        self.connectButton.setObjectName("connectButton")
        self.formLayout_2.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.connectButton)
        self.disconnectButton = QtWidgets.QPushButton(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.disconnectButton.sizePolicy().hasHeightForWidth())
        self.disconnectButton.setSizePolicy(sizePolicy)
        self.disconnectButton.setMinimumSize(QtCore.QSize(100, 0))
        self.disconnectButton.setObjectName("disconnectButton")
        self.formLayout_2.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.disconnectButton)
        self.startButton = QtWidgets.QPushButton(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.startButton.sizePolicy().hasHeightForWidth())
        self.startButton.setSizePolicy(sizePolicy)
        self.startButton.setMinimumSize(QtCore.QSize(100, 0))
        self.startButton.setObjectName("startButton")
        self.formLayout_2.setWidget(4, QtWidgets.QFormLayout.FieldRole, self.startButton)
        self.stopButton = QtWidgets.QPushButton(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.stopButton.sizePolicy().hasHeightForWidth())
        self.stopButton.setSizePolicy(sizePolicy)
        self.stopButton.setMinimumSize(QtCore.QSize(100, 10))
        self.stopButton.setObjectName("stopButton")
        self.formLayout_2.setWidget(5, QtWidgets.QFormLayout.FieldRole, self.stopButton)
        self.in0TriggerSpinBox = QtWidgets.QSpinBox(self.centralwidget)
        self.in0TriggerSpinBox.setMinimumSize(QtCore.QSize(100, 10))
        self.in0TriggerSpinBox.setMinimum(-1)
        self.in0TriggerSpinBox.setProperty("value", -1)
        self.in0TriggerSpinBox.setObjectName("in0TriggerSpinBox")
        self.formLayout_2.setWidget(8, QtWidgets.QFormLayout.FieldRole, self.in0TriggerSpinBox)
        self.in1TriggerSpinBox = QtWidgets.QSpinBox(self.centralwidget)
        self.in1TriggerSpinBox.setMinimumSize(QtCore.QSize(100, 10))
        self.in1TriggerSpinBox.setMinimum(-1)
        self.in1TriggerSpinBox.setSingleStep(1)
        self.in1TriggerSpinBox.setProperty("value", -1)
        self.in1TriggerSpinBox.setObjectName("in1TriggerSpinBox")
        self.formLayout_2.setWidget(9, QtWidgets.QFormLayout.FieldRole, self.in1TriggerSpinBox)
        self.label_7 = QtWidgets.QLabel(self.centralwidget)
        self.label_7.setObjectName("label_7")
        self.formLayout_2.setWidget(8, QtWidgets.QFormLayout.LabelRole, self.label_7)
        self.label_8 = QtWidgets.QLabel(self.centralwidget)
        self.label_8.setObjectName("label_8")
        self.formLayout_2.setWidget(9, QtWidgets.QFormLayout.LabelRole, self.label_8)
        self.label_10 = QtWidgets.QLabel(self.centralwidget)
        self.label_10.setObjectName("label_10")
        self.formLayout_2.setWidget(7, QtWidgets.QFormLayout.FieldRole, self.label_10)
        self.line = QtWidgets.QFrame(self.centralwidget)
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.formLayout_2.setWidget(6, QtWidgets.QFormLayout.FieldRole, self.line)
        self.line_2 = QtWidgets.QFrame(self.centralwidget)
        self.line_2.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_2.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_2.setObjectName("line_2")
        self.formLayout_2.setWidget(3, QtWidgets.QFormLayout.FieldRole, self.line_2)
        self.horizontalLayout.addLayout(self.formLayout_2)
        self.formLayout = QtWidgets.QFormLayout()
        self.formLayout.setObjectName("formLayout")
        self.label_6 = QtWidgets.QLabel(self.centralwidget)
        self.label_6.setObjectName("label_6")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.label_6)
        self.pulseTrainSpinBox = QtWidgets.QSpinBox(self.centralwidget)
        self.pulseTrainSpinBox.setObjectName("pulseTrainSpinBox")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.pulseTrainSpinBox)
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setObjectName("label")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.label)
        self.ch0Mode = QtWidgets.QComboBox(self.centralwidget)
        self.ch0Mode.setObjectName("ch0Mode")
        self.ch0Mode.addItem("")
        self.ch0Mode.addItem("")
        self.ch0Mode.addItem("")
        self.ch0Mode.addItem("")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.ch0Mode)
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setObjectName("label_2")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.label_2)
        self.ch1Mode = QtWidgets.QComboBox(self.centralwidget)
        self.ch1Mode.setObjectName("ch1Mode")
        self.ch1Mode.addItem("")
        self.ch1Mode.addItem("")
        self.ch1Mode.addItem("")
        self.ch1Mode.addItem("")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.ch1Mode)
        self.label_3 = QtWidgets.QLabel(self.centralwidget)
        self.label_3.setObjectName("label_3")
        self.formLayout.setWidget(3, QtWidgets.QFormLayout.LabelRole, self.label_3)
        self.frequency_hz = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.frequency_hz.setDecimals(3)
        self.frequency_hz.setMinimum(0.001)
        self.frequency_hz.setMaximum(100000.0)
        self.frequency_hz.setProperty("value", 100.0)
        self.frequency_hz.setObjectName("frequency_hz")
        self.formLayout.setWidget(3, QtWidgets.QFormLayout.FieldRole, self.frequency_hz)
        self.label_4 = QtWidgets.QLabel(self.centralwidget)
        self.label_4.setObjectName("label_4")
        self.formLayout.setWidget(4, QtWidgets.QFormLayout.LabelRole, self.label_4)
        self.duration_sec = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.duration_sec.setDecimals(3)
        self.duration_sec.setMaximum(1000000.0)
        self.duration_sec.setProperty("value", 1.0)
        self.duration_sec.setObjectName("duration_sec")
        self.formLayout.setWidget(4, QtWidgets.QFormLayout.FieldRole, self.duration_sec)
        self.label_5 = QtWidgets.QLabel(self.centralwidget)
        self.label_5.setObjectName("label_5")
        self.formLayout.setWidget(5, QtWidgets.QFormLayout.LabelRole, self.label_5)
        self.phases = QtWidgets.QTableWidget(self.centralwidget)
        self.phases.setEnabled(True)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.phases.sizePolicy().hasHeightForWidth())
        self.phases.setSizePolicy(sizePolicy)
        self.phases.setMinimumSize(QtCore.QSize(300, 100))
        font = QtGui.QFont()
        font.setPointSize(8)
        self.phases.setFont(font)
        self.phases.setInputMethodHints(QtCore.Qt.ImhDigitsOnly)
        self.phases.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.phases.setLineWidth(1)
        self.phases.setAutoScrollMargin(16)
        self.phases.setAlternatingRowColors(True)
        self.phases.setGridStyle(QtCore.Qt.SolidLine)
        self.phases.setRowCount(10)
        self.phases.setColumnCount(3)
        self.phases.setObjectName("phases")
        item = QtWidgets.QTableWidgetItem()
        self.phases.setHorizontalHeaderItem(0, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setHorizontalHeaderItem(1, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setHorizontalHeaderItem(2, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setItem(0, 0, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setItem(0, 1, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setItem(0, 2, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setItem(1, 0, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setItem(1, 1, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setItem(1, 2, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setItem(2, 0, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setItem(2, 1, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setItem(2, 2, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setItem(3, 0, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setItem(3, 1, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setItem(3, 2, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setItem(4, 0, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setItem(4, 1, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setItem(4, 2, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setItem(5, 0, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setItem(5, 1, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setItem(5, 2, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setItem(6, 0, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setItem(6, 1, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setItem(6, 2, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setItem(7, 0, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setItem(7, 1, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setItem(7, 2, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setItem(8, 0, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setItem(8, 1, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setItem(8, 2, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setItem(9, 0, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setItem(9, 1, item)
        item = QtWidgets.QTableWidgetItem()
        self.phases.setItem(9, 2, item)
        self.phases.horizontalHeader().setVisible(True)
        self.phases.horizontalHeader().setDefaultSectionSize(80)
        self.phases.horizontalHeader().setMinimumSectionSize(60)
        self.phases.verticalHeader().setDefaultSectionSize(20)
        self.phases.verticalHeader().setMinimumSectionSize(20)
        self.formLayout.setWidget(5, QtWidgets.QFormLayout.FieldRole, self.phases)
        self.horizontalLayout.addLayout(self.formLayout)
        self.serialOutputBrowser = QtWidgets.QTextBrowser(self.centralwidget)
        self.serialOutputBrowser.setMaximumSize(QtCore.QSize(500, 16777215))
        font = QtGui.QFont()
        font.setFamily("Courier")
        self.serialOutputBrowser.setFont(font)
        self.serialOutputBrowser.setAcceptRichText(False)
        self.serialOutputBrowser.setObjectName("serialOutputBrowser")
        self.horizontalLayout.addWidget(self.serialOutputBrowser)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1100, 21))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.ch0Mode.setCurrentIndex(3)
        self.ch1Mode.setCurrentIndex(3)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Stimjim control"))
        self.label_9.setText(_translate("MainWindow", "Port"))
        self.connectButton.setText(_translate("MainWindow", "Connect"))
        self.disconnectButton.setText(_translate("MainWindow", "Disconnect"))
        self.startButton.setText(_translate("MainWindow", "Start PulseTrain"))
        self.stopButton.setText(_translate("MainWindow", "Stop PulseTrain"))
        self.label_7.setText(_translate("MainWindow", "Input 0:"))
        self.label_8.setText(_translate("MainWindow", "Input 1:"))
        self.label_10.setText(_translate("MainWindow", "Triggers"))
        self.label_6.setText(_translate("MainWindow", "Settings #:"))
        self.label.setText(_translate("MainWindow", "Ch0 mode"))
        self.ch0Mode.setItemText(0, _translate("MainWindow", "Voltage"))
        self.ch0Mode.setItemText(1, _translate("MainWindow", "Current"))
        self.ch0Mode.setItemText(2, _translate("MainWindow", "Disconnected"))
        self.ch0Mode.setItemText(3, _translate("MainWindow", "Grounded (off)"))
        self.label_2.setText(_translate("MainWindow", "Ch1 mode"))
        self.ch1Mode.setItemText(0, _translate("MainWindow", "Voltage"))
        self.ch1Mode.setItemText(1, _translate("MainWindow", "Current"))
        self.ch1Mode.setItemText(2, _translate("MainWindow", "Disconnected"))
        self.ch1Mode.setItemText(3, _translate("MainWindow", "Grounded (off)"))
        self.label_3.setText(_translate("MainWindow", "Frequency (Hz)"))
        self.label_4.setText(_translate("MainWindow", "Duration (sec)"))
        self.label_5.setText(_translate("MainWindow", "Pulse phases:"))
        item = self.phases.horizontalHeaderItem(0)
        item.setText(_translate("MainWindow", "Ch0 Ampl."))
        item = self.phases.horizontalHeaderItem(1)
        item.setText(_translate("MainWindow", "Ch1 Ampl."))
        item = self.phases.horizontalHeaderItem(2)
        item.setText(_translate("MainWindow", "Duration (us)"))
        __sortingEnabled = self.phases.isSortingEnabled()
        self.phases.setSortingEnabled(False)
        item = self.phases.item(0, 0)
        item.setText(_translate("MainWindow", "100"))
        item = self.phases.item(0, 1)
        item.setText(_translate("MainWindow", "100"))
        item = self.phases.item(0, 2)
        item.setText(_translate("MainWindow", "100"))
        item = self.phases.item(1, 0)
        item.setText(_translate("MainWindow", "0"))
        item = self.phases.item(1, 1)
        item.setText(_translate("MainWindow", "0"))
        item = self.phases.item(1, 2)
        item.setText(_translate("MainWindow", "0"))
        item = self.phases.item(2, 0)
        item.setText(_translate("MainWindow", "0"))
        item = self.phases.item(2, 1)
        item.setText(_translate("MainWindow", "0"))
        item = self.phases.item(2, 2)
        item.setText(_translate("MainWindow", "0"))
        item = self.phases.item(3, 0)
        item.setText(_translate("MainWindow", "0"))
        item = self.phases.item(3, 1)
        item.setText(_translate("MainWindow", "0"))
        item = self.phases.item(3, 2)
        item.setText(_translate("MainWindow", "0"))
        item = self.phases.item(4, 0)
        item.setText(_translate("MainWindow", "0"))
        item = self.phases.item(4, 1)
        item.setText(_translate("MainWindow", "0"))
        item = self.phases.item(4, 2)
        item.setText(_translate("MainWindow", "0"))
        item = self.phases.item(5, 0)
        item.setText(_translate("MainWindow", "0"))
        item = self.phases.item(5, 1)
        item.setText(_translate("MainWindow", "0"))
        item = self.phases.item(5, 2)
        item.setText(_translate("MainWindow", "0"))
        item = self.phases.item(6, 0)
        item.setText(_translate("MainWindow", "0"))
        item = self.phases.item(6, 1)
        item.setText(_translate("MainWindow", "0"))
        item = self.phases.item(6, 2)
        item.setText(_translate("MainWindow", "0"))
        item = self.phases.item(7, 0)
        item.setText(_translate("MainWindow", "0"))
        item = self.phases.item(7, 1)
        item.setText(_translate("MainWindow", "0"))
        item = self.phases.item(7, 2)
        item.setText(_translate("MainWindow", "0"))
        item = self.phases.item(8, 0)
        item.setText(_translate("MainWindow", "0"))
        item = self.phases.item(8, 1)
        item.setText(_translate("MainWindow", "0"))
        item = self.phases.item(8, 2)
        item.setText(_translate("MainWindow", "0"))
        item = self.phases.item(9, 0)
        item.setText(_translate("MainWindow", "0"))
        item = self.phases.item(9, 1)
        item.setText(_translate("MainWindow", "0"))
        item = self.phases.item(9, 2)
        item.setText(_translate("MainWindow", "0"))
        self.phases.setSortingEnabled(__sortingEnabled)
        self.serialOutputBrowser.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Courier\'; font-size:10pt; font-weight:400; font-style:normal;\">\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))

