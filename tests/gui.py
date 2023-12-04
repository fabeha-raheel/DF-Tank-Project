# Ref: https://github.com/lim147/Weather-Radar-GUI

import sys
import os
from PyQt5.QtWidgets import QApplication,QWidget,QMainWindow,QFrame,QAction,qApp,QLabel,QWidget,QHBoxLayout,QVBoxLayout,QPushButton,QFileDialog
from PyQt5.QtGui import QIcon,QPainter,QBrush,QPen,QPolygon, QColor, QFont
from PyQt5.QtCore import Qt, QPointF,QLineF

from PyQt5.QtCore import Qt
import struct
class Window(QMainWindow,QWidget):
    def __init__(self):
        super().__init__()

        #Indicate the size of the window
        self.title = "Weather Radar Interface"
        self.top = 50
        self.left = 20
        self.width = 1160
        self.height = 720


        self.frame_one()
        self.frame_two()
        self.InitWindow()



    def InitWindow(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.top,self.left,self.width,self.height)

        self.show()


    def frame_one(self):
        frame1 = QFrame(self)
        frame1.setStyleSheet("QWidget { background-color: black } ")
        frame1.setGeometry(760, 10, 370, 295)

        self.text()
        self.button()


    def frame_two(self):
        self.remark()


    #The text block:
    def text(self):
        t1 = QLabel(self)
        t1.setText("Name: Strength")
        t1.setStyleSheet("QLabel {color: white}")
        t1.resize(350,20)
        t1.move(820, 20)

        t2 = QLabel(self)
        t2.setText("Location: LiQuan")
        t2.setStyleSheet("QLabel {color: white}")
        t2.resize(350, 20)
        t2.move(803, 45)

        t3 = QLabel(self)
        t3.setText("Longitude: 108.45700")
        t3.setStyleSheet("QLabel {color: white}")
        t3.resize(350, 20)
        t3.move(793, 70)

        t4 = QLabel(self)
        t4.setText("Magnitude: 34.48600")
        t4.setStyleSheet("QLabel {color: white}")
        t4.resize(350, 20)
        t4.move(790, 95)

        t5 = QLabel(self)
        t5.setText("Height: 506.00")
        t5.setStyleSheet("QLabel {color: white}")
        t5.resize(350, 20)
        t5.move(817, 120)

        t6 = QLabel(self)
        t6.setText("Start Time: 2017-10-30 14:44:2")
        t6.setStyleSheet("QLabel {color: white}")
        t6.resize(350, 20)
        t6.move(794, 145)

        t7 = QLabel(self)
        t7.setText("End Time: 2017-10-30 14:40:0")
        t7.setStyleSheet("QLabel {color: white}")
        t7.resize(350, 20)
        t7.move(802, 170)

        t8 = QLabel(self)
        t8.setText("Range: 120.0km")
        t8.setStyleSheet("QLabel {color: white}")
        t8.resize(350, 20)
        t8.move(820, 195)

        t9 = QLabel(self)
        t9.setText("Resolution: 4,000km")
        t9.setStyleSheet("QLabel {color: white}")
        t9.resize(350, 20)
        t9.move(790, 220)

    def button(self):
        b = QPushButton(self)
        b.setText("  Choose File  ")
        b.move(820, 255)

        b.clicked.connect(self.openFiles)

    def openFiles(self):
        fileName = QFileDialog.getOpenFileName(self, 'open file')
        with open(fileName[0],'rb') as f:
            byBuf = f.read(4)
            print(byBuf.decode('utf-8'))

            byVersion = f.read(4)
            fVal = struct.unpack('<f', byVersion)[0]
            print(fVal)

            byLength = f.read(4)
            fLen = struct.unpack('<f', byLength)[0]
            print(fLen)

            country = f.read(30)
            print(country.decode('utf-8'))



    #Draw the image of indicators:
    def paintEvent(self,event):
        qp = QPainter()
        qp.begin(self)
        self.drawFrame2(qp)
        self.drawRadar(qp)
        qp.end()

    #The colorful indicators:
    def drawFrame2(self,qp):
        qp.setBrush(QBrush(QColor(0,0,0),Qt.SolidPattern))
        qp.drawRect(760, 320, 370, 390)


        qp.setBrush(QBrush(QColor(128,255,255),Qt.SolidPattern))
        qp.drawRect(910,650,60,20)

        qp.setBrush(QBrush(QColor(77,219,255),Qt.SolidPattern))
        qp.drawRect(910,630,60,20)

        qp.setBrush(QBrush(QColor(0,102,255), Qt.SolidPattern))
        qp.drawRect(910, 610, 60, 20)

        qp.setBrush(QBrush(QColor(179,255,102), Qt.SolidPattern))
        qp.drawRect(910, 590, 60, 20)

        qp.setBrush(QBrush(QColor(0, 255, 0), Qt.SolidPattern))
        qp.drawRect(910, 570, 60, 20)

        qp.setBrush(QBrush(QColor(0, 179, 89), Qt.SolidPattern))
        qp.drawRect(910, 550, 60, 20)

        qp.setBrush(QBrush(QColor(255, 255, 153), Qt.SolidPattern))
        qp.drawRect(910, 530, 60, 20)

        qp.setBrush(QBrush(QColor(255, 255, 102), Qt.SolidPattern))
        qp.drawRect(910, 510, 60, 20)

        qp.setBrush(QBrush(QColor(255, 219, 77), Qt.SolidPattern))
        qp.drawRect(910, 490, 60, 20)

        qp.setBrush(QBrush(QColor(255, 112, 77), Qt.SolidPattern))
        qp.drawRect(910, 470, 60, 20)

        qp.setBrush(QBrush(QColor(255, 0,0), Qt.SolidPattern))
        qp.drawRect(910, 450, 60, 20)

        qp.setBrush(QBrush(QColor(153,0,0), Qt.SolidPattern))
        qp.drawRect(910, 430, 60, 20)

        qp.setBrush(QBrush(QColor(255, 153, 255), Qt.SolidPattern))
        qp.drawRect(910, 410, 60, 20)

        qp.setBrush(QBrush(QColor(204,51,255), Qt.SolidPattern))
        qp.drawRect(910, 390, 60, 20)

        qp.setBrush(QBrush(QColor(255,255,255), Qt.SolidPattern))
        qp.drawRect(910, 370, 60, 20)


    #Draw the longitude and latitude graticules:
    def drawRadar(selfself,qp):
        #set frame:
        qp.setBrush(QBrush(QColor(0,0,0),Qt.SolidPattern))
        qp.drawRect(20, 10, 700, 700)

        pen_bold = QPen(QColor(102,204,255),0.8,Qt.SolidLine)

        pen = QPen(QColor(102,204,255),0.8,Qt.DashDotLine)

        pen_hide = QPen(QColor(0, 0, 0), 1.5, Qt.SolidLine)

        #add circle:
        qp.setPen(pen)
        qp.drawEllipse(QPointF(370, 360), 347, 347)


        #add scales:
        for i in range(1,360):
            if i%30 == 0:
                pass

            else:
                if i%10 == 0:
                    scaleLine = QLineF()
                    scaleLine.setP1(QPointF(370,360))
                    scaleLine.setAngle(i)
                    scaleLine.setLength(347)
                    qp.setPen(pen_bold)
                    qp.drawLine(scaleLine)

                    hideLine = QLineF()
                    hideLine.setP1(QPointF(370, 360))
                    hideLine.setAngle(i)
                    hideLine.setLength(337)
                    qp.setPen(pen_hide)
                    qp.drawLine(hideLine)





                else:
                    scaleLine = QLineF()
                    scaleLine.setP1(QPointF(370, 360))
                    scaleLine.setAngle(i)
                    scaleLine.setLength(347)
                    qp.setPen(pen_bold)
                    qp.drawLine(scaleLine)

                    hideLine = QLineF()
                    hideLine.setP1(QPointF(370, 360))
                    hideLine.setAngle(i)
                    hideLine.setLength(342)
                    qp.setPen(pen_hide)
                    qp.drawLine(hideLine)

        #add other circles:
        qp.setPen(pen)
        qp.drawEllipse(QPointF(370, 360), 277, 277)

        qp.setPen(pen)
        qp.drawEllipse(QPointF(370, 360), 207, 207)

        qp.setPen(pen)
        qp.drawEllipse(QPointF(370, 360), 137, 137)

        qp.setPen(pen)
        qp.drawEllipse(QPointF(370, 360), 67, 67)

        #add lines:
        qp.setPen(pen_bold)
        qp.drawLine(20, 360, 720, 360)

        qp.setPen(pen_bold)
        qp.drawLine(370, 10, 370, 710)

        angleLine1 = QLineF()
        angleLine1.setP1(QPointF(370, 360))
        angleLine1.setAngle(30)
        angleLine1.setLength(347)
        qp.setPen(pen)
        qp.drawLine(angleLine1)

        angleLine2 = QLineF()
        angleLine2.setP1(QPointF(370, 360))
        angleLine2.setAngle(60)
        angleLine2.setLength(347)
        qp.setPen(pen)
        qp.drawLine(angleLine2)

        angleLine3 = QLineF()
        angleLine3.setP1(QPointF(370, 360))
        angleLine3.setAngle(120)
        angleLine3.setLength(347)
        qp.setPen(pen)
        qp.drawLine(angleLine3)

        angleLine4 = QLineF()
        angleLine4.setP1(QPointF(370, 360))
        angleLine4.setAngle(150)
        angleLine4.setLength(347)
        qp.setPen(pen)
        qp.drawLine(angleLine4)

        angleLine5 = QLineF()
        angleLine5.setP1(QPointF(370, 360))
        angleLine5.setAngle(210)
        angleLine5.setLength(347)
        qp.setPen(pen)
        qp.drawLine(angleLine5)

        angleLine6 = QLineF()
        angleLine6.setP1(QPointF(370, 360))
        angleLine6.setAngle(240)
        angleLine6.setLength(347)
        qp.setPen(pen)
        qp.drawLine(angleLine6)

        angleLine7 = QLineF()
        angleLine7.setP1(QPointF(370, 360))
        angleLine7.setAngle(300)
        angleLine7.setLength(347)
        qp.setPen(pen)
        qp.drawLine(angleLine7)

        angleLine8 = QLineF()
        angleLine8.setP1(QPointF(370, 360))
        angleLine8.setAngle(330)
        angleLine8.setLength(347)
        qp.setPen(pen)
        qp.drawLine(angleLine8)


    def remark(self):
        l1 = QLabel("0",self)
        l1.setStyleSheet("QLabel {color: white}")
        l1.move(980,655)

        l2 = QLabel("5", self)
        l2.setStyleSheet("QLabel {color: white}")
        l2.move(980, 635)

        l3 = QLabel("10", self)
        l3.setStyleSheet("QLabel {color: white}")
        l3.move(980, 615)

        l4 = QLabel("15", self)
        l4.setStyleSheet("QLabel {color: white}")
        l4.move(980, 595)

        l5 = QLabel("20", self)
        l5.setStyleSheet("QLabel {color: white}")
        l5.move(980, 575)

        l6 = QLabel("25", self)
        l6.setStyleSheet("QLabel {color: white}")
        l6.move(980, 555)

        l7 = QLabel("30", self)
        l7.setStyleSheet("QLabel {color: white}")
        l7.move(980, 535)

        l8 = QLabel("35", self)
        l8.setStyleSheet("QLabel {color: white}")
        l8.move(980, 515)

        l9 = QLabel("40", self)
        l9.setStyleSheet("QLabel {color: white}")
        l9.move(980, 495)

        l10 = QLabel("45", self)
        l10.setStyleSheet("QLabel {color: white}")
        l10.move(980, 475)

        l11 = QLabel("50", self)
        l11.setStyleSheet("QLabel {color: white}")
        l11.move(980, 455)

        l12 = QLabel("55", self)
        l12.setStyleSheet("QLabel {color: white}")
        l12.move(980, 435)

        l13 = QLabel("60", self)
        l13.setStyleSheet("QLabel {color: white}")
        l13.move(980, 415)

        l14 = QLabel("65", self)
        l14.setStyleSheet("QLabel {color: white}")
        l14.move(980, 395)

        l15 = QLabel("70", self)
        l15.setStyleSheet("QLabel {color: white}")
        l15.move(980, 375)

        l16 = QLabel(">70", self)
        l16.setStyleSheet("QLabel {color: white}")
        l16.move(980, 355)

        name = QLabel("dbz",self)
        name.setStyleSheet("QLabel {color: white}")
        name.move(930,340)




App = QApplication(sys.argv)
w = QWidget()
a_window = Window()
sys.exit(App.exec_())

