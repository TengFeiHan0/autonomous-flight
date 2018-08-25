from PySide import QtCore, QtGui



class ExDisplay(QtGui.QMainWindow):
    GUI_UPDATE_PERIOD = 20 # ms

    def __init__(self):
        super(ExDisplay, self).__init__()
        self.setWindowTitle("Test click")
        self.imageBox = QtGui.QLabel(self)
        self.setCentralWidget(self.imageBox)
        # self.imageBox.mousePressEvent = self.OnClick

        self.image = QtGui.QPixmap('baloon.png')
        self.text = "Nothing happened"

        self.redrawTimer = QtCore.QTimer(self)
        self.redrawTimer.timeout.connect(self.RedrawCallback)
        self.redrawTimer.start(self.GUI_UPDATE_PERIOD)

    def RedrawCallback(self):
        self.imageBox.setPixmap(self.image)
        self.resize(640, 480)
        self.statusBar().showMessage(self.text)

    def mousePressEvent(self, e):
        print e.x(), e.y(),
        print int(e.button())

if __name__ == '__main__':
    import sys
    app = QtGui.QApplication(sys.argv)
    display = ExDisplay()
    display.show()
    sys.exit(app.exec_())

