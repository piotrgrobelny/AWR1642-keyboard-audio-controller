from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui
import gui_template
import read_and_process_data as rapd

class MainWindow(QtWidgets.QWidget):
    def __init__(self, parent = None):
        QtWidgets.QWidget.__init__(self, parent)
        self.ui = gui_template.Ui_Dialog()
        self.ui.setupUi(self)
        #timer which counts time to update data, set for one 100ms
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.updateData)
        self.timer.start(100)
        self.ui.graphicsView.setBackground('w')
        self.ui.graphicsView.setXRange(-0.3, 0.3)
        self.ui.graphicsView.setYRange(0, 1)
        self.ui.graphicsView.setLabel('left', text='Y position (m)')
        self.ui.graphicsView.setLabel('bottom', text='X position (m)')
        #sets lines that show the sector in which we trigger a particular key, in this case we divide
        #area in 13 sectors - 8cm long and 30cm width each
        y = [0.08, 0.08]
        x = [-0.15, 0.15]
        for i in range(14):
            self.ui.graphicsView.plot(x, y, pen=pg.mkPen(color=(150, 200, 200), width=2, style=QtCore.Qt.DashLine))
            y[0] = y[0]+0.08
            y[1] = y[1]+0.08
        #indicates limits beyond which detected object are not taken into consideration when distance from object is calculated
        y_1 = [-0.1, 1.1]
        x_1 = [[0.15, 0.15], [-0.15, -0.15]]
        self.ui.graphicsView.plot(x_1[0], y_1, name="border", pen=pg.mkPen(color=(150, 10, 10), width=4, style=QtCore.Qt.DashLine))
        self.ui.graphicsView.plot(x_1[1], y_1, pen=pg.mkPen(color=(150, 10, 10), width=4, style=QtCore.Qt.DashLine))
        self.data_plot = self.ui.graphicsView.plot([], [], pen=None, symbol='o')


    #update plot and values from radars
    def updateData(self):
        self.ui.graphicsView_2.setStyleSheet("background-color: rgb(255, 0, 4);")
        #When HB100 detects motion change squer colour to green
        if rapd.high == False:
            self.ui.graphicsView_2.setStyleSheet("background-color: rgb(0, 240, 0);")
        self.ui.lcdNumber.display(int(100 * rapd.distance)) #show distance in real time
        self.data_plot.setData(rapd.x, rapd.y)
        QtGui.QApplication.processEvents()

def plot():
    pg.setConfigOption('background', 'w')
    win = pg.GraphicsWindow(title="2D scatter plot")
    p = win.addPlot()
    p.setXRange(-0.5, 0.5)
    p.setYRange(0, 1.5)
    p.setLabel('left', text='Y position (m)')
    p.setLabel('bottom', text='X position (m)')
    s = p.plot([], [], pen=None, symbol='o')

if __name__ == "__main__":
    rapd.main() #start read and process data from radars
    import sys
    app = QtWidgets.QApplication(sys.argv)
    c = MainWindow()
    c.show()
    sys.exit(app.exec_())
