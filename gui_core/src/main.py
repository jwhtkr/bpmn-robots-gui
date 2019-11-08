import sys
import os
import rospy
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QFileDialog
from Ui_main_window import Ui_GUI


class GuiMainWindow(QtWidgets.QMainWindow, Ui_GUI):
    """Class to setup GUI widgets and connect them."""
    def __init__(self):
        super(GuiMainWindow, self).__init__()
        rospy.init_node("GUI_Node")
        self.setupUi(self)
        self.mainStackedWidget.setCurrentIndex(1)
        self.fileSelectButton.clicked.connect(self.select_file)
        self.submitButton.clicked.connect(self.file_submitted)
    def select_file(self):
        """Open file dialog when file select is pressed."""
        self.filePathLineEdit.setText(QFileDialog.getOpenFileName()[0])
        filename, file_extension = os.path.splitext(self.filePathLineEdit.text())
        if file_extension == ".bpmn":
            self.missionName.setText(os.path.basename(filename))
        else:
            self.filePathLineEdit.clear()
    def file_submitted(self):
        """Switch to main page when a file submitted."""
        self.mainStackedWidget.setCurrentIndex(1)
        

APP = QtWidgets.QApplication(sys.argv)
WINDOW = GuiMainWindow()
WINDOW.show()
APP.exec_()
