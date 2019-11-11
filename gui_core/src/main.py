#!/usr/bin/env python
"""This module contains the GUI"""

import sys
import os
import rospy
from PyQt5 import QtWidgets
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QFileDialog, QMessageBox

from models import DataModel
from Ui_main_window import Ui_GUI


class GuiMainWindow(QtWidgets.QMainWindow, Ui_GUI):
    """
    Class to setup GUI widgets and connect them.
    """

    def __init__(self):
        super(GuiMainWindow, self).__init__()
        rospy.init_node("GUI_Node")
        self.setupUi(self)
        self.main_stacked_widget.setCurrentIndex(1)
        self.setAttribute(Qt.WA_DeleteOnClose)

        self.confirm_button.setVisible(False)
        self.test_signal = [('signal', 'this is a signal', [])]
        self.test_input = [('input', 'this is an input', [])]

        self.signal_list_model = DataModel(self.test_signal)
        self.signal_list.setModel(self.signal_list_model)
        self.signal_list.clicked.connect(self.signal_clicked)

        self.user_input_list_mode = DataModel(self.test_input)
        self.input_list.setModel(self.user_input_list_mode)
        self.input_list.clicked.connect(self.input_clicked)

        self.connect_buttons()

    def connect_buttons(self):
        """
        Connect all the buttons on the gui to their signals.
        """
        self.file_select_button.clicked.connect(self.select_file)
        self.submit_button.clicked.connect(self.file_submitted)
        self.confirm_button.clicked.connect(self.confirm_clicked)
        self.refresh_button.clicked.connect(self.refresh_clicked)

    def confirm_clicked(self):
        """
        Send signal/Send user input when clicked.
        """
        if self.signal_list.selectedIndexes():
            QMessageBox.question(self, "Signal Sent",
                                 "I can't believe this worked!",
                                 QMessageBox.Yes | QMessageBox.No)
        elif self.input_list.selectedIndexes():
            QMessageBox.question(self, "Input Data Sent",
                                 "WOOOOOOOOO!",
                                 QMessageBox.Yes | QMessageBox.No)

    def select_file(self):
        """
        Open file dialog when file select is pressed.
        """
        self.file_path_line_edit.setText(QFileDialog.getOpenFileName()[0])
        filename, file_extension = os.path.splitext(
            self.file_path_line_edit.text())
        if file_extension == ".bpmn":
            self.mission_name.setText(os.path.basename(filename))
        else:
            self.file_path_line_edit.clear()

    def file_submitted(self):
        """
        Switch to main page when a file submitted.
        """
        self.main_stacked_widget.setCurrentIndex(1)

    def signal_clicked(self, index):
        """
        Display signal information when clicked in the list.
        """
        self.input_list.selectionModel().clearSelection()
        self.confirm_button.setVisible(True)
        self.name_of_selected.setText(
            self.signal_list_model.data_list[index.row()][0])
        self.description_of_selected.setText(
            self.signal_list_model.data_list[index.row()][1])
        self.prompt_of_selected.setText("Would you like to send this signal?")

    def input_clicked(self, index):
        """
        Display input information when clicked in the list.
        """
        self.signal_list.selectionModel().clearSelection()
        self.confirm_button.setVisible(True)
        self.name_of_selected.setText(
            self.user_input_list_mode.data_list[index.row()][0])
        self.description_of_selected.setText(
            self.user_input_list_mode.data_list[index.row()][1])
        self.prompt_of_selected.setText("Would fufill this request?")

    def refresh_clicked(self):
        """
        Update the signal/input list.
        """
        self.signal_list_model.data_list = []
        self.user_input_list_mode.data_list = []
        self.signal_list_model.layoutChanged.emit()
        self.user_input_list_mode.layoutChanged.emit()
        self.reset_labels()

        temp_data = [('signal1', 'this is a signal', []),
                     ('signal2', 'adding a signal worked', [])]
        add_data(self.signal_list_model, temp_data)

    def reset_labels(self):
        """
        Reset the labels as if no signal was selected
        """
        self.name_of_selected.setText("")
        self.prompt_of_selected.setText("")
        self.description_of_selected.setText("")
        self.signal_list.selectionModel().clearSelection()
        self.input_list.selectionModel().clearSelection()


def add_data(model, input_data_list):
    """
    Add signal/input to model.
    """
    for data in input_data_list:
        model.data_list.append(data)
        model.layoutChanged.emit()


APP = QtWidgets.QApplication(sys.argv)
WINDOW = GuiMainWindow()
WINDOW.show()
APP.exec_()
