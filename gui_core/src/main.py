#!/usr/bin/env python
"""This module contains the GUI"""

import sys
import os
import rospy
from PyQt5 import QtWidgets
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QFileDialog, QMessageBox

from gui_models import DataModel
from Ui_main_window import Ui_GUI
from gui_msgs.msg import GuiData        # pylint: disable=import-error
from gui_msgs.srv import (SignalList,   # pylint: disable=import-error
                          StartMission,
                          SignalSend, SignalSendRequest,
                          RequestUserInput, RequestUserInputResponse,
                          ProvideUserInput, ProvideUserInputRequest)


class GuiMainWindow(QtWidgets.QMainWindow, Ui_GUI):
    """
    Class to setup GUI widgets and connect them.
    """

    def __init__(self):
        super(GuiMainWindow, self).__init__()
        self.variable_form = None
        self.request_input_service = None
        self.provide_input_service = None
        rospy.init_node("GUI_Node")
        self.setupUi(self)
        self.main_stacked_widget.setCurrentIndex(0)
        self.setAttribute(Qt.WA_DeleteOnClose)

        self.confirm_button.setVisible(False)

        self.signal_list_model = DataModel()
        self.signal_list.setModel(self.signal_list_model)
        self.signal_list.clicked.connect(self.signal_clicked)

        self.input_list_model = DataModel()
        self.input_list.setModel(self.input_list_model)
        self.input_list.clicked.connect(self.input_clicked)

        self.connect_buttons()
        self.service_setup()

    def connect_buttons(self):
        """
        Connect all the buttons on the gui to their signals.
        """
        self.file_select_button.clicked.connect(self.select_file)
        self.submit_button.clicked.connect(self.file_submitted)
        self.confirm_button.clicked.connect(self.confirm_clicked)
        self.refresh_button.clicked.connect(self.refresh_clicked)
        self.start_mission_button.clicked.connect(self.start_mission_clicked)

    def service_setup(self):
        """
        Ready all service proxies for use.
        """
        self.signal_list_topic = rospy.get_param('~signal_list_topic')
        self.start_mission_topic = rospy.get_param('~start_mission_topic')
        self.signal_send_topic = rospy.get_param('~signal_send_topic')
        self.request_input_topic = rospy.get_param('~request_user_input_topic')
        self.provide_input_topic = rospy.get_param('~provide_user_input_topic')

        self.get_signal_list = rospy.ServiceProxy(
            self.signal_list_topic,
            SignalList)

        self.start_mission = rospy.ServiceProxy(
            self.start_mission_topic,
            StartMission)

        self.send_signal = rospy.ServiceProxy(
            self.signal_send_topic,
            SignalSend)

        self.provide_input = rospy.ServiceProxy(
            self.provide_input_topic,
            ProvideUserInput
        )

        self.request_input_service = rospy.Service(self.request_input_topic,
                                                   RequestUserInput,
                                                   self.input_request_callback)

    def input_request_callback(self, request):
        """
        Add a input request to the input request list.
        """
        print request.data
        add_data(self.input_list_model, [request.data])
        return RequestUserInputResponse(True, '')

    def start_mission_clicked(self):
        """
        Request a mission to be started, grey out the button if successful
        """
        rospy.wait_for_service(self.start_mission_topic)
        start_mission_response = self.start_mission(
            self.file_path_line_edit.text())
        if start_mission_response.success:
            self.start_mission_button.setEnabled(False)
            self.start_mission_button.setText("Mission Started")
        else:
            self.clear_second_page()
            self.main_stacked_widget.setCurrentIndex(0)
            QMessageBox.warning(self,
                                "Failed to open mission!",
                                start_mission_response.failure)

    def confirm_clicked(self):
        """
        Send signal/Send user input when clicked.
        """
        if self.signal_list.selectedIndexes():
            signal = self.get_data('signal')
            signal_send_request = SignalSendRequest(signal)
            response = self.send_signal(signal_send_request)
            if response.success:
                QMessageBox.information(self, "Signal Sent",
                                        "Signal sent successfully!")
            else:
                printable_failure_messages = ''
                for message in response.failure_messages:
                    printable_failure_messages += message + '\n'
                QMessageBox.warning(self, "Failed to Send Signal",
                                    printable_failure_messages)
        elif self.input_list.selectedIndexes():
            user_input = self.get_data('input')
            input_request = ProvideUserInputRequest(user_input)
            response = self.provide_input(input_request)

            if response.success:
                self.input_list_model.data_list.pop(
                    self.input_list.selectedIndexes()[0].row())
                self.refresh_clicked()
                QMessageBox.information(self, "Provided User Input!",
                                        "Input request fufilled!")
            else:
                printable_failure_messages = ''
                for message in response.failure_messages:
                    printable_failure_messages += message + '\n'
                QMessageBox.warning(
                    self, "Failed to Send Signal", printable_failure_messages)

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
        signal_list_response = self.get_signal_list()
        add_data(self.signal_list_model, signal_list_response.signal_list)

    def signal_clicked(self, index):
        """
        Display signal information when clicked in the list.
        """
        self.confirm_button.setVisible(True)
        self.input_list.selectionModel().clearSelection()
        for idx in reversed(range(self.variable_form.count())):
            self.variable_form.itemAt(idx).widget().setParent(None)
        self.variable_form = QtWidgets.QFormLayout()
        self.variable_form.setObjectName("variable_form")
        self.verticalLayout.addLayout(self.variable_form)

        self.confirm_button.setVisible(True)
        self.name_of_selected.setText(
            self.signal_list_model.data_list[index.row()].name)
        self.description_of_selected.setText(
            self.signal_list_model.data_list[index.row()].description)
        self.prompt_of_selected.setText("Would you like to send this signal?")
        self.display_form(
            self.signal_list_model.data_list[index.row()].variables)

    def input_clicked(self, index):
        """
        Display input information when clicked in the list.
        """
        self.confirm_button.setVisible(True)
        self.signal_list.selectionModel().clearSelection()

        for idx in reversed(range(self.variable_form.count())):
            self.variable_form.itemAt(idx).widget().setParent(None)
        self.variable_form = QtWidgets.QFormLayout()
        self.variable_form.setObjectName("variable_form")
        self.verticalLayout.addLayout(self.variable_form)

        self.confirm_button.setVisible(True)
        self.name_of_selected.setText(
            self.input_list_model.data_list[index.row()].name)
        self.description_of_selected.setText(
            self.input_list_model.data_list[index.row()].description)
        self.prompt_of_selected.setText(
            "Would you please fufill this request?")
        self.display_form(
            self.input_list_model.data_list[index.row()].variables)

    def refresh_clicked(self):
        """
        Update the signal/input list.
        """
        self.clear_second_page()
        rospy.wait_for_service(self.signal_list_topic, 5)

        add_data(self.signal_list_model, self.get_signal_list().signal_list)

    def reset_labels(self):
        """
        Reset the labels as if no signal was selected
        """
        self.name_of_selected.setText("")
        self.prompt_of_selected.setText("")
        self.description_of_selected.setText("")
        self.signal_list.selectionModel().clearSelection()
        self.input_list.selectionModel().clearSelection()

    def display_form(self, data):
        """
        Display labels of variables and line edits for taking user inputs.
        """
        for idx, variable in enumerate(data):
            label = QtWidgets.QLabel(self.user_input_page)
            label.setObjectName(variable.name)
            label.setText(variable.name)
            self.variable_form.setWidget(
                idx, QtWidgets.QFormLayout.LabelRole, label)
            line_edit = QtWidgets.QLineEdit(self.user_input_page)
            line_edit.setObjectName(variable.name + "_line_edit")
            self.variable_form.setWidget(
                idx, QtWidgets.QFormLayout.FieldRole, line_edit)

    def clear_second_page(self):
        """
        Clear the models, turn buttons off, remove variable menus.
        """
        self.signal_list.selectionModel().reset()
        self.input_list.selectionModel().reset()
        self.confirm_button.setVisible(False)
        self.signal_list_model.data_list = []
        self.signal_list_model.layoutChanged.emit()
        self.reset_labels()
        for idx in reversed(range(self.variable_form.count())):
            self.variable_form.itemAt(idx).widget().setParent(None)

    def get_data(self, data_type):
        """
        Return the raw_data corresponding to the selected signal or input.
        """
        if data_type == "signal":
            raw_signal = self.signal_list_model.data_list[
                (self.signal_list.selectedIndexes())[0].row()]
            signal = GuiData(
                raw_signal.name, raw_signal.description, raw_signal.uid, raw_signal.variables)
            for row in range(self.variable_form.rowCount()):
                item = self.variable_form.itemAt(
                    row, QtWidgets.QFormLayout.FieldRole)
                signal.variables[row].value = item.widget().text()
            return signal
        elif data_type == "input":
            raw_user_input = self.input_list_model.data_list[
                (self.input_list.selectedIndexes())[0].row()]
            user_input = GuiData(
                raw_user_input.name,
                raw_user_input.description,
                raw_user_input.uid,
                raw_user_input.variables)
            for row in range(self.variable_form.rowCount()):
                item = self.variable_form.itemAt(
                    row, QtWidgets.QFormLayout.FieldRole)
                user_input.variables[row].value = item.widget().text()
            return user_input
        return None


def add_data(model, input_data_list):
    """
    Add signal/input to model.
    """
    for data in input_data_list:
        model.data_list.append(data)
        model.layoutChanged.emit()


if __name__ == "__main__":
    APP = QtWidgets.QApplication(sys.argv)
    WINDOW = GuiMainWindow()
    WINDOW.show()
    APP.exec_()
