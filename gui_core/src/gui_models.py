"""This module holds the models used in the GUI"""

from PyQt5 import QtCore
from PyQt5.QtCore import Qt


class DataModel(QtCore.QAbstractListModel):
    """
    Class to represent avaliable signals to send or inputs to receive.
    """

    def __init__(self, *args, **kwargs):
        super(DataModel, self).__init__(*args, **kwargs)
        self.data_list = []

    def data(self, index, role):
        """
        Return name of signal/input for display.
        """
        if role == Qt.DisplayRole:
            data = self.data_list[index.row()]
            return data.name
        return None

    def rowCount(self, index):  # pylint: disable=invalid-name, unused-argument
        """
        Return length of signal/input model.
        """
        return len(self.data_list)
