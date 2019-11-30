#!/usr/bin/env python
"""This dummy node handles the signal list."""
import rospy
import unique_id
from gui_msgs.msg import GuiData, Variable  # pylint: disable=import-error
from gui_msgs.srv import SignalList, SignalListResponse, SignalSend, SignalSendResponse  # pylint: disable=import-error


class SignalHandler(object):
    """
    Class for dummy signal handler
    """

    def __init__(self):
        """
        Start a signal_list_handler node.
        """
        rospy.init_node('signal_list_handler_dummy')
        signal_list_topic = rospy.get_param('~signal_list_topic')
        rospy.Service(signal_list_topic,
                      SignalList,
                      self.provide_signal_list)
        signal_send_topic = rospy.get_param('~signal_send_topic')
        rospy.Service(signal_send_topic,
                      SignalSend,
                      self.signal_start)

        self.test_variable1 = Variable("var1", "string", None)
        self.test_variable2 = Variable("var2", "int", None)
        self.uuid_list = []

        for _ in range(2):
            self.uuid_list.append(unique_id.toMsg(unique_id.fromRandom()))

        rospy.spin()

    def provide_signal_list(self, request):  # pylint: disable=unused-argument
        """
        Provide two signals to start. One with no variables and one with.
        """

        test_signal1 = GuiData(
            "signal1", "this is a signal without variables", self.uuid_list[0], [])
        test_signal2 = GuiData("signal2", "this is a signal with variables", self.uuid_list[1],
                               [self.test_variable1, self.test_variable2])
        signal_list = [test_signal1, test_signal2]
        return SignalListResponse(signal_list)

    def signal_start(self, request):
        """
        Take the request signal, check to see if all variables have values.
        """
        print request.signal
        fail_messages = []
        for variable in request.signal.variables:
            if variable.value == '':
                fail_messages.append(variable.name + " does not have a value!")
            if (variable.type == ('int' or 'float')) and (not variable.value.isdigit()):
                fail_messages.append(
                    variable.name + " does not have the correct value for type: " + variable.type)
        if fail_messages == []:
            return SignalSendResponse(True, [])
        return SignalSendResponse(False, fail_messages)


if __name__ == "__main__":
    TEST = SignalHandler()
