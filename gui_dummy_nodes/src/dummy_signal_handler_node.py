#!/usr/bin/env python
"""This dummy node handles the signal list."""
import rospy
from gui_msgs.msg import GuiData, Variable  # pylint: disable=import-error
from gui_msgs.srv import SignalList, SignalListResponse, SignalSend, SignalSendResponse  # pylint: disable=import-error


def provide_signal_list(request):  # pylint: disable=unused-argument
    """
    Provide two signals to start. One with no variables and one with.
    """
    test_variable1 = Variable("var1", "string", None)
    test_variable2 = Variable("var2", "int", None)
    test_signal1 = GuiData("signal1", "this is a signal without variables", [])
    test_signal2 = GuiData("signal2", "this is a signal with variables",
                           [test_variable1, test_variable2])
    signal_list = [test_signal1, test_signal2]
    return SignalListResponse(signal_list)


def signal_list_handler():
    """
    Start a signal_list_handler node.
    """
    rospy.init_node('signal_list_handler_dummy')
    signal_list_topic = rospy.get_param('~signal_list_topic')
    rospy.Service(signal_list_topic,
                  SignalList,
                  provide_signal_list)
    signal_send_topic = rospy.get_param('~signal_send_topic')
    rospy.Service(signal_send_topic,
                  SignalSend,
                  signal_start)
    rospy.spin()


def signal_start(request):
    """
    Take the request signal, check to see if all variables have values.
    """
    print request
    fail_messages = []
    for variable in request.signal.variables:
        if variable.value == '':
            fail_messages.append(variable.name + " does not have a value!")
    if fail_messages == []:
        return SignalSendResponse(True, [])
    return SignalSendResponse(False, fail_messages)


if __name__ == "__main__":
    signal_list_handler()