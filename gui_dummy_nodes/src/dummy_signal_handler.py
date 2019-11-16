#!/usr/bin/env python
""" This dummy node handles the signal list."""
import rospy
from gui_msgs.msg import GuiData, Variable  # pylint: disable=import-error
from gui_msgs.srv import SignalList, SignalListResponse  # pylint: disable=import-error


def provide_signal_list(request):  # pylint: disable=unused-argument
    test_variable1 = Variable("var1", "string", None)
    test_variable2 = Variable("var2", "int", None)
    test_signal1 = GuiData("signal1", "this is a signal without variables", [])
    test_signal2 = GuiData("signal2", "this is a signal with variables",
                           [test_variable1, test_variable2])
    signal_list = [test_signal1, test_signal2]
    return SignalListResponse(signal_list)


def signal_list_server():
    rospy.init_node('signal_list_handler_dummy')
    test = rospy.get_param('~signal_list_topic')
    print test
    rospy.Service(test,
                  SignalList,
                  provide_signal_list)
    rospy.spin()


if __name__ == "__main__":
    signal_list_server()
