#!/usr/bin/env python
""" This module contains the temporary input handler for the GUI"""

import rospy
from gui_msgs.msg import GuiData, Variable  # pylint: disable=import-error
from gui_msgs.srv import RequestUserInput, ProvideUserInput, ProvideUserInputResponse  # pylint: disable=import-error


def input_handler():
    """
    Start the input handler node.

    This node will request user input and pretend it accepts the response.
    """
    rospy.init_node("input_handler_dummy")
    request_topic = rospy.get_param("~request_user_input_topic")
    response_topic = rospy.get_param("~provide_user_input_topic")
    test_variable1 = Variable("var1", "string", None)
    test_variable2 = Variable("var2", "int", None)
    test_variable3 = Variable("var2", "int", None)
    test_input1 = GuiData(
        "input1", "this is an input without variables", 0, [])
    test_input2 = GuiData("input2", "this is an input with variables", 0,
                          [test_variable1, test_variable2])
    test_input3 = GuiData("input3", "this is an input with variables", 0,
                          [test_variable1, test_variable2, test_variable3])
    rospy.wait_for_service(request_topic)
    request_input = rospy.ServiceProxy(request_topic, RequestUserInput)
    request_input(test_input1)
    request_input(test_input2)
    request_input(test_input3)
    rospy.Service(response_topic, ProvideUserInput, accept_user_input)
    rospy.spin()


def accept_user_input(request):
    """
    Print whether or not user input got accepted.
    """
    fail_messages = []
    for variable in request.data.variables:
        if variable.value == '':
            fail_messages.append(variable.name + " does not have a value!")
    if fail_messages == []:
        return ProvideUserInputResponse(True, [])
    return ProvideUserInputResponse(False, fail_messages)


if __name__ == "__main__":
    input_handler()
