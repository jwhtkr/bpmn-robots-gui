#!/usr/bin/env python
""" This module contains the temporary input handler for the GUI"""
import unique_id
import rospy
from gui_msgs.msg import GuiData, Variable  # pylint: disable=import-error
from gui_msgs.srv import RequestUserInput, ProvideUserInput, ProvideUserInputResponse  # pylint: disable=import-error


class InputHandler(object):
    """
    Dummy node for handling input.
    """

    def __init__(self):
        """
        Start the input handler node.

        This node will request user input and pretend it accepts the response.
        """
        rospy.init_node("input_handler_dummy")
        self.request_topic = rospy.get_param("~request_user_input_topic")
        self.response_topic = rospy.get_param("~provide_user_input_topic")
        self.test_variable1 = Variable("var1", "string", None)
        self.test_variable2 = Variable("var2", "int", None)
        self.test_variable3 = Variable("var2", "int", "12312312")

        uuid_list = []

        for _ in range(3):
            uuid_list.append(unique_id.toMsg(unique_id.fromRandom()))

        self.test_input1 = GuiData(
            "input1", "this is an input without variables", uuid_list[0], [])
        self.test_input2 = GuiData("input2", "this is an input with variables", uuid_list[1],
                                   [self.test_variable1, self.test_variable2])
        self.test_input3 = GuiData("input3", "this is an input with variables", uuid_list[2],
                                   [self.test_variable1, self.test_variable2, self.test_variable3])
        rospy.wait_for_service(self.request_topic)
        self.request_input = rospy.ServiceProxy(
            self.request_topic, RequestUserInput)
        self.request_input(self.test_input1)
        self.request_input(self.test_input2)
        self.request_input(self.test_input3)

        rospy.Service(self.response_topic, ProvideUserInput,
                      self.accept_user_input)
        self.input_timer = rospy.Timer(rospy.Duration(1), self.send_input)

        rospy.spin()

    def send_input(self, timer_event):  # pylint: disable=unused-argument
        """
        Send a single input every second.
        """
        self.request_input(self.test_input1)

    def accept_user_input(self, request):
        """
        Print whether or not user input got accepted.
        """
        print request.data
        fail_messages = []
        for variable in request.data.variables:
            if variable.value == '':
                fail_messages.append(variable.name + " does not have a value!")
            if (variable.type == ('int' or 'float')) and (not variable.value.isdigit()):
                fail_messages.append(
                    variable.name + " does not have the correct value for type: " + variable.type)
        if fail_messages == []:
            return ProvideUserInputResponse(True, [])
        return ProvideUserInputResponse(False, fail_messages)


if __name__ == "__main__":
    INPUT_HANDLER = InputHandler()
