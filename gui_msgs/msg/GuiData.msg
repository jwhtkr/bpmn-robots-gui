##
# @File: GuiData.msg
# @Date: 11 November 2019
# @Author: Ryan Jacobson
#
# @Brief
# This is a message that represents either a signal or a user input request
#
# @name
# Name of either the signal or the input request
#
# @description
# A description of the signal or input request
#
# @uuid
# The unique identifier for the signal or input request
#
# @variables
# An array of variable messages to either populate before sending a signal, or
# to request to be populated if asking for user input.
#
##

string name
string description
uuid_msgs/UniqueID uuid
gui_msgs/Variable[] variables
