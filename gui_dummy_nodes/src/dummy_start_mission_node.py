#!/usr/bin/env python
"""This dummy node pretends that it started a signal."""
import rospy
from gui_msgs.srv import StartMission, StartMissionResponse  # pylint: disable=import-error


def start_mission(request):
    """Return true after printing the request."""
    if request.filepath == "/home/ryanjacobson/testfolder/bad.bpmn":
        return StartMissionResponse(False, 'Bad bpmn file, please choose another.')
    return StartMissionResponse(True, '')


def mission_starter():
    """Start mission starting dummy node."""
    rospy.init_node('start_mission_dummy')
    rospy.Service(rospy.get_param('~start_mission_topic'),
                  StartMission,
                  start_mission)
    rospy.spin()


if __name__ == "__main__":
    mission_starter()
