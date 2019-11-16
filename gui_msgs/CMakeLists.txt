cmake_minimum_required(VERSION 2.8.3)
project(gui_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    message_runtime
    message_generation
)


add_message_files(
   FILES
    Variable.msg
    GuiData.msg
)
add_service_files(
  DIRECTORY srv
  FILES 
    SignalList.srv
    StartMission.srv
)
generate_messages(
  DEPENDENCIES
)



catkin_package(
  CATKIN_DEPENDS
    message_runtime
    message_generation
)
