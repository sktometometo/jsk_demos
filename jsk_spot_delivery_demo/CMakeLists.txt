cmake_minimum_required(VERSION 3.0.2)
project(jsk_spot_delivery_demo)

find_package(catkin REQUIRED genmsg actionlib_msgs std_msgs
)

add_message_files(
DIRECTORY msg
FILES
    DeliveryTask.msg
    DeliveryTaskArray.msg
)

add_action_files(
DIRECTORY action
FILES
    ExecuteTask.action
    PickupPackage.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
)
