#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

global service_proxy
global pressed
global pub

pressed = False


def callback(msg: Joy):
    global pressed

    if msg.buttons[0] == 1 and not pressed:
        pressed = True
        try:
            service_proxy(TriggerRequest())
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    if msg.buttons[0] == 0:
        pressed = False


if __name__ == "__main__":
    rospy.init_node("lightweight_speech_recognition_caller")

    service_proxy = rospy.ServiceProxy("/run_lightweight_speech_recognition", Trigger)
    pub = rospy.Publisher("/smart_device_protocol/led_color", ColorRGBA, queue_size=1)
    sub = rospy.Subscriber("/smart_device_protocol/joy", Joy, callback)

    rospy.spin()
