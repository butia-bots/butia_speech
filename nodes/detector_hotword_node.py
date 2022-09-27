#!/usr/bin/env python3
import rospy
import os
import rospkg

from butia_speech.detect_hotword import DetectHotWord

from std_msgs.msg import Empty

from termcolor import colored
import warnings
warning = rospy.get_param("warnings", False)
if not warning:
    warnings.filterwarnings("ignore")

PACK_DIR = rospkg.RosPack().get_path('butia_speech')

if __name__ == '__main__':
    rospy.init_node('detector_hotword_node', anonymous = True)
    
    sensibility = rospy.get_param("/butia_hotword_detection/sensibility", 0.5)

    keyword = [PACK_DIR + '/resources/Hello-Doris_en_linux_v2_1_0.ppn', PACK_DIR + '/resources/Follow-me_en_linux_v2_1_0.ppn']

    sensibility = [sensibility]*len(keyword)

    detector_publisher_param = rospy.get_param("publishers/butia_hotword_detection/topic","/butia_speech/bhd/detected")
    detector_subscriber_param = rospy.get_param("subscribers/butia_hotword_detection/topic","/butia_speech/bhd/hot_word")
    detector_publisher = rospy.Publisher(detector_publisher_param, Empty, queue_size=1)

    detector = DetectHotWord(keyword, sensibility)

    detector.hear()
    while not rospy.is_shutdown():
        result = detector.process()
        if result:
            detector_publisher.publish(Empty())