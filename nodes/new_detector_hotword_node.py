#!/usr/bin/env python3
import rospy
import os
import rospkg

from butia_speech.new_detect_hotword import newDetectHotWord

from std_msgs.msg import String

from termcolor import colored
import warnings
warning = rospy.get_param("warnings", False)
if not warning:
    warnings.filterwarnings("ignore")

PACK_DIR = rospkg.RosPack().get_path('butia_speech')

if __name__ == '__main__':
    rospy.init_node('detector_hotword_node', anonymous = True)
    count = 1
    
    sensibility = rospy.get_param("/butia_hotword_detection/sensibility", 0.5)

    result_arr = [
        'Hello Boris',
        'Yes Boris',
        'No Boris',
        ]

    keyword = [
        PACK_DIR + '/resources/Hello-Boris_en_linux_v3_0_0.ppn',
        PACK_DIR + '/resources/It--s-right_en_linux_v3_0_0.ppn',
        PACK_DIR + '/resources/It--s-wrong_en_linux_v3_0_0.ppn'
    ] 

    sensibility = [sensibility]*len(keyword)

    detector_publisher_param = rospy.get_param("publishers/butia_hotword_detection/topic","/butia_speech/bhd/detected")

    detector_subscriber_param = rospy.get_param("subscribers/butia_hotword_detection/topic","/butia_speech/bhd/hot_word")
    detector_publisher = rospy.Publisher(detector_publisher_param, String, queue_size=1)

    detector = newDetectHotWord(keyword, sensibility)

    detector.hear()
    
    while not rospy.is_shutdown():
        
        result = detector.process()
        if result>=0:
            rospy.logwarn(f"ouviu {result}")
            detector_publisher.publish(result_arr[result])