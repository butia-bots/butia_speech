#!/usr/bin/env python
import rospy
import os
import rospkg

from butia_speech.detect_hotword import DetectHotWord

from std_msgs.msg import Empty, String

PACK_DIR = rospkg.RosPack().get_path('butia_speech')

if __name__ == '__main__':
    rospy.init_node('detector_hotword_node', anonymous = True)
    sensibility = rospy.get_param("/butia_hotword_detection/sensibility", 0.5)
    library = PACK_DIR + '/include/lib/libpv_porcupine.so'
    model = PACK_DIR + '/include/model/porcupine_params.pv'
    keyword = PACK_DIR + '/resources/hello doris_linux.ppn'
    detector_publisher_param = rospy.get_param('/topics/butia_hotword_detection/detected','/butia_speech/bhd/detected')
    detector_subscriber_param = rospy.get_param('/topics/butia_hotword_detection/hot_word','/butia_speech/bhd/hot_word')
    detector_publisher = rospy.Publisher(detector_publisher_param, Empty, queue_size=1)
    detector = DetectHotWord(library, model, keyword, sensibility)
    detector.hear()
    while not rospy.is_shutdown():
        result = detector.process()
        if result:
            detector_publisher.publish(Empty())