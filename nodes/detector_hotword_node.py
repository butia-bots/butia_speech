#!/usr/bin/env python
import rospy
import os
import rospkg
from threading import Lock

from butia_speech.detect_hotword import DetectHotWord

from std_msgs.msg import Empty, String

PACK_DIR = rospkg.RosPack().get_path('butia_speech')

library = None
model = None
keyword = None

detector = None

mutex = Lock()  

def detectionCallBack(hotword):
    mutex.acquire()
    if detector is not None:
        detector = DetectHotWord(library, model, keyword + hotword.data + '_linux.ppn', sensibility)
        detector.hear()
    mutex.release()

if __name__ == '__main__':
    rospy.init_node('detector_hotword_node', anonymous = True)
    sensibility = rospy.get_param("/butia_hotword_detection/sensibility", 0.5)
    library = PACK_DIR + '/include/lib/libpv_porcupine.so'
    model = PACK_DIR + '/include/model/porcupine_params.pv'
    keyword = PACK_DIR + '/resources/'
    #detector_publisher_param = rospy.get_param('/topics/butia_hotword_detection/detected','/butia_speech/bhd/detected')
    #detector_subscriber_param = rospy.get_param('/topics/butia_hotword_detection/hot_word','/butia_speech/bhd/hot_word')
    detector_subscriber = rospy.Subscriber('/butia_speech/bhd/hot_word', String, detectionCallBack, queue_size=1)
    detector_publisher = rospy.Publisher('/butia_speech/bhd/detected', Empty, queue_size=1)
    
    if detector is None:
        detector = DetectHotWord(library, model, keyword + 'hello doris_linux.ppn', sensibility)
        detector.hear()
    while not rospy.is_shutdown():
        mutex.acquire()
        result = detector.process()
        if result:
            detector_publisher.publish(Empty())
        mutex.release()
    
