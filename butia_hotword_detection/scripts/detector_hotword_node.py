import rospy
import os
import rospkg

from DetectHotWord.DetectHotWord import DetectHotWord

from std_msgs.msg import Empty, String

PACK_DIR = rospkg.RosPack().get_path('butia_hotword_detection')

sensibility = None

detector_publisher = None
detector_subscriber = None

detector = None

library = None
model = None
keyword = None

def detectionCallBack(hot_word):
    keyword = os.path.join(keyword, hot_word + '_linux.ppn')
    detector = DetectHotWord(library, model, keyword, sensibility)
    rate = rospy.Rate(1)
    detector.hear()
    while not rospy.is_shutdown():
        result = detector.process()
        if result:
            detector_publisher.publish(Empty())
            del detector
            break


if __name__ = '__main__':
    rospy.init_node('butia_hotword_detection', anonymous = True)

    sensibility = rospy.get_param("/butia_hotword_detection/sensibility", 0.5)

    library = os.path.join(PACK_DIR, '/include/lib/libpv_porcupine.so')
    model = os.path.join(PACK_DIR, '/include/model/porcupine_params.pv')
    keyword = os.path.join(PACK_DIR, '/resources')

    detector_publisher_param = rospy.get_param('/topics/butia_hotword_detection/detected','/butia_speech/bhd/detected')
    detector_subscriber_param = rospy.get_param('/topics/butia_hotword_detection/hot_word','/butia_speech/bhd/hot_word')

    detector_publisher = rospy.Publisher(detector_publisher_param, Empty, queue_size=1)
    detector_subscriber = rospy.Subscriber("/butia_speech/bhd/hot_word", String, detectionCallBack, queue_size=1)

    rospy.spin()