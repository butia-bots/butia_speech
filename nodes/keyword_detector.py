#!/usr/bin/env python
# -*- coding: utf-8 -*-
import platform
import rospkg
from porcupine import Porcupine
import rospy
import pyaudio
import struct
from datetime import datetime
from std_msgs.msg import Empty
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--input_audio_device_index', help='index of input audio device', type=int, default=None)
    args = parser.parse_args()
    package_path = rospkg.RosPack().get_path('butia_speech')
    system = platform.system().lower()
    machine = platform.machine()
    library_path = '{package}/lib/Porcupine/{system}/{machine}/libpv_porcupine.so'.format(
        package=package_path,
        system=system,
        machine=machine
    ) # Path to Porcupine's C library available under lib/${SYSTEM}/${MACHINE}/
    model_file_path = '{package}/lib/Porcupine/common/porcupine_params.pv'.format(
        package=package_path
    ) # It is available at lib/common/porcupine_params.pv
    keyword_file_paths = ['{package}/resources/hello doris_linux.ppn'.format(package=package_path)]
    sensitivities = [0.5]
    handle = Porcupine(library_path, model_file_path, keyword_file_paths=keyword_file_paths, sensitivities=sensitivities)
    rospy.init_node('keyword_detector')
    wakeup_pub = rospy.Publisher('butia/wakeup', Empty, queue_size=10)
    rate = rospy.Rate(1)
    pa = pyaudio.PyAudio()
    audio_stream = pa.open(
        rate=handle.sample_rate,
        channels=1,
        format=pyaudio.paInt16,
        input=True,
        frames_per_buffer=handle.frame_length)
    while not rospy.is_shutdown():
        pcm = audio_stream.read(handle.frame_length)
        pcm = struct.unpack_from("h" * handle.frame_length, pcm)
        recorded_frames = []
        recorded_frames.append(pcm)
        result = handle.process(pcm)
        num_keywords = len(keyword_file_paths)
        if num_keywords == 1 and result:
            print('[%s] detected keyword' % str(datetime.now()))
            wakeup_pub.publish(Empty())
            rate.sleep()
