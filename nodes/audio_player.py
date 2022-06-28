#!/usr/bin/env python3

import rospy

from butia_speech.wav_to_mouth import WavToMouth

if __name__ == "__main__":
    rospy.init_node("audio_to_angle", anonymous=False)

    w2m = WavToMouth()
    w2m.read_audio()