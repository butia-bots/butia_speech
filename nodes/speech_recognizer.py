#!/usr/bin/env python3

import rospy
import rospkg
from butia_speech.srv import SpeechToText, SpeechToTextResponse
from speech_recognition import Microphone, Recognizer, WaitTimeoutError, AudioData
import os
import numpy as np
from transformers import Wav2Vec2Processor, Wav2Vec2ForCTC, pipeline
from playsound import playsound

AUDIO_DIR = os.path.join(rospkg.RosPack().get_path("butia_speech"), "audios/")
FILENAME = os.path.join(AUDIO_DIR, "speech_input.wav")
TALK_AUDIO = os.path.join(AUDIO_DIR, "beep.wav")

def handle_recognition(req):
    with Microphone(sample_rate=16000) as source:
        recognizer.adjust_for_ambient_noise(source, duration=1)

    playsound(TALK_AUDIO, block=True)

    with Microphone(sample_rate=16000) as source:
        try:
            audio = recognizer.listen(source, phrase_time_limit=20, timeout=5)
            # audio = recognizer.listen(source)
            
            #fs = audio.sample_rate
            #audio = np.frombuffer(audio.frame_data, np.double)
            #print(audio.shape)
            # tokenize
            #input_values = processor(audio, return_tensors="pt", sampling_rate=16000)  # Batch size 1
            
            # retrieve logits
            #with torch.no_grad():
            #    logits = model(**input_values).logits
            
            # take argmax and decode
            #predicted_ids = torch.argmax(logits, dim=-1)
            #transcription = processor.batch_decode(predicted_ids)
            #text = transcription[0]
            with open(FILENAME, 'wb') as f:
                f.write(audio.get_wav_data())
            text = asr_pipeline(FILENAME)['text'].lower()
        except WaitTimeoutError:
            text = ''
    return SpeechToTextResponse(
        text=text
    )

if __name__ == '__main__':
    #processor = Wav2Vec2Processor.from_pretrained("facebook/wav2vec2-large-960h-lv60")
    #model = Wav2Vec2ForCTC.from_pretrained("facebook/wav2vec2-large-960h-lv60")
    asr_pipeline = pipeline(task="automatic-speech-recognition", model="facebook/wav2vec2-large-960h-lv60")
    recognizer = Recognizer()
    rospy.init_node('speech_recognizer')
    
    recognizer_service_param = rospy.get_param("/services/speech_recognizer/service", "/butia_speech/sr/speech_recognizer")

    recognition_service = rospy.Service(recognizer_service_param, SpeechToText, handle_recognition)
    rospy.spin()
