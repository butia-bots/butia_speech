#pip install numpy
#pip install nvidia-riva-client
#pip install soundfile

import numpy as np
import soundfile as sf
import riva.client

auth = riva.client.Auth(uri='jetson:50051')

riva_tts = riva.client.SpeechSynthesisService(auth)

sample_rate_hz = 44100
req = { 
        "language_code"  : "en-US",
        "encoding"       : riva.client.AudioEncoding.LINEAR_PCM ,   # LINEAR_PCM and OGGOPUS encodings are supported
        "sample_rate_hz" : sample_rate_hz,                          # Generate 44.1KHz audio
        "voice_name"     : "English-US.Female-1",                 # The name of the voice to generate
}
print(req.items())

text = "Hello, i am called Boris, a helper open source robot designed to help at domestic chores. I am a product of development of the team FBOT from FURG university. It is a pleasure to meet you!"
resp = riva_tts.synthesize(custom_dictionary=req, text=text)
audio_samples = np.frombuffer(resp.audio, dtype=np.int16)

sf.write('test_riva_audio.wav', audio_samples, sample_rate_hz)
