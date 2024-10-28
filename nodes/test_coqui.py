import requests
import urllib.parse

def synthesize(text, speaker_id="", style_wav="", language_id=""):
    # Preprocess the text
    encoded_text = urllib.parse.quote(text)
    
    url = f"http://jetson:5002/api/tts?text={encoded_text}&speaker_id={speaker_id}&style_wav={style_wav}&language_id={language_id}"
    response = requests.get(url, headers={'Cache-Control': 'no-cache'})
    
    if response.status_code != 200:
        raise Exception(f"Error: {response.status_code} {response.reason}")
    
    # Save the response content as a WAV file
    with open("output.wav", "wb") as f:
        f.write(response.content)
    
    print("WAV file saved as output.wav")

# Example usage
synthesize("Hi! I am ready to answer six of your best questions. Always ask me after the beep. Choose carefully your questions and when you are ready, say hello Dolris to start", "p374")