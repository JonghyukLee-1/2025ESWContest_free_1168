import librosa 
import librosa.display 
import matplotlib.pyplot as plt
import numpy as np
audio_path = "home/tunnel/Desktop/riot/sound_code/abnormal_resampled.wav"
y,sr = librosa.load(audio_path)

D =np.abs(librosa.stft(y))

S_db = librosa.amplitude_to_db(D,ref=np.max)

plt.figure(figsize=(10,4))
librosa.display.specshow(S_db, sr=sr, x_axis='time',y_axis='hz')
plt.colorbar(format='%+2.0f dB')
plt.title('graph')
plt.show()
