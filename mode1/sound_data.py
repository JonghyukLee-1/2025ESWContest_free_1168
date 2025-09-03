# import numpy as np
# import sounddevice as sd
# import time

# FS = 44100         # 샘플링 주파수
# DURATION = 0.5   # 프레임 길이 (한 번에 0.3초)
# FFT_N = 8192       # FFT 크기
# DEVICE = 11        # 마이크 장치 번호
# min_freq = 0
# max_freq = 10000
# total_duration = 6000  # 총 실행 시간 (초)

# print("Started recording for 10 seconds...")

# start_time = time.time()

# try:
#     while True:
#         current_time = time.time()
#         if current_time - start_time > total_duration:
#             break

#         0.3초간 녹음
#         audio = sd.rec(int(DURATION * FS), samplerate=FS, channels=1, dtype='float32', device=DEVICE)
#         sd.wait()
#         sig = audio[:, 0]

#         FFT
#         window = np.hanning(len(sig))
#         X = np.fft.rfft(sig * window, n=FFT_N)
#         f = np.fft.rfftfreq(FFT_N, 1 / FS)
#         mag = np.abs(X)

#         peak frequency
#         peak_freq = f[np.argmax(mag)]

#         조건에 맞을 경우 출력
#         if min_freq <= peak_freq <= max_freq:
#             print(f"{time.strftime('%H:%M:%S')} - Peak Frequency: {peak_freq:.1f} Hz")

# except KeyboardInterrupt:
#     print("Stopped by user.")

# print("Done.")





import numpy as np
import sounddevice as sd
import time
from sound_for_answersheet import quest
freqs_ref, spectrum = quest()
from sklearn.metrics.pairwise import cosine_similarity
import sys 


FS = 44100      # Sampling rate. set as 44100 ( this usb mike = 44100 )
ref_record = 3 # pre_record, 3 second
real_record = 5 
window_fft = 2205

# changeable value list
min_freq = 2000 # threshold. change if needed
max_freq = 4000
mag_diff = 0.1
simil_threshold = 0.1





print("Started. Press Ctrl+C to stop.")

try:
    while True:

        # ref
        print("ref_record start")
        ref_audio = sd.rec(int(ref_record * FS), samplerate=FS, channels=1, dtype='float32', device=11) # record_time*FS >> record just for 5 second
        #In basic, it is 2 dimensional array 
        sd.wait()
       
        print("real_record start")
        real_audio = sd.rec(int(real_record * FS), samplerate=FS, channels=1, dtype='float32', device=11) 
        sd.wait()

        ref_sig = ref_audio[:,0]
        real_sig = real_audio[:,0]
        ref_abs = np.abs(ref_sig)  # extract data from audio and make it into sig, one dimensional array. it is magnitude + do abs
        real_abs = np.abs(real_sig)

        ref_amp = np.mean(ref_abs) # do mean to magnitude, for db setting 

        epsilon=1e-10
        sig_db = 20* np.log10( ( real_abs + epsilon )  / (ref_amp + epsilon) ) # caculate db compare to ref_amp
        # there is a problem... what if noise of a tunnel is overcome a hitting sound? hmm....

        index = np.where(real_abs >=ref_amp + mag_diff)[0]

        if index.size > 0:
            target_candidate = real_abs[index]
            real_target_idx = np.argmax(target_candidate)
            center = index[real_target_idx]
            if ( center < window_fft or center +window_fft >=len(real_sig)):
                print("center out of range")
                continue

            center_time = center / FS
            print(f"hit time : {center_time}")

            start = max(center - window_fft,0)
            end = min(center + window_fft,len(real_sig))
            segment=real_abs[start:end]
            # fft part 
            window = np.hanning(len(segment)) # to avoid gibs
            fft_result = np.fft.rfft(segment*window) # changed rffted data. real data is in here. it contains 1 dimensional array of ffted complex value data of freq.
            # it contains magnitude and phase data.
            rfft_freq = np.fft.rfftfreq(len(segment*window), 1/FS)
            rfft_mag = np.abs(fft_result)

            band_mask_live = (rfft_freq >=min_freq) & (rfft_freq <=max_freq)
            filter_freq = rfft_freq[band_mask_live] # freq vec 
            filter_mag = rfft_mag[band_mask_live] # mag vec
                
            print(f"filter_freq : {filter_freq}")
            print(f"filter_mag : {filter_mag}")

            band_mask_ref = (freqs_ref >=min_freq) & (freqs_ref <=max_freq) # freqs_ref is a data from given data 
                 
            mag_ref = spectrum[band_mask_ref] # mag vec
            if (filter_mag.size == 0 or mag_ref.size == 0 ):
                print("no vaild signal")
                continue


            print(f"mag_ref : {mag_ref}") # now, no data in here!

            mag_ref_norm = mag_ref / np.linalg.norm(mag_ref)
            mag_live_norm = filter_mag / np.linalg.norm(filter_mag)

            print(f"mag_ref_norm : {mag_ref_norm}")
            print(f"mag_live_norm : {mag_live_norm}")


            similarity = cosine_similarity(mag_ref_norm.reshape(1,-1),mag_live_norm.reshape(1,-1))[0][0]

            print(f"similarity : {similarity}")

            if (similarity >= simil_threshold): #assume that answersheet is good tile 
                print("bad tile")
            else:
                print("good tile")
            
            sys.exit()
        else:
            print("no wrong signal")
            sys.exit()

            '''
                visual_freq = np.fft.rfftfreq(len(segment), 1 / FS) # to visualize, act as x plot
                # until here, whole rffted data for 10 second is contained and stored. how should i eliminate noise and just keep our real data..?
                # i don't know fuucking high level vibe, so let's suppose that the min is around 2500, max is around 4000.
            '''
except KeyboardInterrupt:
    print("Recording Stopped.")










'''

import numpy as np
import sounddevice as sd
import matplotlib.pyplot as plt
import time

FS = 44100      # Sampling rate
DURATION = 0.1  # Duration = 0.1s (100ms)

print("Started. Press Ctrl+C to stop.")

# Set up interactive plot
plt.ion()
fig, ax = plt.subplots(figsize=(10, 4))
line, = ax.plot([], [], lw=2)
ax.set_xlim(0, DURATION)
ax.set_ylim(-0.5, 0.5)  # Assuming normalized float32 audio
ax.set_xlabel("Time [s]")
ax.set_ylabel("Amplitude")
ax.set_title("Live Audio Signal")
ax.grid(True)

try:
    while True:
        # Record audio
        audio = sd.rec(int(DURATION * FS), samplerate=FS, channels=1, dtype='float32', device=11)
        sd.wait()
        sig = audio[:, 0]
        t = np.linspace(0, DURATION, len(sig), endpoint=False)

        # Update the plot
        line.set_data(t, sig)
        ax.set_xlim(0, DURATION)  # Keep x-axis fixed
        plt.pause(0.001)          # Short pause to refresh plot

except KeyboardInterrupt:
    print("Stopped.")
    plt.ioff()
    plt.show()

       
        
        

except KeyboardInterrupt:
    print("Recording Stopped.")

'''

'''
import numpy as np
import sounddevice as sd
import matplotlib.pyplot as plt
import time

FS = 44100      # Sampling rate
DURATION = 0.1  # Duration = 0.1s (100ms)

print("Started. Press Ctrl+C to stop.")

# Set up interactive plot
plt.ion()
fig, ax = plt.subplots(figsize=(10, 4))
line, = ax.plot([], [], lw=2)
ax.set_xlim(0, DURATION)
ax.set_ylim(-0.5, )  # Assuming normalized float32 audio
ax.set_xlabel("Time [s]")
ax.set_ylabel("Amplitude")
ax.set_title("Live Audio Signal")
ax.grid(True)

try:
    while True:
        # Record audio
        audio = sd.rec(int(DURATION * FS), samplerate=FS, channels=1, dtype='float32', device=11)
        sd.wait()
        sig = audio[:, 0]
        t = np.linspace(0, DURATION, len(sig), endpoint=False)

        # Update the plot
        line.set_data(t, sig)
        ax.set_xlim(0, DURATION)  # Keep x-axis fixed
        plt.pause(0.001)          # Short pause to refresh plot

except KeyboardInterrupt:
    print("Stopped.")
    plt.ioff()
    plt.show()
'''
