import numpy as np
import os
import sys
import time
import RPi.GPIO as GPIO
import sounddevice as sd
import librosa
import joblib
from scipy.signal import butter, sosfiltfilt
from typing import Tuple

# --- 전역 설정 (학습 코드와 동일하게 설정) ---
SAMPLE_RATE = 48000
CHANNELS = 2
SOLENOID_GPIO_PIN = 17
MODEL_FILE = "one_class_svm_classifier.joblib"
MIC_DEVICE_NAME = 'snd_rpi_googlevoicehat_soundcar'
BP_LOW = 1000.0
BP_HIGH = 4500.0
CLIP_DURATION_SEC = 5.0
N_MFCC = 20
THRESHOLD = 0.5

# --- DSP 유틸리티 함수 (기존 코드와 동일) ---
def butter_bandpass_sos(low_cut: float, high_cut: float, fs: int, order: int = 4):
    nyq = 0.5 * fs
    return butter(order, [low_cut / nyq, high_cut / nyq], btype='band', output='sos')

def gcc_phat(sig: np.ndarray, ref: np.ndarray, fs: int, max_tau: float | None = None) -> float:
    n = 1
    L = len(sig) + len(ref)
    while n < L:
        n <<= 1
    SIG = np.fft.rfft(sig, n=n)
    REF = np.fft.rfft(ref, n=n)
    R = SIG * np.conj(REF)
    R /= np.abs(R) + 1e-12
    cc = np.fft.irfft(R, n=n)
    max_shift = int(n // 2)
    if max_tau is not None:
        max_shift = min(max_shift, int(max_tau * fs))
    cc = np.concatenate((cc[-max_shift:], cc[:max_shift + 1]))
    shift = np.argmax(np.abs(cc)) - max_shift
    return shift / fs

def align_by_delay(x: np.ndarray, y: np.ndarray, fs: int, tau: float) -> Tuple[np.ndarray, np.ndarray]:
    shift = int(round(tau * fs))
    if shift > 0:
        y2 = np.concatenate([y[shift:], np.zeros(shift, dtype=y.dtype)])
    elif shift < 0:
        y2 = np.concatenate([np.zeros(-shift, dtype=y.dtype), y[:shift]])
    else:
        y2 = y.copy()
    n = min(len(x), len(y2))
    return x[:n], y2[:n]

def estimate_alpha(front: np.ndarray, rear: np.ndarray, fs: int) -> float:
    pre_s = 1
    pre_samples = int(pre_s * fs)
    if pre_samples >= len(front) or pre_samples >= len(rear):
        return 1.0
    f = front[:pre_samples]
    r = rear[:pre_samples]
    den = np.dot(r, r) + 1e-9
    return float(np.clip(np.dot(f, r) / den, 0.0, 1.0))

# --- 스펙트럼 중심 특징 추출 함수로 변경 ---
def extract_spectral_centroid_features(audio_data, sr):
    if audio_data.size == 0:
        return np.zeros((1, 1))
    
    spectral_centroids = librosa.feature.spectral_centroid(y=audio_data, sr=sr, hop_length=128)
    return spectral_centroids.flatten()

def normalize_features(features: np.ndarray) -> np.ndarray:
    """특징 배열을 0과 1 사이로 정규화합니다."""
    min_val = np.min(features)
    max_val = np.max(features)
    if (max_val - min_val) > 0:
        normalized_features = (features - min_val) / (max_val - min_val)
    else:
        normalized_features = np.zeros_like(features)
    return normalized_features

# --- 솔레노이드 제어 함수 (기존 코드와 동일) ---
def activate_solenoid():
    GPIO.output(SOLENOID_GPIO_PIN, GPIO.HIGH)
    time.sleep(0.1)
    GPIO.output(SOLENOID_GPIO_PIN, GPIO.LOW)

# --- 메인 실행 함수 ---
def main_run_process():
    try:
        model = joblib.load(MODEL_FILE)

        # GPIO 설정
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SOLENOID_GPIO_PIN, GPIO.OUT)

        # 마이크 장치 인덱스 찾기
        device_index = None
        for i, device in enumerate(sd.query_devices()):
            if MIC_DEVICE_NAME in device['name']:
                device_index = i
                break
        if device_index is None:
            print(f"[오류] '{MIC_DEVICE_NAME}' 장치를 찾을 수 없습니다.")
            sys.exit(1)

        print("[INFO] 실시간 타일 추론을 시작합니다. 'Enter' 키를 누르면 타격 및 추론을 시작합니다.")
        input("준비되면 Enter 키를 누르세요...")

        frames = []
        with sd.InputStream(samplerate=SAMPLE_RATE, channels=CHANNELS, device=device_index, dtype='float32') as stream:
            print("-> 타격음을 녹음합니다...")
            start_time = time.time()
            solenoid_triggered = False

            while True:
                data, overflowed = stream.read(1024)
                frames.append(data.copy())
                current_rec_time = time.time() - start_time
                
                if current_rec_time > 1.0 and not solenoid_triggered:
                    activate_solenoid()
                    solenoid_triggered = True

                if current_rec_time > 1.0 + CLIP_DURATION_SEC:
                    break
        
        recording = np.concatenate(frames)
        left_audio = recording[:, 0]
        right_audio = recording[:, 1]
        
        # --- 전처리 과정 ---
        gain_factor = estimate_alpha(right_audio, left_audio, SAMPLE_RATE)
        tau = gcc_phat(left_audio, right_audio, SAMPLE_RATE, max_tau=0.010)
        right_aligned, left_aligned = align_by_delay(right_audio, left_audio, SAMPLE_RATE, tau)
        sos = butter_bandpass_sos(BP_LOW, BP_HIGH, SAMPLE_RATE)
        right_bp = sosfiltfilt(sos, right_aligned)
        left_bp = sosfiltfilt(sos, left_aligned)
        clean_audio = right_bp - gain_factor * left_bp

        # 스펙트럼 중심 특징 추출
        features = extract_spectral_centroid_features(clean_audio, SAMPLE_RATE)
        
        # 정규화
        normalized_features = normalize_features(features)
        
        # 모델에 맞는 길이로 패딩
        # `train_oneclass.py` 실행 후 출력되는 '가장 긴 스펙트럼 중심 특징 길이'로 변경
        fixed_length = 1841 # <-- 이 값을 새로운 값으로 변경하세요!
        
        if len(normalized_features) > fixed_length:
            final_features = normalized_features[:fixed_length]
        else:
            pad_width = fixed_length - len(normalized_features)
            final_features = np.pad(normalized_features, (0, pad_width), 'constant')
            
        final_features = final_features.reshape(1, -1)

        # 모델 추론
        score = model.score_samples(final_features)
        
        print("\n--- 추론 결과 ---")
        print(f"-> 이상 타일 유사도 점수: {score[0]:.4f}")
        
        if score[0] > THRESHOLD:
            print("-> 이 타일은 '이상'으로 판단됩니다. 🚨")
        else:
            print("-> 이 타일은 '정상'으로 판단됩니다. ✅")

    except Exception as e:
        print(f"[오류] 추론 중 오류가 발생했습니다: {e}")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main_run_process()
