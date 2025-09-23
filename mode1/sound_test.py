import numpy as np
import os
import sys
import librosa
import joblib
import glob
from sklearn.svm import OneClassSVM
from typing import List

# --- 전역 설정 ---
ABNORMAL_AUDIO_DIR = "abnormal_audio" # <-- .wav 파일이 있는 폴더
MODEL_FILE = "one_class_svm_classifier.joblib"
SAMPLE_RATE = 48000
N_FEATURES = 1 

# --- 스펙트럼 중심 특징 추출 함수 ---
def extract_spectral_centroid_from_wav(audio_path: str) -> np.ndarray:
    """원본 .wav 파일에서 스펙트럼 중심 특징을 추출합니다."""
    audio_data, sr = librosa.load(audio_path, sr=SAMPLE_RATE, mono=True)
    
    # hop_length 값을 128로 줄여 더 많은 특징 프레임을 얻습니다.
    spectral_centroids = librosa.feature.spectral_centroid(
        y=audio_data, sr=sr, hop_length=128
    )
    
    # 1차원 배열로 평탄화
    features = spectral_centroids.flatten()
    
    return features

def normalize_features(features: np.ndarray) -> np.ndarray:
    """특징 배열을 0과 1 사이로 정규화합니다."""
    min_val = np.min(features)
    max_val = np.max(features)
    if (max_val - min_val) > 0:
        normalized_features = (features - min_val) / (max_val - min_val)
    else:
        normalized_features = np.zeros_like(features)
    return normalized_features

# --- 메인 학습 함수 ---
def main_training_process():
    # 1. 이상 타일 오디오 파일 목록 불러오기
    abnormal_files = glob.glob(os.path.join(ABNORMAL_AUDIO_DIR, "*.wav"))
    
    if not abnormal_files:
        print(f"[오류] '{ABNORMAL_AUDIO_DIR}' 폴더에 .wav 파일이 없습니다.")
        return

    # 2. 모든 .wav 파일에서 특징 추출
    all_features: List[np.ndarray] = []
    print("원본 .wav 파일에서 스펙트럼 중심 특징 추출 중...")
    for audio_file in abnormal_files:
        features = extract_spectral_centroid_from_wav(audio_file)
        all_features.append(features)
            
    # 3. 모든 특징 데이터를 가장 긴 길이에 맞춰 패딩
    if not all_features:
        print("[오류] 특징 추출에 실패했습니다.")
        return
        
    max_length = max(len(feat) for feat in all_features)
    print(f"-> 전체 데이터 중 가장 긴 스펙트럼 중심 특징 길이: {max_length}")
            
    padded_features = []
    for features in all_features:
        pad_width = max_length - len(features)
        padded = np.pad(features, (0, pad_width), 'constant')
        padded_features.append(padded)
        
    final_features = np.array(padded_features)
    
    # 4. 모델 학습에 사용할 데이터를 정규화합니다.
    normalized_data = np.apply_along_axis(normalize_features, 1, final_features)

    print(f"-> 총 {normalized_data.shape[0]}개의 샘플로 학습 시작.")

    # 5. One-Class SVM 모델 학습
    model = OneClassSVM(kernel='rbf', gamma='scale', nu=0.1)
    model.fit(normalized_data)
    
    # 6. 모델 저장
    joblib.dump(model, MODEL_FILE)
    print(f"\n-> 학습된 One-Class SVM 모델이 '{MODEL_FILE}' 파일로 저장되었습니다.")

if __name__ == "__main__":
    main_training_process()
