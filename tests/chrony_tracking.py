import subprocess
import re
import time
import sys
import math
import matplotlib.pyplot as plt
from datetime import datetime

# ================= 설정 변수 =================
MASTER_IP = "192.168.0.12"  # 그래프 제목 표시용
INTERVAL = 2.0
TOTAL_SAMPLES = 100         # 총 수집 횟수 제한
# ============================================

def get_chrony_offset():
    """chronyc tracking에서 System time(최종 정제된 오차)을 추출합니다."""
    try:
        result = subprocess.run(["chronyc", "tracking"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True)
        
        for line in result.stdout.strip().split("\n"):
            if "System time" in line:
                match = re.search(r'([\d.]+)\s+seconds\s+(fast|slow)', line)
                if match:
                    val = float(match.group(1))
                    direction = match.group(2)
                    
                    offset_sec = val if direction == "fast" else -val
                    offset_ms = offset_sec * 1000.0
                    return offset_ms
    except Exception:
        return None
    return None

def save_statistics_image(samples, timestamps):
    """오차 데이터를 그래프로 그려서 이미지로 저장합니다."""
    plt.figure(figsize=(10, 6))
    
    # 1. 시계열 그래프 그리기
    plt.plot(timestamps, samples, marker='o', linestyle='-', color='g', markersize=4, label='System Time Offset (ms)')
    plt.axhline(0, color='r', linestyle='--', alpha=0.5)
    
    plt.title(f"Chrony System Time Offset Trend (Tracking Mode - 100 Samples)")
    plt.xlabel("Time (seconds elapsed)")
    plt.ylabel("Offset (ms)")
    plt.grid(True, linestyle='--', alpha=0.7)
    
    # 2. 통계 텍스트 박스 추가
    stats_text = (
        f"Count: {len(samples)}\n"
        f"Mean: {sum(samples)/len(samples):.4f} ms\n"
        f"Max: {max(samples):.4f} ms\n"
        f"Min: {min(samples):.4f} ms\n"
        f"StdDev: {math.sqrt(sum((x - sum(samples)/len(samples))**2 for x in samples)/len(samples)):.4f} ms"
    )
    plt.text(0.02, 0.95, stats_text, transform=plt.gca().transAxes, fontsize=10,
             verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    # 3. 파일 저장
    filename = "chrony_tracking_stats.png"
    plt.savefig(filename)
    print(f"\n[성공] 100회 측정 완료! 정밀 통계 그래프가 '{filename}'으로 저장되었습니다.")

def main():
    print(f"=== Chrony Tracking 100회 제한 모니터링 시작 ===")
    print(f"목표 마스터: {MASTER_IP} | 인터벌: {INTERVAL}초 (총 예상 시간: 약 {TOTAL_SAMPLES * INTERVAL}초)")
    print("-" * 50)
    
    offset_samples = []
    timestamps = []
    start_time = time.time()
    
    try:
        # 1부터 100까지 지정된 횟수만큼 반복하는 for문으로 교체
        for i in range(1, TOTAL_SAMPLES + 1):
            offset = get_chrony_offset()
            if offset is not None:
                offset_samples.append(offset)
                timestamps.append(time.time() - start_time)
                print(f"수집 {i}/{TOTAL_SAMPLES}회 | 실제 시계 오차: {offset:.6f} ms")
            else:
                print(f"수집 {i}/{TOTAL_SAMPLES}회 | Chrony 데이터를 가져오는 데 실패했습니다.")
                
            time.sleep(INTERVAL)
            
        # 100번 다 돌면 자동으로 그래프 저장
        if len(offset_samples) > 1:
            save_statistics_image(offset_samples, timestamps)
            
    except KeyboardInterrupt:
        # 중간에 사용자가 종료하더라도 모인 데이터가 있다면 저장
        print("\n\n[알림] 사용자에 의해 모니터링이 중단되었습니다.")
        if len(offset_samples) > 1:
            save_statistics_image(offset_samples, timestamps)
        else:
            print("[알림] 수집된 데이터가 부족하여 그래프를 생성하지 않고 종료합니다.")

if __name__ == "__main__":
    main()