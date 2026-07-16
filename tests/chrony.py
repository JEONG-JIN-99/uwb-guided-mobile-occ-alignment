import subprocess
import re
import time
import sys
import math
import matplotlib.pyplot as plt
from datetime import datetime

# ================= 설정 변수 =================
MASTER_IP = "192.168.0.12"
INTERVAL = 2.0
# ============================================

def get_chrony_offset(target_ip):
    try:
        result = subprocess.run(["chronyc", "-n", "sources"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True)
        for line in result.stdout.strip().split("\n"):
            if target_ip in line:
                match = re.search(r'\[\s*([+-]?\d+(?:\.\d+)?)\s*([µu]s|ms|ns|s)\]', line)
                if match:
                    val = float(match.group(1))
                    unit = match.group(2)
                    if unit in ("µs", "us"): offset_ms = val / 1000.0
                    elif unit == "ns": offset_ms = val / 1000000.0
                    elif unit == "s": offset_ms = val * 1000.0
                    else: offset_ms = val
                    return offset_ms
    except Exception: return None
    return None

def save_statistics_image(samples, timestamps):
    """오차 데이터를 그래프로 그려서 이미지로 저장합니다."""
    plt.figure(figsize=(10, 6))
    
    # 1. 시계열 그래프 그리기
    plt.plot(timestamps, samples, marker='o', linestyle='-', color='b', markersize=4, label='Offset (ms)')
    plt.axhline(0, color='r', linestyle='--', alpha=0.5) # 0 지점 기준선
    
    plt.title(f"Chrony Sync Offset Trend ({MASTER_IP})")
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
    filename = "chrony_sync_stats.png"
    plt.savefig(filename)
    print(f"\n[성공] 통계 그래프가 '{filename}'으로 저장되었습니다.")

def main():
    print(f"=== {MASTER_IP} 모니터링 시작 (Ctrl+C로 종료) ===")
    
    offset_samples = []
    timestamps = []
    start_time = time.time()
    
    try:
        while True:
            offset = get_chrony_offset(MASTER_IP)
            if offset is not None:
                offset_samples.append(offset)
                timestamps.append(time.time() - start_time)
                print(f"수집 {len(offset_samples)}회 | 오차: {offset:.4f} ms")
            time.sleep(INTERVAL)
    except KeyboardInterrupt:
        if len(offset_samples) > 1:
            save_statistics_image(offset_samples, timestamps)
        else:
            print("\n[알림] 데이터가 부족하여 그래프를 생성할 수 없습니다.")

if __name__ == "__main__":
    main()