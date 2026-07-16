import os
import csv

class ResultLogger:
    def __init__(self, target_dir_name="result"):
        # 1. 현재 파일의 절대 경로
        current_file = os.path.abspath(__file__)
        
        # 2. os.path.dirname을 이용해 부모 폴더로 3번 거슬러 올라감
        logger_dir = os.path.dirname(current_file) # .../code/logger
        code_dir = os.path.dirname(logger_dir)     # .../code
        project_root = os.path.dirname(code_dir)   # .../2026icufn(최상단 루트)
        
        # print("project_root : ", project_root)

        # 3. 프로젝트 최상단 루트 아래에 "result" 폴더 경로를 합침
        # 예: /home/ciderlab/2026icufn/result
        self.base_dir = os.path.join(project_root, target_dir_name)
        
        # 4. 폴더가 없으면 생성
        if not os.path.exists(self.base_dir):
            os.makedirs(self.base_dir)

    def log_t_result(self, mode, data_dict):
        """
        t_result 기록용 (mode: "uwb" 또는 "gps")
        """
        filename = f"{mode.lower()}_t_result.csv"
        self._write_to_csv(filename, data_dict)

    def log_a_result(self, mode, data_dict):
        """
        a_result 기록용 (mode: "uwb" 또는 "gps")
        """
        filename = f"{mode.lower()}_a_result.csv"
        self._write_to_csv(filename, data_dict)

    def _write_to_csv(self, filename, data_dict):
        filepath = os.path.join(self.base_dir, filename)
        file_exists = os.path.isfile(filepath)

        # 딕셔너리의 키(Key)들을 CSV의 첫 줄(헤더)로 사용
        with open(filepath, mode='a', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=data_dict.keys())
            
            # 파일이 처음 만들어질 때만 맨 위에 헤더(컬럼명) 추가
            if not file_exists:
                writer.writeheader()
                
            writer.writerow(data_dict)


if __name__ == "__main__":
    logger = ResultLogger("result")
