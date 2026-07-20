import csv
import datetime
import os
import threading

class ResultLogger:
    def __init__(
        self,
        target_dir_name="result",
        experiment_id=None,
        node_id=None,
        fieldnames=None,
    ):
        """
        결과 로거를 생성한다.

        node_id가 없으면 기존 방식처럼 result/ 바로 아래의 CSV에 기록한다.
        node_id가 "tx" 또는 "rx"이면 실험별 폴더를 만들고 <node_id>.csv에
        기록한다. Tx/Rx가 같은 실험 폴더를 공유해야 할 때는 양쪽에 동일한
        experiment_id를 전달한다.
        """
        # 1. 현재 파일의 절대 경로
        current_file = os.path.abspath(__file__)

        # 2. os.path.dirname을 이용해 부모 폴더로 3번 거슬러 올라감
        logger_dir = os.path.dirname(current_file) # .../code/logger
        code_dir = os.path.dirname(logger_dir)     # .../code
        project_root = os.path.dirname(code_dir)   # .../2026icufn(최상단 루트)

        # 3. 프로젝트 최상단 루트 아래에 "result" 폴더 경로를 합침
        # 예: /home/ciderlab/2026icufn/result
        self.base_dir = os.path.join(project_root, target_dir_name)
        os.makedirs(self.base_dir, exist_ok=True)

        self.experiment_id = None
        self.node_id = None
        self.experiment_dir = None
        self.csv_path = None
        self._csv_file = None
        self._writer = None
        self._fieldnames = None
        self._lock = threading.Lock()

        if node_id is not None:
            self._initialize_experiment_log(experiment_id, node_id, fieldnames)

    def _initialize_experiment_log(self, experiment_id, node_id, fieldnames):
        node_id = node_id.lower()
        if node_id not in ("tx", "rx"):
            raise ValueError("node_id must be 'tx' or 'rx'")
        if not fieldnames:
            raise ValueError("fieldnames are required for an experiment log")

        fieldnames = tuple(fieldnames)
        required_fields = {"experiment_id", "node_id"}
        if not required_fields.issubset(fieldnames):
            raise ValueError(
                "fieldnames must contain experiment_id and node_id"
            )

        if experiment_id is None:
            base_id = datetime.datetime.now().strftime(
                f"%Y%m%d_%H%M%S_{node_id}"
            )
            experiment_id = self._find_available_experiment_id(base_id)

        self.experiment_id = str(experiment_id)
        self.node_id = node_id
        self._fieldnames = fieldnames
        self.experiment_dir = os.path.join(
            self.base_dir,
            self.experiment_id,
        )
        os.makedirs(self.experiment_dir, exist_ok=True)
        self.csv_path = os.path.join(self.experiment_dir, f"{node_id}.csv")

        file_exists = os.path.isfile(self.csv_path)
        if file_exists:
            with open(self.csv_path, newline="", encoding="utf-8") as csv_file:
                existing_header = next(csv.reader(csv_file), [])
            if existing_header != list(fieldnames):
                raise ValueError(
                    f"CSV header mismatch in {self.csv_path}: "
                    f"{existing_header!r}"
                )

        self._csv_file = open(
            self.csv_path,
            mode="a",
            newline="",
            encoding="utf-8",
        )
        self._writer = csv.DictWriter(
            self._csv_file,
            fieldnames=self._fieldnames,
        )
        if not file_exists:
            self._writer.writeheader()
            self._csv_file.flush()

    def _find_available_experiment_id(self, base_id):
        experiment_id = base_id
        suffix = 1
        while os.path.exists(os.path.join(self.base_dir, experiment_id)):
            experiment_id = f"{base_id}_{suffix:02d}"
            suffix += 1
        return experiment_id

    def log_sample(self, data_dict):
        """실험별 CSV에 하나의 Tx/Rx 샘플을 기록하고 즉시 반영한다."""
        if self._writer is None:
            raise RuntimeError(
                "log_sample requires node_id and fieldnames in ResultLogger"
            )

        row = dict(data_dict)
        row.setdefault("experiment_id", self.experiment_id)
        row.setdefault("node_id", self.node_id)

        unknown_fields = set(row) - set(self._fieldnames)
        if unknown_fields:
            raise ValueError(
                f"unknown log fields: {sorted(unknown_fields)!r}"
            )

        # 아직 측정할 수 없는 스키마 필드는 빈 CSV 셀로 남긴다.
        complete_row = {
            fieldname: row.get(fieldname, "")
            for fieldname in self._fieldnames
        }
        with self._lock:
            self._writer.writerow(complete_row)
            self._csv_file.flush()

    def close(self):
        """열려 있는 실험 CSV를 닫는다."""
        with self._lock:
            if self._csv_file is not None:
                self._csv_file.close()
                self._csv_file = None
                self._writer = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

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
        with self._lock:
            with open(filepath, mode='a', newline='', encoding='utf-8') as f:
                writer = csv.DictWriter(f, fieldnames=data_dict.keys())

                # 파일이 처음 만들어질 때만 맨 위에 헤더(컬럼명) 추가
                if not file_exists:
                    writer.writeheader()

                writer.writerow(data_dict)


if __name__ == "__main__":
    logger = ResultLogger("result")
