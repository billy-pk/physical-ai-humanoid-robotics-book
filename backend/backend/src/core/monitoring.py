import time
from collections import defaultdict

class Metrics:
    def __init__(self):
        self.request_count = defaultdict(int)
        self.error_count = defaultdict(int)
        self.status_codes = defaultdict(lambda: defaultdict(int))
        self.start_times = {}

    def increment_request(self, endpoint: str, method: str):
        self.request_count[f"{method} {endpoint}"] += 1

    def increment_error(self, endpoint: str, method: str):
        self.error_count[f"{method} {endpoint}"] += 1

    def record_status_code(self, endpoint: str, method: str, status_code: int):
        self.status_codes[f"{method} {endpoint}"][status_code] += 1

    def start_timer(self, request_id: str):
        self.start_times[request_id] = time.time()

    def stop_timer(self, request_id: str) -> float:
        if request_id in self.start_times:
            duration = (time.time() - self.start_times[request_id]) * 1000  # milliseconds
            del self.start_times[request_id]
            return duration
        return 0.0

    def get_metrics(self):
        return {
            "request_count": dict(self.request_count),
            "error_count": dict(self.error_count),
            "status_codes": {ep: dict(codes) for ep, codes in self.status_codes.items()}
        }

metrics = Metrics()