"""Small wall-clock timing summaries; these are not GPU kernel timings."""


class TimingStats:
    def __init__(self):
        self.count = 0
        self.total = {}
        self.maximum = {}
        self.latest = {}

    def record(self, **seconds):
        self.count += 1
        self.latest = {key: value * 1000 for key, value in seconds.items()}
        for key, value in self.latest.items():
            self.total[key] = self.total.get(key, 0.0) + value
            self.maximum[key] = max(self.maximum.get(key, 0.0), value)

    def summary(self):
        return {
            "samples": self.count,
            "mean_ms": {key: value / self.count for key, value in self.total.items()},
            "max_ms": dict(self.maximum),
            "latest_ms": dict(self.latest),
        }
