import time
from collections import Counter


class FrequencyCounter:
    def __init__(self, period):
        self.period = period
        self.counter = Counter()
        self.start_time = {}

    def update(self, name: str):
        if name not in self.start_time:
            self.start_time[name] = time.time()
            self.counter[name] = 0

        self.counter[name] += 1
        if time.time() - self.start_time[name] >= self.period:
            print(f"Frequency {name}: {self.counter[name] / self.period}")
            self.counter[name] = 0
            self.start_time[name] = time.time()
