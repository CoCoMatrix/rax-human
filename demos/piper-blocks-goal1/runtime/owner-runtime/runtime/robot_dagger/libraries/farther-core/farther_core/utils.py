import time
from collections import Counter
from datetime import datetime, timezone

import numpy as np

try:
    from datetime import UTC
except ImportError:
    UTC = timezone.utc


def get_timestamp_ns():
    return int(datetime.now(UTC).timestamp() * 1e9)


def get_timestamp_us():
    return int(datetime.now(UTC).timestamp() * 1e6)


def get_timestamp_ms():
    return int(datetime.now(UTC).timestamp() * 1e3)


def datetime_from_ns(ns: int, tz: timezone = UTC):
    return datetime.fromtimestamp(ns / 1e9, tz)


def print_once(message: str):
    if message not in print_once.printed:
        print(message)
        print_once.printed.add(message)


print_once.printed = set()


def print_every(message: str, period: float):
    if message not in print_every.last_time:
        print_every.last_time[message] = time.monotonic()
    if time.monotonic() - print_every.last_time[message] >= period:
        print(message)
        print_every.last_time[message] = time.monotonic()


print_every.last_time = {}


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


class Interpolator:
    def __init__(self, start: np.ndarray | float, end: np.ndarray | float, duration: float, step: float = 0.01):
        self.start = start
        self.end = end
        self.duration = duration
        self.step = step
        self.t = 0
        self._start_time = None

    def _interpolate(self, t: float):
        return self.start + (self.end - self.start) * t

    def __next__(self):
        if self._start_time is None:
            self._start_time = time.time()
        else:
            time.sleep(self.step)
        if time.time() - self._start_time >= self.duration:
            raise StopIteration
        t = (time.time() - self._start_time) / self.duration
        return self._interpolate(t)

    def __iter__(self):
        return self


class DictInterpolator:
    def __init__(self, start: dict[str, float], end: dict[str, float], duration: float, step: float = 0.01):
        self.start = start
        self.end = end
        self.duration = duration
        self.step = step
        self.t = 0
        self._start_time = None

    def update(self, current_time: float, end: dict[str, float] | None = None):
        if self._start_time is None:
            self._start_time = current_time
        if current_time - self._start_time >= self.duration:
            return None
        t = (current_time - self._start_time) / self.duration
        if end is not None:
            self.end = end
        return self._interpolate(t)

    def _interpolate(self, t: float):
        if self.start.keys() != self.end.keys():
            self.start.update({k: 0.0 for k in self.end if k not in self.start})

        return {k: self.start[k] + (self.end[k] - self.start[k]) * t for k in self.end}

    def __next__(self):
        if self._start_time is None:
            self._start_time = time.time()
        else:
            time.sleep(self.step)
        if time.time() - self._start_time >= self.duration:
            raise StopIteration
        t = (time.time() - self._start_time) / self.duration
        return self._interpolate(t)

    def __iter__(self):
        return self


def dict_of_array_to_array_of_dict(d: dict[str, np.ndarray]):
    length = next(iter(d.values())).shape[0]
    return [{k: v[i] for k, v in d.items()} for i in range(length)]


def dict_of_array_iter(d: dict[str, np.ndarray], factor: int = 1):
    """
    Iterate over a dictionary of numpy arrays, yielding a dictionary of the same shape
    with each array element as a numpy array.

    Args:
        d (dict[str, np.ndarray]): A dictionary of numpy arrays.
        factor (int): The downsampling factor.

    Yields:
        dict[str, np.ndarray]: A dictionary of numpy arrays with each array element as a numpy array.
    """
    length = next(iter(d.values())).shape[0]
    for i in range(length):
        if i % factor == 0:
            yield {k: v[i] for k, v in d.items()}


def downsample(iterable, factor):
    """
    A generator that downsamples elements from an iterable by a given factor.

    Args:
        iterable: The input iterable (e.g., list, tuple, iterator).
        factor: The downsampling factor (must be a positive integer).

    Yields:
        The elements from the iterable at the specified interval.
    """
    if not isinstance(factor, int) or factor <= 0:
        raise ValueError("Factor must be a positive integer.")

    for i, item in enumerate(iterable):
        if i % factor == 0:
            yield item
