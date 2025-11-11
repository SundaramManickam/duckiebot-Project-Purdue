import collections, time

class MedianFilter:
    def __init__(self, k=5):
        self.k = k
        self.buf = collections.deque(maxlen=k)
    def add(self, x):
        self.buf.append(x)
    def value(self):
        b = sorted(self.buf)
        n = len(b)
        return b[n//2] if n else None

class HysteresisGate:
    def __init__(self, on_th, off_th):
        self.on_th = on_th
        self.off_th = off_th
        self.state = False
    def update(self, x):
        if not self.state and x <= self.on_th:
            self.state = True
        elif self.state and x >= self.off_th:
            self.state = False
        return self.state

class HoldTrue:
    # Keep True if condition stays True for hold_s seconds (debounce)
    def __init__(self, hold_s=0.5):
        self.hold_s = hold_s
        self.ts = None
        self.latched = False
    def update(self, cond):
        t = time.time()
        if cond:
            if self.ts is None: self.ts = t
            self.latched = (t - self.ts) >= self.hold_s
        else:
            self.ts = None
            self.latched = False
        return self.latched
