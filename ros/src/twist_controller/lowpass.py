
class LowPassFilter(object):
    def __init__(self, tau):
        self.tau = tau

        self.last_val = 0.
        self.ready = False

    def get(self):
        return self.last_val

    def filt(self, val, ts):
        if self.ready:
            a = 1. / (self.tau / ts + 1.)
            b = self.tau / ts / (self.tau / ts + 1.);
        
            val = a * val + b * self.last_val
        else:
            self.ready = True

        self.last_val = val
        return val

    def reset(self):
        self.ready = False
