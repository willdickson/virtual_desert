import math

class LowpassFilter(object):

    def __init__(self, fcut):
        self.fcut = fcut
        self.is_first = True
        self.value = 0.0

    @property
    def time_constant(self):
        rc = 1.0/(2.0*math.pi*self.fcut)
        return rc

    def update(self,x,dt):
        if self.is_first:
            self.value = x
            self.is_first = False
        else:
            coeff_0 = dt/(self.time_constant + dt)
            coeff_1 = 1.0 - coeff_0 
            self.value = coeff_0*x + coeff_1*self.value

    def reset(self):
        self.is_first = True
