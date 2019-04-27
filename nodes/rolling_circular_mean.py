import numpy as np

class RollingCircularMean(object):

    def __init__(self, size=800):
        self.size = size
        self.data = []

    def insert_data(self,item):
        self.data.append(np.deg2rad(item))
        if len(self.data) > self.size:
            self.data.pop(0)

    def value(self):
        if self.data:
            return np.rad2deg(circmean(self.data))
        else:
            return 0.0 

# utility func
# ---------------------------------------------------------------------------------
def circmean(alpha,axis=None):
    mean_angle = np.arctan2(np.mean(np.sin(alpha),axis),np.mean(np.cos(alpha),axis))
    return mean_angle
