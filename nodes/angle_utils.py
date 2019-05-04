import numpy as np

class AngleAccumulator(object):

    def __init__(self):
        self.value = None
        self.is_first = True
        
    def reset(self):
        self.is_first = True
                
    def update(self,angle):
        if self.is_first:
            self.value = angle  
            self.is_first = False
        else:
            delta_theta = smallest_signed_angle_between(self.value, angle)
            self.value = self.value + delta_theta
        return self.value
        

        
        
class AngleFixer(object):
    
    def __init__(self, size=30):
        self.size = size
        self.data = []

    def fix_data(self,item):
        self.data.append(item)
        if len(self.data) > self.size:
            self.mean_angle = np.mean(self.data)
            self.data.pop(0)
            if (self.data[-1]-self.mean_angle)>=130:
                self.data[-1] = self.data[-1]-180.0
                return self.data[-1]
            elif (self.data[-1]-self.mean_angle)<=-130:
                self.data[-1] = self.data[-1]+180.0
                return self.data[-1]
            else:
                return self.data[-1]
        else:
            return self.data[-1]


def smallest_signed_angle_between(theta1, theta2):
    x = float(np.radians(theta1))
    y = float(np.radians(theta2))
    a = np.arctan2(np.sin(y-x), np.cos(y-x))
    a = (np.degrees(a))
    return a
