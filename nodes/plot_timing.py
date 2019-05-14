import numpy as np 
import matplotlib.pyplot as plt

data = np.loadtxt('timing.txt')

dt_image = data[:,0]
dt_recv = data[:,1]

plt.plot(dt_image,'.-')
plt.plot(dt_recv,'.r')
plt.xlabel('(index)')
plt.ylabel('(sec)')
plt.grid('on')
plt.show()
