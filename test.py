import numpy as np
import wrap
import matplotlib.pyplot as plt
import math
from numpy.random import randn
import filterpy.kalman

def compute_dog_data(z_var, process_var, count=1, dt=1.):
    "returns track, measurements 1D ndarrays"
    x, vel = 0., 1.
    z_std = math.sqrt(z_var) 
    p_std = math.sqrt(process_var)
    xs, zs = [], []
    for _ in range(count):
        v = vel + (randn() * p_std)
        x += v*dt        
        xs.append(x)
        zs.append(x + randn() * z_std)        
    return np.array(xs), np.array(zs)

x0 = np.array([[0, 0]], dtype=np.float32)
cov0 = np.eye(2, dtype=np.float32)
A = np.array([[1, 0.1], 
              [0, 1]], dtype=np.float32)
C = np.array([[1., 0]], dtype=np.float32)
W = np.array( [[0., 0.   ],
              [0.,  0.001]], dtype=np.float32)
V = 5

kf = wrap.KalmanFilter()
#import pdb; pdb.set_trace()
kf.set(A, C, W, np.array([[5]], dtype=np.float32), x0.T, cov0)
kf2 = filterpy.kalman.KalmanFilter(dim_x=2, dim_z=1)
kf2.F = A
kf2.H = C
kf2.x = x0.T
kf2.P = cov0
kf2.Q = W
kf2.R *= V  
track, zs = compute_dog_data(3., .02, 50)

xs = []
for z in zs:
    #import pdb; pdb.set_trace()
    kf.predict()
    kf.update(np.array([[z]], dtype=np.float32))
    kf2.predict()
    kf2.update(z)
    
    print('ours', kf.get_state())
    print('==================')
    print('baseline', kf2.x)
    print('*********************')
