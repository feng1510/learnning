import numpy as np
from math import *
import matplotlib.pyplot as plt


theta = np.linspace(0, np.pi, 10)
x = 5.0*np.cos(theta)
y = 5.0*np.sin(theta)
s = theta*5.0

fx = np.poly1d(np.polyfit(s,x,5))
dfx = fx.deriv()

fy = np.poly1d(np.polyfit(s,y,5))
dfy = fy.deriv()

snew = np.linspace(0, 5*np.pi, 1000)

newx = fx(snew)
newy = fy(snew)

plt.plot(np.zeros(1000), snew, 'y')
plt.plot(np.zeros(1000)+1.5, snew, 'b')
plt.plot(np.zeros(1000)-1.5, snew, 'b')

def polyfit(coeffs, t):
    return coeffs[0] + coeffs[1]*t + coeffs[2]*t*t + coeffs[3]*t*t*t


def cubicPolyCurve1d(x0, dx0, x1, dx1, T):
    coeffs = np.zeros(4)
    coeffs[0] = x0
    coeffs[1] = dx0
    
    T2 = T*T
    
    coeffs[2] = (3*x1 - T*dx1 - 3*coeffs[0] - 2*coeffs[1]*T)/T2
    coeffs[3] = (dx1 - coeffs[1] - 2*coeffs[2]*T)/(3.0*T2)
    return coeffs
    
targets = []
for i in range(11):
    x0 = 0
    dx0 = 0  # np.tan(np.pi/6)
    x1 = -1.5 + i * 0.3
    dx1 = 0    
    targets.append(np.array([x0, dx0, x1, dx1]))
    

t = np.linspace(0, 10, 1000)
for i in range(11):
    tar = targets[i]
    coeffs = cubicPolyCurve1d(tar[0], tar[1], tar[2], tar[3], 10.0)
    d = polyfit(coeffs, t)
    plt.plot(d, t, 'g')
plt.axis([-8, 8, 0, 16])    
plt.show()


def frenet_to_cartesian1D(rs, rx, ry, rtheta, s_condition, d_condition):
    if fabs(rs - s_condition[0])>= 1.0e-6:
        print("The reference point s and s_condition[0] don't match")
        
    cos_theta_r = cos(rtheta)
    sin_theta_r = sin(rtheta)
    
    x = rx - sin_theta_r * d_condition[0]
    y = ry + cos_theta_r * d_condition[0]    

    return x, y

left_bound = []
right_bound = []

for i in range(1000):
    rs = snew[i]
    rx = fx(rs)
    ry = fy(rs)
    
    drx = dfx(rs)
    dry = dfy(rs)
    
    rtheta = atan2(dry, drx)
    
    l_s_condition = np.array([rs])
    l_d_condition = np.array([1.5])
    lx, ly = frenet_to_cartesian1D(rs, rx, ry, rtheta, l_s_condition, l_d_condition)
    left_bound.append(np.array([lx, ly]))
    
    r_s_condition = np.array([rs])
    r_d_condition = np.array([-1.5])
    rx, ry = frenet_to_cartesian1D(rs, rx, ry, rtheta, r_s_condition, r_d_condition)   
    right_bound.append(np.array([rx, ry]))
left_bound = np.array(left_bound)
right_bound = np.array(right_bound)

plt.plot(newx, newy, 'y')
plt.plot(left_bound[:,0],left_bound[:,1], 'b')
plt.plot(right_bound[:,0],right_bound[:,1], 'b')


#################################################


for i in range(11):
    tar = targets[i]
    coeffs = cubicPolyCurve1d(tar[0], tar[1], tar[2], tar[3], 10.0)
    d = polyfit(coeffs, t)
    traj = []
    for j in range(1000):
        rs = t[j]
        rx = fx(rs)
        ry = fy(rs)
        rtheta = atan2(dfy(rs), dfx(rs))
        
        s_condition = np.array([rs])
        d_condition = np.array([d[j]])
        x, y = frenet_to_cartesian1D(rs, rx, ry, rtheta, s_condition, d_condition)
        traj.append(np.array([x, y]))
    traj = np.array(traj)
    plt.plot(traj[:,0], traj[:,1], 'g')
plt.plot()
plt.show()

