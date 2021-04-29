import numpy as np
from math import *
import matplotlib.pyplot as plt

theta = np.linspace(0, np.pi, 10)
x = 5.0*np.cos(theta)
y = 5.0*np.sin(theta)
s = theta*5.0

fx = np.poly1d(np.polyfit(s,x,5))
dfx = fx.deriv()
ddfx = dfx.deriv()
dddfx = ddfx.deriv()


fy = np.poly1d(np.polyfit(s,y,5))
dfy = fy.deriv()
ddfy = dfy.deriv()
dddfy = ddfy.deriv()

snew = np.linspace(0, 5*np.pi, 1000)

newx = fx(snew)
dnewx = dfx(snew)
ddnewx = ddfx(snew)

newy = fy(snew)
dnewy = dfy(snew)
ddnewy = ddfy(snew)

# alpha = np.arctan2(dnewy, dnewx)
# kappa = (ddnewx*dnewy-ddnewy*dnewx)/(dnewx*dnewx+dnewy*dnewy)**(3.0/2.0)
# norm to [-pi, pi]
def NormalizeAngle(angle):
    a = fmod(angle+np.pi, 2*np.pi)
    if a < 0.0:
        a += (2.0*np.pi)        
    return a - np.pi

def polyfit(coeffs, t, order):
    if order == 0:
        return coeffs[0] + coeffs[1]*t + coeffs[2]*t*t + coeffs[3]*t*t*t
    if order == 1:
        return coeffs[1] + 2*coeffs[2]*t+3*coeffs[3]*t*t
    if order == 2:
        return 2*coeffs[2]+6*coeffs[3]*t
    if order == 3:
        return 6*coeffs[3]
    else:
        return 0.0


def cubicPolyCurve1d(x0, dx0, x1, dx1, T):
    coeffs = np.zeros(4)
    coeffs[0] = x0
    coeffs[1] = dx0
    
    T2 = T*T
    
    coeffs[2] = (3*x1 - T*dx1 - 3*coeffs[0] - 2*coeffs[1]*T)/T2
    coeffs[3] = (dx1 - coeffs[1] - 2*coeffs[2]*T)/(3.0*T2)
    return coeffs

def ComputeCurvature(dx, ddx, dy, ddy):
    a = dx*ddy - dy*ddx
    norm_square = dx*dx+dy*dy
    norm = sqrt(norm_square)
    b = norm*norm_square
    return a/b

def ComputeCurvatureDerivative(dx, ddx, dddx, dy, ddy, dddy):
    a = dx*ddy-dy*ddx
    b = dx*dddy-dy*dddx
    c = dx*ddx+dy*ddy
    d = dx*dx+dy*dy
    return (b*d-3.0*a*c)/(d*d*d)

def CalculateTheta(rtheta, rkappa, l, dl):
    return NormalizeAngle(rtheta + atan2(dl, 1-l*rkappa))

def CalculateKappa(rkappa, rdkappa, l, dl, ddl):
    denominator = (dl * dl + (1 - l * rkappa) * (1 - l * rkappa))
    if fabs(denominator) < 1e-8:
        return 0.0
    denominator = pow(denominator, 1.5)
    numerator = (rkappa + ddl - 2 * l * rkappa * rkappa -
                           l * ddl * rkappa + l * l * rkappa * rkappa * rkappa +
                           l * dl * rdkappa + 2 * dl * dl * rkappa)
    return numerator / denominator

snew = np.linspace(0, 5*np.pi, 1000)


def frenet_to_cartesian1D(rs, rx, ry, rtheta, s_condition, d_condition):
    if fabs(rs - s_condition[0])>= 1.0e-6:
        print("The reference point s and s_condition[0] don't match")
        
    cos_theta_r = cos(rtheta)
    sin_theta_r = sin(rtheta)
    
    x = rx - sin_theta_r * d_condition[0]
    y = ry + cos_theta_r * d_condition[0]    

    return x, y
t = np.linspace(0, 15.71, 1000)
left_bound = []
right_bound = []

# for i in range(1000):
#     rs = snew[i]
#     rx = fx(rs)
#     ry = fy(rs)
    
#     drx = dfx(rs)
#     dry = dfy(rs)
    
#     rtheta = atan2(dry, drx)
    
#     l_s_condition = np.array([rs])
#     l_d_condition = np.array([1.5])
#     lx, ly = frenet_to_cartesian1D(rs, rx, ry, rtheta, l_s_condition, l_d_condition)
#     left_bound.append(np.array([lx, ly]))
    
#     r_s_condition = np.array([rs])
#     r_d_condition = np.array([-1.5])
#     rx, ry = frenet_to_cartesian1D(rs, rx, ry, rtheta, r_s_condition, r_d_condition)   
#     right_bound.append(np.array([rx, ry]))
# left_bound = np.array(left_bound)
# right_bound = np.array(right_bound)

# plt.plot(newx, newy, 'y')
# plt.plot(left_bound[:,0],left_bound[:,1], 'b')
# plt.plot(right_bound[:,0],right_bound[:,1], 'b')

targets = []
for i in range(5):
    x0 = 0
    dx0 = 0 #np.tan(np.pi/6)
    x1 = -1.2 + i * 0.6
    dx1 = 0    
    targets.append(np.array([x0, dx0, x1, dx1]))

thetas_buf = []
kappas_buf = []
# for i in range(5):
#     tar = targets[i]
#     coeffs = cubicPolyCurve1d(tar[0], tar[1], tar[2], tar[3], 15.71)
#     traj = []
#     theta_buf = []
#     kappa_buf = []
#     for j in range(1000):
#         rs = t[j]
#         rx = fx(rs)
#         ry = fy(rs)
#         drx = dfx(rs)
#         dry = dfy(rs)
#         ddrx = ddfx(rs)
#         ddry = ddfy(rs)
#         dddrx = dddfx(rs)
#         dddry = dddfy(rs)
#         rtheta = atan2(dry, drx)
#         rkappa = ComputeCurvature(drx, ddrx, dry, ddry)
#         rdkappa = ComputeCurvatureDerivative(drx, ddrx, dddrx, dry, ddry, dddry)
        
#         l = polyfit(coeffs, t[j], 0)
#         dl = polyfit(coeffs, t[j], 1)
        
#         theta = CalculateTheta(rtheta, rkappa, l, dl)
#         theta_buf.append(theta)
        
#         ddl = polyfit(coeffs, t[j], 2)
        
#         kappa = CalculateKappa(rkappa, rdkappa, l, dl, ddl)
#         kappa_buf.append(kappa)
        
#         s_condition = np.array([rs])
#         d_condition = np.array([l])
#         x, y = frenet_to_cartesian1D(rs, rx, ry, rtheta, s_condition, d_condition)
#         traj.append(np.array([x, y]))
#     thetas_buf.append(np.array(theta_buf))
#     kappas_buf.append(np.array(kappa_buf))
#     traj = np.array(traj)
#     plt.plot(traj[:,0], traj[:,1], 'g')

# for i in range(5):
#     T = 15
#     x0 = 5
#     dx0 = 0
#     x1 = -5 - 1.2 + 0.6*i;
#     dx1 = 0;
#     dcoeffs = cubicPolyCurve1d(x0, dx0, x1, dx1, T)

#     x0 = 0
#     dx0 = 1
#     x1 = 0
#     dx1 = -1;
#     scoeffs = cubicPolyCurve1d(x0, dx0, x1, dx1, T)
#     traj = []

#     for j in range(1000):
#         t = 0.015 * j
#         d = polyfit(dcoeffs, t, 0)
#         s = polyfit(scoeffs, t, 0)
#         traj.append(np.array([d, s]))
#     traj = np.array(traj)
#     plt.plot(traj[:,0], traj[:,1], 'k')



# plt.show()
# for i in range(11):
#     plt.plot(thetas_buf[i])   
# plt.show()


# for i in range(11):
#     plt.plot(kappas_buf[i])   
# plt.show()

ycoeffs = np.zeros(4)
ycoeffs[0] = 10
ycoeffs[1] = 12
ycoeffs[2] = -7
ycoeffs[3] = 1

xcoeffs = np.zeros(4)
xcoeffs[0] = 0
xcoeffs[1] = 7
xcoeffs[2] = 0
xcoeffs[3] = 0

xyX = []
xyY = []
left_bound = []
right_bound = []
t = 0
for i in range(6000):
    t = i*0.001
    xyX.append(polyfit(xcoeffs, t, 0))
    xyY.append(polyfit(ycoeffs, t, 0))
plt.plot(xyX, xyY)   
plt.axis('equal')
plt.show()

