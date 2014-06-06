#!/usr/bin/python
# Attempt to simulate gyro unbiasing with KF

import numpy as np
import Gnuplot
import Gnuplot.funcutils

# Simulate heading/gyro measurements
dt = 0.050
x = np.matrix('250.0; 0.0; 0.0')
z = np.matrix('0.0; 0.0')
A = np.matrix('1.0, 0.010, 0.0; 0.0, 1.0, 0.0; 0.0, 0.0, 1.0')
H = np.matrix('0.0, 0.0, 0.0; 0.0, 0.0, 0.0')
K = np.matrix('0.0, 0.0; 0.0, 0.0; 0.0, 0.0')
Q = np.matrix('0.25, 0.0, 0.0; 0.0, 0.1, 0.0; 0.0, 0.0, 0.00001')
R = np.matrix('1.0, 0.0; 0.0, 0.01')
P = np.matrix('1.0, 0.0, 0.0; 0.0, 1000.0, 0.0; 0.0, 0.0, 1000.0')
I = np.matrix('1.0, 0.0, 0.0; 0.0, 1.0, 0.0; 0.0, 0.0, 1.0')

def kf():
    global x, z, A, H, K, P, Q, R
    x = A * x
    P = A * P * A.T + Q
    tmp = H*P*H.T + R
    K = P * H.T * np.linalg.inv(tmp)
    x = x + K * (z - H * x)
    P = (I - K * H) * P

for i in range(0, 30):
    z[0] = 250.0
    z[1] = 0
    H = np.matrix("1.0, 0.0, 0.0; 0.0, 0.0, 0.0")
    kf()
    for j in range(0, 100):
        z[0] = 0.0
        z[1] = np.random.normal(loc=2.0, scale=0.2, size=None)
        H = np.matrix('0.0, 0.0, 0.0; 0.0, 1.0, 1.0')
        kf()
        print "{0} {1} {2} {3}".format(x.item(0), z.item(1), x.item(1), x.item(2))

print '-------------'

for i in range(0, 10):
    z[0] = 250.0 - i*2
    z[1] = 0
    H = np.matrix("1.0, 0.0, 0.0; 0.0, 0.0, 0.0")
    kf()
    for j in range(0, 100):
        z[0] = 0.0
        z[1] = -2 + np.random.normal(loc=2.0, scale=0.2, size=None)
        H = np.matrix('0.0, 0.0, 0.0; 0.0, 1.0, 1.0')
        kf()
        print "{0} {1} {2} {3}".format(x.item(0), z.item(1), x.item(1), x.item(2))


# g = Gnuplot.Gnuplot(debug=1)
# g('set style data linespoints')
# g.title('Test')
# g.xlabel('x')
# g.ylabel('y')
# d = []
# y = 0
# x = 0
    # x += 1
    # y = np.random.normal(loc=2.0, scale=0.2, size=None)
    # d.append([x, y])
# g.plot(d)
# raw_input('Press return to continue\n')
    # print "x = ", x
    # print "\n"

