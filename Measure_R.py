#!/usr/bin/python

import rosbag
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
import easygui
import numpy.polynomial.polynomial as poly
from array import array
from scipy import interpolate

bags=1

for i in range(bags):
	filename = easygui.fileopenbox()
	bag = rosbag.Bag(filename)
	M1_speed = []
	M2_speed = []
	M3_speed = []
	M4_speed = []

	for topic, msg, t in bag.read_messages(topics='/crazyflie1/m1'):
		if msg.values[1] >= 20000:
			M1_speed.append((168000000/msg.values[0])* 2 * np.pi)

	for topic, msg, t in bag.read_messages(topics='/crazyflie1/m2'):
		if msg.values[1] >= 20000:
			M2_speed.append((168000000/msg.values[0])* 2 * np.pi)

	for topic, msg, t in bag.read_messages(topics='/crazyflie1/m3'):
		if msg.values[1] >= 20000:
			M3_speed.append((168000000/msg.values[0])* 2 * np.pi)

	for topic, msg, t in bag.read_messages(topics='/crazyflie1/m4'):
		if msg.values[1] >= 20000:
			M4_speed.append((168000000/msg.values[0])* 2 * np.pi)
		
	bag.close()

std_dev = [np.std(M1_speed), np.std(M2_speed), np.std(M3_speed), np.std(M3_speed)]
print "Standartd deviation for M1 sensor :"
print np.std(M1_speed)
print "Standartd deviation for M2 sensor :"
print np.std(M2_speed)
print "Standartd deviation for M3 sensor :"
print np.std(M3_speed)
print "Standartd deviation for M4 sensor :"
print np.std(M4_speed)

print "Standartd deviation for average of 4 motors :"
print np.mean(std_dev)

n, bins, patches = plt.hist(M1_speed, 50, normed=1, facecolor='green', alpha=0.75)

# add a 'best fit' line
mu = np.mean(M1_speed)
sigma = np.std(M1_speed)
y = mlab.normpdf( bins, mu, sigma)
l = plt.plot(bins, y, 'r--', linewidth=1)
plt.show()