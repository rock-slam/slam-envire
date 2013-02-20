#!/usr/bin/env python
import numpy as np
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt

x = np.genfromtxt( 'samples2.dat' )
xmodel = x[:,0]
xmap = x[:,1]

print "model mu: {} sigma: {}".format( np.mean(xmodel), np.std(xmodel) )
print "map mu: {} sigma: {}".format( np.mean(xmap), np.std(xmap) )

# the histogram of the data
n, bins, patches = plt.hist(x[:,1], 50, normed=1, facecolor='green', alpha=0.75)

print np.mean(x[:,1])
print np.std(x[:,1]) 

# add a 'best fit' line
y = mlab.normpdf( bins, 0, 1)
l = plt.plot(bins, y, 'r--', linewidth=1)

plt.xlabel('Smarts')
plt.ylabel('Probability')
plt.title(r'$\mathrm{Histogram\ of\ IQ:}\ \mu=100,\ \sigma=15$')
#plt.axis([40, 160, 0, 0.03])
plt.grid(True)

plt.show()
