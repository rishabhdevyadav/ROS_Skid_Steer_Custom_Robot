#!/usr/bin/python
from __future__ import division

from matplotlib import pyplot as plt
import numpy as np
lines = [line.rstrip('\n') for line in open('angles9.txt')]
lines = lines[20:]
print lines
lines = map(float, lines)

values = np.asarray(lines)
t = range(1, len(lines))

plt.plot(values)
plt.show()