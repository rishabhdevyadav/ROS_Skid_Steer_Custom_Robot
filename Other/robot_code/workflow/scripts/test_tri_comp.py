#!/usr/bin/env python

tagsLocations = [ [0, 0], [10, 0], [5, 10] ]
d1 = 15.911
d2 =  15.711 
d3 = 4.95
# 15.911,15.711, 4.95
A = tagsLocations[0][0]**2 + tagsLocations[0][1]**2 - d1**2
B = tagsLocations[1][0]**2 + tagsLocations[1][1]**2 - d2**2
C = tagsLocations[2][0]**2 + tagsLocations[2][1]**2 - d3**2
X32 = tagsLocations[2][0] - tagsLocations[1][0]
X13 = tagsLocations[0][0] - tagsLocations[2][0]
X21 = tagsLocations[1][0] - tagsLocations[0][0]

Y32 = tagsLocations[2][1] - tagsLocations[1][1]
Y13 = tagsLocations[0][1] - tagsLocations[2][1]
Y21 = tagsLocations[1][1] - tagsLocations[0][1]

x = (A * Y32 + B * Y13 + C * Y21)/(2.0*(tagsLocations[0][0]*Y32 + tagsLocations[1][0]*Y13 + tagsLocations[2][0]*Y21))
y = (A * X32 + B * X13 + C * X21)/(2.0*(tagsLocations[0][1]*X32 + tagsLocations[1][1]*X13 + tagsLocations[2][1]*X21))
# prev_x = x
# prev_y = y
print x, y, "location"
