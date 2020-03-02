import math

def circle_intersection(circle1, circle2):
        # return self.circle_intersection_sympy(circle1,circle2)
        x1,y1,r1 = circle1
        x2,y2,r2 = circle2
        # http://stackoverflow.com/a/3349134/798588
        dx,dy = x2-x1,y2-y1
        d = math.sqrt(dx*dx+dy*dy)
        if d > r1+r2:
            #print "#1"
            return None # no solutions, the circles are separate
        if d < abs(r1-r2):
            #print "#2"
            return None # no solutions because one circle is contained within the other
        if d == 0 and r1 == r2:
            #print "#3"
            return None # circles are coincident and there are an infinite number of solutions

        a = (r1*r1-r2*r2+d*d)/(2*d)
	#print 'r1', r1, 'a', a
        h = math.sqrt(round(r1, 2)**2 - round(a,2)**2)
        xm = x1 + a*dx/d
        ym = y1 + a*dy/d
        xs1 = xm + h*dy/d
        xs2 = xm - h*dy/d
        ys1 = ym - h*dx/d
        ys2 = ym + h*dx/d

        return (xs1,ys1),(xs2,ys2)

def dist_from_circ(circle, point):
	d = math.sqrt((circle[0] - point[0])**2 + (circle[1] - point[1])**2)
	if d <= circle[2]:
		return circle[2] - d
	else:
		return d - circle[2]

def trilateration():
	tagsLocations = [[0.0, 0.0], [10.0, 0.0], [5.0, 10.0]]

	global all_dist, prev_y, prev_x
	all_dist = [15.711, 15.811, 4.95]
        # all_dist = filter(lambda x:x != -1, all_dist)
        if len(all_dist) < 3:
                d1 = 0
                d2 = 0
                d3 = 0
                # print "No distances found"
        else:
                d1 = all_dist[0]
                d2 = all_dist[1]
                d3 = all_dist[2]
        if (d1 <= 0 or d2 <= 0 or d3 <= 0) or (d1 > 20 or d2 > 20 or d3 > 20):
                return prev_x, prev_y


	c1 = (tagsLocations[0][0], tagsLocations[0][1], d1)
	c2 = (tagsLocations[1][0], tagsLocations[1][1], d2)
	c3 = (tagsLocations[2][0], tagsLocations[2][1], d3)

	intersect1 = circle_intersection(c1, c2)
	intersect2 = circle_intersection(c2, c3)
	intersect3 = circle_intersection(c3, c1)

	allX, allY = [], []
	
	if intersect1 is not None:
		if dist_from_circ(c3, intersect1[0]) > dist_from_circ(c3, intersect1[1]):
			allX.append(intersect1[1][0])
			allY.append(intersect1[1][1])
		elif dist_from_circ(c3, intersect1[0]) < dist_from_circ(c3, intersect1[1]):
			allX.append(intersect1[0][0])
			allY.append(intersect1[0][1])
		else:
			allX.append(intersect1[0][0])
			allX.append(intersect1[1][0])
			allY.append(intersect1[0][1])
			allY.append(intersect1[1][1])


	if intersect2 is not None:
		if dist_from_circ(c1, intersect2[0]) > dist_from_circ(c1, intersect2[1]):
			allX.append(intersect2[1][0])
			allY.append(intersect2[1][1])
		elif dist_from_circ(c1, intersect2[0]) < dist_from_circ(c1, intersect2[1]):
			allX.append(intersect2[0][0])
			allY.append(intersect2[0][1])
		else:
			allX.append(intersect2[0][0])
			allX.append(intersect2[1][0])
			allY.append(intersect2[0][1])
			allY.append(intersect2[1][1])


	if intersect3 is not None:
		if dist_from_circ(c2, intersect3[0]) > dist_from_circ(c2, intersect3[1]):
			allX.append(intersect3[1][0])
			allY.append(intersect3[1][1])
		elif dist_from_circ(c2, intersect3[0]) < dist_from_circ(c2, intersect3[1]):
			allX.append(intersect3[0][0])
			allY.append(intersect3[0][1])
		else:
			allX.append(intersect3[0][0])
			allX.append(intersect3[1][0])
			allY.append(intersect3[0][1])
			allY.append(intersect3[1][1])


	x = float(sum(allX))/len(allX)
	y = float(sum(allY))/len(allY)

	return x, y





























if __name__ == '__main__':
	print trilateration()
