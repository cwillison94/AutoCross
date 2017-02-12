import math
  

def slope(p1, p2):
    #p1 = [x1, y1]
    #p2 = [x2, y2]

    return (p2[1]-p1[1])/float(p2[0]-p1[0])

def line(p1, p2):
	m = slope(p1, p2)
	x_0, y_0 = p1
	b = y_0 - m * x_0
	return m, b

def line_calc(m, b, x):
    return m*x + b

def line_inv_calc(m, b, y):
    return (y - b)/m
    
def split_lines(lines, mid_point, height):
	lines_left = []
	lines_right = []
	
	for i in range(len(lines)):
		x1, y1, x2, y2 = lines[i]
		m, b = line((x1, y1), (x2, y2))

		x = line_inv_calc(m, b, height)
		print pt
		if pt < mid_point:
                    lines_left += [[x1, y1, x2, y2]]
		else:
                    lines_right+= [[x1, y1, x2, y2]]
	return lines_left, lines_right

	

def distance(p1, p2):
    return math.sqrt((p2[0] - p1[0])**2 +(p2[1] - p1[1])**2 )


if __name__=="__main__":
    #testing
    p1 = [100, 500]
    p2 = [150, 100]

    m = slope(p1, p2)
    dist = distance(p1, p2)

    print "P1 = ", p1
    print "p2 = ", p2
    print "slope = ", m
    print "distance = ", dist
    print "line = ", line(p1,p2)
