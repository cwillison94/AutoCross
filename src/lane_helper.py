import math

def slope(p1, p2):
    #p1 = [x1, y1]
    #p2 = [x2, y2]

    return (p2[1]-p1[1])/float(p2[0]-p1[0])

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
