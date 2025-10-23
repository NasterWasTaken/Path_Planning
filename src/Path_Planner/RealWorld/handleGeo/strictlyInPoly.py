import sys
import numpy as np

#Checks if q is in the line segment 'pr' with p, q, r being colinear points

def onSegment(p, q, r):
    
    output = 0
    
    if max(p[0], r[0]) >= q[0] >= min(p[0], r[0]) and max(p[1], r[1]) >= q[1] >= min(p[1], r[1]):
        
        output = 1
    
    return output

def orientation(p, q, r):

    output = 0
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    
    if val == 0.0:
        
        output = 0 #Colinear
    
    elif val > 0.0:
        
        output = 1 #Clock Wise
        
    else:
        
        output = 2 #Counterclock Wise
        
    return output

def doIntersect(p1, q1, p2, q2):
    
    output = 0
    
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)
    
    if (o1 != o2 and o3 != o4) or (o1 == 0 and onSegment(p1, p2, q1)) or (o2 == 0 and onSegment(p1, q2, q1)) or (o3 == 0 and onSegment(p2, p1, q2)) or (o4 == 0 and onSegment(p2, q1, q2)):
        
        output = 1
    
    return output

def check(p, coords):
    
    # Point for the line segment that goes from p to infinite
    extreme = [sys.maxsize, p[1]]
    
    count, output, i, next, condition = 0, 0,0, 0, 1
    vertices = coords.shape[0]
    
    while condition == 1:
        
        next = (i+1) % vertices
        
        if doIntersect(coords[i], coords[next], p, extreme):
            
            if orientation(coords[i], p, coords[next]) == 0:
                
                return onSegment(coords[i], p, coords[next])
            
            count += 1
        
        i = next
        
        if i != 0:
            
            condition = 1
        
        else:
            
            condition = 0
    
    if count % 2 == 1:
        
        output = 1
    
    return output

def all_zero(array):
    
    output = 1
    
    i_max = array.shape[0]
    j_max = array.shape[1]
    h_max = array.shape[2]
    
    for i in range(i_max):
        for j in range(j_max):
            for h in range(h_max):
                
                if array[i][j][h] != 0:
                    
                    output = 0
                    return output
    
    return output

def strictlyInPoly(megaNodes, xMin, yMin, xBoxMax, xBoxMin, yBoxMax, yBoxMin, 
                   nodeDistance, nodeIntervalOffset, shiftX, shiftY, polygonCoordinates, xNodes, yNodes, cartObst):
    
    megaNodesCount = 0
    a, b, c, d = [], [], [], []
    
    for i in range (xNodes):
        
        aux0 = xMin + i * nodeDistance + shiftX
        
        for j in range(yNodes):
            
            aux1 = yMin + j * nodeDistance + shiftY
            a[0] = aux0 - nodeIntervalOffset
            a[1] = aux1 + nodeIntervalOffset
            b[0] = aux0 + nodeIntervalOffset
            b[1] = aux1 + nodeIntervalOffset
            c[0] = aux0 + nodeIntervalOffset
            c[1] = aux1 - nodeIntervalOffset
            d[0] = aux0 - nodeIntervalOffset
            d[1] = aux1 - nodeIntervalOffset
        
            if check(a, polygonCoordinates) and check(b, polygonCoordinates) and check(c, polygonCoordinates) and check(d, polygonCoordinates):
                
                are_all_zero = all_zero(cartObst)
                
                if are_all_zero == 0:
                    
                    len_cartObst = cartObst.shape[0]
                    k = 0
                    
                    while megaNodes[i][j] != 1 and k < len_cartObst:
                        
                        if check(a, cartObst[k]) or check(b, cartObst[k]) or check(c, cartObst[k]) or check(d, cartObst[k]):

                            megaNodes[i][j] = 1
                        
                        else:
                            
                            megaNodesCount += 1
                        
                        k += 1
                
                if megaNodes[i][j] == 0:
                    
                    xBoxMax = max(xBoxMax, b[0], d[0])
                    xBoxMin = min(xBoxMin, a[0], c[0])
                    yBoxMax = max(yBoxMax, a[1], b[1])
                    yBoxMin = min(yBoxMin, c[1], d[1])
                    
                    megaNodesCount += 1 #Clear block
            
            else:  
                megaNodes[i][j] = 1 #Obstacle
        
    return megaNodesCount, xBoxMax, xBoxMin, yBoxMax, yBoxMin
    