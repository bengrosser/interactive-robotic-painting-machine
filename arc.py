import math
from math import pow

TINY = 1e-12
M_PI_2l = 1.5707963267948966192313216916397514
G_2 = "cw"
G_3 = "ccw"

def getcenter(current_x,current_y,end_x,end_y,radius,move):

    abs_radius = math.fabs(radius)
    mid_x = (end_x + current_x) / 2.0
    mid_y = (end_y + current_y) / 2.0
    half_length = math.hypot((mid_x - end_x), (mid_y - end_y))

    # allow a small error for semicircle 
    if ((half_length / abs_radius) > (1 - TINY)):
        half_length = abs_radius   

    # check needed before calling asin   
    if (((move == G_2) and (radius > 0)) or ((move == G_3) and (radius < 0))):
        theta = math.atan2((end_y - current_y), (end_x - current_x)) - M_PI_2l
    else:
        theta = math.atan2((end_y - current_y), (end_x - current_x)) + M_PI_2l

    turn2 = math.asin(half_length / abs_radius)
    offset = abs_radius * math.cos(turn2)
    center_x = mid_x + (offset * math.cos(theta))
    center_y = mid_y + (offset * math.sin(theta))

    #print "center x = ", center_x, ", center y = ", center_y
    return (center_x,center_y)


def arcbounds(x0, y0, x1, y1, r, winding):
   """Return the bounding box of an arc given center, first point, final
   point, and winding direction (-1 means CW, +1 means CCW)"""

   center = getcenter(x0,y0,x1,y1,r,winding)
   cx = center[0]
   cy = center[1]

   rad = math.hypot(y0-cy, x0-cx)
   theta1 = math.atan2(y0-cy, x0-cx)
   theta2 = math.atan2(y1-cy, x1-cx)

   if winding == "cw":
       while theta2 - theta1 > -1e-12: theta2 -= 2*math.pi
   else:
       while theta2 - theta1 < 1e-12: theta2 += 2*math.pi

   if theta2 < theta1: theta1, theta2 = theta2, theta1

   # xpts, ypts accumulate the points that could be the most extreme points of
   # the arc -- the endpoints
   xpts = [x0, x1]
   ypts = [y0, y1]

   # .. as well as the quadrant points that are included in the arc
   for j in range(-6, 7):
       th = j * math.pi / 2
       if th < theta1: continue  # before start of arc
       if th > theta2: break     # after end of arc

       x = cx + rad * math.cos(th)
       y = cy + rad * math.sin(th)

       xpts.append(x)
       ypts.append(y)

   return min(xpts), max(xpts), min(ypts), max(ypts), cx, cy

def circley(xc, yc, r, newX):
    yA = yc - math.sqrt(pow(r,2)-pow(newX,2)-pow(xc,2) + 2*newX*xc)
    yB = yc + math.sqrt(pow(r,2)-pow(newX,2)-pow(xc,2) + 2*newX*xc)
    return (yA, yB)

def circlex(xc, yc, r, newY):
    xA = xc - math.sqrt(pow(r,2)-pow(newY,2)-pow(yc,2) + 2*newY*yc)
    xB = xc + math.sqrt(pow(r,2)-pow(newY,2)-pow(yc,2) + 2*newY*yc)
    return (xA, xB)

def boundcrossings(x0,y0,x1,y1,r,crossed,drc):
    center = getcenter(x0,y0,x1,y1,r,drc) 
    crossed = -5

    xc = center[0]
    yc = center[1]

    newYs = circley(xc,yc,r,crossed)
    
    print newYs[0],newYs[1]

  #  yA = yc - math.sqrt(pow(r,2)-pow(crossed,2)-pow(xc,2) + 2*crossed*xc)
  #  yB = yc + math.sqrt(pow(r,2)-pow(crossed,2)-pow(xc,2) + 2*crossed*xc)
  #  print yA, yB

maxX = 5
radius = 2
answer = arcbounds(-4.5,0,-4.5,4,2,"cw")
print answer[0], answer[1], answer[2], answer[3], answer[4],answer[5]
boundcrossings(-4.5,0,-4.5,4,2,1,"cw")

#print radius * math.cos(math.asin((maxX-answer[4])/radius))
