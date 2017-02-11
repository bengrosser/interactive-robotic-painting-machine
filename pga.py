#!/opt/local/bin/python

# Interactive Robotic Painting Machine
# Benjamin Grosser (2010-2011)
# Primary Event Driver Code
# ver 1.0


# python modules
import sys
import math
from decimal import *
from math import pow
from random import *
import time, threading
import Queue
import termios, atexit
from select import select

# my external modules
import OSC
import EMCsocket
import Decoders

# pyevolve
from pyevolve import G1DBinaryString
from pyevolve import GSimpleGA
from pyevolve import Selectors
from pyevolve import Mutators
from pyevolve import Crossovers

# stats
from statlib import stats

# queues for thread management

# holds captured sound information for use in fitness()
sndq = Queue.Queue(maxsize = 0)

# blocks fitness() until painting has been done and sound has been captured
fitq = Queue.Queue(maxsize = 0)

# blocks listener until it's time to capture sound
capq = Queue.Queue(maxsize = 0)

# blocks gastepper until it's time to step
stepq = Queue.Queue(maxsize = 0)

# blocks capture from ending until the subsequent gesture is painted
stopq = Queue.Queue(maxsize = 0)

# blocks main from ending until all generations are complete
endq = Queue.Queue(maxsize = 0)

# sent to step() in GSimpleGA.py so it can let us know when the unsorted
# list is created.  that way we can move on!!
createdq = Queue.Queue(maxsize = 0)

# not sure what this is for?
q = Queue.Queue(maxsize=0)
lock = threading.Lock()



# GLOBALS

TINY = 1e-12
M_PI_2l = 1.5707963267948966192313216916397514
G_2 = "cw"
G_3 = "ccw"

# GA parms
#PERFORMANCE popsize = 9
#PERFORMANCE numgenerations = 10
popsize = 16 
numgenerations = 20
stringlength = 109
seed = 0
# default is .02 which represents .2%
mutation = 0.06
# for concert
mutation = 0.06
elitist = False
pauseaftereach = True
paused = False
dynamicson = False


# capture parms
ampcutoff = 65

# a list to hold an individual's sound info
currentcapture = []
maxamptest = []

# support 10x10
supportoriginX = -5
supportoriginY = -5
supportwidth = 10
supportheight = 10
minX    = supportoriginX
minY    = supportoriginY
maxX    = supportoriginX+supportwidth
maxY    = supportoriginY+supportheight

# painting speed limits
minpaintspeed   = 100
maxpaintspeed   = 400
zpaintspeed     = 160.0
zpaintspeed     = 80.0

#volumes
volumes = [120,125,129,129,132,132,133,135,137]

# z value settings
zpaint          = 0.2
zsafe           = 0.8
zfinal          = 2.0
zpaintwell      = -1.7335

# z value settings for TESTING
# (e.g. all higher than 0!!)
#zpaint          = 0.2
#zsafe           = 0.4
#zfinal          = 2.0
#zpaintwell      = 1.7335

# z value settings for SIMULATOR
zpaint          = -0.4
zsafe           = 0.0
zfinal          = 0.0
#zpaintwell      = -0.7205
zpaintwell      = -1.7335

# brush pressure management
# zpaint0-3 are four Z values 
# for different brush pressures (lo-hi)
zpaint0         =  0.0
zpaint1         = -0.03
zpaint2         = -0.06
zpaint3         = -0.09

#TESTING
##zpaint0         =  1.2
#zpaint1         =  1.2
#zpaint2         =  1.2
#zpaint3         =  1.2

# bwidth0-3 are the corresponding widths
# of a line drawn by those different brush
# pressures
bwidth0         = 0.08
bwidth1         = 0.110
bwidth2         = 0.140
bwidth3         = 0.17

# paint well settings
paintwelldiameter = 1.0

# tracking
colormap        = ["c1","c2","c3","c4"]
lastcolor       = "-1"
gesturemap      = ["line","arc","point","fill"]
lastgesture     = "-1"
origins         = []
lastorigins     = []
points          = 0
arcs            = 0
lines           = 0
fills           = 0
mutationbump    = 0
radiusfix       = 0
reversefix      = 0
nofix           = 0
arcnolength     = 0
exceedminX      = 0
exceedmaxX      = 0
exceedminY      = 0
exceedmaxY      = 0
crossings = lastcrossings = morethanone = 0
start = 0
end = 0
minutes = 0
sec = 0
FAILSAFE = 120
MINTIME = 1

# brush settings
#brushdiameter   = 1.0
brushdiameter   = .1

# GCODE init codes for Machine.init()
initcodes       = "G61"

# general data management
lastx           = 0.0
lasty           = 0.0
initwait        = 0.0
#PERFORMANCEstartdelay      = 35
startdelay      = 1 


# global booleans
paused          = False
outputtofile    = False
outputtonetwork = True
dbug            = False 
aborting        = False
aborted         = False
resuming        = False
capturing       = False
#capturing       = True


# OSCSource IP
recvaddr        = '127.0.0.1', 8081


# support overshoot management
# amount past each edge that a gesture can legally extend
edgeslop        = .05

# points don't need so much edgeslop
pointslop       = 0.0 + (brushdiameter/2.0) 


# arc constants
arcdelta = 1.01
arcfac = 1.03
arcfacfac = 1.03
cw = 'cw'
ccw = 'ccw'



## manage keyboard input for pausing
## ----------------------------------

# save the terminal settings
fd = sys.stdin.fileno()
new_term = termios.tcgetattr(fd)
old_term = termios.tcgetattr(fd)

# new terminal setting unbuffered
new_term[3] = (new_term[3] & ~termios.ICANON & ~termios.ECHO)

# switch to normal terminal
def set_normal_term():
    termios.tcsetattr(fd, termios.TCSAFLUSH, old_term)

# switch to unbuffered terminal
def set_curses_term():
    termios.tcsetattr(fd, termios.TCSAFLUSH, new_term)

def putch(ch):
    sys.stdout.write(ch)

def getch():
    return sys.stdin.read(1)

def getche():
    ch = getch()
    putch(ch)
    return ch

def kbhit():
    dr,dw,de = select([sys.stdin], [], [], 0)
    return dr <> []

# ------------------------------------
# end of keyboard management



class Machine:
    lastx = 0.0
    lasty = 0.0
    iszup = True
    iszdown = False
    emc = EMCsocket.EMCsocket()
    #emcpos = EMCpossocket.EMCpossocket()
    #emcabort = EMCabortsocket.EMCabortsocket()

    def __init__(self):
        if outputtonetwork:
            self.emc.init(lock)
            time.sleep(1)
            #self.emcpos.init()
            #self.emcabort.init()
            time.sleep(startdelay)
        elif outputtofile:
            self.command("%")
            self.command(initcodes + "\n")
        self.reset()

    def reset(self):
        self.comment("; reset.  zpaint=%.2f, zsafe=%.2f" % (zpaint,zsafe))
        self.command("G00 Z%.4f" % zsafe)
        self.command("G00 X0.0 Y0.0") 
        self.command("G04 P%f" % initwait)
        self.lastx = 0.0
        self.lasty = 0.0
        #self.command("\n")
        #print self

    def end(self):
        self.command("G00 Z%.4f" % zsafe)
        self.command("G00 x0 y0")
        self.command("G00 Z%.4f" % zfinal)
        if outputtofile: self.command("%")
        #self.emc.printlines()
            
    def getelapsedtime(self):
        global start

        end = time.time()
        elapsed = end - start
        minutes = elapsed/60
        seconds = elapsed%60 
        if minutes < 0:
            minutes = 0

        return (minutes,seconds)

    def printelapsedtime(self,gen,ind):
        global start
        global FAILSAFE

        end = time.time()
        elapsed = end - start
        minutes = elapsed/60
        seconds = elapsed%60 
        if minutes < 0:
            minutes = 0

        print "--------------------"
        print "ELAPSED TIME: %.02d:%02d" % (minutes,seconds)
        print "GEN: %d, IND: %d" % (gen,ind)
        print "--------------------"

        if minutes >= FAILSAFE:
            time.sleep(6)
            self.end()
            print "OVER TIME"
            sys.exit("OVER TIME")


    def feedrate(self,rate):
        self.command("F%.4f" % rate)

    def edistance(self,x1,y1,x2,y2):
        return math.sqrt(math.pow(x2-x1,2) + math.pow(y2-y1,2))
        
    def setlastxy(self,x,y):
        Machine.lastx = x
        Machine.lasty = y
        if dbug: print "setlastxy(%f,%f)" % (x,y)
        
    def getpaint(self,name,x,y):
        #self.x = float(Color.paints[name].x)
        #self.y = float(Color.paints[name].y)

        d = paintwelldiameter/2.0
        d *= uniform(1,1.9)
        radius = d/2.0

        self.comment("\n; getting %s paint from %d,%d" % (name,x,y))

        px = x
        py = y-radius
        self.rapid(px, py)
        #self.zdown()
        self.command("G00 Z%.4f" % zpaintwell)
        Machine.iszdown = True
        self.command("G02 J%.4f F%.4f" % (radius, zpaintspeed))
        #self.command("M1")
        self.setlastxy(px,py)

    def zup(self):
        if Machine.iszdown:
            Machine.iszup = True
            Machine.iszdown = False
            self.command("G00 Z%.4f" % zsafe)

    def zdown(self):
        if Machine.iszup:
            Machine.iszdown = True
            Machine.iszup = False
            #self.command("G01 Z%.4f F%.0f" % (zpaint, feedrate))
            self.command("G01 Z%.4f" % (zpaint))

    def rapid(self,x,y):
        if Machine.needtomove(self,x,y):
            if dbug: print "in rapid"
            self.zup()
            self.command("G00 X%.4f Y%4f" % (x,y))
            self.setlastxy(x,y)

    def line(self,x,y):
        self.zdown()
        self.command("G01 X%.4f Y%4f" % (x,y))
        self.setlastxy(x,y)

    def point(self):
        if not Machine.iszup:
            self.zup()
            self.zdown()
        else:
            self.zdown()

    def needtomove(self,x,y):
        if dbug: print "in needtomove: checking %f,%f" % (x,y)
        if dbug: print "in needtomove: lastx = %f lasty = %f" % (Machine.lastx,Machine.lasty)
        if x != Machine.lastx or y != Machine.lasty:
            if dbug: print "needtomove true"
            Machine.lastx = x 
            Machine.lasty = y
            return True
        else:
            if dbug: print "needtomove false"
            return False

    def command(self,msg):
        if outputtofile:
            print msg
        if outputtonetwork:
            self.emc.wr("SET MDI "+msg)
            #print "SET MDI "+msg

    def setwait(self, msg):
        if outputtofile:
            print("SET SET_WAIT %s" % msg)
        if outputtonetwork:
            self.emc.w("SET SET_WAIT %s" % msg)
        
    def abort(self):
        print "in machine.abort()"

        if not paused:
            if outputtofile:
                print("SET ABORT")
            if outputtonetwork:
                #self.emc.w("SET ABORT")
                self.emcabort.abort(self)
        else:
            print "NOT SENDING ABORT DUE TO PAUSE STATE"

    def resume(self):
        global resuming

        #self.emc.init() 
        print "in machine.resume()"
        print "resuming is %s" %(resuming)
        #self.emc.recontrol()
        print "machine pausing"
        resuming = True
        print "set resuming to %s" %(resuming)


        recontrol = threading.Thread(target=self.emc.recontrol)
        #recontrol.setDaemon(True)
        recontrol.start()
        recontrol.join()

        resuming = False


    def checkabortfinished(self):
        self.resume()
    
    def getpos(self):
        if outputtofile:
            print "calling getpos()"
        if outputtonetwork:
            read = self.emcpos.wr("GET REL_CMD_POS") 
            print("READ IN GETPOS: %s" % read)
            if read != None:
                xyz = read.split()
                print("x = %s, y = %s, z = %s" % (xyz[1],xyz[2],xyz[3]))
                return (xyz[1],xyz[2],xyz[3])
            else:
                print("read = none")

    def comment(self,msg):
        #if outputtofile:
        print msg

    def audioevent(self,data):
        print("in Machine.audioevent(), data = %s" % data)



# manages what paints are available, where they are, and which 
# copy of a paint is closest
class Paints():
    
    # holds a list of tuples describing all paints available, such as
    # [("red",x,y),("blue",x,y),etc.]
    colors = []

    # adds another paint color to the object
    # send name of the paint as a string, and it's x,y center
    def addcolor(self,name,x,y):
        self.name = name
        self.x = x  
        self.y = y 

        self.colors.append((name,x,y))


    # euclidian distance .. may not need this duped here
    def edistance(self,x1,y1,x2,y2):
        return math.sqrt(math.pow(x2-x1,2) + math.pow(y2-y1,2))
            

    # return the name and coordinates to the closest paint matching 
    # the submitted name.  oX and oY are the current location of the 
    # machine (may be able to remove that and depend on globals?)
    def getclosestpaint(self,name,oX,oY):

        # find all paints matching the requested name
        # could return 1 or more (or 0!)
        options = self.getpaintoptions(name)
        answer = None
        best = 1000

        for item in options:
            dist = self.edistance(oX,oY,item[1],item[2])
            if dist < best:
                answer = item
                best = dist

        return answer


    # get the closest color for the color name requested
    def getpaint(self,name):
        #print "lastx = %f, lasty = %f" % (Machine.lastx,Machine.lasty)
        answer = self.getclosestpaint(name,Machine.lastx,Machine.lasty)
        if answer:
            machine.getpaint(name,answer[1],answer[2])
            print "getting paint %s from %f,%f" % (name,answer[1],answer[2])

        else:
            print "ERROR: requested paint \"%s\" is not available" % name


    # return all matching options for paint "name"
    def getpaintoptions(self,name):

        options = []
        for color in self.colors:
            if color[0] == name:
                options.append(color)

        #for color in options:
        #    print color 

        return options






class Color():
    
    paints = {}
    names = []
    numpaints = 0
    
    def __init__(self,name,x,y):
        self.name = name
        self.x = x  
        self.y = y 

        Color.paints[self.name] = self
        Color.names.append(self.name)
        Color.numpaints += 1
        

class Gesture(Machine):
    count = 0

    def __init__(self,s): 
        self.support  = s

    def __str__(self):
        return "; Gesture Count = %d" % Gesture.count

    def comment(self,defname,arglist):
        self.comment = "\n; "+defname+"("
        self.max = len(arglist)
        self.i = 0
        while self.i < self.max:
            arglist[self.i] = str(arglist[self.i]) 
            self.comment += arglist[self.i]
            self.i += 1
            if(self.i<self.max):
                self.comment += ", "
        self.comment = self.comment +")"
        return self.comment

    def paint(self,x,y):
        self.x = float(x)
        self.y = float(y)

    def reportcount(self):
        print Gesture.__str__(self)


    def inbounds(self,coords):

        if( (coords[0] > maxX or coords[0] < minX) or
            (coords[1] > maxY or coords[1] < minY) or
            (coords[2] > maxX or coords[2] < minX) or
            (coords[3] > maxY or coords[3] < minY)):

            return False
        else:
            return True


    def testlength(self,x1,y1,x2,y2):

        x1 = float(x1)
        y1 = float(y1)
        x2 = float(x2)
        y2 = float(y2)
        answer = []

        #print "TESTLENGTH b4 check: %f,%f,%f,%f" % (x1,y1,x2,y2)
        
        # if they're all outside, then what???
        if ((x1 >= maxX and y1 >= maxY and x2 >= maxX and y2 >= maxY) or
            (x1 <= minX and y1 <= minY and x2 <= minX and y2 <= minY) or
            (x1 <= minX and y1 <= minY and x2 >= maxX and y2 >= maxY) or
            (x1 >= maxX and y1 >= maxY and x2 <= minX and y2 <= minY) 
           ):
            # fuck it, if it's that outside, then whatever
            print "ALL coords are outside the boundary: fuck it."
            return (maxX,maxY,maxX,maxY)
            #return (x1,y1,x2,y2)

        # dealing with divide by zero errors
        xdiff = x2-x1
        ydiff = y2-y1

        if xdiff == 0:
            #xdiff = .000001
            xdiff = 1

        if ydiff == 0:
            ydiff = 1

        # x1,y1 
        if x1 > maxX and y1 > maxY:
            if x1 > y1:
                #print "case1"
                newx1 = maxX
                #newy1 = (y2-y1)/(x2-x1) * (maxX - x1) + y1
                newy1 = (ydiff)/(xdiff) * (maxX - x1) + y1
            else:
                #print "case2"
                newy1 = maxY
                #newx1 = (x2-x1)/(y2-y1) * (maxY - y1) + x1
                newx1 = (xdiff)/(ydiff) * (maxY - y1) + x1

        elif x1 > maxX:
            #print "case3"
            newx1 = maxX
            #newy1 = (y2-y1)/(x2-x1) * (maxX - x1) + y1
            newy1 = (ydiff)/(xdiff) * (maxX - x1) + y1

        elif y1 > maxY:
            #print "case4"
            newy1 = maxY
            #newx1 = (x2-x1)/(y2-y1) * (maxY - y1) + x1
            newx1 = (xdiff)/(ydiff) * (maxY - y1) + x1

        elif x1 < minX and y1 < minY:
            if x2 < y2:
                #print "case5"
                newx1 = minX
                #newy1 = (y2-y1)/(x2-x1) * (minX - x1) + y1
                newy1 = (ydiff)/(xdiff) * (minX - x1) + y1
            else:
                #print "case6"
                newy1 = minY
                #newx1 = (x2-x1)/(y2-y1) * (minY - y1) + x1
                newx1 = (xdiff)/(ydiff) * (minY - y1) + x1

        elif x1 < minX:
            #print "case7"
            newx1 = minX
            #newy1 = (y2-y1)/(x2-x1) * (minX - x1) + y1
            newy1 = (ydiff)/(xdiff) * (minX - x1) + y1

        elif y1 < minY:
            #print "case8"
            newy1 = minY
            #newx1 = (x2-x1)/(y2-y1) * (minY - y1) + x1    
            newx1 = (xdiff)/(ydiff) * (minY - y1) + x1    

        else:
            newx1 = x1
            newy1 = y1


        # x2,y2 
        if x2 > maxX and y2 > maxY:
            if x2 > y2:
                #print "case1"
                newx2 = maxX
                #newy2 = (y2-y1)/(x2-x1) * (maxX - x1) + y1
                newy2 = (ydiff)/(xdiff) * (maxX - x1) + y1
            else:
                #print "case2"
                newy2 = maxY
                #newx2 = (x2-x1)/(y2-y1) * (maxY - y1) + x1
                newx2 = (xdiff)/(ydiff) * (maxY - y1) + x1

        elif x2 > maxX:
            #print "case3"
            newx2 = maxX
            #newy2 = (y2-y1)/(x2-x1) * (maxX - x1) + y1
            newy2 = (ydiff)/(xdiff) * (maxX - x1) + y1

        elif y2 > maxY:
            #print "case4"
            newy2 = maxY
            #newx2 = (x2-x1)/(y2-y1) * (maxY - y1) + x1
            newx2 = (xdiff)/(ydiff) * (maxY - y1) + x1

        elif x2 < minX and y2 < minY:
            if x2 < y2:
                #print "case5"
                newx2 = minX
                #newy2 = (y2-y1)/(x2-x1) * (minX - x1) + y1
                newy2 = (ydiff)/(xdiff) * (minX - x1) + y1
            else:
                #print "case6"
                newy2 = minY
                #newx2 = -x1)/(y2-y1) * (minY - y1) + x1
                newx2 = (xdiff)/(ydiff) * (minY - y1) + x1

        elif x2 < minX:
            #print "case7"
            newx2 = minX
            #newy2 = (y2-y1)/(x2-x1) * (minX - x1) + y1
            newy2 = (ydiff)/(xdiff) * (minX - x1) + y1

        elif y2 < minY:
            #print "case8"
            newy2 = minY
            #newx2 = (x2-x1)/(y2-y1) * (minY - y1) + x1       
            newx2 = (xdiff)/(ydiff) * (minY - y1) + x1       

        else:
            newx2 = x2
            newy2 = y2

        #print "TESTLENGTH after check: %f,%f,%f,%f" % (newx1,newy1,newx2,newy2)

        answer = (newx1,newy1,newx2,newy2)

        #if (newx1 > maxX or newy1 > maxY or newx1 < minX or newy1 < minY or
        #   newx2 > maxX or newy2 > maxY or newx2 < minX or newy2 < minY):
        #    print "blah"
            #answer = tl(newx1,newy1,newx2,newy2)


        #print "%f,%f,%f,%f" % (answer[0],answer[1],answer[2],answer[3])

        return answer


class Arc(Gesture):


    def __str__(self):
        return str(Gesture.comment(self,"arc", [self.x1,self.y1,
                                                self.x2,self.y2,
                                                self.crv,self.drc]))

    # return a list of arc values as a list
    # the larger the value (and the index) the less extreme the arc kicks out
    def arclist(self,x0,y0,x1,y1,numarcs):
        global arcdelta, arcfac, arcfacfac
        ad = arcdelta
        af = arcfac
        aff = arcfacfac

        i = 0
        vals = []

        startradius = self.edistance(x0,y0,x1,y1)/2.0
        vals.append(startradius)

        #while i < numarcs:
        while i < 8:
            newradius = startradius * ad
            ad *= af 
            af *= aff 
            vals.append(newradius)
            i += 1

        return vals 

    def circley(self,xc, yc, r, newX):
        yA = yc - math.sqrt(pow(r,2)-pow(newX,2)-pow(xc,2) + 2*newX*xc)
        yB = yc + math.sqrt(pow(r,2)-pow(newX,2)-pow(xc,2) + 2*newX*xc)
        return (yA, yB)

    def circlex(self,xc, yc, r, newY):
        xA = xc - math.sqrt(pow(r,2)-pow(newY,2)-pow(yc,2) + 2*newY*yc)
        xB = xc + math.sqrt(pow(r,2)-pow(newY,2)-pow(yc,2) + 2*newY*yc)
        return (xA, xB)

    def drawarc(self,x0,y0,x1,y1,crv,drc):
        # tracking
        global radiusfix
        global reversefix
        global nofix
        global arcnolength
        global crossings
        global lastcrossings
        global morethanone

        exceedminX = False
        exceedmaxX = False
        exceedminY = False
        exceedmaxY = False

        #print "arc: befor testlength: ", x0,y0,x1,y1
        test = self.testlength(x0,y0,x1,y1) 
        if not (test[0] == x0 and test[1] == y0 and test[2] == x1 and test[3] == y1):
            print "ARC LENGTH TRUNCATED"
        x0 = test[0]
        y0 = test[1]
        x1 = test[2]
        y1 = test[3]
        #print "arc: after testlength: ", x0,y0,x1,y1

        if x0 == x1 and y0 == y1:
            arcnolength += 1
            print "arc start and end are the same, moving on"
            return

#        index = 0

        arcvals = self.arclist(x0,y0,x1,y1,crv)

        #for item in arcvals:
        #    print "arcval[%d]: %f" % (index,item)
        #    index += 1
        #print "arcval selection: #%d, %f" % (crv,arcvals[crv])

        radius = arcvals[crv]

        boundingbox = self.arcbounds(x0,y0,x1,y1,arcvals[crv],drc)

        #for item in boundingbox:
        #    print "arc bound: %f" % item

        lastcrossings = crossings

        if boundingbox[0] < minX:
            exceedminX = True
            crossings += 1
        if boundingbox[1] > maxX: 
            exceedmaxX = True
            crossings += 1
        if boundingbox[2] < minY: 
            exceedminY = True
            crossings += 1
        if boundingbox[3] > maxY:
            exceedmaxY = True
            crossings += 1

        # if it crosses multiple boundaries then fuck it
        # pretty sure that means it exceeds on two points anyway
        if crossings > lastcrossings+1:
            morethanone += 1
            print "arc crosses multiple boundaries, won't draw"
            return
        # we are crossing a boundary, so we'll draw two arcs 
        # (unless one of them doesn't work)
        elif crossings > lastcrossings:

            # calc center
            center = self.getcenter(x0,y0,x1,y1,radius,drc)
            xc = center[0]
            yc = center[1]
            
            if exceedminX:
                newys = self.circley(xc,yc,radius,minX)
            
                if y0 < y1:
                    arc1y1 = newys[0]
                    arc2y0 = newys[1]
                else:
                    arc1y1 = newys[1]
                    arc2y0 = newys[0]

                arc1x0 = x0
                arc1y0 = y0
                arc1x1 = minX

                arc2x0 = minX
                arc2x1 = x1
                arc2y1 = y1

            elif exceedmaxX:
                newys = self.circley(xc,yc,radius,maxX)
            
                if y0 < y1:
                    arc1y1 = newys[0]
                    arc2y0 = newys[1]
                else:
                    arc1y1 = newys[1]
                    arc2y0 = newys[0]

                arc1x0 = x0
                arc1y0 = y0
                arc1x1 = maxX

                arc2x0 = maxX
                arc2x1 = x1
                arc2y1 = y1

            elif exceedminY:
                newxs = self.circlex(xc,yc,radius,minY)
            
                if x0 < x1:
                    arc1x1 = newxs[0]
                    arc2x0 = newxs[1]
                else:
                    arc1x1 = newxs[1]
                    arc2x0 = newxs[0]

                arc1x0 = x0
                arc1y0 = y0
                arc1y1 = minY

                arc2y0 = minY
                arc2x1 = x1
                arc2y1 = y1

            elif exceedmaxY:
                newxs = self.circlex(xc,yc,radius,maxY)
            
                if x0 < x1:
                    arc1x1 = newxs[0]
                    arc2x0 = newxs[1]
                else:
                    arc1x1 = newxs[1]
                    arc2x0 = newxs[0]

                arc1x0 = x0
                arc1y0 = y0
                arc1y1 = maxY

                arc2y0 = maxY
                arc2x1 = x1
                arc2y1 = y1

            if drc == 'cw':
                gcode = "G02"
            elif drc == 'ccw':
                gcode = "G03"
            else:
                print "ERROR: bad direction in drawarc"


            #print "arc1x0 = ", Decimal(str(arc1x0)), ", arc1y0 = ", Decimal(str(arc1y0))
            #print "arc1x1 = ", Decimal(str(arc1x1)), ", arc1y1 = ", Decimal(str(arc1y1))

            #if(Decimal(str(arc1x0)) == Decimal(str(arc1x1)) and 
            #   Decimal(str(arc1y0)) == Decimal(str(arc1y1))):
            #if arc1x0 == arc1x1 and arc1y0 == arc1y1:

            # if arc 1 has no length, don't draw it
            if(self.nearly_equal(arc1x0,arc1x1) and
               self.nearly_equal(arc1y0,arc1y1)):
                arcnolength += 1
                print "arc start and end are the same, skipping"

            # else draw it
            else:
                self.rapid(arc1x0,arc1y0)
                self.zdown()
                Machine.command(self,"%s x%f y%f r%f" % (gcode,arc1x1,arc1y1,radius))
                self.zup()
                Machine.setlastxy(self,arc1x1,arc1y1)


            if(self.nearly_equal(arc2x0,arc2x1) and
               self.nearly_equal(arc2y0,arc2y1)):
                arcnolength += 1
                print "arc start and end are the same"
            else:
                self.rapid(arc2x0,arc2y0)
                self.zdown()
                Machine.command(self,"%s x%f y%f r%f" % (gcode,arc2x1,arc2y1,radius))
                Machine.setlastxy(self,arc2x1,arc2y1)
            
        else:
            if drc == 'cw':
                gcode = "G02"
            elif drc == 'ccw':
                gcode = "G03"
            else:
                print "ERROR: bad direction in drawarc"

            self.rapid(x0,y0)
            self.zdown()
            Machine.command(self,"%s x%f y%f r%f" % (gcode,x1,y1,radius))
            Machine.setlastxy(self,x1,y1)
            

    def nearly_equal(self,x,y):
        epsilon=1e-7
        return abs(x - y) < epsilon


    def edistance(self,x1,y1,x2,y2):
        return math.sqrt(math.pow(x2-x1,2) + math.pow(y2-y1,2))

    def paint(self,x1,y1,x2,y2,curvyness,direction):

        self.x1 = float(x1)
        self.x2 = float(x2)
        self.y1 = float(y1)
        self.y2 = float(y2)
        self.crv = curvyness
        self.drc = direction

        print self

        if not self.support.inbounds(self.x1,self.y1,self.x2,self.y2,edgeslop):
            #sys.exit("BOUNDS ERROR")
            print "BOUNDS ERROR: skipping arc"
        else:
            #self.rapid(self.x1,self.y1)
            #self.zdown()
            self.drawarc(self.x1,self.y1,self.x2,self.y2,self.crv,self.drc)
            Gesture.count += 1
            Gesture.reportcount(self)


    def getcenter(self,current_x,current_y,end_x,end_y,radius,move):

        abs_radius = math.fabs(radius)
        mid_x = (end_x + current_x) / 2.0
        mid_y = (end_y + current_y) / 2.0
        half_length = math.hypot((mid_x - end_x), (mid_y - end_y))

        # allow a small error for semicircle 
        if ((half_length / abs_radius) > (1 - TINY)):
            half_length = abs_radius   

        # check needed before calling asin   
        if (((move == cw) and (radius > 0)) or ((move == ccw) and (radius < 0))):
            theta = math.atan2((end_y - current_y), (end_x - current_x)) - M_PI_2l
        else:
            theta = math.atan2((end_y - current_y), (end_x - current_x)) + M_PI_2l

        turn2 = math.asin(half_length / abs_radius)
        offset = abs_radius * math.cos(turn2)
        center_x = mid_x + (offset * math.cos(theta))
        center_y = mid_y + (offset * math.sin(theta))

        #print "center x = ", center_x, ", center y = ", center_y
        return (center_x,center_y)


    def arcbounds(self,x0, y0, x1, y1, r, winding):
       """Return the bounding box of an arc given center, first point, final
       point, and winding direction (-1 means CW, +1 means CCW)"""

       center = self.getcenter(x0,y0,x1,y1,r,winding)
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

       return min(xpts), max(xpts), min(ypts), max(ypts)



class Line(Gesture):

    def __str__(self):
        return str(Gesture.comment(self,"line", [self.x1,self.y1,
                                                 self.x2,self.y2]))

    def paint(self,x1,y1,x2,y2):

        self.x1 = float(x1)
        self.x2 = float(x2)
        self.y1 = float(y1)
        self.y2 = float(y2)

        print self

        tested = self.testlength(self.x1,self.y1,self.x2,self.y2)
        self.x1 = tested[0]
        self.y1 = tested[1]
        self.x2 = tested[2]
        self.y2 = tested[3]

        if self.x1 == self.x2 and self.y1 == self.y2:
            print "start and end of line are same, skipping line"
        elif not self.support.inbounds(self.x1,self.y1,self.x2,self.y2,edgeslop):
            #sys.exit("BOUNDS ERROR")
            print "BOUNDS ERROR: skipping line"
        else:
            self.rapid(self.x1,self.y1)
            self.line(self.x2,self.y2)
            #self.zup()
            Gesture.count += 1
            Gesture.reportcount(self)

    def getdirection(self):
        
        if self.x1 < self.x2:
            xdir = 1
        else:
            xdir = 0
        if self.y1 < self.y2:
            ydir = 1
        else:
            ydir = 0

        return (xdir,ydir)

    def getvector(self,x1,y1,length,angle):

        self.x1 = float(x1)
        self.y1 = float(y1)
        self.length = float(length)
        self.angle = float(angle)

        x2 = self.x1 + (self.length * math.cos(math.radians(self.angle)))
        y2 = self.y1 + (self.length * math.sin(math.radians(self.angle)))
        
        return (x2,y2)

   

class Fill(Gesture):

    def __str__(self):
        return str(Gesture.comment(self,"fill", [self.x1,self.y1,
                                                 self.angle,self.length,
                                                 self.fillwidth,self.variance]))

    # euclidian distance .. may not need this duped here
    def edistance(self,x1,y1,x2,y2):
        return math.sqrt(math.pow(x2-x1,2) + math.pow(y2-y1,2))

    def paint(self,x1,y1,angle,length,width,variance,color,speed):
        global zpaint

        self.x1 = x1
        self.y1 = y1
        self.angle = angle
        self.length = length
        self.fillwidth = width
        self.variance = variance

        print self

        minv = 0
        maxv = 0

        if self.variance == 0:
            maxv = 1.1
            minv = 1.0
        elif self.variance == 1:
            maxv = 1.2
            minv = 1.0
        elif self.variance == 2:
            maxv = 1.3
            minv = 1.0
        elif self.variance == 3:
            maxv = 1.4
            minv = 1.0

        variance = uniform(minv,maxv)

        #print "FILL: testing end points"
        xy2 = line.getvector(self.x1,self.y1,self.length*variance,self.angle)
        #xy2 = self.testlength(xy2)
        #xy2 = self.tl(self.x1,self.x2,self.y1,self.y2)

        test = self.testlength(self.x1,self.y1,xy2[0],xy2[1])
        self.x1 = test[0]
        self.y1 = test[1]
        xy2 = (test[2],test[3])

        # not sure I need this.  won't they always be inside the support????
        #print "FILL: testing start points"
        #txy = self.testlength((x1,y1))
        #sx1 = txy[0]
        #sy1 = txy[1]

        sx1 = self.x1
        sy1 = self.y1

        # draw centerline
        #line.paint(self.x1,self.y1,xy2[0],xy2[1])
        line.paint(sx1,sy1,xy2[0],xy2[1])

        # replace with pressure calculation later
        if zpaint == zpaint0:
            linethickness = bwidth0
        elif zpaint == zpaint1:
            linethickness = bwidth1
        elif zpaint == zpaint2:
            linethickness = bwidth2
        elif zpaint == zpaint3:
            linethickness = bwidth3

        # for manual testing
        #linethickness = 0.1

        # total num of lines
        totallines = self.fillwidth/linethickness
         
        linesdrawn = 0
        lengthpainted = 0

        #print "FILL: lengthpainted = %f" % lengthpainted

        #sx1 = self.x1
        #sy1 = self.y1
        # draw half on one side of centerline


        while linesdrawn < totallines/2:
            #print "FILL: lengthpainted = %f" % lengthpainted
            variance = uniform(minv,maxv)
            if lengthpainted > 3:
                paints.getpaint(color)
                lengthpainted = 0
                #machine.feedrate(speed*variance)
                machine.feedrate(speed)
            variedlength = self.length*variance
            neworigin = line.getvector(sx1,sy1,linethickness,(self.angle+90))
            newend = line.getvector(neworigin[0],neworigin[1],variedlength,self.angle)
            variance = uniform(minv,maxv)
            variedlength = self.length*variance
            neworigin = line.getvector(newend[0],newend[1],variedlength,self.angle+180)
            test = self.testlength(neworigin[0],neworigin[1],newend[0],newend[1])
            neworigin = (test[0],test[1])
            newend = (test[2],test[3])
            if linesdrawn%2:
                line.paint(neworigin[0],neworigin[1],newend[0],newend[1])
            else:
                line.paint(newend[0],newend[1],neworigin[0],neworigin[1])
            linesdrawn += 1
            sx1 = neworigin[0]
            sy1 = neworigin[1]
            actuallength = self.edistance(neworigin[0],neworigin[1],newend[0],newend[1])
            #lengthpainted += self.length
            lengthpainted += actuallength



        sx1 = self.x1
        sy1 = self.y1
        linesdrawn = 0

        while linesdrawn < totallines/2:
            #print "FILL: lengthpainted = %f" % lengthpainted
            variance = uniform(minv,maxv)
            if lengthpainted > 7:
                paints.getpaint(color)
                lengthpainted = 0
                #machine.feedrate(speed*variance)
                machine.feedrate(speed)
            neworigin = line.getvector(sx1,sy1,linethickness,self.angle-90)
            variedlength = self.length*variance
            newend = line.getvector(neworigin[0],neworigin[1],variedlength,self.angle)
            variance = uniform(minv,maxv)
            variedlength = self.length*variance
            neworigin = line.getvector(newend[0],newend[1],variedlength,self.angle+180)
            test = self.testlength(neworigin[0],neworigin[1],newend[0],newend[1])
            neworigin = (test[0],test[1])
            newend = (test[2],test[3])
            if linesdrawn%2:
                line.paint(neworigin[0],neworigin[1],newend[0],newend[1])
            else:
                line.paint(newend[0],newend[1],neworigin[0],neworigin[1])
            linesdrawn += 1
            sx1 = neworigin[0]
            sy1 = neworigin[1]
            actuallength = self.edistance(neworigin[0],neworigin[1],newend[0],newend[1])
            #lengthpainted += self.length
            lengthpainted += actuallength





class Point(Gesture):

    def __str__(self):
        return str(Gesture.comment(self,"point", [self.x,self.y]))
    
    def paint(self,x,y):

        Gesture.paint(self,x,y)

        halfbrush = float(brushdiameter)/2.0
        
        print self

        # edgeslop isn't very useful for points since they take up
        # so little space, so we'll add edgeslop into the val before 
        # testing it

        if(not self.support.inbounds(self.x-halfbrush,
                                     self.y-halfbrush,
                                     self.x+halfbrush,
                                     self.y+halfbrush,
                                     pointslop)):
            #sys.exit("BOUNDS ERROR") 
            print "BOUNDS ERROR: skipping point"
        else:
            self.rapid(self.x,self.y)
            self.point()
            Gesture.count += 1
            Gesture.reportcount(self)


class Square(Gesture):

    def __str__(self):
        return str(Gesture.comment(self,"square", [self.x,self.y,self.size]))

    def paint(self,x,y,size):

        Gesture.paint(self,x,y)
        self.size = float(size)

        self.xstart = self.x-(self.size/2)
        self.ystart = self.y-(self.size/2)

        print self

        if(not self.support.inbounds(self.xstart,
                                     self.ystart,
                                     self.xstart+size,
                                     self.ystart+size,
                                     edgeslop)):
            sys.exit("BOUNDS ERROR") 

        self.rapid(self.xstart,self.ystart)
        self.line(self.xstart+self.size, self.ystart)
        self.line(self.xstart+self.size, self.ystart+self.size)
        self.line(self.xstart, self.ystart+self.size)
        self.line(self.xstart, self.ystart)
        Gesture.count += 4
        Gesture.reportcount(self)

    def fill(self,x,y,size):
        self.x = float(x)
        self.y = float(y)
        self.size = float(size)

        self.x = self.x-(self.size/2.0)
        self.y = self.y-(self.size/2.0)

        brushradius = brushdiameter/2.0
        delta = brushradius * 0.9

        finished = False
        newx = (self.x+delta) 
        finalx = self.x+self.size
        finaly = self.y+self.size

        Machine.comment(self,"; square fill => %.2f,%.2f,%.2f" % \
                            (self.x,self.y,self.size))

        self.rapid(newx,self.y)

        while not finished:
            if newx < finalx-delta: 
                self.rapid(newx,self.y)
                self.line(newx,finaly)
                newx += delta 
                #newx *= .9 
                self.rapid(newx,finaly)
                self.line(newx,self.y)
                newx += delta 
                #newx *= .9 
            else:
                self.rapid(finalx-delta,self.y)
                self.line(finalx-delta,finaly)
                finished = True


# draw a circle centered on x,y with size diameter 
class Circle(Gesture):

    def __str__(self):
        return str(Gesture.comment(self,"circle",[self.x,self.y,self.diameter]))

    def paint(self,x,y,diameter):
        Gesture.paint(self,x,y)
        self.diameter = diameter
        self.radius = self.diameter/2.0

        Machine.comment(self,self)

        if(not self.support.inbounds(self.x-self.radius,
                                     self.y-self.radius,
                                     self.x+self.radius,
                                     self.y+self.radius,
                                     edgeslop)):
            sys.exit("BOUNDS ERROR") 

        Machine.rapid(self,self.x, self.y-self.radius)
        Machine.zdown(self)
        Machine.command(self,"G02 J%.4f" % self.radius)
        Gesture.count += 1
        Gesture.reportcount(self)

    def fill(self,x,y,diameter):
        self.diameter = float(diameter)
        finished = False
        newdiameter = (diameter-brushdiameter) * 1.1

        Machine.comment(self,"; circle fill => %.2f,%.2f,%.2f" % \
                            (self.x,self.y,self.diameter))

        while not finished:
            if newdiameter > 0: 
                self.paint(x,y,newdiameter)
                newdiameter -= brushdiameter 
                newdiameter *= 1.1
            else:
                finished = True
                

class Support:
    
    def __init__(self,x,y,w,h,name):
        self.x = float(x)
        self.y = float(y)
        self.w = float(w)
        self.h = float(h)
        self.name = name
        print self

    def getsize(self):
        return ((self.w),(self.h))

    def getbounds(self):
        return ((self.x,self.y,self.w,self.h))

    def __str__(self):
        return "\n; support %s => x,y=%.2f,%.2f , w,h=%.2f,%.2f" \
                % (self.name,self.x,self.y,self.w,self.h)

    def error(self,msg):
        print("; " + msg)
        #print >> sys.stderr, msg

    def inbounds(self,x1,y1,x2,y2,slop): 
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.slop = slop
        maxx = self.x+self.w
        maxy = self.y+self.h

        #print >> sys.stderr, (x1, y1, x2, y2, slop)
        # x = y = -5
        # w = h = 10

        failed = False

        if(self.x1 < self.x): 
            delta = self.x - self.x1
            if(delta <= slop):
                self.error("WARNING: into edgeslop on x1")
            else:
                self.error("ERROR: out of bounds on support " + self.name + 
                      " => [x1 = %.2f, minx = %.2f]" % (self.x1,self.x))
                failed = True

        if(self.x2 < self.x): 
            delta = self.x - self.x2
            if(delta <= slop):
                self.error("WARNING: into edgeslop on x2")
            else:
                self.error("ERROR: out of bounds on support " + self.name + 
                      " => [x2 = %.2f, minx = %.2f]" % (self.x2,self.x))
                failed = True

        if(self.y1 < self.y): 
            delta = self.y - self.y1
            if(delta <= slop):
                self.error("WARNING: into edgeslop on y1")
            else:
                self.error("ERROR: out of bounds on support " + self.name + 
                      " => [y1 = %.2f, miny = %.2f]" % (self.y1,self.y))
                failed = True

        if(self.y2 < self.y): 
            delta = self.y - self.y2
            if(delta <= slop):
                self.error("WARNING: into edgeslop on y2")
            else:
                self.error("ERROR: out of bounds on support " + self.name + 
                      " => [y2 = %.2f, miny = %.2f]" % (self.y2,self.y))
                failed = True

        if(self.x1 > maxx): 
            delta = self.x1 - maxx
            if(delta <= slop):
                self.error("WARNING: into edgeslop on x1")
            else:
                self.error("ERROR: out of bounds on support " + self.name + 
                      " => [x1 = %.2f, maxx = %.2f]" % (self.x1,maxx))
                failed = True

        if(self.x2 > maxx): 
            delta = self.x2 - maxx
            if(delta <= slop):
                self.error("WARNING: into edgeslop on x2")
            else:
                self.error("ERROR: out of bounds on support " + self.name + 
                      " => [x2 = %.2f, maxx = %.2f]" % (self.x2,maxx))
                failed = True

        #print("y2 = %.2f, maxy = %.2f" % (self.y2,maxy))

        if(self.y1 > maxy): 
            delta = self.y1 - maxy
            if(delta <= slop):
                self.error("WARNING: into edgeslop on y1")
            else:
                self.error("ERROR: out of bounds on support " + self.name + 
                      " => [y1 = %.2f, maxy = %.2f]" % (self.y1,maxy))
                failed = True

        if(self.y2 > maxy): 
            delta = self.y2 - maxy
            if(delta <= slop):
                self.error("WARNING: into edgeslop on y2")
            else:
                self.error("ERROR: out of bounds on support " + self.name + 
                      " => [y2 = %.2f, maxy = %.2f]" % (self.y2,maxy))
                failed = True

        if(failed):
            return False
        else:
            return True


class Dynamics():

    global dynamicson

    client = OSC.OSCClient()

    def __init__(self):
        if dynamicson:
            self.client.connect( ('192.168.1.137', 7503) ) 
        else:
            print "init"

    def setvolume(self,v):
        if dynamicson:
            msg = OSC.OSCMessage()
            msg.setAddress("/volume")
            msg.append(v)
            self.client.send(msg) 
        else:
            print "setvol"



# this is the driver.  launches two threads: loop() and gastepper().  
# the loop runs for all generations and drives the action, getting info
# from the GA, sending that info for decoding, asking for sound capture
# getting that info to fitness(), etc.
class Loop(Machine,Gesture):

    # This function is the evaluation function, we want
    # to give high score to more fit chromosomes
    # runs to evaluate each individual
    def fitness(self,chromosome):
        global lastorigins
        global ampcutoff
        global mutationbump
         
        # init the fitness scoree
        score = 0.0

        # block fitness until generation is done painting
        # fitq holds a value for each individual in generation
        # and fitness() will run for each of those individuals
        # then when it runs out, it blocks again
        run = fitq.get()

        # get individual number and list of sound info for that individual
        sndinfo = sndq.get()

        # the bit string decoder
        decoder = Decoders.BitStringDecoder(support.getbounds(),
                                       (minpaintspeed,maxpaintspeed))

        decoded = decoder.decode(chromosome)
        
        for item in decoded:
            # get the tag and the value
            tag = item[0]
            val = item[1]

            if tag == "type":
                gtype = val 
            elif tag == "color":
                color = val 
            elif tag == "originX":
                x1 = val
            elif tag == "originY":
                y1 = val
            elif tag == "angle":
                angle = val
            elif tag == "length":
                length = val
            elif tag == "speed":
                speed = val
            elif tag == "pressure":
                pressure = val

            elif tag == "volume":
                volume = val

            elif tag == "width":
                fillwidth = val
            elif tag == "variance":
                variance = val

            elif tag == "arcness":
                arcness = val
            elif tag == "arcdir":
                arcdir = val
            elif tag == "arcseg":
                arcseg = val


            elif tag == "angle1":
                angle1 = val
            elif tag == "length1":
                length1 = val
            elif tag == "arcness1":
                arcness1 = val
            elif tag == "arcdir1":
                arcdir1 = val

            elif tag == "angle2":
                angle2 = val
            elif tag == "length2":
                length2 = val
            elif tag == "arcness2":
                arcness2 = val
            elif tag == "arcdir2":
                arcdir2 = val

            elif tag == "angle3":
                angle3 = val
            elif tag == "length3":
                length3 = val
            elif tag == "arcness3":
                arcness3 = val
            elif tag == "arcdir3":
                arcdir3 = val

            elif tag == "volume":
                volume = val


        # gtype scoring
        # 9 points possible
        if gtype == "line":
            score += 6.0
        elif gtype == "arc":
            score += 9.1
        elif gtype == "fill":
            #score += 7.8
            score += 9.2
        elif gtype == "point":
            score += 7.0

        if volume > 4:
            score += 3.0
            print "+3 for volume!!"

        # color
        # 5 points possible
        if color == colormap[0]:
            score += 4.6
        elif color == colormap[1]:
            score += 5.0
        elif color == colormap[2]:
            score += 4.5
        elif color == colormap[3]:
            score += 4.0

        # origin
        # if origin of current individual matches the origin of any
        # previous individual, then decrement it a bit
        # 3 points possible
        #for item in lastorigins:
        #    print "lastorigins: ", item
        if gtype == "fill":
            if ((x1,y1)) in lastorigins:
                score -= 3.0
                print "-3 for origin!"
            elif currentgeneration != 0:
                score += 1.0
                print "+1 for origin."

        # angle
        # perfect vertical rocks
        # 8 points possible
        if (angle == 90 or angle == 270):
            score += 8.0
            print "+9 for angle!!"
        # near vertical is good too
        elif ((angle > 85 and angle < 95) or (angle > 265 and angle < 275)):
            score += 3.0
            print "+6 for angle."
        # near horizontal sucks
        elif ((angle > 175 and angle < 185) or (angle > 355 and angle < 361) or
              (angle < 5 and angle > 0)):
            score -= 5.0
            print "-3 for angle."
        else:
            score -= 1.0
            print "-1 for angle."

        # length
        # 4 points possible
        if length < 2 and length > 0.5:
            score += 4.0
            print "+4 for length!"
        else:
            score += 1.0
            print "+1 for length."
        
        # short length fills rock
        # 3 points possible
        if gtype == "fill" and length < 0.5:
            score += 5.0
            print "+3 for short fill!"
        elif gtype == "fill" and length < 1.0:
            score += 4.0
            print "+1 for short fill."
        elif gtype == "fill" and length > 2.0:
            score -= 3.0
            print "-3 for long fill!"



        # points at pressure 0 are best
        # 3 points possible
        if gtype == "point" and pressure == 0:
            score += 3.0
        elif gtype == "point" and pressure > 0:
            score -= 1.0

        # speed
        # favor a speed range.  not sure how htis will work.
        # should end up clustering events within a particular pitch area
        # 3 points possible
        #if speed > 160 and speed < 180:
        #    score += 3.0
        #    print "+3 for speed!"
        #elif speed > 220 and speed < 240:
        #    score += 3.0
        #    print "+3 for speed!"
        #elif speed > 110 and speed < 120:
        #    score += 3.0
        #    print "+3 for speed!"
        #else:
        #    score += 1.0
        #    print "+1 for speed."

        # fillwidth?
        # width
        # favor short length but wide fills
        # 4 points possible
        if gtype == "fill" and length < 1:
            if fillwidth > 3:
                score += 4.0
                print "+4 for fillwidth!"
            elif fillwidth > 2:
                score += 3.0
                print "+3 for fillwidth!"
            elif fillwidth > 1:
                score += 2.0
                print "+2 for fillwidth."
            else:
                score += 1.5
                print "+1.5 for fillwidth."

        elif gtype == "fill" and length < 1.5:
            if fillwidth > 3:
                score += 2.0
                print "+3 for fillwidth!"
            elif fillwidth > 2:
                score += 1.0
                print "+2 for fillwidth."
            elif fillwidth > 1:
                score += 0.5
                print "+1 for fillwidth."
            else:
                score += 0.25
                print "+0.5 for fillwidth."


        # variance
        if variance > 2:
            score += 2.0
            print "+2 for variance!"

        # arcs
        # favor 4 seg arcs
        # 4 points possible
        if gtype == "arc":
            if arcseg == 3:
                score += 4.0
            elif arcseg == 2:
                score += 2.0
            else:
                score += 1.0

            arcdirlist = [arcdir,arcdir1,arcdir2,arcdir3]

            # be within +/- angledelta degrees for eval
            angledelta = 10

            # favor alternating arcsegs
            # 5 points possible
            if arcseg > 0:
                if arcseg == 1:
                    upperangle = angle + angledelta
                    lowerangle = angle - angledelta
                    if angle1 < upperangle and angle1 > lowerangle:
                        score += 5.0
                    else:
                        score -= 2.0

                    if arcdirlist[0] != arcdirlist[1]:
                        score += 5.0
                    else:
                        score -= 1.0

                elif arcseg == 2:
                    upperangle = angle1 + angledelta
                    lowerangle = angle1 - angledelta
                    if angle2 < upperangle and angle2 > lowerangle:
                        score += 0.5
                    else:
                        score -= 0.2

                    if arcdirlist[0] == arcdirlist[2] and arcdirlist[0] != arcdirlist[1]:
                        score += 5.0
                    else:
                        score -= 2.0

                elif arcseg == 3:
                    upperangle = angle2 + angledelta
                    lowerangle = angle2 - angledelta
                    if angle3 < upperangle and angle3 > lowerangle:
                        score += 5.0
                    else:
                        score -= 2.0

                    if (arcdirlist[0] == arcdirlist[2] and arcdirlist[1] == arcdirlist[3] and
                        arcdirlist[0] != arcdirlist[1]):
                        score += 5.0
                    else:
                        score -= 1.0



        # deal with the sound input
        # -------------------------

        # sound tracking values in Hz
        malefemalethreshold = 170
        childthreshold = 230

        # analysis parms
        # if it gets louder than this then you're crazy!! (and it's valued)
        maxampthreshold = ampcutoff + 40

        # initialize the vars
        freqs       = []
        amps        = []
        minmaxfreq  = []
        minmaxamp   = []
        source      = ""
        numattacks  = 0
        meanfreq    = 0
        meanamp     = 0
        rangefreq   = 0
        rangeamp    = 0
        stdevfreq   = 0
        stdevamp    = 0
        skewfreq    = 0
        modefreq    = 0
        modeamp     = 0

        # first check if there was any sound collected!
        if sndinfo:
            # if so, do some statistical analysis of the information

            # sndinfo contains a list of 0-n tuples: (amp,freq)
            # grab the info and put it in freqs and amps
            for item in sndinfo:
                # print out sound info for now
                # print("sndinfo = "), item
                amps.append(item[0])
                freqs.append(item[1])

            #print "freqs = "
            #print freqs
            #print "amps = "
            #print amps 

            # source detection
            # (but what happens with music instead of speech??)
            meanfreq = stats.mean(freqs)

            if meanfreq < malefemalethreshold:
                source = "male"
            elif meanfreq > malefemalethreshold and meanfreq < childthreshold:
                source = "female"
            else:
                source = "music"

            # store the number of attacks
            numattacks = len(sndinfo)

            # get the average loudness
            meanamp = stats.mean(amps)

            # get minmax
            minmaxfreq.append(min(freqs))
            minmaxfreq.append(max(freqs))
            minmaxamp.append(min(amps))
            minmaxamp.append(max(amps))

            # get the ranges
            # if there's more than one freq
            if len(freqs) > 1:
                rangefreq = max(freqs) - min(freqs)
            if len(amps) > 1:
                rangeamp = max(amps) - min(amps)

            # get the mode of the frequencies (e.g. what are the most common freqs --
            # are any repeated?)  modefreq will contain two values, the number of 
            # times the frequency occurred, and what the recurring frequency is/are
            # if modefreq[0] > 1, then there's a recurring freq
            modefreq = stats.mode(freqs)

            # get the mode of the amps as well
            modeamp = stats.mode(amps)

            # calc the standard deviation of freqs and amps
            # describes how much variance there is in the freqs and amps, and the value
            # returned is a measure of how far away from the median the freqs/amps are
            stdevfreq = stats.samplestdev(freqs)
            stdevamp = stats.samplestdev(amps)

            # calc the skew of freqs
            # positive values indicate bulk of the freqs are higher than the mean
            # negative values indicate bulk of the freqs are lower than the mean
            # need more than one value to calc a skew!
            #if len(freqs) > 1:
            #    print "in skew:"
            #    print freqs
            #    skewfreq = stats.skew(freqs)
            #    print "out of skew."


            # apply fitness values based on sound stats and my aesthetics
            # -----------------------------------------------------------

            # source: max = +3
            # favor female speech or music over male speech
            if source == "female" or source == "music":
                score += 4.0
                print "+3 for female speech or music!"
            else:
                score += 1.0

            # frequency range: if it is widely varying, give it a boost
            # or if it is barely varying, give it a boost
            if rangefreq > 400:
                score += 4.0
                print "+3 for wide range of frequencies!"
            elif rangefreq < 10:
                score += 4.0
                print "+3 for low range of frequencies!"
            else: 
                score += 1.0

            # amp range
            # big boost for widely varying amps or for dead same on the amp
            if rangeamp > 20:
                score += 4.0
                print "+3 for high range of amps!"
            elif rangeamp == 1 and numattacks > 1:
                score += 5.0
                print "+4 for only having ONE amp!"
            else:
                score += 1.0

            # freq recurrence
            # value repeated freqs, and value more based on numattacks
            # as long as there's more than one attack
            if modefreq[0] > 1:
                # if recurring freqs at least half of the total freqs
                # then add +2
                if modefreq[0] >= numattacks/8.0:
                    score += 4.0
                    print("+3 for %d out of %d attacks being same freq!" 
                                    % (modefreq[0],int(numattacks/2.0)))
                else:
                    score += 1.0

            # frequency variance from the mean
            # value high variance or low variance.  middle variance is boring
            # need at least 3 attacks or this doesn't get measured
            if numattacks > 2:
                if stdevfreq > 125:
                    score += 4.0
                    print "+3 for high frequency variance from the mean!"
                elif stdevfreq < 25:
                    score += 4.0
                    print "+3 for low frequency variance from the mean!"
                else:
                    score += 1.0

            # amplitude variation
            # value high variance or low variance.  middle variance is boring
            # need at least 3 attacks or this doesn't get measured
            if numattacks > 2:
                if stdevamp > 4:
                    score += 4.0
                    print "+3 for high amplitude variance from the mean!"
                elif stdevfreq < 2:
                    score += 4.0
                    print "+3 for low amplitude variance from the mean!"
                else:
                    score += 1.0

                                                                        
            # skew ?

            # max amp
            if minmaxamp[1] > maxampthreshold:
                score += 5.0
                mutationbump = .01
                print "+5 AND increasing mutation rate by .01 for seriously fucking loud!"
            elif minmaxamp[1] < ampcutoff + 2:
                score += 3.0
                print "+3 for making sound but being barely above the noise!"
            
        # there's no snd for this event
        else:
            score -= 2.0
            print "-2 for no sound!"

        
        # can't have <= 0 fitness, so reset to 0 if that's the case
        # (I'm not sure this should happen anymore, though)
        if score < 0:
            score = 0
            print "score was less than 0!! (reset to 0)"

        # print some statistics about the sound info used to calc this fitness
        print "----------------"
        print "sound statistics"
        print "----------------"
        if source:
            print "source      = %s" % source 
        else:
            print "source      = undefined"
        print "numattacks  = %d" % numattacks
        print "meanfreq    = %f" % meanfreq
        print "meanamp     = %f" % meanamp
        print "rangefreq   = %f" % rangefreq
        print "rangeamp    = %f" % rangeamp
        if modefreq:
            print "modefreq    = %f" % modefreq[0]
        else:
            print "modefreq    = undefined"
        if modeamp:
            print "modeamp     = %f" % modeamp[0]
        else:
            print "modeamp     = undefined"
        print "stdevfreq   = %f" % stdevfreq
        print "stdevamp    = %f" % stdevamp
        print "skewfreq    = %f" % skewfreq
        print "----------------"

        # print the fitness and individual specifics
        print("fitness: individual = %s" % (chromosome.getBinary()))
        print decoded
        print "SCORE = %f\n" % score

        # if this is last individual in generation, then signal the
        # calling thread to unblock using task_done()
        fitq.task_done()

        # return the score!
        return score


    # loop init
    # startup and init GA
    # startup gastepper
    # startup the loop
    # do anything else
    def __init__(self,support):
        global seed

        # the bit string decoder
        decoder = Decoders.BitStringDecoder(support.getbounds(),
                                       (minpaintspeed,maxpaintspeed))
        
        # for controlling dynamics from here
        dynamics = Dynamics()

        # Genome instance
        genome = G1DBinaryString.G1DBinaryString(stringlength)

        # The evaluator function (objective function)
        genome.evaluator.set(self.fitness)

        # using two-point crossover
        genome.crossover.set(Crossovers.G1DBinaryStringXTwoPoint)

        # using a standard bit-flip mutator
        genome.mutator.set(Mutators.G1DBinaryStringMutatorFlip)

        # make our own seed so that I can reuse it later on
        # if I want to redo a particularly good run
        seed = randint(0, sys.maxint)
        print "SEED: ", seed

        # Genetic Algorithm Instance
        ga = GSimpleGA.GSimpleGA(genome,seed)
        ga.selector.set(Selectors.GTournamentSelector)
        ga.setGenerations(numgenerations)
        ga.setPopulationSize(popsize)
        ga.setElitism(elitist)
        ga.setMutationRate(mutation)

        # initialize the initial population, but don't start
        # evolving yet (leave that for step() in the loop())
        ga.initialize(createdq)

        # startup gastepper() as a thread
        gastepper = threading.Thread(target=self.gastepper,args=(ga,))
        gastepper.setDaemon(True)
        gastepper.start()

        # startup loop() as a thread
        loop = threading.Thread(target=self.loop,args=(ga,decoder,dynamics))
        loop.setDaemon(True)
        loop.start()
        loop.join()


    # blocks on a q.get() until it's time to calc another generation
    def gastepper(self,ga):
        while True:
            # block until someone else puts a val on stpq
            step = stepq.get()

            # calc a generation
            ga.step(createdq)

            # notify caller that we're done!
            stepq.task_done()



    # everything happens from here:
    # manage stepping forward the evolution engine
    # painting of individuals
    # storing of sound info
    # as well as some stats printing and waiting for sound information
    def loop(self,ga,decoder,dynamics):

        global seed
        global currentgeneration
        global lastorigins
        global origins
        global fills 
        global arcs 
        global points 
        global lines
        global mutationbump
        global paused
        global numgenerations


        # fitqlist holds popsize values so that fitness 
        # runs as many times as necessary at the end of each 
        # generation
        fitqlist = []

        # generic counter var used for filling fitqlist
        cnt = 0

        # loop for just generation 0 -- special case!!
        while True:

            #print "mutation in loop:"
            #print ga.pMutation
            #ga.pMutation += .01
            if mutationbump:
                ga.pMutation += mutationbump
                mutationbump = 0
                print "new mutation: %f" % ga.pMutation

            # check for a keyboard pause event
            if kbhit() or paused:
                print "paused..."
                paused = True
                ch = getch()
                while True:
                    if kbhit():
                        print "unpaused..."
                        paused = False
                        ch = getch()
                        break

            # fill fitqlist 
            while cnt < popsize:
                fitqlist.append(cnt)
                cnt += 1

            # individual 0 is a special case, so deal with it
            # before the while loop below

            # new generation, so (re)set current individual to 0
            currentindividual = 0

            # block until the step() function lets us know that it's finished
            # creating the unsorted list we'll use in a moment
            resume = createdq.get()

            # store the current generation
            currentgeneration = ga.getCurrentGeneration()

            # get a list of individuals for the current generation
            # using my bastardized function getUnsorted... to get around
            # the issue of presenting individuals before evaling them
            poplist = ga.getUnsortedPopListBinary()

            # get the first individual from the population
            individual = poplist[currentindividual]

            # print info
            print("\n\nNEW GENERATION: %d" % (currentgeneration))
            print("--------------------")
            print("generation# %d, individual#%d = %s" 
                  % (currentgeneration,currentindividual,individual))
            
            # start capturing for currentindividual -1
            # (putting a value on capq starts the capture)
            #print "putting a val on capq"
            #capq.put(currentindividual)

            machine.printelapsedtime(currentgeneration,currentindividual)
            # paint individual
            print "PAINTING individual#%d ..." % (currentindividual)


            decodedindividual = decoder.decode(individual)
            print decodedindividual
            self.paint(decodedindividual,currentindividual,dynamics)

            # wait a bit (simulation - remove later)
            print "PAINTING DONE\n"

            # we're done with individual 0 (and the special case for ind 0)
            currentindividual = 1

            # loop on the rest of the individuals in this generation
            while currentindividual < popsize:

                # check for a keyboard pause event
                if kbhit() or paused:
                    print "paused..."
                    paused = True
                    ch = getch()
                    while True:
                        if kbhit():
                            print "unpaused..."
                            paused = False
                            ch = getch()
                            break
               


                # get a list of individuals for the new generation
                # using my bastardized function getUnsorted... to get around
                # the issue of presenting individuals before evaling them
                poplist = ga.getUnsortedPopListBinary()

                # get the next individual from the population
                individual = poplist[currentindividual]

                # print info
                print("generation# %d, individual#%d = %s" 
                      % (currentgeneration,currentindividual,individual))
                
                # start capturing for currentindividual -1
                # (putting a value on capq starts the capture)
                #print "putting a val on capq"
                #capq.put(currentindividual)

                machine.printelapsedtime(currentgeneration,currentindividual)
                # paint individual
                print "PAINTING individual#%d" % (currentindividual)

                decodedindividual = decoder.decode(individual)
                print decodedindividual
                self.paint(decodedindividual,currentindividual,dynamics)

                print "PAINTING DONE\n"

                # increment currentindividual
                currentindividual += 1


            # now that we've painted all individuals in this generation, 
            # pause and grab snd for the last individual in the generation
            gdelay = 2 + uniform(2,5)

            capq.put(currentindividual)
            print "end of generation %d, waiting for %f seconds" % (currentgeneration,gdelay)
            time.sleep(gdelay)
            stopq.put(1)
            stopq.join()

            # eval fitness for generation
            # (putting a val on fitq allows it to proceed)
            for item in fitqlist:
                #print "putting %d on fitq" % (item)
                # block the loop until fitness processing is complete
                fitq.put(item)
                #fitq.join()

            # evaluate (fitness) generation 0
            ga.internalPop.evaluate()

            # block the loop until fitness processing is complete
            fitq.join()

            # sort generation 0
            ga.internalPop.sort()

            # increment currentGeneration
            ga.currentGeneration += 1

            # setup lastorigins for next generation
            lastorigins = list(origins)

            # now that we've unblocked step(), wait for it to finish
            stepq.join()

            ga.printStats()
            print ga.bestIndividual()

            #if pauseaftereach:
            #    paused = True

            break



        # the main loop that runs for generations 1 until complete
        while ga.getCurrentGeneration() < numgenerations:
            #print "mutation in loop:"
            #print ga.pMutation
            #ga.pMutation += .01

            if mutationbump:
                ga.pMutation += mutationbump
                mutationbump = 0
                print "new mutation: %f" % ga.pMutation

            # check for a keyboard pause event
            if kbhit() or paused:
                print "paused..."
                paused = True
                ch = getch()
                while True:
                    if kbhit():
                        print "unpaused..."
                        paused = False
                        ch = getch()
                        break

            # fill fitqlist 
            while cnt < popsize:
                fitqlist.append(cnt)
                cnt += 1

            # individual 0 is a special case, so deal with it
            # before the while loop below

            # new generation, so (re)set current individual to 0
            currentindividual = 0

            # calc a generation
            stepq.put(1)

            # block until the step() function lets us know that it's finished
            # creating the unsorted list we'll use in a moment
            resume = createdq.get()

            # store the current generation
            currentgeneration = ga.getCurrentGeneration()

            # get a list of individuals for the current generation
            # using my bastardized function getUnsorted... to get around
            # the issue of presenting individuals before evaling them
            poplist = ga.getUnsortedPopListBinary()

            # get the first individual from the population
            individual = poplist[currentindividual]

            # print info
            print("\n\nNEW GENERATION: %d" % (currentgeneration))
            print("--------------------")
            print("generation# %d, individual#%d = %s" 
                  % (currentgeneration,currentindividual,individual))
            
            # start capturing for currentindividual -1
            # (putting a value on capq starts the capture)
            print "putting a val on capq"
            capq.put(currentindividual)

            machine.printelapsedtime(currentgeneration,currentindividual)
            # paint individual
            print "PAINTING individual#%d ..." % (currentindividual)

            decodedindividual = decoder.decode(individual)
            print decodedindividual
            self.paint(decodedindividual,currentindividual,dynamics)

            # wait a bit (simulation - remove later)
            print "PAINTING DONE\n"

            # we're done with individual 0 (and the special case for ind 0)
            currentindividual = 1

            # loop on the rest of the individuals in this generation
            while currentindividual < popsize:

                # check for a keyboard pause event
                if kbhit() or paused:
                    print "paused..."
                    paused = True
                    ch = getch()
                    while True:
                        if kbhit():
                            print "unpaused..."
                            paused = False
                            ch = getch()
                            break
               


                # get a list of individuals for the new generation
                # using my bastardized function getUnsorted... to get around
                # the issue of presenting individuals before evaling them
                poplist = ga.getUnsortedPopListBinary()

                # get the next individual from the population
                individual = poplist[currentindividual]

                # print info
                print("generation# %d, individual#%d = %s" 
                      % (currentgeneration,currentindividual,individual))
                
                # start capturing for currentindividual -1
                # (putting a value on capq starts the capture)
                print "putting a val on capq"
                capq.put(currentindividual)

                machine.printelapsedtime(currentgeneration,currentindividual)

                # paint individual
                print "PAINTING individual#%d" % (currentindividual)

                decodedindividual = decoder.decode(individual)
                print decodedindividual
                self.paint(decodedindividual,currentindividual,dynamics)

                print "PAINTING DONE\n"

                # increment currentindividual
                currentindividual += 1


            # now that we've painted all individuals in this generation, 
            # pause and grab snd for the last individual in the generation
            capq.put(currentindividual-1)
            gdelay = 2 + uniform(2,5)
            #if currentgeneration+1 != numgenerations:
            #    time.sleep(gdelay)
            print "end of generation %d, waiting for %f seconds" % (currentgeneration,gdelay)
            time.sleep(gdelay)
            stopq.put(1)
            stopq.join()

            # eval fitness for generation
            # (putting a val on fitq allows it to proceed)
            for item in fitqlist:
                #print "putting %d on fitq" % (item)
                # block the loop until fitness processing is complete
                fitq.put(item)
                fitq.join()

            # block the loop until fitness processing is complete
            #fitq.join()

            # setup lastorigins for next generation
            lastorigins = list(origins)

            # now that we've unblocked step(), wait for it to finish
            stepq.join()

            ga.printStats()
            print ga.bestIndividual()

            #numgenerations = numgenerations + 1
            
            if currentgeneration + 1 == numgenerations:
                print "end of generations, checking elapsed time"
                elapsed = machine.getelapsedtime()
                if elapsed[0] < MINTIME:
                    print "time is less than mintime, adding a generation"
                    ga.setGenerations(numgenerations+1)
                    print "now numgenerations = ",ga.getGenerations()
                    numgenerations = numgenerations + 1


#            if pauseaftereach and currentgeneration > 8:
#                paused = True

        # some final statistics
        ga.printStats()
        print "GA finished.  Painting finished.  Seed = %d" % (seed)
        print("popsize = %d, numgenerations = %d, mutation = %f" % 
              (popsize,numgenerations,mutation))
        print("fills = %d, arcs = %d, lines = %d, points = %d" % 
              (fills,arcs,lines,points))
        print "elitist = ", elitist
        print "arc fixes: reverse = %d, new radius = %d, nofix = %d, arcnolength = %d" % (reversefix,radiusfix, nofix,arcnolength)
        print "arc crossings: minX %d, minY %d, maxX %d, maxY %d" % (exceedminX, exceedminY, exceedmaxX, exceedmaxY)
        print "arc crossings totals: crossings: %d, morethanone: %d" % (crossings, morethanone)

        endq.put(1)


    def paint(self,gesture,currentindividual,dynamics):
        global zpaint
        global lastcolor
        global lastgesture
        global origins
        global currentgeneration
        global fills
        global arcs
        global lines
        global points
        
        for item in gesture:
            # get the tag and the value
            tag = item[0]
            val = item[1]

            if tag == "type":
                gtype = val 
            elif tag == "color":
                color = val 
            elif tag == "originX":
                x1 = val
            elif tag == "originY":
                y1 = val
            elif tag == "angle":
                angle = val
            elif tag == "length":
                length = val
            elif tag == "speed":
                speed = val
            elif tag == "pressure":
                pressure = val
            elif tag == "volume":
                volume = val


            elif tag == "width":
                fillwidth = val
            elif tag == "variance":
                variance = val

            elif tag == "arcness":
                arcness = val
            elif tag == "arcdir":
                arcdir = val
            elif tag == "arcseg":
                arcseg = val


            elif tag == "angle1":
                angle1 = val
            elif tag == "length1":
                length1 = val
            elif tag == "arcness1":
                arcness1 = val
            elif tag == "arcdir1":
                arcdir1 = val

            elif tag == "angle2":
                angle2 = val
            elif tag == "length2":
                length2 = val
            elif tag == "arcness2":
                arcness2 = val
            elif tag == "arcdir2":
                arcdir2 = val

            elif tag == "angle3":
                angle3 = val
            elif tag == "length3":
                length3 = val
            elif tag == "arcness3":
                arcness3 = val
            elif tag == "arcdir3":
                arcdir3 = val

        # keep a running track of all origins for the fitness() eval
        origins.append((x1,y1))

        xy2 = line.getvector(x1,y1,length,angle)

        # test to make sure the length doesn't send x2,y2 outside
        # of the support boundaries.  if it does, truncate it
        tested = self.testlength(x1,y1,xy2[0],xy2[1])
        x1 = tested[0]
        y1 = tested[1]
        xy2 = (tested[2],tested[3])
        

        # set the pressure
        if pressure == 0:
            zpaint = zpaint0
        elif pressure == 1:
            zpaint = zpaint1
        elif pressure == 2:
            zpaint = zpaint2
        elif pressure == 3:
            zpaint = zpaint3

        
        # if we need the same color as last gesture
        # and if we didn't just do a fill or an arc
        # then we don't need to get any new paint
        if lastcolor == colormap[color] and lastgesture != "fill" and lastgesture != "arc" and (gtype != "arc" and arcseg != 0):
                print "-- using paint already on brush --"

        # otherwise, it's a new color or a fill, so get some paint
        else:
            paints.getpaint(colormap[color])


        # signal to stop capture and wait for it
        # stopping capture here means that all sounds up through the getting of the paint
        # for the new gesture counts for the last gesture painted
        if lastgesture != "-1":
            print "stop capturing"
            stopq.put(1)
            stopq.join()

        print "start capturing for individual %d" % currentindividual
        capq.put(currentindividual)

        # track the last color
        lastcolor = colormap[color]

        # set the speed for this gesture
        print "speed = %f" % speed
        machine.feedrate(speed)

        # set volume
        print "volume = ",volume
        dynamics.setvolume(volumes[volume])
        # paint the gesture!

        #gtype = "arc"

        if gtype == "line":
            lastgesture = "line"
            lines += 1
            line.paint(x1,y1,xy2[0],xy2[1])

        elif gtype == "arc":
            lastgesture = "arc"
            lengthpainted = 0
            arcs +=1

            # paint segment 1
            arc.paint(x1,y1,xy2[0],xy2[1],arcness,arcdir)
            lengthpainted += self.edistance(x1,y1,xy2[0],xy2[1])

            # if 1 segments was all we needed, return
            if arcseg == 0:
                return

            # otherwise its a multiseg, so we know we have to paint seg 2
            #if lengthpainted > 2.5:
            #    paints.getpaint(colormap[color])
            #    lengthpainted = 0
            #    machine.feedrate(speed)
            newx1 = xy2[0]
            newy1 = xy2[1]
            xy2 = line.getvector(newx1,newy1,length1,angle1)
            #xy2 = self.testlength(x1,y1,xy2)
            tested = self.testlength(newx1,newy1,xy2[0],xy2[1])
            newx1 = tested[0]
            newy1 = tested[1]
            xy2 = (tested[2],tested[3])
            if (newx1 != xy2[0] and newy1 != xy2[1]):
                arc.paint(newx1,newy1,xy2[0],xy2[1],arcness1,arcdir1)
            lengthpainted += self.edistance(newx1,newy1,xy2[0],xy2[1])

            # if 2 segments was all we needed, return
            if arcseg == 1:
                return

            # otherwise paint seg 3
            #if lengthpainted > 2.5:
            #    paints.getpaint(colormap[color])
            #    lengthpainted = 0
            #    machine.feedrate(speed)
            newx1 = xy2[0]
            newy1 = xy2[1]
            xy2 = line.getvector(newx1,newy1,length2,angle2)
            #xy2 = self.testlength(x1,y1,xy2)
            tested = self.testlength(newx1,newy1,xy2[0],xy2[1])
            newx1 = tested[0]
            newy1 = tested[1]
            xy2 = (tested[2],tested[3])
            if (newx1 != xy2[0] and newy1 != xy2[1]):
                arc.paint(newx1,newy1,xy2[0],xy2[1],arcness1,arcdir1)
            lengthpainted += self.edistance(newx1,newy1,xy2[0],xy2[1])
                
            # if 3 segments was all we needed, return
            if arcseg == 2:
                return

            # otherwise paint seg 4
            #if lengthpainted > 2.5:
            #    paints.getpaint(colormap[color])
            #    lengthpainted = 0
            #    machine.feedrate(speed)
            newx1 = xy2[0]
            newy1 = xy2[1]
            xy2 = line.getvector(newx1,newy1,length3,angle3)
            #xy2 = self.testlength(x1,y1,xy2)
            tested = self.testlength(newx1,newy1,xy2[0],xy2[1])
            newx1 = tested[0]
            newy1 = tested[1]
            xy2 = (tested[2],tested[3])
            if (newx1 != xy2[0] and newy1 != xy2[1]):
                arc.paint(newx1,newy1,xy2[0],xy2[1],arcness3,arcdir3)

        elif gtype == "point":
            lastgesture = "point"
            points += 1
            point.paint(x1,y1)
        
        elif gtype == "fill":
            lastgesture = "fill"
            fills += 1
            fill.paint(x1,y1,angle,length,fillwidth,variance,colormap[color],speed)

    def edistance(self,x1,y1,x2,y2):
        return math.sqrt(math.pow(x2-x1,2) + math.pow(y2-y1,2))
    


# starts up the OSC server thread, which waits for messages from 
# an OSC client.  for each message, the audiohandler() function 
# is called.  audiohandler() filters the messages to weed out the
# noise, looks for attacks, and sends on new attacks along with their
# amplitude and frequency
class Listener():

    lastamp = 0
    inbang = False

    waitingforevent = True
    capturingevent = False
    grabbing = -1
    lastfreq = -1
    lastfreq2 = -1
    inEvent = False


    def __init__(self):
        global recvaddr;


        # OSC Server. there are three different types of server. 
        #s = OSC.OSCServer(receive_address) # basic
        self.s = OSC.ThreadingOSCServer(recvaddr) # threading
        ##s = OSC.ForkingOSCServer(receive_address) # forking

        # this registers a 'default' handler (for unmatched messages), 
        # an /'error' handler, an '/info' handler.
        # And, if client supports it, a '/subscribe' & '/unsubscribe' handler
        self.s.addDefaultHandlers()

        # just checking which handlers we have added
        print "Registered Callback-functions are :"
        for addr in self.s.getOSCAddressSpace():
            print addr

        # Start OSCServer
        print "\nStarting OSCServer. Use ctrl-C to quit."
        st = threading.Thread( target = self.s.serve_forever )
        st.setDaemon(True)
        st.start()

        # define a message-handler function for the server to call.
        def audiohandler(addr, tags, stuff, source):
            global ampcutoff
            global capturing
            global currentcapture

            amp = stuff[0]
            freq = stuff[1]

            # uncomment this block for noise testing
            #global maxamptest
            #maxamptest.append(amp)
            #print "ALL: amp = %d, freq = %d, MAXAMP = %d" % (amp,freq,max(maxamptest))
            #print "ALL: amp = %d, freq = %d" % (amp,freq)



            # stopq has a value, so reset bool and get out of here
            if not stopq.empty():
                print "AUDIOHANDLER: stopping capture"
                stop = stopq.get()
                capturing = False
                #print "set capturing to false"
                sndq.put(currentcapture)
                currentcapture = []
                stopq.task_done()
                return

            # if we're not capturing and capq isn't empty
            # then we're starting capture
            elif not capq.empty() and not capturing:
                print "AUDIOHANDLER: starting capture"
                currentindividual = capq.get()
                capturing = True
                #print "capturing so store snd"
                
            # and if we aren't capturing, bolt
            elif not capturing:
                #print "not capturing so returning"
                #print "AUDIOHANDLER: ignoring"
                return

            # uninterested, must be waiting for event 
            if amp < ampcutoff:
                self.waitingforevent=True

            elif(amp > ampcutoff and (self.lastfreq + 10 < freq or self.lastfreq - 10 > freq)):
                self.waitingforevent = True

            # previous spike indicated a grab coming up, here it is 
            if self.grabbing == 0:
                # only really grab it if it's above noise 
                if amp > ampcutoff:
                    # comment out next line for straight testing of audio 
                    currentcapture.append((amp,freq))
                    self.capturingevent = False
                    print "\ncaptured: amp = %d, freq = %d\n" % (amp,freq)

                self.grabbing = -1

            elif self.grabbing > 0:
                self.grabbing -= 1

            # if noise, dump it 
            elif self.lastfreq == freq:
                #print "noise"
                return

            # a spike: it's useful, setup a grab 
            elif amp > ampcutoff and self.waitingforevent: 
                #print "useful -- will grab"
                self.waitingforevent = False
                self.capturingevent = True
                self.grabbing = 2

            self.lastfreq = freq

        self.s.addMsgHandler("/print", audiohandler) # adding our function


    def shutdown(self):
        print "\nClosing OSCServer."
        s.close()
        print "Waiting for Server-thread to finish"
        st.join() ##!!!
        print "Done"


        

if __name__ == '__main__':


    start = time.time()


    atexit.register(set_normal_term)
    set_curses_term()

    # create and initialize the machine
    machine = Machine()
    machine.printelapsedtime(0,0)

    # setup paint colors and positions
    paints = Paints()

    # 10x15
    negY = -7.3311
    paints.addcolor("c1",-3.7807,negY)
    paints.addcolor("c2",-1.2914,negY)
    paints.addcolor("c3", 1.2219,negY)
    paints.addcolor("c4", 3.7260,negY)
    posY = 7.5296
    paints.addcolor("c1",-3.7351,posY)
    paints.addcolor("c2",-1.2526,posY)
    paints.addcolor("c3", 1.2515,posY)
    paints.addcolor("c4", 3.7670,posY)

    # create a new support (x1,y1,w,h,name)
    support = Support(supportoriginX,supportoriginY,
                      supportwidth,supportheight,"s1")

    # create new shape objects for painting on the support
    square = Square(support)
    point = Point(support)
    circle = Circle(support)
    line = Line(support)
    arc = Arc(support)
    fill = Fill(support)

    drawbound = True
    #drawbound = False

    if drawbound:
        machine.rapid(minX,minY)
        machine.rapid(maxX,minY)
        machine.rapid(maxX,maxY)
        machine.rapid(minX,maxY)
        machine.rapid(minX,minY)

    # start listening for audio events and store them when asked for by the loop
    listener = Listener()

    # volume control
    #dynamics = Dynamics()

    # startup the loop, which runs everything from the GA to painting 
    loop = Loop(support)

    endq.get()
    machine.printelapsedtime(0,0)
    machine.end()
