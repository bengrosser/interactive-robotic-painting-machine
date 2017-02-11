#!/usr/bin/python

import math

# for temporary here
#minspeed = 80
#maxspeed = 220
minspeed = 100
maxspeed = 400

class BitStringDecoder():

    # list of tuples defining the individual bit string coding
    # format is "parameter name", #bits

    # volume was after originX

    stringdef = [
        ("type",2),
        ("color",2),
        ("angle",8),
        ("pressure",2),
        ("arcness",3),
        ("length3",4),
        ("originX",8),
        ("arcdir",1),
        ("arcseg",2),
        ("speed",6),
        ("angle1",8),
        ("length1",4),
        ("variance",2),
        ("arcness1",3),
        ("arcdir1",1),
        ("width",8),
        ("angle2",8),
        ("length2",4),
        ("arcness2",3),
        ("arcdir2",1),
        ("originY",8),
        ("angle3",8),
        ("arcness3",3),
        ("length",4),
        ("arcdir3",1),
        ("volume",3)
        ]

    types = [
        ("line"),
        ("arc"),
        ("point"),
        ("fill")
        ]

    arcdirs = [
        "cw",
        "ccw"
        ]

    # maximum decimal value of X/Y in the bitstring
    maxX = 255
    maxY = 255

    # maximum decimal value of angle in the bitstring
    maxA = 255

    # max dec for length
    maxL = 15

    # max dec for speed
    #maxS = 255
    maxS = 63
    #maxS = 31

    # max dec for speed
    maxP = 3

    # max dec for variance
    maxV = 3

    # max dec for fill width
    maxW = 255 

    # maximum width of a fill
    maxfillwidth = 2


    def __init__(self,supportsize,speedvals):
        self.oX = supportsize[0]
        self.oY = supportsize[1]
        self.width = supportsize[2]
        self.height = supportsize[3]
        self.segX = float(self.width)/float(self.maxX)
        self.segY = float(self.height)/float(self.maxY)
        self.segA = float(360)/float(self.maxA)
        self.segW = float(self.maxfillwidth)/float(self.maxW)

        maxdistance = self.edistance(self.oX,self.oY,self.oX+self.width,
                                self.oY+self.height)
        # arbitrary for testing
        # was 4
        maxdistance = 4
        maxdistance = 3

        self.segL = float(maxdistance)/float(self.maxL)

        self.minspeed = speedvals[0]
        self.maxspeed = speedvals[1]

        self.segS = float(self.maxspeed-self.minspeed)/float(self.maxS)

        stringlength = 0
        for item in self.stringdef:
            stringlength += item[1]

        print "---Decoder Initialized---"
        print "bit string length = %d" % (stringlength)


    def binary2decimal(self,s):
        decimal = 0
        for i in s:
            decimal = (decimal << 1) | int(i)
        return decimal 

    def edistance(self,x1,y1,x2,y2):
        return math.sqrt(math.pow(x2-x1,2) + math.pow(y2-y1,2))

    # decodes a bit string into whatever parameters it represents
    # return is a list of tuples, such as: [("type","line"),("color",0),etc.]
    # up to calling function to use the information intelligently
    def decode(self,bitstring):
        # manages where we are in the bitstring
        start = 0
        finish = 0

        minlength = 1.00

        # lists to hold decoded values
        decvals = []
        decoded = []

        # run through every item in stringdef and pull out values from 
        # the bitstring.  if you run out of bitstring then the values 
        # show up as zero (maybe error check this?)
        for item in self.stringdef:
            parametername = item[0]
            parameterlength = item[1]
            end = start+parameterlength
            parametervalue = bitstring[start:end]
            pvaldecimal = self.binary2decimal(parametervalue)
            decvals.append((parametername,pvaldecimal,parametervalue))
            start = end

        # now work through the decimal values and turn them into
        # parameter values, (each one is a special case?)
        for item in decvals:
            # get the tag and the value
            tag = item[0]
            pick = item[1]

            if tag == "type":
                decoded.append((item[0],self.types[pick]))
            elif tag == "color":
                decoded.append((item[0],pick))
            elif tag == "volume":
                decoded.append((item[0],pick))
            elif tag == "originX":
                decoded.append((item[0],(self.segX * pick) + self.oX))
            elif tag == "originY":
                decoded.append((item[0],(self.segY * pick) + self.oY))
            elif tag == "angle":
                decoded.append((item[0],(self.segA * pick)))
            elif tag == "length":
                #print "length: orig pick = %f" % pick
                if pick < minlength:
                    pick = minlength
                    #print "length: adj pick = %f" % pick
                x = self.segL * pick
                #print "length: segL = %f" % self.segL
                #print "length: x = %f" % x
                decoded.append((item[0],x))
            elif tag == "speed":
                decoded.append((item[0],(self.segS * pick)+self.minspeed))
            elif tag == "pressure":
                decoded.append((item[0],pick))

            elif tag == "width":
                decoded.append((item[0],self.segW * pick))
            elif tag == "variance":
                decoded.append((item[0],pick))

            elif tag == "arcness":
                decoded.append((item[0],pick))
            elif tag == "arcdir":
                decoded.append((item[0],self.arcdirs[pick]))

            elif tag == "arcseg":
                decoded.append((item[0],pick))

            elif tag == "angle1":
                decoded.append((item[0],(self.segA * pick)))
            elif tag == "length1":
                if pick == 0:
                    pick = minlength
                decoded.append((item[0],(self.segL * pick)))
            elif tag == "arcness1":
                decoded.append((item[0],pick))
            elif tag == "arcdir1":
                decoded.append((item[0],self.arcdirs[pick]))

            elif tag == "angle2":
                decoded.append((item[0],(self.segA * pick)))
            elif tag == "length2":
                if pick == 0:
                    pick = minlength
                decoded.append((item[0],(self.segL * pick)))
            elif tag == "arcness2":
                decoded.append((item[0],pick))
            elif tag == "arcdir2":
                decoded.append((item[0],self.arcdirs[pick]))

            elif tag == "angle3":
                decoded.append((item[0],(self.segA * pick)))
            elif tag == "length3":
                if pick == 0:
                    pick = minlength
                decoded.append((item[0],(self.segL * pick)))
            elif tag == "arcness3":
                decoded.append((item[0],pick))
            elif tag == "arcdir3":
                decoded.append((item[0],self.arcdirs[pick]))

        return decoded
    

if __name__ == '__main__':

    #for item in stringdef:
    #    print "%s: %d" % (item[0],item[1])


    # speedvals[0] is min, speed[1] is max
    # decoder = BitStringDecoder((support.getbounds()),speedvals)
    decoder = BitStringDecoder((-5,-5,10,10),(minspeed,maxspeed))


    print "\n"

    #                         t c x1      y1      angle   len speed   p an dS angle   len an dangle   len an dangle   len an d
    decoded = decoder.decode("000111111111110100011111111100000000000000000000111111110000000011111111000000001111111100000000")
    for x in decoded:
        print "%s: %s" % (x[0],x[1])

    print "\n"

    #                         t c x1      y1      angle   len speed   p an dS angle   len an dangle   len an dangle   len an d
    decoded = decoder.decode("100000000000011011110000000011111111111111111111000000001111111100000000111111110000000011111111")
    for x in decoded:
        print "%s: %s" % (x[0],x[1])


