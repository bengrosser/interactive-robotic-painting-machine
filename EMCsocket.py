#!/usr/bin/python

#host = "localhost"
#host = "192.168.1.140"

# simulator
host = "172.16.240.128"

## studio linux
#host = "128.174.42.10" 
# PERFORMANCE host = "192.168.1.138"

port = 5007
startupmsg = "HELLO EMC 1 1"

import socket
import sys
import time

dbug = False
reading = False

class EMCsocket():
    skt = None
    rw = None
    line = ""
    readlines = []

    def init(self,lock):
        self.lock = lock
        self.connect()
        self.makefd()
        self.wr(startupmsg)
        self.wr("SET ECHO OFF")
        self.wr("SET VERBOSE ON")
        self.wr("SET ENABLE EMCTOO")
        self.wr("SET SET_WAIT DONE")
        #EMCsocket.r.flush()

    def recontrol(self):
        global reading

        if reading:
            self.recontrol2()
        else:
            self.wr("SET ENABLE EMCTOO")


    def recontrol2(self):
        global reading

        reading = False
        print "IN recontrol()"
        self.skt.close()
        self.connect()
        self.makefd()
        self.wr(startupmsg)
        self.wr("SET ECHO OFF")
        self.wr("SET VERBOSE ON")
        self.wr("SET ENABLE EMCTOO")
        self.wr("SET SET_WAIT DONE")
        #self.init(self.lock)
        #self.lock.acquire()


#        try:
#            EMCsocket.rw.flush()
#            print("flushing socket in recontrol()")
#        except socket.error, e:
#            print("couldn't flush socket: %s" % e)
#            
        #print "b4 set echo"
        #self.wr("SET ENABLE EMCTOO")
        #self.wr("SET SET_WAIT RECEIVED")
        #self.wr("SET ECHO OFF")
        print "end of recontrol()"
        return 1

    def flushall(self):
        #EMCsocket.r.flush()
        EMCsocket.rw.flush()

    def connect(self):
        try:
            EMCsocket.skt = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            EMCsocket.skt.settimeout(30)
        except socket.error, e:
            print("Error creating socket: %s" % e)
            sys.exit(1)

        try:
            print("host=%s,port=%d" % (host,port))
            EMCsocket.skt.connect((host,port))
        except socket.gaierror, e:
            print("Address-related error connecting to server: %s" % e)
            sys.exit(1)
        except socket.error, e:
            print("Error connecting to socket: %s" % e)
            sys.exit(1)

        print("\n** setting up command socket **")
        print("Connected from: ", EMCsocket.skt.getsockname())
        print("Connected to: ", EMCsocket.skt.getpeername())

    def makefd(self):
        EMCsocket.rw = EMCsocket.skt.makefile('rwU',0)

    def write(self, msg):
        try:
            #EMCsocket.rw.write(msg)
            EMCsocket.rw.write(msg+"\r\n")
            sys.stdout.write("\r\nSEND: ")
            sys.stdout.write(msg)
            EMCsocket.rw.flush()
        except socket.error, e:
            print("couldn't write to socket: %s" % e)

    def wrr(self, msg):
        self.write(msg)
        self.read()
        self.read()

    def w(self, msg):
        #self.write("SET ENABLE EMCTOO")
        self.write(msg)
        EMCsocket.rw.flush()

    def wr(self, msg):
        global reading
        self.write(msg)
        EMCsocket.rw.flush()
        if not reading:
            return self.read()
        else:
            print "*************ERROR!!!"
        #return self.read()

    def rwr(self, msg):
        self.read()
        self.write(msg)
        EMCsocket.rw.flush()
        self.read()

    def read(self):
        global reading

        try:
            reading = True
            line = EMCsocket.rw.readline()
            reading = False
            if dbug:
                print "PRINT TESTS\n"
                print line
                print repr(line)
                print len(line)
                print len(repr(line))
                print line.find('\r')
                print line.startswith('\r')
                print "PRINT TESTS END\n"
                if line.startswith('\r'):
                    print "STARTS WITH R"
            if line.find("NAK") == -1: 
                if line.startswith('\r'):
                    print "\nREAD: "+line[1:]
                    self.readlines.append(line[1:])
                    return line[1:]
                else:
                    print "\nREAD: "+line
                    self.readlines.append(line)
                    return line
            else:
                print "ERROR: GOT A NAK"
        except socket.error, e:
            print("couldn't read from socket: %s " % e)

    def printlines(self):
        for line in self.readlines:
            print repr("123456789012345678901234567890: "+line)


if __name__ == '__main__':
    emc = EMCsocket()
    emc.wr("SET VERBOSE ON\r\n")
    emc.wr("SET ECHO OFF\r\n")
    emc.wr("SET ENABLE EMCTOO\r\n")
    emc.wr("SET MDI G0 x0 y0\r\n")
    emc.wr("SET MDI G0 x2 y2\r\n")
    emc.wr("SET MDI G0 x0 y0\r\n")
