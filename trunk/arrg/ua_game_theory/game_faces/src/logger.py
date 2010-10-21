#!/usr/bin/env python

import os.path
import datetime
import time


class logger:

    filename = ''
    
    def __init__(self):
        
        return
    
    def openlog(self, filename, header = ''):
        self.filename = filename;

        print "Logging to " + str( self.filename )
        FILE = open(self.filename, 'a')
        FILE.write("Game Faces playing log ")
        FILE.write(time.asctime( time.localtime(time.time()) ))
        FILE.write("\n")
        FILE.write(header)
        FILE.write("\n")
        # output time.asctime( time.localtime(time.time()) )
        #
        # print datetime.datetime.now()
        FILE.close()

    def log(self, event):
        
        if (self.filename == ''):
            print "Logger not yet initialized!"
            return

        FILE = open(self.filename, 'a')
        FILE.write( str(time.time()) + ' ' + str(event) )
        FILE.write("\n")
        FILE.close()
        
        return
        

