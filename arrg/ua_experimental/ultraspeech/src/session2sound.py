#!/usr/bin/env python

import optparse
import os
import glob
from bag2sound import exportBag

if __name__ == "__main__":
    parser = optparse.OptionParser()
    parser.add_option('-b', '--bag-directory', help='directory containing bagfiles to have sound extracted', dest='bagname')
    parser.add_option('-d', '--out-directory', help='path where new soundfiles will go', dest='dirname', default = '.')
    parser.add_option('-p', '--pre', help='amount of audio before click to include', dest='time_before', type='int', default = 0)
    parser.add_option('-a', '--after', help='amount of audio after click to include', dest='time_after', type='int', default = 8)
    parser.add_option('-s', '--single-file', help='Don''t split sound into clips', dest='singlefile', default=False, action='store_true')
    (opts, args) = parser.parse_args()

    if opts.time_after < 1:
        opts.time_after = 1
    if opts.singlefile:
        print "Storing as a single file."
    if opts.bagname is None:
        print "bagfile option must be input on command line"
        parser.print_help()
        exit()

    bags = glob.glob( os.path.join( opts.bagname, '*.bag') )
    if len(bags) is 0:
        print "No bagfiles found"
        exit()
    for infile in bags:
        print "current file is: " + infile
        opts.bagname = infile
        exportBag(opts)

