#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Antons Rebguns. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of Arizona nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import time
import serial
import math
from binascii import b2a_hex

from chr_6dm_const import *

class SerialIO(object):
    """ Provides low level IO with the CHR-6dm IMU through pyserial. """

    def __init__(self, port):
        """ Constructor takes serial port as argument. """
        try:
            self.ser = None
            self.ser = serial.Serial(port)
            self.ser.setTimeout(0.015)
            self.ser.baudrate = 115200
            self.ser.bytesize = serial.EIGHTBITS
            self.ser.stopbits = serial.STOPBITS_ONE
            self.ser.parity = serial.PARITY_NONE
        except:
           raise(SerialOpenError(port, baudrate))

    def __del__(self):
        """ Destructor calls self.close_serial_port() """
        self.close_serial_port()

    def write_to_imu(self, command, data=tuple()):
        self.ser.flushInput()

        dataStr = ''
        chkSum = 0

        for b in data:
            dataStr += chr(byte)
            chkSum += b

        # packet: s  n  p  INSTRUCTION  LENGTH  DATA  CHECKSUM
        # bytes:  1  1  1       1          1     N       2
        packetStr = 's' + 'n' + 'p' + chr(command) + chr(len(data)) + chr(chkSum)
        self.ser.write(packetStr)

        # wait for response
        time.sleep(0.0005)

    def read_from_imu(self):
        self.ser.read() # 's'
        self.ser.read() # 'n'
        self.ser.read() # 'p'
        command = self.ser.read()
        n = self.ser.read()

        if command == BAD_CHECKSUM:
            print 'bad checksum'
        elif command == BAD_DATA_LENGTH:
            print 'bad data length'
        elif command == UNRECOGNIZED_PACKET:
            print 'unrecognized command'
        elif command == SENSOR_DATA:
            data = self.ser.read(n)
            if data[0] & 0b10000000:
                print 'yaw channel active'
            elif data[0] & 0b01000000:
                print 'pitch channel active'
            elif data[0] & 0b00100000:
                print 'roll channel active'
            elif data[0] & 0b00010000:
                print 'yaw rate channel active'
            elif data[0] & 0b00001000:
                print 'pitch rate channel active'
            elif data[0] & 0b00000100:
                print 'roll rate channel active'
            elif data[0] & 0b00000010:
                print 'mx channel active'
            elif data[0] & 0b00000001:
                print 'my channel active'

    def close_serial_port(self):
        """
        Be nice, close the serial port.
        """
        if self.ser:
            self.ser.flushInput()
            self.ser.flushOutput()
            self.ser.close()

    def handle_common_responses(self, packet):
        if packet == COMMAND_COMPLETE:
            print 'command complete'
        elif packet == COMMAND_FAILED:
            print 'command failed'
        elif packet == BAD_CHECKSUM:
            print 'bad checksum'
        elif packet == BAD_DATA_LENGTH:
            print 'bad data length'
        elif packet == UNRECOGNIZED_PACKET:
            print 'unrecognized packet'
        elif packet == BUFFER_OVERFLOW:
            print 'buffer overflow'

    def set_active_channels(self, channels):
        """
        channels is a dictionary
        """
        high_byte = str()
        high_byte += '1' if channels['yaw'] else '0'
        high_byte += '1' if channels['pitch'] else '0'
        high_byte += '1' if channels['roll'] else '0'
        high_byte += '1' if channels['yaw_rate'] else '0'
        high_byte += '1' if channels['pitch_rate'] else '0'
        high_byte += '1' if channels['roll_rate'] else '0'
        high_byte += '1' if channels['mx'] else '0'
        high_byte += '1' if channels['my'] else '0'

        low_byte = str()
        low_byte += '1' if channels['mz'] else '0'
        low_byte += '1' if channels['gx'] else '0'
        low_byte += '1' if channels['gy'] else '0'
        low_byte += '1' if channels['gz'] else '0'
        low_byte += '1' if channels['ax'] else '0'
        low_byte += '1' if channels['ay'] else '0'
        low_byte += '1' if channels['az'] else '0'
        low_byte += '0'

        high_byte = int(high_byte, 2)
        low_byte = int(low_byte, 2)

        self.write_to_imu(SET_ACTIVE_CHANNELS, (high_byte, low_byte))

    def set_silent_mode(self):
        self.write_to_imu(SET_SILENT_MODE, 0)

    def set_broadcast_mode(self, hz):
        x = int((hz - 20) / (280.0/255.0))
        self.write_to_imu(SET_BROADCAST_MODE, (x, ))

    def auto_set_accel_ref(self):
        self.write_to_imu(AUTO_SET_ACCEL_REF)

    def get_data(self):
        self.write_to_imu(GET_DATA)
        self.read_from_imu()
