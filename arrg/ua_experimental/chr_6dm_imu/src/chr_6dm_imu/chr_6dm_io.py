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
# Author: Antons Rebguns
#

import time
import serial
import math
import struct
from binascii import b2a_hex

from chr_6dm_const import *

DEG_TO_RAD = 0.017453293    # degrees to radians
MILIG_TO_MSS = 0.00980665   # mili g to m/s^2

class CHR6dmIMU(object):
    """ Provides low level IO with the CHR-6dm IMU through pyserial. """

    def __init__(self, port='/dev/ttyUSB0'):
        """ Constructor takes serial port as argument. """
        self.ser = None
        self.ser = serial.Serial(port)
        self.ser.timeout = 0.015
        self.ser.baudrate = 115200
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.parity = serial.PARITY_NONE
        
        print "Connected to IMU on %s" % port

    def __del__(self):
        """ Destructor calls self.close_serial_port() """
        self.close()

    def calculate_checksum(self, command, data):
        chkSum = reduce(int.__add__, map(ord, ['s', 'n', 'p']), 0)
        chkSum = chkSum + command + len(data) + reduce(int.__add__, data, 0)
        return chkSum

    def write_to_imu(self, command, data=tuple()):
        self.ser.flushInput()
        
        chkSum = self.calculate_checksum(command, data)
        dataStr = ''.join(map(chr, data))
        
        # packet: s  n  p  INSTRUCTION  LENGTH  DATA  CHECKSUM
        # bytes:  1  1  1       1          1     N       2
        packetStr = 'snp' + chr(command) + chr(len(dataStr)) + dataStr + chr(chkSum >> 8) + chr(chkSum & 0x0FF)
        self.ser.write(packetStr)
        
        print 'Command: %s, data packet: %s' % (hex(command).upper(), str(data))
        
        # wait for response
        time.sleep(0.005)
        
        self.read_from_imu()

    def read_from_imu(self):
        # look for packet prefix 'snp'
        while self.ser.read() != 's':
            print "looking for packet prefix 'snp'"
        packet = 's' + self.ser.read(2) # read 'n' and 'p'
        
        if packet != 'snp':
            print "Received corrupted packet, prefix was %s" % packet
            return
            
        command = ord(self.ser.read())
        n = ord(self.ser.read())
        
        dataStr = self.ser.read(n)
        data = map(b2a_hex, dataStr)
        data = map(int, data, [16] * len(data))
        
        print "Received reply: %s, data: %s" % (hex(command).upper(), str(data))
        
        chkSumStr = self.ser.read(2)
        chkSumData = map(b2a_hex, chkSumStr)
        chkSumData = map(int, chkSumData, [16] * len(chkSumData))
        chkSumRx = (chkSumData[0] << 8) | chkSumData[1]
        
        chkSum = self.calculate_checksum(command, data)
        
        if chkSumRx != chkSum:
            print "Checksums don't match %d != %d" % (chkSumRx, chkSum)
            return
            
        if command == COMMAND_COMPLETE:
            print 'Command %s complete' % hex(data[0]).upper()
        elif command == COMMAND_FAILED:
            print 'Command %s failed' % hex(data[0]).upper()
        elif command == BAD_CHECKSUM:
            print 'Bad checksum'
        elif command == BAD_DATA_LENGTH:
            print 'Bad data length for command %s' % hex(data[0]).upper()
        elif command == UNRECOGNIZED_PACKET:
            print 'Unrecognized packet %s' % hex(data[0]).upper()
        elif command == BUFFER_OVERFLOW:
            print 'Buffer overflow'
        elif command == STATUS_REPORT:
            print 'Self test status report'
            if (data[0] >> 5) & 0x01:
                print 'FAILED self-test: gyro_z'
            if (data[0] >> 4) & 0x01:
                print 'FAILED self-test: gyro_y'
            if (data[0] >> 3) & 0x01:
                print 'FAILED self-test: gyro_x'
            if (data[0] >> 2) & 0x01:
                print 'FAILED self-test: accel_z'
            if (data[0] >> 1) & 0x01:
                print 'FAILED self-test: accel_y'
            if data[0] & 0x01:
                print 'FAILED self-test: accel_x'
        elif command == SENSOR_DATA:
            active_channels = (data[0] << 8) | data[1]
            
            i = 2
            
            # yaw angle
            if active_channels & 0x8000:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                yaw = value[0] * SCALE_YAW * DEG_TO_RAD
                print 'yaw = %f' % yaw
                i += 2
            
            # pitch angle
            if active_channels & 0x4000:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                pitch = value[0] * SCALE_PITCH * DEG_TO_RAD
                print 'pitch = %f' % pitch
                i += 2
            
            # roll angle
            if active_channels & 0x2000:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                roll = value[0] * SCALE_ROLL * DEG_TO_RAD
                print 'roll = %f' % roll
                i += 2
            
            # yaw rate
            if active_channels & 0x1000:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                yaw_rate = value[0] * SCALE_YAW_RATE * DEG_TO_RAD
                print 'yaw_rate = %f' % yaw_rate
                i += 2
            
            # pitch rate
            if active_channels & 0x0800:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                pitch_rate = value[0] * SCALE_PITCH_RATE * DEG_TO_RAD
                print 'pitch_rate = %f' % pitch_rate
                i += 2
            
            # roll rate
            if active_channels & 0x0400:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                roll_rate = value[0] * SCALE_ROLL_RATE * DEG_TO_RAD
                print 'roll_rate = %f' % roll_rate
                i += 2
            
            # mag X
            if active_channels & 0x0200:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                mx = value[0] * SCALE_MAG_X
                print 'mx = %f' % mx
                i += 2
            
            # mag Y
            if active_channels & 0x0100:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                my = value[0] * SCALE_MAG_Y
                print 'my = %f' % my
                i += 2
            
            # mag Z
            if active_channels & 0x0080:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                mz = value[0] * SCALE_MAG_Z
                print 'mz = %f' % mz
                i += 2
            
            # gyro X
            if active_channels & 0x0040:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                gx = value[0] * SCALE_GYRO_X * DEG_TO_RAD
                print 'gx = %f' % gx
                i += 2
            
            # gyro Y
            if active_channels & 0x0020:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                gy = value[0] * SCALE_GYRO_Y * DEG_TO_RAD
                print 'gy = %f' % gy
                i += 2
            
            # gyro Z
            if active_channels & 0x0010:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                gz = value[0] * SCALE_GYRO_Z * DEG_TO_RAD
                print 'gz = %f' % gz
                i += 2
            
            # accel X
            if active_channels & 0x0008:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                ax = value[0] * SCALE_ACCEL_X * MILIG_TO_MSS
                print 'ax = %f' % ax
                i += 2
            
            # accel Y
            if active_channels & 0x0004:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                ay = value[0] * SCALE_ACCEL_Y * MILIG_TO_MSS
                print 'ay = %f' % ay
                i += 2
            
            # accel Z
            if active_channels & 0x0002:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                az = value[0] * SCALE_ACCEL_Z * MILIG_TO_MSS
                print 'az = %f' % az
                i += 2
        elif command == GYRO_BIAS_REPORT:
            value = struct.unpack('>h', dataStr[0] + dataStr[1])
            gyro_z_bias = value[0] * SCALE_GYRO_Z * DEG_TO_RAD
            
            value = struct.unpack('>h', dataStr[2] + dataStr[3])
            gyro_y_bias = value[0] * SCALE_GYRO_Y * DEG_TO_RAD
            
            value = struct.unpack('>h', dataStr[4] + dataStr[5])
            gyro_x_bias = value[0] * SCALE_GYRO_X * DEG_TO_RAD
            
            print 'gyro biases: (%f, %f, %f)' % (gyro_x_bias, gyro_y_bias, gyro_z_bias)
        elif command == GYRO_SCALE_REPORT:
            value = struct.unpack('>f', dataStr[0:4])
            gyro_z_scale = value[0]
            
            value = struct.unpack('>f', dataStr[4:8])
            gyro_y_scale = value[0]
            
            value = struct.unpack('>f', dataStr[8:12])
            gyro_x_scale = value[0]
            
            print 'gyro scales: (%f, %f, %f)' % (gyro_x_scale, gyro_y_scale, gyro_z_scale)
        elif command == START_CAL_REPORT:
            if data[0] & 0x01:
                print 'Gyro bias stratup calibration is ENABLED'
            else:
                print 'Gyro bias stratup calibration is DISABLED'
        elif command == ACCEL_BIAS_REPORT:
            value = struct.unpack('>h', dataStr[0] + dataStr[1])
            accel_z_bias = value[0] * SCALE_ACCEL_Z * MILIG_TO_MSS
            
            value = struct.unpack('>h', dataStr[2] + dataStr[3])
            accel_y_bias = value[0] * SCALE_ACCEL_Y * MILIG_TO_MSS
            
            value = struct.unpack('>h', dataStr[4] + dataStr[5])
            accel_x_bias = value[0] * SCALE_ACCEL_X * MILIG_TO_MSS
            
            print 'accel biases: (%f, %f, %f)' % (accel_x_bias, accel_y_bias, accel_z_bias)
        elif command == ACCEL_REF_VECTOR_REPORT:
            value = struct.unpack('>h', dataStr[0] + dataStr[1])
            accel_z_ref = value[0] * SCALE_ACCEL_Z * MILIG_TO_MSS
            
            value = struct.unpack('>h', dataStr[2] + dataStr[3])
            accel_y_ref = value[0] * SCALE_ACCEL_Y * MILIG_TO_MSS
            
            value = struct.unpack('>h', dataStr[4] + dataStr[5])
            accel_x_ref = value[0] * SCALE_ACCEL_X * MILIG_TO_MSS
            
            print 'accel ref vector: (%f, %f, %f)' % (accel_x_ref, accel_y_ref, accel_z_ref)
        elif command == ACTIVE_CHANNEL_REPORT:
            print "Active channel report received"
            active_channels = data[1] | (data[0] << 8)
            
            if (active_channels >> 15) & 0x01:
                print 'yaw channel active'
            if (active_channels >> 14) & 0x01:
                print 'pitch channel active'
            if (active_channels >> 13) & 0x01:
                print 'roll channel active'
            if (active_channels >> 12) & 0x01:
                print 'yaw rate channel active'
            if (active_channels >> 11) & 0x01:
                print 'pitch rate channel active'
            if (active_channels >> 10) & 0x01:
                print 'roll rate channel active'
            if (active_channels >> 9) & 0x01:
                print 'mx channel active'
            if (active_channels >> 8) & 0x01:
                print 'my channel active'
            if (active_channels >> 7) & 0x01:
                print 'mz channel active'
            if (active_channels >> 6) & 0x01:
                print 'gx channel active'
            if (active_channels >> 5) & 0x01:
                print 'gy channel active'
            if (active_channels >> 4) & 0x01:
                print 'gz rate channel active'
            if (active_channels >> 3) & 0x01:
                print 'ax rate channel active'
            if (active_channels >> 2) & 0x01:
                print 'ay rate channel active'
            if (active_channels >> 1) & 0x01:
                print 'az channel active'
        elif command == ACCEL_COVARIANCE_REPORT:
            value = struct.unpack('>f', dataStr[0:4])
            accel_covariance = value[0]
            print 'Accel covariance is %f' % accel_covariance
        elif command == MAG_COVARIANCE_REPORT:
            value = struct.unpack('>f', dataStr[0:4])
            mag_covariance = value[0]
            print 'Mag covariance is %f' % mag_covariance
        elif command == PROCESS_COVARIANCE_REPORT:
            value = struct.unpack('>f', dataStr[0:4])
            process_covariance = value[0]
            print 'Process covariance is %f' % process_covariance
        elif command == STATE_COVARIANCE_REPORT:
            value = struct.unpack('>f', dataStr[0:4])
            mat00 = value[0]
            value = struct.unpack('>f', dataStr[4:8])
            mat01 = value[0]
            value = struct.unpack('>f', dataStr[8:12])
            mat02 = value[0]
            value = struct.unpack('>f', dataStr[12:16])
            mat10 = value[0]
            value = struct.unpack('>f', dataStr[16:20])
            mat11 = value[0]
            value = struct.unpack('>f', dataStr[20:24])
            mat12 = value[0]
            value = struct.unpack('>f', dataStr[24:28])
            mat20 = value[0]
            value = struct.unpack('>f', dataStr[28:32])
            mat21 = value[0]
            value = struct.unpack('>f', dataStr[32:36])
            mat22 = value[0]
            print 'State covariance is [%f %f %f | %f %f %f | %f %f %f]' % (mat00, mat01, mat02, mat10, mat11, mat12, mat20, mat21, mat22)
        elif command == EKF_CONFIG_REPORT:
            if data[0] & 0x02:
                print 'Accelerometer updates ARE used for pitch and roll angle correction'
            else:
                print 'Accelerometer updates ARE NOT used for pitch and roll angle correction'
            
            if data[0] & 0x01:
                print 'Magnetometer updates ARE used for yaw angle correction'
            else:
                print 'Magnetometer updates ARE NOT used for yaw angle correction'
        elif command == GYRO_ALIGNMENT_REPORT:
            pass
        elif command == ACCEL_ALIGNMENT_REPORT:
            pass
        elif command == MAG_REF_VECTOR_REPORT:
            pass
        elif command == MAG_CAL_REPORT:
            pass
        elif command == MAG_BIAS_REPORT:
            pass
        elif command == BROADCAST_MODE_REPORT:
            pass
        else:
            print "Unsupported packet %d" % command

    def close(self):
        """
        Be nice, close the serial port.
        """
        if self.ser:
            self.ser.flushInput()
            self.ser.flushOutput()
            self.ser.close()

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
        print "Setting silent mode"
        self.write_to_imu(SET_SILENT_MODE)

    def set_broadcast_mode(self, hz):
        x = int((hz - 20) / (280.0/255.0))
        self.write_to_imu(SET_BROADCAST_MODE, (x, ))

    def auto_set_accel_ref(self):
        self.write_to_imu(AUTO_SET_ACCEL_REF)

    def get_data(self):
        self.write_to_imu(GET_DATA)

    def get_active_channels(self):
        self.write_to_imu(GET_ACTIVE_CHANNELS)
        
    def enable_accel_angrate_orientation(self):
        ch = {'yaw':1, 'pitch':1, 'roll':1,
              'yaw_rate':1, 'pitch_rate':1, 'roll_rate':1,
              'mx':0, 'my':0, 'mz':0,
              'gx':0, 'gy':0, 'gz':0,
              'ax':1, 'ay':1, 'az':1,}
              
        self.set_active_channels(ch)
        
    def read_accel_angrate_orientation(self):
        # look for packet prefix 'snp'
        while self.ser.read() != 's':
            print "looking for packet prefix 'snp'"
        packet = 's' + self.ser.read(2) # read 'n' and 'p'
        
        if packet != 'snp':
            print "Received corrupted packet, prefix was %s" % packet
            return

        command = ord(self.ser.read())
        print "command = %s" % hex(command).upper()

        n = ord(self.ser.read())
        print "data bytes = %d" % n

        dataStr = self.ser.read(n)
        data = map(b2a_hex, dataStr)
        data = map(int, data, [16] * len(data))
        print data

        chkSumStr = self.ser.read(2)
        chkSumData = map(b2a_hex, chkSumStr)
        chkSumData = map(int, chkSumData, [16] * len(chkSumData))
        chkSumRx = (chkSumData[0] << 8) | chkSumData[1]

        chkSum = self.calculate_checksum(command, data)
        
        if chkSumRx != chkSum:
            print "Checksums don't match %d != %d" % (chkSumRx, chkSum)
            return

        res = {}

        if command == SENSOR_DATA:
            active_channels = (data[0] << 8) | data[1]
            
            i = 2
            
            # yaw angle
            if active_channels & 0x8000:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                yaw = value[0] * SCALE_YAW * DEG_TO_RAD
                print 'yaw = %f' % yaw
                res['yaw'] = yaw
                i += 2
            
            # pitch angle
            if active_channels & 0x4000:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                pitch = value[0] * SCALE_PITCH * DEG_TO_RAD
                print 'pitch = %f' % pitch
                res['pitch'] = pitch
                i += 2
            
            # roll angle
            if active_channels & 0x2000:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                roll = value[0] * SCALE_ROLL * DEG_TO_RAD
                print 'roll = %f' % roll
                res['roll'] = roll
                i += 2
            
            # yaw rate
            if active_channels & 0x1000:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                yaw_rate = value[0] * SCALE_YAW_RATE * DEG_TO_RAD
                print 'yaw_rate = %f' % yaw_rate
                res['yaw_rate'] = yaw_rate
                i += 2
            
            # pitch rate
            if active_channels & 0x0800:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                pitch_rate = value[0] * SCALE_PITCH_RATE * DEG_TO_RAD
                print 'pitch_rate = %f' % pitch_rate
                res['pitch_rate'] = pitch_rate
                i += 2
            
            # roll rate
            if active_channels & 0x0400:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                roll_rate = value[0] * SCALE_ROLL_RATE * DEG_TO_RAD
                print 'roll_rate = %f' % roll_rate
                res['roll_rate'] = roll_rate
                i += 2

            # accel X
            if active_channels & 0x0008:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                ax = value[0] * SCALE_ACCEL_X * MILIG_TO_MSS
                print 'ax = %f' % ax
                res['accel_x'] = ax
                i += 2
            
            # accel Y
            if active_channels & 0x0004:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                ay = value[0] * SCALE_ACCEL_Y * MILIG_TO_MSS
                print 'ay = %f' % ay
                res['accel_y'] = ay
                i += 2
            
            # accel Z
            if active_channels & 0x0002:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                az = value[0] * SCALE_ACCEL_Z * MILIG_TO_MSS
                print 'az = %f' % az
                res['accel_z'] = az
                i += 2
                
        return res

if __name__ == "__main__":
    sio = CHR6dmIMU()
    sio.set_silent_mode()
    #sio.set_broadcast_mode(100)
    
    ch = {'yaw':1, 'pitch':1, 'roll':1,
          'yaw_rate':1, 'pitch_rate':1, 'roll_rate':1,
          'mx':1, 'my':1, 'mz':1,
          'gx':1, 'gy':1, 'gz':1,
          'ax':1, 'ay':1, 'az':1,}
    
    sio.set_active_channels(ch)
    #sio.get_active_channels()
    #while True:
    #    sio.get_data()
        #time.sleep(0.2)

