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
import struct
from math import sqrt
from binascii import b2a_hex

from chr_6dm_const import *

DEG_TO_RAD = 0.017453293    # degrees to radians
MILIG_TO_MSS = 0.00980665   # mili g to m/s^2

MAX_BYTES_SKIPPED = 1000

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
        
        # IMU state variables
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw_rate = 0.0
        self.pitch_rate = 0.0
        self.roll_rate = 0.0
        self.mag_x = 0.0
        self.mag_y = 0.0
        self.mag_z = 0.0
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        
        self.imu_data = {
            'timestamp':  time.time(),
            'yaw':        0.0,
            'pitch':      0.0,
            'roll':       0.0,
            'yaw_rate':   0.0,
            'pitch_rate': 0.0,
            'roll_rate':  0.0,
            'mag_x':      0.0,
            'mag_y':      0.0,
            'mag_z':      0.0,
            'gyro_x':     0.0,
            'gyro_y':     0.0,
            'gyro_z':     0.0,
            'accel_x':    0.0,
            'accel_y':    0.0,
            'accel_z':    0.0,
        }
        
        self.accel_covariance = 0.0
        self.mag_covariance = 0.0
        self.process_covariance = 0.0

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
        
        #print 'Command: %s (%s), data packet: %s' % (CODE_TO_STR[command], hex(command).upper(), str(data))
        
        # wait for response
        time.sleep(0.006)
        
        self.read_from_imu()

    def read_from_imu(self):
        # look for packet prefix 'snp'
        skipped_bytes = 0
        
        while skipped_bytes < MAX_BYTES_SKIPPED:
            if self.ser.read() != 's': skipped_bytes += 1
            else: break
        else:
            print 'Unable to find packet prefix. Throw exception.'
            return
        
        packet = 's' + self.ser.read(2) # read 'n' and 'p'
        
        if packet != 'snp':
            print "Received corrupted packet, prefix was %s" % packet
            return
            
        command = ord(self.ser.read())
        n = ord(self.ser.read())
        
        dataStr = self.ser.read(n)
        data = map(b2a_hex, dataStr)
        data = map(int, data, [16] * len(data))
        
        #print "Received reply: %s (%s), data: %s" % (CODE_TO_STR[command], hex(command).upper(), str(data))
        
        chkSumStr = self.ser.read(2)
        chkSumData = map(b2a_hex, chkSumStr)
        chkSumData = map(int, chkSumData, [16] * len(chkSumData))
        chkSumRx = (chkSumData[0] << 8) | chkSumData[1]
        
        chkSum = self.calculate_checksum(command, data)
        
        if chkSumRx != chkSum:
            print "Checksums don't match %d != %d" % (chkSumRx, chkSum)
            return
            
        if command == COMMAND_COMPLETE:
            print 'Command %s (%s) complete' % (CODE_TO_STR[data[0]], hex(data[0]).upper())
        elif command == COMMAND_FAILED:
            print 'Command %s (%s) failed' % (CODE_TO_STR[data[0]], hex(data[0]).upper())
        elif command == BAD_CHECKSUM:
            print 'Bad checksum'
        elif command == BAD_DATA_LENGTH:
            print 'Bad data length for command %s (%s)' % (CODE_TO_STR[data[0]], hex(data[0]).upper())
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
            self.imu_data['timestamp'] = time.time()
            active_channels = (data[0] << 8) | data[1]
            
            i = 2
            
            # yaw angle
            if active_channels & 0x8000:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                self.imu_data['yaw'] = value[0] * SCALE_YAW * DEG_TO_RAD
                #print 'yaw = %f' % self.imu_data['yaw']
                i += 2
                
            # pitch angle
            if active_channels & 0x4000:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                self.imu_data['pitch'] = value[0] * SCALE_PITCH * DEG_TO_RAD
                #print 'pitch = %f' % self.imu_data['pitch']
                i += 2
                
            # roll angle
            if active_channels & 0x2000:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                self.imu_data['roll'] = value[0] * SCALE_ROLL * DEG_TO_RAD
                #print 'roll = %f' % self.imu_data['roll']
                i += 2
                
            # yaw rate
            if active_channels & 0x1000:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                self.imu_data['yaw_rate'] = value[0] * SCALE_YAW_RATE * DEG_TO_RAD
                #print 'yaw_rate = %f' % self.imu_data['yaw_rate']
                i += 2
                
            # pitch rate
            if active_channels & 0x0800:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                self.imu_data['pitch_rate'] = value[0] * SCALE_PITCH_RATE * DEG_TO_RAD
                #print 'pitch_rate = %f' % self.imu_data['pitch_rate']
                i += 2
                
            # roll rate
            if active_channels & 0x0400:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                self.imu_data['roll_rate'] = value[0] * SCALE_ROLL_RATE * DEG_TO_RAD
                #print 'roll_rate = %f' % self.imu_data['roll_rate']
                i += 2
                
            # mag X
            if active_channels & 0x0200:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                self.imu_data['mag_x'] = value[0] * SCALE_MAG_X
                #print 'mx = %f' % self.imu_data['mag_x']
                i += 2
                
            # mag Y
            if active_channels & 0x0100:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                self.imu_data['mag_y'] = value[0] * SCALE_MAG_Y
                #print 'my = %f' % self.imu_data['mag_y']
                i += 2
                
            # mag Z
            if active_channels & 0x0080:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                self.imu_data['mag_z'] = value[0] * SCALE_MAG_Z
                #print 'mz = %f' % self.imu_data['mag_z']
                i += 2
                
            # gyro X
            if active_channels & 0x0040:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                self.imu_data['gyro_x'] = value[0] * SCALE_GYRO_X * DEG_TO_RAD
                #print 'gx = %f' % self.imu_data['gyro_x']
                i += 2
                
            # gyro Y
            if active_channels & 0x0020:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                self.imu_data['gyro_y'] = value[0] * SCALE_GYRO_Y * DEG_TO_RAD
                #print 'gy = %f' % self.imu_data['gyro_y']
                i += 2
                
            # gyro Z
            if active_channels & 0x0010:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                self.imu_data['gyro_z'] = value[0] * SCALE_GYRO_Z * DEG_TO_RAD
                #print 'gz = %f' % self.imu_data['gyro_z']
                i += 2
                
            # accel X
            if active_channels & 0x0008:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                self.imu_data['accel_x'] = value[0] * SCALE_ACCEL_X * MILIG_TO_MSS
                #print 'ax = %f' % self.imu_data['accel_x']
                i += 2
                
            # accel Y
            if active_channels & 0x0004:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                self.imu_data['accel_y'] = value[0] * SCALE_ACCEL_Y * MILIG_TO_MSS
                #print 'ay = %f' % self.imu_data['accel_y']
                i += 2
                
            # accel Z
            if active_channels & 0x0002:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                self.imu_data['accel_z'] = value[0] * SCALE_ACCEL_Z * MILIG_TO_MSS
                #print 'az = %f' % self.imu_data['accel_z']
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
            self.accel_covariance = pow(sqrt(value[0]) * MILIG_TO_MSS, 2.0)
            print 'Accel covariance is %f' % self.accel_covariance
        elif command == MAG_COVARIANCE_REPORT:
            value = struct.unpack('>f', dataStr[0:4])
            self.mag_covariance = value[0]
            print 'Mag covariance is %f' % self.mag_covariance
        elif command == PROCESS_COVARIANCE_REPORT:
            value = struct.unpack('>f', dataStr[0:4])
            self.process_covariance = value[0]
            print 'Process covariance is %f' % self.process_covariance
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
            print 'gyro alignment is [%f %f %f | %f %f %f | %f %f %f]' % (mat00, mat01, mat02, mat10, mat11, mat12, mat20, mat21, mat22)
        elif command == ACCEL_ALIGNMENT_REPORT:
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
            print 'accel alignment is [%f %f %f | %f %f %f | %f %f %f]' % (mat00, mat01, mat02, mat10, mat11, mat12, mat20, mat21, mat22)
        elif command == MAG_REF_VECTOR_REPORT:
            value = struct.unpack('>h', dataStr[0] + dataStr[1])
            mag_z_ref = value[0] * SCALE_MAG_Z
            
            value = struct.unpack('>h', dataStr[2] + dataStr[3])
            mag_y_ref = value[0] * SCALE_MAG_Y
            
            value = struct.unpack('>h', dataStr[4] + dataStr[5])
            mag_x_ref = value[0] * SCALE_MAG_X
            
            print 'mag ref vector: (%f, %f, %f)' % (mag_x_ref, mag_y_ref, mag_z_ref)
        elif command == MAG_CAL_REPORT:
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
            print 'mag calibration is [%f %f %f | %f %f %f | %f %f %f]' % (mat00, mat01, mat02, mat10, mat11, mat12, mat20, mat21, mat22)
        elif command == MAG_BIAS_REPORT:
            value = struct.unpack('>h', dataStr[0] + dataStr[1])
            mag_z_bias = value[0] * SCALE_MAG_Z
            
            value = struct.unpack('>h', dataStr[2] + dataStr[3])
            mag_y_bias = value[0] * SCALE_MAG_Y
            
            value = struct.unpack('>h', dataStr[4] + dataStr[5])
            mag_x_bias = value[0] * SCALE_MAG_X
            
            print 'mag bias vector: (%f, %f, %f)' % (mag_x_bias, mag_y_bias, mag_z_bias)
        elif command == BROADCAST_MODE_REPORT:
            if data[1] & 0x01:
                rate = (280.0 / 255.0) * data[0] + 20.0
                print "IMU is in BROADCAST mode, data rate is %d Hz" % rate
            else:
                print "IMU is in SILENT mode"
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
        Specifies which channel data should be transmitted over the UART in
        response to a GET_DATA packet, or periodically in Broadcast Mode. Any
        combination of sensor channels can be set as active or inactive.
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
        """
        Enables "Silent Mode." In Silent Mode, the AHRS only reports data when a
        GET_DATA packet is received.
        """
        self.write_to_imu(SET_SILENT_MODE)

    def set_broadcast_mode(self, hz):
        """
        Enables "Broadcast Mode." In Broadcast Mode, the AHRS automatically
        transmits sensor data at regular time intervals.
        """
        if hz < 20: hz = 20
        if hz > 300: hz = 300
        x = int((hz - 20) / (280.0/255.0))
        self.write_to_imu(SET_BROADCAST_MODE, (x, ))

    def set_gyro_bias(self, gyro_x_bias, gyro_y_bias, gyro_z_bias):
        """
        Manually sets the rate gyro bias on the X, Y, and Z rate gyros. The bias
        can be automatically set for all gyro axes by sending a ZERO_RATE_GYROS
        packet.
        """
        data = struct.pack('>hhh', gyro_z_bias, gyro_y_bias, gyro_x_bias)
        self.write_to_imu(SET_GYRO_BIAS, map(ord, data))

    def set_accel_bias(self, accel_x_bias, accel_y_bias, accel_z_bias):
        """
        Manually sets the accelerometer biases on the X, Y, and Z accelerometers.
        """
        data = struct.pack('>hhh', accel_z_bias, accel_y_bias, accel_x_bias)
        self.write_to_imu(SET_ACCEL_BIAS, map(ord, data))

    def set_accel_ref_vector(self, accel_x_ref, accel_y_ref, accel_z_ref):
        """
        Manually sets the accelerometer reference vector. The data in the
        SET_ACCEL_REF_VECTOR packet corresponds to the raw accelerometer
        measurements expected when the pitch and roll angles are zero.
        """
        data = struct.pack('>hhh', accel_z_ref, accel_y_ref, accel_x_ref)
        self.write_to_imu(SET_ACCEL_REF_VECTOR, map(ord, data))

    def auto_set_accel_ref(self):
        """
        Sets the accel reference vector to the most recent data acquired by the 
        accelerometers. Returns the new reference vector in a
        ACCEL_REF_VECTOR_REPORT packet.
        """
        self.write_to_imu(AUTO_SET_ACCEL_REF)

    def zero_rate_gyros(self):
        """
        Starts internal self-calibration of all three rate gyro axes. By
        default, rate gyros are zeroed on AHRS startup, but gyro startup
        calibration can be disabled (or re-enabled) by sending a SET_START_CAL
        packet.
        """
        self.write_to_imu(ZERO_RATE_GYROS)

    def self_test(self):
        """
        Instructs the AHRS to perform a self-test of the accelerometer and gyro
        sensor channels. The self-test sequence takes approximately 570
        milliseconds to complete. During this time, the AHRS should be kept
        stationary. A STATUS_REPORT packet is transmitted after the self-test is
        complete.
        """
        self.write_to_imu(SELF_TEST)

    def set_start_cal(self, enable_calibration):
        """
        Enables or disables automatic startup calibration of rate gyros.
        """
        data = 0x01 if enable_calibration else 0x00
        self.write_to_imu(SET_START_CAL, (data,))

    def set_process_covariance(self, process_covariance):
        """
        Sets the process covariance to be used in the prediction step of the
        EKF. The unit assumes that the process covariance will be a diagonal
        matrix with equivalent diagonal entries. The SET_PROCESS_COVARIANCE
        packet thus includes only one 32-bit floating point value for the
        covariance matrix.
        """
        data = struct.pack('>f', process_covariance)
        self.write_to_imu(SET_PROCESS_COVARIANCE, map(ord, data))

    def set_mag_covariance(self, mag_covariance):
        """
        Sets the covariance to be used in the magnetometer update step of the
        EKF. The unit assumes that the magnetometer covariance will be a 
        diagonal matrix with equivalent diagonal entries. The SET_MAG_COVARIANCE
        packet thus includes only one 32-bit floating point value for the
        covariance matrix.
        """
        data = struct.pack('>f', mag_covariance)
        self.write_to_imu(SET_MAG_COVARIANCE, map(ord, data))

    def set_accel_covariance(self, accel_covariance):
        """
        Sets the covariance to be used in the accelerometer update step of the
        EKF. The unit assumes that the accelerometer covariance will be a 
        diagonal matrix with equivalent diagonal entries. The 
        SET_ACCEL_COVARIANCE packet thus includes only one 32-bit floating point
        value for the covariance matrix.
        """
        data = struct.pack('>f', accel_covariance)
        self.write_to_imu(SET_ACCEL_COVARIANCE, map(ord, data))

    def set_ekf_config(self, enable_accel, enable_mag):
        """
        Sets the EKF configuration register. This packet is used to enable/
        disable accelerometer and magnetometer updates to the angle estimates.
        """
        data = 0
        if enable_accel: data |= 0x02
        if enable_mag: data |= 0x01
        self.write_to_imu(SET_EKF_CONFIG, (data,))

    def set_gyro_alignment(self, gyro_alignment):
        """
        Sets the 3x3 calibration matrix used to correct cross-axis misalignment
        of the rate gyro outputs. Each element in the matrix is a 32-bit IEEE
        floating point value.
        
        [mat00, mat01, mat02, mat10, mat11, mat12, mat20, mat21, mat22]
        """
        data = struct.pack('>' + 9*'f', gyro_alignment)
        self.write_to_imu(SET_GYRO_ALIGNMENT, map(ord, data))

    def set_accel_alignment(self, accel_alignment):
        """
        Sets the 3x3 calibration matrix used to correct cross-axis misalignment
        of the acclerometer outputs. Each element in the matrix is a 32-bit IEEE
        floating point value.
        
        [mat00, mat01, mat02, mat10, mat11, mat12, mat20, mat21, mat22]
        """
        data = struct.pack('>' + 9*'f', accel_alignment)
        self.write_to_imu(SET_ACCEL_ALIGNMENT, map(ord, data))

    def set_mag_ref_vector(self, mag_x_ref, mag_y_ref, mag_z_ref):
        """
        Manually sets the magnetometer reference vector. The data in the
        SET_MAG_REF_VECTOR packet corresponds to the raw magnetometer
        measurements expected when the pitch, roll, and yaw angles are zero.
        """
        data = struct.pack('>hhh', mag_z_ref, mag_y_ref, mag_x_ref)
        self.write_to_imu(SET_MAG_REF_VECTOR, map(ord, data))

    def auto_set_mag_ref(self):
        """
        Sets the magnetic field reference vector to the most recent magnetic
        sensor measurement. Returns a MAG_REF_VECTOR_REPORT PACKET with the new
        reference vector.
        """
        self.write_to_imu(AUTO_SET_MAG_REF)

    def set_mag_cal(self, mag_cal):
        """
        Sets the 3x3 calibration matrix used for soft and hard iron calibration,
        axis misalignment calibration, and scale calibration of the magnetometer.
        
        [mat00, mat01, mat02, mat10, mat11, mat12, mat20, mat21, mat22]
        """
        data = struct.pack('>' + 9*'f', mag_cal)
        self.write_to_imu(SET_MAG_CAL, map(ord, data))

    def set_mag_bias(self, mag_x_bias, mag_y_bias, mag_z_bias):
        """
        Sets the magnetic field bias term to compensate for hard-iron
        distortions of the magnetic field.
        """
        data = struct.pack('>hhh', mag_z_bias, mag_y_bias, mag_x_bias)
        self.write_to_imu(SET_MAG_BIAS, map(ord, data))

    def set_gyro_scale(self, gyro_x_scale, gyro_y_scale, gyro_z_scale):
        """
        Sets the scale factors used to compute rates from raw gyro data on all
        axes.
        """
        data = struct.pack('>fff', gyro_z_scale, gyro_y_scale, gyro_x_scale)
        self.write_to_imu(SET_GYRO_SCALE, map(ord, data))

    def ekf_reset(self):
        """
        Sets each term in the state covariance matrix to zero and re-initializes
        the EKF. This command can be used for recovery if the state is corrupted
        by passing too close to the singularity at pitch = 90 degrees.
        """
        self.write_to_imu(EKF_RESET)

    def reset_to_factory(self):
        """
        Resets all AHRS configuration to the factory defaults. This includes
        calibration parameters, biases, communication settings, etc. The factory
        defaults are written to RAM. To make them persistent, a WRITE_TO_FLASH
        packet must be sent following the RESET_TO_FACTORY command.
        """
        self.write_to_imu(RESET_TO_FACTORY)

    def write_to_flash(self):
        """
        Writes AHRS configuration to on-board flash so that the configuration
        persists when the power is cycled.
        """
        self.write_to_imu(WRITE_TO_FLASH)

    def get_data(self):
        """
        In Silent Mode, the AHRS waits to receive a GET_DATA packet before
        transmitting sensor data. The most recent data from all active sensor
        channels is transmitted in response to a GET_DATA packet. In Broadcast
        Mode, a GET_DATA packet is ignored.
        """
        self.write_to_imu(GET_DATA)
        return self.imu_data

    def get_active_channels(self):
        """
        Reports which channels are "active." Active channels are sensor channels
        that are measured and transmitted in response to a GET_DATA packet, or
        periodically in Broadcast Mode. Active channels are reported in an
        ACTIVE_CHANNEL_REPORT packet.
        """
        self.write_to_imu(GET_ACTIVE_CHANNELS)

    def get_broadcast_mode(self):
        """
        Causes the AHRS to send a BROADCAST_MODE_REPORT packet, which specifies
        whether the AHRS is in Broadcast Mode or Silent Mode.
        """
        self.write_to_imu(GET_BROADCAST_MODE)

    def get_accel_bias(self):
        """
        Return the bias values for all three accel axes in an ACCEL_BIAS_REPORT
        packet.
        """
        self.write_to_imu(GET_ACCEL_BIAS)

    def get_accel_ref_vector(self):
        """
        Return the accelerometer reference vector in a ACCEL_REF_VECTOR_REPORT.
        """
        self.write_to_imu(GET_ACCEL_REF_VECTOR)

    def get_gyro_bias(self):
        """
        Returns the bias values for all three rate gyros in a GYRO_BIAS_REPORT
        packet.
        """
        self.write_to_imu(GET_GYRO_BIAS)

    def get_gyro_scale(self):
        """
        Returns the scale factors used to convert raw gyro measurements to
        angular rates. Data is returned in a GYRO_SCALE_REPORT packet.
        """
        self.write_to_imu(GET_GYRO_SCALE)

    def get_start_cal(self):
        """
        Reports whether gyro startup calibration is enabled by sending a
        START_CAL_REPORT packet.
        """
        self.write_to_imu(GET_START_CAL)

    def get_ekf_config(self):
        """
        Returns the value of the EKF configuration register in a
        EKF_CONFIG_REPORT packet.
        """
        self.write_to_imu(GET_EKF_CONFIG)

    def get_accel_covariance(self):
        """
        Returns the covariance to be used in the accelerometer update step of
        the EKF. Covariance is transmitted in a ACCEL_COVARIANCE_REPORT packet.
        """
        self.write_to_imu(GET_ACCEL_COVARIANCE)
        return self.accel_covariance

    def get_mag_covariance(self):
        """
        Returns the covariance to be used in the magnetometer update step of the
        EKF. Covariance is transmitted in a MAG_COVARIANCE_REPORT packet.
        """
        self.write_to_imu(GET_MAG_COVARIANCE)
        return self.mag_covariance

    def get_process_covariance(self):
        """
        Returns the covariance to be used in the prediction step of the EKF.
        Covariance is transmitted in a PROCESS_COVARIANCE_REPORT packet.
        """
        self.write_to_imu(GET_PROCESS_COVARIANCE)
        return self.process_covariance

    def get_state_covariance(self):
        """
        Returns the 3x3 covariance matrix of the current state estimates in the
        EKF. The covariance matrix is sent in a STATE_COVARIANCE_REPORT packet.
        """
        self.write_to_imu(GET_STATE_COVARIANCE)

    def get_gyro_alignment(self):
        """
        Returns the 3x3 matrix used to correct gyro cross-axis misalignment. The
        alignment matrix is returned in a GYRO_ALIGNMENT_REPORT packet.
        """
        self.write_to_imu(GET_GYRO_ALIGNMENT)

    def get_accel_alignment(self):
        """
        Returns the 3x3 matrix used to correct accelerometer cross-axis
        misalignment. The alignment matrix is returned in an
        ACCEL_ALIGNMENT_REPORT packet.
        """
        self.write_to_imu(GET_ACCEL_ALIGNMENT)

    def get_mag_ref_vector(self):
        """
        Returns the magnetic field reference vector in a MAG_REF_VECTOR_REPORT
        packet.
        """
        self.write_to_imu(GET_MAG_REF_VECTOR)

    def get_mag_cal(self):
        """
        Returns the 3x3 matrix used to correct magnetometer soft iron
        distortions, axis misalignment, and scale factors. The calibration
        matrix is returned in a MAG_CAL_REPORT packet.
        """
        self.write_to_imu(GET_MAG_CAL)

    def get_mag_bias(self):
        """
        Returns the magnetometer biases used to correct hard iron distortions.
        Biases are returned in a MAG_BIAS_REPORT packet.
        """
        self.write_to_imu(GET_MAG_BIAS)



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
        #print "packet type = %s" % hex(command).upper()

        n = ord(self.ser.read())
        #print "data bytes = %d" % n

        dataStr = self.ser.read(n)
        data = map(b2a_hex, dataStr)
        data = map(int, data, [16] * len(data))
        #print data

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
            res['timestamp'] = time.time()
            active_channels = (data[0] << 8) | data[1]
            
            i = 2
            
            # yaw angle
            if active_channels & 0x8000:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                yaw = value[0] * SCALE_YAW * DEG_TO_RAD
                #print 'yaw = %f' % yaw
                res['yaw'] = yaw
                i += 2
            
            # pitch angle
            if active_channels & 0x4000:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                pitch = value[0] * SCALE_PITCH * DEG_TO_RAD
                #print 'pitch = %f' % pitch
                res['pitch'] = pitch
                i += 2
            
            # roll angle
            if active_channels & 0x2000:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                roll = value[0] * SCALE_ROLL * DEG_TO_RAD
                #print 'roll = %f' % roll
                res['roll'] = roll
                i += 2
            
            # yaw rate
            if active_channels & 0x1000:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                yaw_rate = value[0] * SCALE_YAW_RATE * DEG_TO_RAD
                #print 'yaw_rate = %f' % yaw_rate
                res['yaw_rate'] = yaw_rate
                i += 2
            
            # pitch rate
            if active_channels & 0x0800:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                pitch_rate = value[0] * SCALE_PITCH_RATE * DEG_TO_RAD
                #print 'pitch_rate = %f' % pitch_rate
                res['pitch_rate'] = pitch_rate
                i += 2
            
            # roll rate
            if active_channels & 0x0400:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                roll_rate = value[0] * SCALE_ROLL_RATE * DEG_TO_RAD
                #print 'roll_rate = %f' % roll_rate
                res['roll_rate'] = roll_rate
                i += 2

            # accel X
            if active_channels & 0x0008:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                ax = value[0] * SCALE_ACCEL_X * MILIG_TO_MSS
                #print 'ax = %f' % ax
                res['accel_x'] = ax
                i += 2
            
            # accel Y
            if active_channels & 0x0004:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                ay = value[0] * SCALE_ACCEL_Y * MILIG_TO_MSS
                #print 'ay = %f' % ay
                res['accel_y'] = ay
                i += 2
            
            # accel Z
            if active_channels & 0x0002:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                az = value[0] * SCALE_ACCEL_Z * MILIG_TO_MSS
                #print 'az = %f' % az
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

    sio.get_accel_alignment()
    sio.get_accel_bias()
    sio.get_accel_covariance()
    sio.get_accel_ref_vector()
    sio.get_active_channels()
    
    sio.get_broadcast_mode()
    sio.get_ekf_config()
    sio.get_gyro_alignment()
    sio.get_gyro_bias()
    sio.get_gyro_scale()
    
    sio.get_mag_bias()
    sio.get_mag_cal()
    sio.get_mag_covariance()
    sio.get_mag_ref_vector()
    
    sio.get_process_covariance()
    sio.get_start_cal()
    sio.get_state_covariance()
    
    sio.self_test()
    
    #sio.get_active_channels()
    #while True:
    #    sio.get_data()
        #time.sleep(0.2)

