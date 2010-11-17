#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Arizona Robotics Research Group,
# University of Arizona. All rights reserved.
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
# Author: Cody Jorgensen
# Author: Antons Rebguns
#

import time
import serial
from array import array
from binascii import b2a_hex

from ax12_const import *
from ax12_user_commands import *

exception = None

class SerialIO(object):
    """ Provides low level IO with the AX-12+ servos through pyserial. Has the
    ability to write instruction packets, request and read register value
    packets, send and receive a response to a ping packet, and send a SYNC WRITE
    multi-servo instruction packet.
    """

    def __init__(self, port, baudrate):
        """ Constructor takes serial port and baudrate as arguments. """
        try:
            self.ser = None
            self.ser = serial.Serial(port)
            self.ser.setTimeout(0.015)
            self.ser.baudrate = baudrate
        except:
           raise(SerialOpenError(port, baudrate))

    def __del__(self):
        """ Destructor calls self.close_serial_port() """
        self.close_serial_port()

    def __read_response(self, servoId):
        data = []
        try:
            data.extend(self.ser.read(4))
            if not data[0:2] == ['\xff', '\xff']: raise Exception('Wrong packet prefix %s' % data[0:2])
            data.extend(self.ser.read(ord(data[3])))
            data = array('B', ''.join(data)).tolist() # [int(b2a_hex(byte), 16) for byte in data]
        except Exception, e:
            raise DroppedPacketError('Invalid response received from motor %d. %s' % (servoId, e))

        # verify checksum
        checksum = 255 - sum(data[2:-1]) % 256
        if not checksum == data[-1]: raise ChecksumError(data, checksum)

        return data

    def read_from_servo(self, servoId, address, size):
        """ Read "size" bytes of data from servo with "servoId" starting at the
        register with "address". "address" is an integer between 0 and 49. It is
        recommended to use the constants in module ax12_const for readability.

        To read the position from servo with id 1, the method should be called
        like:
            read_from_servo(1, AX_GOAL_POSITION_L, 2)
        """
        self.ser.flushInput()

        # Number of bytes following standard header (0xFF, 0xFF, id, length)
        length = 4  # instruction, address, size, checksum

        # directly from AX-12 manual:
        # Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
        # If the calculated value is > 255, the lower byte is the check sum.
        checksum = 255 - ( (servoId + length + AX_READ_DATA + address + size) % 256 )

        # packet: FF  FF  ID LENGTH INSTRUCTION PARAM_1 ... CHECKSUM
        packet = [0xFF, 0xFF, servoId, length, AX_READ_DATA, address, size, checksum]
        packetStr = array('B', packet).tostring() # same as: packetStr = ''.join([chr(byte) for byte in packet])
        self.ser.write(packetStr)

        # wait for response packet from the motor
        timestamp = time.time()
        time.sleep(0.0013)#0.00235)

        # read response
        data = self.__read_response(servoId)
        data.append(timestamp)

        return data

    def write_to_servo(self, servoId, address, data):
        """ Write the values from the "data" list to the servo with "servoId"
        starting with data[0] at "address", continuing through data[n-1] at
        "address" + (n-1), where n = len(data). "address" is an integer between
        0 and 49. It is recommended to use the constants in module ax12_const
        for readability. "data" is a list/tuple of integers.

        To set servo with id 1 to position 276, the method should be called
        like:
            read_from_servo(1, AX_GOAL_POSITION_L, (20, 1))
        """
        self.ser.flushInput()

        # Number of bytes following standard header (0xFF, 0xFF, id, length)
        length = 3 + len(data)  # instruction, address, len(data), checksum

        # directly from AX-12 manual:
        # Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
        # If the calculated value is > 255, the lower byte is the check sum.
        checksum = 255 - ((servoId + length + AX_WRITE_DATA + address + sum(data)) % 256)

        # packet: FF  FF  ID LENGTH INSTRUCTION PARAM_1 ... CHECKSUM
        packet = [0xFF, 0xFF, servoId, length, AX_WRITE_DATA, address]
        packet.extend(data)
        packet.append(checksum)

        packetStr = array('B', packet).tostring() # packetStr = ''.join([chr(byte) for byte in packet])
        self.ser.write(packetStr)

        # wait for response packet from the motor
        timestamp = time.time()
        time.sleep(0.0013)

        # read response
        data = self.__read_response(servoId)
        data.append(timestamp)

        return data

    def ping_servo(self, servoId):
        """ Ping the servo with "servoId". This causes the servo to return a
        "status packet". This can tell us if the servo is attached and powered,
        and if so, if there is any errors.

        To ping the servo with id 1 to position 276, the method should be called
        like:
            ping_servo(1)
        """
        self.ser.flushInput()

        # Number of bytes following standard header (0xFF, 0xFF, id, length)
        length = 2  # instruction, checksum

        # directly from AX-12 manual:
        # Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
        # If the calculated value is > 255, the lower byte is the check sum.
        checksum = 255 - ((servoId + length + AX_PING) % 256)

        # packet: FF  FF  ID LENGTH INSTRUCTION CHECKSUM
        packet = [0xFF, 0xFF, servoId, length, AX_PING, checksum]
        packetStr = array('B', packet).tostring()
        self.ser.write(packetStr)

        # wait for response packet from the motor
        timestamp = time.time()
        time.sleep(0.0013)

        # read response
        try:
            data = self.__read_response(servoId)
            data.append(timestamp)
        except Exception, e:
            data = []

        return data

    def sync_write_to_servos(self, address, data):
        """ Use Broadcast message to send multiple servos instructions at the
        same time. No "status packet" will be returned from any servos.
        "address" is an integer between 0 and 49. It is recommended to use the
        constants in module ax12_const for readability. "data" is a tuple of
        tuples. Each tuple in "data" must contain the servo id followed by the
        data that should be written from the starting address. The amount of
        data can be as long as needed.

        To set servo with id 1 to position 276 and servo with id 2 to position
        550, the method should be called like:
            sync_write_to_servos(AX_GOAL_POSITION_L, ( (1, 20, 1), (2 ,38, 2) ))
        """
        self.ser.flushInput()

        # Calculate length and sum of all data
        flattened = [value for servo in data for value in servo]

        # Number of bytes following standard header (0xFF, 0xFF, id, length) plus data
        length = 4 + len(flattened)

        checksum = 255 - ((AX_BROADCAST + length + \
                          AX_SYNC_WRITE + address + len(data[0][1:]) + \
                          sum(flattened)) % 256)

        # packet: FF  FF  ID LENGTH INSTRUCTION PARAM_1 ... CHECKSUM
        packet = [0xFF, 0xFF, AX_BROADCAST, length, AX_SYNC_WRITE, address, len(data[0][1:])]
        packet.extend(flattened)
        packet.append(checksum)

        packetStr = array('B', packet).tostring() # packetStr = ''.join([chr(byte) for byte in packet])
        self.ser.write(packetStr)

    def close_serial_port(self):
        """
        Be nice, close the serial port.
        """
        if self.ser:
            self.ser.flushInput()
            self.ser.flushOutput()
            self.ser.close()

class AX12_IO(object):
    def __init__(self, port, baudrate):
        try:
            self.port = port
            self.baud = baudrate
            self.__sio = SerialIO(port, baudrate)
        except SerialOpenError, e:
            raise(e)

        # Avoid writing facade code
        self.close_serial_port = self.__sio.close_serial_port
        self.read_from_servo = self.__sio.read_from_servo
        self.write_to_servo = self.__sio.write_to_servo

    def test_bit(self, number, offset):
        mask = 1 << offset
        return (number & mask)

    def ping(self, servoId):
        response = self.__sio.ping_servo(servoId)
        if response:
            self.exception_on_error(response[4], servoId, 'ping')
        return response

    def write_packet(self, packet):
        command = packet[0]

        register = 0
        values = []
        two_byte_cmd = False

        if command == DMXL_SET_TORQUE_ENABLE:
            register = AX_TORQUE_ENABLE
        elif command == DMXL_SET_CW_COMPLIANCE_MARGIN:
            register = AX_CW_COMPLIANCE_MARGIN
        elif command == DMXL_SET_CCW_COMPLIANCE_MARGIN:
            register = AX_CCW_COMPLIANCE_MARGIN
        elif command == DMXL_SET_COMPLIANCE_MARGINS:
            two_byte_cmd = True
            register = AX_CW_COMPLIANCE_MARGIN
        elif command == DMXL_SET_CW_COMPLIANCE_SLOPE:
            register = AX_CW_COMPLIANCE_SLOPE
        elif command == DMXL_SET_CCW_COMPLIANCE_SLOPE:
            register = AX_CCW_COMPLIANCE_SLOPE
        elif command == DMXL_SET_COMPLIANCE_SLOPES:
            two_byte_cmd = True
            register = AX_CW_COMPLIANCE_SLOPE
        elif command == DMXL_SET_GOAL_POSITION:
            two_byte_cmd = True
            register = AX_GOAL_POSITION_L
        elif command == DMXL_SET_GOAL_SPEED:
            two_byte_cmd = True
            register = AX_GOAL_SPEED_L
        elif command == DMXL_SET_TORQUE_LIMIT:
            two_byte_cmd = True
            register = AX_TORQUE_LIMIT_L
        elif command == DMXL_SET_PUNCH:
            two_byte_cmd = True
            register = AX_PUNCH_L

        for val in packet[1]:
            motor_id = val[0]
            value = val[1]
            if two_byte_cmd:
                if value >= 0:
                    loByte = int(value % 256)
                    hiByte = int(value >> 8)
                else:
                    loByte = int((1023 - value) % 256)
                    hiByte = int((1023 - value) >> 8)
                values.append((motor_id, loByte, hiByte))
            else:
                values.append((motor_id, value))
        self.__sio.sync_write_to_servos(register, tuple(values))

    ######################################################################
    # These function modify EEPROM data which persists after power cycle #
    ######################################################################

    def set_servo_id(self, old_id, new_id):
        """
        Sets a new unique number to identify a motor. The range from 1 to 253
        (0xFD) can be used.
        """
        response = self.__sio.write_to_servo(old_id, AX_ID, [new_id])
        if response:
            self.exception_on_error(response[4], old_id, 'setting id to %d' % new_id)
        return response

    def set_servo_baud_rate(self, servoId, baud_rate):
        """
        Sets servo communication speed. The range from 0 to 254.
        """
        response = self.__sio.write_to_servo(servoId, AX_BAUD_RATE, [baud_rate])
        if response:
            self.exception_on_error(response[4], servoId, 'setting baud rate to %d' % baud_rate)
        return response

    def set_servo_return_delay_time(self, servoId, delay):
        """
        Sets the delay time from the transmission of Instruction Packet until
        the return of Status Packet. 0 to 254 (0xFE) can be used, and the delay
        time per data value is 2 usec.
        """
        response = self.__sio.write_to_servo(servoId, AX_RETURN_DELAY_TIME, [delay])
        if response:
            self.exception_on_error(response[4], servoId, 'setting return delay time to %d' % delay)
        return response

    def set_servo_angle_limit_cw(self, servoId, angle_cw):
        """
        Set the min (CW) angle of rotation limit.
        """
        loVal = int(angle_cw % 256)
        hiVal = int(angle_cw >> 8)
        # set 4 register values with low and high bytes for min and max angles
        response = self.__sio.write_to_servo(servoId, AX_CW_ANGLE_LIMIT_L, (loVal, hiVal))
        if response:
            self.exception_on_error(response[4], servoId, 'setting CW angle limits to %d' % angle_cw)
        return response

    def set_servo_angle_limit_ccw(self, servoId, angle_ccw):
        """
        Set the max (CCW) angle of rotation limit.
        """
        loVal = int(angle_ccw % 256)
        hiVal = int(angle_ccw >> 8)
        # set 4 register values with low and high bytes for min and max angles
        response = self.__sio.write_to_servo(servoId, AX_CCW_ANGLE_LIMIT_L, (loVal, hiVal))
        if response:
            self.exception_on_error(response[4], servoId, 'setting CCW angle limits to %d' % angle_ccw)
        return response

    def set_min_max_angle_limits(self, servoId, minAngle, maxAngle):
        """
        Set the min (CW) and max (CCW) angle of rotation limits.
        """
        loMinVal = int(minAngle % 256)
        hiMinVal = int(minAngle >> 8)
        loMaxVal = int(maxAngle % 256)
        hiMaxVal = int(maxAngle >> 8)
        # set 4 register values with low and high bytes for min and max angles
        response = self.__sio.write_to_servo(servoId, AX_CW_ANGLE_LIMIT_L,
                                      (loMinVal, hiMinVal, loMaxVal, hiMaxVal))
        if response:
            self.exception_on_error(response[4], servoId, 'setting CW and CCW angle limits to %d and %d' %(minAngle, maxAngle))
        return response

    def set_min_max_voltage_limits(self, servoId, minVoltage, maxVoltage):
        """
        Set the min and max voltage limits.
        NOTE: the absolute min is 5v and the absolute max is 25v
        """

        if minVoltage < 5: minVoltage = 5
        if maxVoltage > 25: maxVoltage = 25

        minVal = int(minVoltage * 10)
        maxVal = int(maxVoltage * 10)

        # set 4 register values with low and high bytes for min and max angles
        response = self.__sio.write_to_servo(servoId, AX_DOWN_LIMIT_VOLTAGE, (minVal, maxVal))

        if response:
            self.exception_on_error(response[4], servoId, 'setting min and max voltage levels to %d and %d' %(minVoltage, maxVoltage))
        return response

    ###############################################################
    # These functions can send a single command to a single servo #
    ###############################################################

    def set_torque_enabled(self, servoId, enabled):
        """
        Sets the value of the torque enabled register to 1 or 0. When the
        torque is disabled the servo can be moved manually while the motor is
        still powered.
        """
        response = self.__sio.write_to_servo(servoId, AX_TORQUE_ENABLE, [enabled])
        if response:
            self.exception_on_error(response[4], servoId, '%sabling torque' % 'en' if enabled else 'dis')
        return response

    def set_servo_compliance_margin_cw(self, servoId, margin):
        """
        The error between goal position and present position in CW direction.
        The range of the value is 0 to 255, and the unit is the same as Goal Position.
        The greater the value, the more difference occurs.
        """
        response = self.__sio.write_to_servo(servoId, AX_CW_COMPLIANCE_MARGIN, [margin])
        if response:
            self.exception_on_error(response[4], servoId, 'setting CW compliance margin to %d' % margin)
        return response

    def set_servo_compliance_margin_ccw(self, servoId, margin):
        """
        The error between goal position and present position in CCW direction.
        The range of the value is 0 to 255, and the unit is the same as Goal Position.
        The greater the value, the more difference occurs.
        """
        response = self.__sio.write_to_servo(servoId, AX_CCW_COMPLIANCE_MARGIN, [margin])
        if response:
            self.exception_on_error(response[4], servoId, 'setting CCW compliance margin to %d' % margin)
        return response

    def set_servo_compliance_margins(self, servoId, margin_cw, margin_ccw):
        """
        The error between goal position and present position in CCW direction.
        The range of the value is 0 to 255, and the unit is the same as Goal Position.
        The greater the value, the more difference occurs.
        """
        response = self.__sio.write_to_servo(servoId, AX_CW_COMPLIANCE_MARGIN, (margin_cw, margin_ccw))
        if response:
            self.exception_on_error(response[4], servoId, 'setting CW and CCW compliance margins to values %d and %d' %(margin_cw, margin_ccw))
        return response

    def set_servo_compliance_slope_cw(self, servoId, slope):
        """
        Sets the level of Torque near the goal position in CW direction.
        Compliance Slope is set in 7 steps, the higher the value, the more flexibility is obtained.
        """
        response = self.__sio.write_to_servo(servoId, AX_CW_COMPLIANCE_SLOPE, [slope])
        if response:
            self.exception_on_error(response[4], servoId, 'setting CW compliance slope to %d' %  slope)
        return response

    def set_servo_compliance_slope_ccw(self, servoId, slope):
        """
        Sets the level of Torque near the goal position in CCW direction.
        Compliance Slope is set in 7 steps, the higher the value, the more flexibility is obtained.
        """
        response = self.__sio.write_to_servo(servoId, AX_CCW_COMPLIANCE_SLOPE, [slope])
        if response:
            self.exception_on_error(response[4], servoId, 'setting CCW compliance slope to %d' % slope)
        return response

    def set_servo_compliance_slopes(self, servoId, slope_cw, slope_ccw):
        """
        Sets the level of Torque near the goal position in CW/CCW direction.
        Compliance Slope is set in 7 steps, the higher the value, the more flexibility is obtained.
        """
        response = self.__sio.write_to_servo(servoId, AX_CW_COMPLIANCE_SLOPE, (slope_cw, slope_ccw))
        if response:
            self.exception_on_error(response[4], servoId, 'setting CW and CCW compliance slopes to %d and %d' %(slope_cw, slope_ccw))
        return response

    def set_servo_punch(self, servoId, punch):
        """
        Sets the limit value of torque being reduced when the output torque is
        decreased in the Compliance Slope area. In other words, it is the mimimum
        torque. The initial value is 32 (0x20) and can be extended up to 1023
        (0x3FF). (Refer to Compliance margin & Slope)
        """
        loVal = int(punch % 256)
        hiVal = int(punch >> 8)
        response = self.__sio.write_to_servo(servoId, AX_PUNCH_L, (loVal, hiVal))
        if response:
            self.exception_on_error(response[4], servoId, 'setting punch to %d' % punch)
        return response

    def set_servo_position(self, servoId, position):
        """
        Set the servo with servoId to the specified goal position.
        Position value must be positive.
        """
        # split position into 2 bytes
        loVal = int(position % 256)
        hiVal = int(position >> 8)
        # set two register values with low and high byte for the position
        response = self.__sio.write_to_servo(servoId, AX_GOAL_POSITION_L,
                                             (loVal, hiVal))
        if response:
            self.exception_on_error(response[4], servoId, 'setting goal position to %d' % position)
        return response

    def set_servo_speed(self, servoId, speed):
        """
        Set the servo with servoId to the specified goal speed.
        Speed can be negative only if the dynamixel is in "freespin" mode.
        """
        # split speed into 2 bytes
        if speed >= 0:
            loVal = int(speed % 256)
            hiVal = int(speed >> 8)
        else:
            loVal = int((1023 - speed) % 256)
            hiVal = int((1023 - speed) >> 8)
        # set two register values with low and high byte for the speed
        response = self.__sio.write_to_servo(servoId, AX_GOAL_SPEED_L,
                                             (loVal, hiVal))
        if response:
            self.exception_on_error(response[4], servoId, 'setting moving speed to %d' % speed)
        return response

    def set_servo_torque_limit(self, servoId, torque):
        """
        Sets the value of the maximum torque limit for servo with id servoID.
        Valid values are 0 to 1023 (0x3FF), and the unit is about 0.1%.
        For example, if the value is 512 only 50% of the maximum torque will be used.
        If the power is turned on, the value of Max Torque (Address 14, 15) is used as the initial value.
        """
        loVal = int(torque % 256)
        hiVal = int(torque >> 8)
        response = self.__sio.write_to_servo(servoId, AX_TORQUE_LIMIT_L, (loVal, hiVal))
        if response:
            self.exception_on_error(response[4], servoId, 'setting torque limit to %d' % torque)
        return response

    #################################################################
    # These functions can send multiple commands to multiple servos #
    #################################################################

    def set_servo_position_and_speed(self, servoId, position, speed):
        """
        Set the servo with servoId to specified position and speed.
        Speed can be negative only if the dynamixel is in "freespin" mode.
        """
        # split speed into 2 bytes
        if speed >= 0:
            loSpeedVal = int(speed % 256)
            hiSpeedVal = int(speed >> 8)
        else:
            loSpeedVal = int((1023 - speed) % 256)
            hiSpeedVal = int((1023 - speed) >> 8)
        # split position into 2 bytes
        loPositionVal = int(position % 256)
        hiPositionVal = int(position >> 8)
        response = self.__sio.write_to_servo(servoId, AX_GOAL_POSITION_L,
                                             (loPositionVal, hiPositionVal,
                                              loSpeedVal, hiSpeedVal))
        if response:
            self.exception_on_error(response[4], servoId, 'setting goal position to %d and moving speed to %d' %(position, speed))
        return response

    def set_multi_servo_speeds(self, valueTuples):
        """
        Set different speeds for multiple servos.
        Should be called as such:
        set_multi_servo_speeds( ( (id1, speed1), (id2, speed2), (id3, speed3) ) )
        """
        # prepare value tuples for call to syncwrite
        writeableVals = []
        for vals in valueTuples:
            sid = vals[0]
            speed = vals[1]
            # split speed into 2 bytes
            if speed >= 0:
                loVal = int(speed % 256)
                hiVal = int(speed >> 8)
            else:
                loVal = int((1023 - speed) % 256)
                hiVal = int((1023 - speed) >> 8)
            writeableVals.append( (sid, loVal, hiVal) )
        # use sync write to broadcast multi servo message
        self.__sio.sync_write_to_servos(AX_GOAL_SPEED_L, tuple(writeableVals))

    def set_multi_servo_positions(self, valueTuples):
        """
        Set different positions for multiple servos.
        Should be called as such:
        set_multi_servo_positions( ( (id1, position1), (id2, position2), (id3, position3) ) )
        """
        # prepare value tuples for call to syncwrite
        writeableVals = []
        for vals in valueTuples:
            sid = vals[0]
            position = vals[1]
            # split position into 2 bytes
            loVal = int(position % 256)
            hiVal = int(position >> 8)
            writeableVals.append( (sid, loVal, hiVal) )
        # use sync write to broadcast multi servo message
        self.__sio.sync_write_to_servos(AX_GOAL_POSITION_L, tuple(writeableVals))

    def set_multi_servo_positions_and_speeds(self, valueTuples):
        """
        Set different positions and speeds for multiple servos.
        Should be called as such:
        set_multi_servo_speeds( ( (id1, position1, speed1), (id2, position2, speed2), (id3, position3, speed3) ) )
        """
        # prepare value tuples for call to syncwrite
        writeableVals = []
        for vals in valueTuples:
            sid = vals[0]
            position = vals[1]
            speed = vals[2]
            # split speed into 2 bytes
            if speed >= 0:
                loSpeedVal = int(speed % 256)
                hiSpeedVal = int(speed >> 8)
            else:
                loSpeedVal = int((1023 - speed) % 256)
                hiSpeedVal = int((1023 - speed) >> 8)
            # split position into 2 bytes
            loPositionVal = int(position % 256)
            hiPositionVal = int(position >> 8)
            writeableVals.append( (sid, loPositionVal, hiPositionVal, loSpeedVal, hiSpeedVal) )
        # use sync write to broadcast multi servo message
        self.__sio.sync_write_to_servos(AX_GOAL_POSITION_L, tuple(writeableVals))

    def set_multi_servos_to_torque_enabled(self, servoIds, enabled):
        """
        Method to set multiple servos torque enabled.
        Should be called as such:
        set_multi_servos_to_torque_enabled( (id1, id2, id3), True)
        """
        # prepare value tuples for call to syncwrite
        writeableVals = []
        for sid in servoIds:
            # Choose 1 or 0 for torque enable value
            if enabled:
                val = 1
            else:
                val = 0
            writeableVals.append( (sid, val) )
        # use sync write to broadcast multi servo message
        self.__sio.sync_write_to_servos(AX_TORQUE_ENABLE, tuple(writeableVals))

    #################################
    # Servo status access functions #
    #################################

    def get_servo_model_number(self, servoId):
        """ Reads the servo's model number (e.g. 12 for AX-12+). """
        response = self.__sio.read_from_servo(servoId, AX_MODEL_NUMBER_L, 2)
        if response:
            self.exception_on_error(response[4], servoId, 'fetching model number')
        return response[5] + (response[6] << 8)

    def get_servo_firmware_version(self, servoId):
        """ Reads the servo's firmware version. """
        response = self.__sio.read_from_servo(servoId, AX_VERSION, 1)
        if response:
            self.exception_on_error(response[4], servoId, 'fetching firmware version')
        return response[5]

    def get_servo_return_delay_time(self, servoId):
        """ Reads the servo's return delay time. """
        response = self.__sio.read_from_servo(servoId, AX_RETURN_DELAY_TIME, 1)
        if response:
            self.exception_on_error(response[4], servoId, 'fetching return delay time')
        return response[5]

    def get_servo_min_max_angle_limits(self, servoId):
        """
        Returns the min and max angle limits from the specified servo.
        """
        # read in 4 consecutive bytes starting with low value of clockwise angle limit
        response = self.__sio.read_from_servo(servoId, AX_CW_ANGLE_LIMIT_L, 4)
        if response:
            self.exception_on_error(response[4], servoId, 'fetching CW/CCW angle limits')
        # extract data valus from the raw data
        cwLimit = response[5] + (response[6] << 8)
        ccwLimit = response[7] + (response[8] << 8)

        # return the data in a dictionary
        return {'min':cwLimit, 'max':ccwLimit}

    def get_servo_position(self, servoId):
        """ Reads the servo's position value from its registers. """
        response = self.__sio.read_from_servo(servoId, AX_PRESENT_POSITION_L, 2)
        if response:
            self.exception_on_error(response[4], servoId, 'fetching present position')
        position = response[5] + (response[6] << 8)
        return position

    def get_servo_speed(self, servoId):
        """ Reads the servo's speed value from its registers. """
        response = self.__sio.read_from_servo(servoId, AX_PRESENT_SPEED_L, 2)
        if response:
            self.exception_on_error(response[4], servoId, 'fetching present speed')
        speed = response[5] + (response[6] << 8)
        if speed > 1023:
            return 1023 - speed
        return speed

    def get_servo_feedback(self, servoId):
        """
        Returns the id, goal, position, error, speed, load, voltage, temperature
        and moving values from the specified servo.
        """
        # read in 17 consecutive bytes starting with low value for goal position
        response = self.__sio.read_from_servo(servoId, AX_GOAL_POSITION_L, 17)

        if response:
            self.exception_on_error(response[4], servoId, 'fetching full servo status')
        if len(response) == 24:
            # extract data values from the raw data
            goal = response[5] + (response[6] << 8)
            position = response[11] + (response[12] << 8)
            error = position - goal
            speed = response[13] + ( response[14] << 8)
            if speed > 1023: speed = 1023 - speed
            load_raw = response[15] + (response[16] << 8)
            load_direction = 1 if self.test_bit(load_raw, 10) else 0
            load = (load_raw & int('1111111111', 2)) / 1024.0
            if load_direction == 1: load = -load
            voltage = response[17] / 10.0
            temperature = response[18]
            moving = response[21]
            timestamp = response[-1]

            # return the data in a dictionary
            return { 'timestamp': timestamp,
                     'id': servoId,
                     'goal': goal,
                     'position': position,
                     'error': error,
                     'speed': speed,
                     'load': load,
                     'voltage': voltage,
                     'temperature': temperature,
                     'moving': bool(moving) }

    def exception_on_error(self, error_code, servo_id, command_failed):
        global exception
        exception = None
        ex_message = '[servo #%d on %s@%sbps]: %s failed' % (servo_id, self.port, self.baud, command_failed)
        if not error_code & AX_OVERHEATING_ERROR == 0:
            msg = 'Overheating Error ' + ex_message
            exception = FatalErrorCodeError(msg, error_code)
        if not error_code & AX_OVERLOAD_ERROR == 0:
            msg = 'Overload Error ' + ex_message
            exception = FatalErrorCodeError(msg, error_code)
        if not error_code & AX_INPUT_VOLTAGE_ERROR == 0:
            msg = 'Input Voltage Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if not error_code & AX_ANGLE_LIMIT_ERROR == 0:
            msg = 'Angle Limit Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if not error_code & AX_RANGE_ERROR == 0:
            msg = 'Range Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if not error_code & AX_CHECKSUM_ERROR == 0:
            msg = 'Checksum Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if not error_code & AX_INSTRUCTION_ERROR == 0:
            msg = 'Instruction Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)

class SerialOpenError(Exception):
    def __init__(self, port, baud):
        Exception.__init__(self)
        self.message = "Cannot open port '%s' at %d bps" %(port, baud)
        self.port = port
        self.baudrate = baud
    def __str__(self):
        return self.message

class ChecksumError(Exception):
    def __init__(self, response, checksum):
        Exception.__init__(self)
        self.message = 'Checksum of %d does not match the checksum from servo of %d' \
                       %(response[-1], checksum)
        self.response_data = response
        self.expected_checksum = checksum
    def __str__(self):
        return self.message

class FatalErrorCodeError(Exception):
    def __init__(self, message, ec_const):
        Exception.__init__(self)
        self.message = message
        self.error_code = ec_const
    def __str__(self):
        return self.message

class NonfatalErrorCodeError(Exception):
    def __init__(self, message, ec_const):
        Exception.__init__(self)
        self.message = message
        self.error_code = ec_const
    def __str__(self):
        return self.message

class ErrorCodeError(Exception):
    def __init__(self, message, ec_const):
        Exception.__init__(self)
        self.message = message
        self.error_code = ec_const
    def __str__(self):
        return self.message

class DroppedPacketError(Exception):
    def __init__(self, message):
        Exception.__init__(self)
        self.message = message
    def __str__(self):
        return self.message

