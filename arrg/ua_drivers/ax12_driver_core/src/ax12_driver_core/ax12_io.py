#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Arizona Robotics Research Group,
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

import time
import serial
from binascii import b2a_hex

from ax12_const import *
from ax12_user_commands import AX_GOAL_POSITION, AX_GOAL_SPEED, AX_TORQUE_ENABLE as AX_TORQUE_EN

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
        checksum = 255 - ( (servoId + length + AX_READ_DATA + \
                            address + size) % 256 )
        
        # packet: FF  FF  ID LENGTH INSTRUCTION PARAM_1 ... CHECKSUM
        packetStr = chr(0xFF) + chr(0xFF) + chr(servoId) + chr(length) + \
                    chr(AX_READ_DATA) + chr(address) + chr(size) + \
                    chr(checksum)
        self.ser.write(packetStr)
        
        # wait for response packet from AX-12+
        time.sleep(0.0005)
        
        # read response
        data = []
        data.append(self.ser.read()) # read 0xFF
        if not b2a_hex(data[0]) == 'ff': return []
        data.append(self.ser.read()) # read 0xFF
        if not b2a_hex(data[1]) == 'ff': return []
        data.append(self.ser.read()) # read id
        data.append(self.ser.read()) # read length
        try:
            length = ord(data[3])
            while length > 0:
                data.append(self.ser.read())
                length -= 1
            data = map(b2a_hex, data)
            data = map( int, data, ([16] * len(data)) )
        except Exception, e:
            raise DroppedPacketError('Invalid response received from motor %d.' %servoId)
        
        # verify checksum
        checksum = 255 - reduce(int.__add__, data[2:-1]) % 256
        if not checksum == data[-1]:
            raise ChecksumError(data, checksum)
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
        checksum = 255 - ((servoId + length + AX_WRITE_DATA + \
                           address + sum(data)) % 256)
        
        # packet: FF  FF  ID LENGTH INSTRUCTION PARAM_1 ... CHECKSUM
        packetStr = chr(0xFF) + chr(0xFF) + chr(servoId) + chr(length) + \
                       chr(AX_WRITE_DATA) + chr(address)
        self.ser.write(packetStr)
        for d in data:
            self.ser.write(chr(d))
        self.ser.write(chr(checksum))
        
        # wait for response packet from AX-12+
        time.sleep(0.0005)
        
        # read response
        data = []
        data.append(self.ser.read()) # read 0xFF
        if not b2a_hex(data[0]) == 'ff':
            return []
        data.append(self.ser.read()) # read 0xFF
        data.append(self.ser.read()) # read id
        data.append(self.ser.read()) # read length
        try:
            length = ord(data[3])
            while length > 0:
                data.append(self.ser.read())
                length -= 1
            data = map(b2a_hex, data)
            data = map( int, data, ([16] * len(data)) )
        except Exception, e:
            raise DroppedPacketError('Invalid response received from motor %d.' %servoId)
        
        # verify checksum
        checksum = 255 - reduce(int.__add__, data[2:-1]) % 256
        if not checksum == data[-1]:
            raise ChecksumError(data, checksum)
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
        packetStr = chr(0xFF) + chr(0xFF) + chr(servoId) + chr(length) + \
                       chr(AX_PING) + chr(checksum)
        self.ser.write(packetStr)
        
        # wait for response packet from AX-12+
        time.sleep(0.0005)
        
        # read response
        data = []
        data.append(self.ser.read()) # read 0xFF
        if not b2a_hex(data[0]) == 'ff':
            return []
        data.append(self.ser.read()) # read 0xFF
        data.append(self.ser.read()) # read id
        data.append(self.ser.read()) # read length
        length = ord(data[3])
        while length > 0:
            data.append(self.ser.read())
            length -= 1
        data = map(b2a_hex, data)
        data = map( int, data, ([16] * len(data)) )
        
        # verify checksum
        checksum = 255 - reduce(int.__add__, data[2:-1]) % 256
        if not checksum == data[-1]:
            raise ChecksumError(data, checksum)
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
        
        # Number of bytes following standard header (0xFF, 0xFF, id, length)
        length = 4  # instruction, address, length, checksum
        # Must iterate through data to calculate length and keep running sum
        # for the checksum
        valsum = 0
        for d in data:
            length += len(d)    
            valsum += sum(d)
        
        checksum = 255 - ((AX_BROADCAST + length + \
                          AX_SYNC_WRITE + address + len(data[0][1:]) + \
                          valsum) % 256)
        
        # packet: FF  FF  ID LENGTH INSTRUCTION PARAM_1 ... CHECKSUM
        packetStr = chr(0xFF) + chr(0xFF) + chr(AX_BROADCAST) + \
                    chr(length) + chr(AX_SYNC_WRITE) + chr(address) + \
                    chr(len(data[0][1:]))
        self.ser.write(packetStr)
        for servo in data:
            for value in servo:
                self.ser.write(chr(value))
        self.ser.write(chr(checksum))

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
            self.exception_on_error(response[4], 'when pinging servo with id %d' %(servoId))
        return response

    def write_packet(self, packet):
        command = packet[0]
        
        register = 0
        values = []
        
        if command == AX_GOAL_POSITION:
            parse2 = True
            register = AX_GOAL_POSITION_L
        elif command == AX_GOAL_SPEED:
            parse2 = True
            register = AX_GOAL_SPEED_L
        elif command == AX_TORQUE_EN:
            parse2 = False
            register = AX_TORQUE_ENABLE                 

        for val in packet[1]:
            motor_id = val[0]
            value = val[1]
            if parse2:
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
            self.exception_on_error(response[4], 'when setting servo with id %d to speed %d' %(servoId, speed))
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
            self.exception_on_error(response[4], 'when setting servo with id %d to position %d' %(servoId, position))
        return response

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
            self.exception_on_error(response[4], 'when setting servo with id %d to position %d and speed %d' %(servoId, position, speed))
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

    def set_min_max_angle_limits(self, servoId, minAngle, maxAngle):
        """
        Set the min and max angle of rotation limits.
        NOTE: the absolute min is 0 and the absolute max is 300
        """
        loMinVal = int(minAngle % 256)
        hiMinVal = int(minAngle >> 8)
        loMaxVal = int(maxAngle % 256)
        hiMaxVal = int(maxAngle >> 8)
        # set 4 register values with low and high bytes for min and max angles
        response = self.__sio.write_to_servo(servoId, AX_CW_ANGLE_LIMIT_L,
                                      (loMinVal, hiMinVal, loMaxVal, hiMaxVal))
        if response:
            self.exception_on_error(response[4], 'when setting servo with id %d to min angle %d and max angle %d' %(servoId, minAngle, maxAngle))
        return response

    def set_torque_enabled(self, servoId, enabled):
        """ Sets the value of the torque enabled register to 1 or 0. When the
        torque is disabled the servo can be moved manually while the motor is
        still powered.
        """
        if enabled:
            value = (1,)
        else:
            value = (0,)
        response = self.__sio.write_to_servo(servoId, AX_TORQUE_ENABLE,
                                             value)
        if response:
            self.exception_on_error(response[4], 'when setting servo with id %d to torque_enabled ' + str(enabled))
        return response

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

    def get_servo_speed(self, servoId):
        """ Reads the servo's speed value from its registers. """
        response = self.__sio.read_from_servo(servoId, AX_PRESENT_SPEED_L, 2)
        if response:
            self.exception_on_error(response[4], 'when getting speed of servo with id %d' %(servoId))
        speed = response[5] + (response[6] << 8)
        if speed > 1023:
            return 1023 - speed
        return speed
    
    def get_servo_position(self, servoId):
        """ Reads the servo's position value from its registers. """
        response = self.__sio.read_from_servo(servoId, AX_PRESENT_POSITION_L, 2)
        if response:
            self.exception_on_error(response[4], 'when getting position of servo with id %d' %(servoId))
        position = response[5] + (response[6] << 8)
        return position

    def get_min_max_angle_limits(self, servoId):
        """
        Returns the min and max angle limits from the specified servo.
        """
        # read in 4 consecutive bytes starting with low value of clockwise angle limit
        response = self.__sio.read_from_servo(servoId, AX_CW_ANGLE_LIMIT_L, 4)
        if response:
            self.exception_on_error(response[4], 'when getting min/max angle limits of servo with id %d' %(servoId))
        # extract data valus from the raw data
        cwLimit = response[5] + (response[6] << 8)
        ccwLimit = response[7] + (response[8] << 8)
        
        # return the data in a dictionary
        return {'min':cwLimit, 'max':ccwLimit}

    def get_servo_feedback(self, servoId):
        """
        Returns the position, speed, load, voltage, and temperature values
        from the specified servo.
        """
        # read in 8 consecutive bytes starting with low value for position
        response = self.__sio.read_from_servo(servoId,
                                        AX_PRESENT_POSITION_L, 11)
        if response:
            self.exception_on_error(response[4], 'when getting feedback from servo with id %d' %(servoId))
        if len(response) == 17:
            # extract data values from the raw data
            position = response[5] + (response[6] << 8)
            speed = response[7] + ( response[8] << 8)
            load_raw = response[9] + (response[10] << 8)
            load_direction = 1 if self.test_bit(load_raw, 10) else 0
            load = load_raw & int('1111111111', 2)
            if load_direction == 1:
                load = -load
            voltage = response[11]
            temperature = response[12]
            moving = response[15]
            
            if speed > 1023:
                speed = 1023 - speed
            
            # return the data in a dictionary
            return {'id':servoId, 'position':position, 'speed':speed, 'load':load, 'voltage':voltage, 'temperature':temperature, 'moving':bool(moving)}

    def exception_on_error(self, error_code, ex_message):
        if not error_code & AX_OVERHEATING_ERROR == 0:
            msg = 'Overheating Error ' + ex_message
            raise FatalErrorCodeError(msg, error_code)
        if not error_code & AX_OVERLOAD_ERROR == 0:
            msg = 'Overload Error ' + ex_message
            raise FatalErrorCodeError(msg, error_code)
        if not error_code & AX_INPUT_VOLTAGE_ERROR == 0:
            msg = 'Input Voltage Error ' + ex_message
            raise NonfatalErrorCodeError(msg, error_code)
        if not error_code & AX_ANGLE_LIMIT_ERROR == 0:
            msg = 'Angle Limit Error ' + ex_message
            raise NonfatalErrorCodeError(msg, error_code)
        if not error_code & AX_RANGE_ERROR == 0:
            msg = 'Range Error ' + ex_message
            raise NonfatalErrorCodeError(msg, error_code)
        if not error_code & AX_CHECKSUM_ERROR == 0:
            msg = 'Checksum Error ' + ex_message
            raise NonfatalErrorCodeError(msg, error_code)
        if not error_code & AX_INSTRUCTION_ERROR == 0:
            msg = 'Instruction Error ' + ex_message
            raise NonfatalErrorCodeError(msg, error_code)

class SerialOpenError(Exception):
    def __init__(self, port, baud):
        Exception.__init__(self)
        self.message = "Cannot open port '%s' at %d bps" %(port, baud)
        self.port = port
        self.baudrate = baud
    def __str__(self):
        return self.message

class ChecksumError(Exception):
    def __init__(self, reponse, checksum):
        Exception.__init__(self)
        self.message = 'Checksum of %d does not match the checksum from servo of %d' \
                       %(response[-1], checksum)
        self.response_data = reponse
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
