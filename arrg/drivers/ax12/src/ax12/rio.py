#
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

"""
IO for Bioloid Control System
"""
import time
import serial
from binascii import b2a_hex

# Control Table Constants
AX_MODEL_NUMBER_L = 0
AX_MODOEL_NUMBER_H = 1
AX_VERSION = 2
AX_ID = 3
AX_BAUD_RATE = 4
AX_RETURN_DELAY_TIME = 5
AX_CW_ANGLE_LIMIT_L = 6
AX_CW_ANGLE_LIMIT_H = 7
AX_CCW_ANGLE_LIMIT_L = 8
AX_CCW_ANGLE_LIMIT_H = 9
AX_SYSTEM_DATA2 = 10
AX_LIMIT_TEMPERATURE = 11
AX_DOWN_LIMIT_VOLTAGE = 12
AX_UP_LIMIT_VOLTAGE = 13
AX_MAX_TORQUE_L = 14
AX_MAX_TORQUE_H = 15
AX_RETURN_LEVEL = 16
AX_ALARM_LED = 17
AX_ALARM_SHUTDOWN = 18
AX_OPERATING_MODE = 19
AX_DOWN_CALIBRATION_L = 20
AX_DOWN_CALIBRATION_H = 21
AX_UP_CALIBRATION_L = 22
AX_UP_CALIBRATION_H = 23
AX_TORQUE_ENABLE = 24
AX_LED = 25
AX_CW_COMPLIANCE_MARGIN = 26
AX_CCW_COMPLIANCE_MARGIN = 27
AX_CW_COMPLIANCE_SLOPE = 28
AX_CCW_COMPLIANCE_SLOPE = 29
AX_GOAL_POSITION_L = 30
AX_GOAL_POSITION_H = 31
AX_GOAL_SPEED_L = 32
AX_GOAL_SPEED_H = 33
AX_TORQUE_LIMIT_L = 34
AX_TORQUE_LIMIT_H = 35
AX_PRESENT_POSITION_L = 36
AX_PRESENT_POSITION_H = 37
AX_PRESENT_SPEED_L = 38
AX_PRESENT_SPEED_H = 39
AX_PRESENT_LOAD_L = 40
AX_PRESENT_LOAD_H = 41
AX_PRESENT_VOLTAGE = 42
AX_PRESENT_TEMPERATURE = 43
AX_REGISTERED_INSTRUCTION = 44
AX_PAUSE_TIME = 45
AX_MOVING = 46
AX_LOCK = 47
AX_PUNCH_L = 48
AX_PUNCH_H = 49

# Status Return Levels
AX_RETURN_NONE = 0
AX_RETURN_READ = 1
AX_RETURN_ALL = 2

# Instruction Set
AX_PING = 1
AX_READ_DATA = 2
AX_WRITE_DATA = 3
AX_REG_WRITE = 4
AX_ACTION = 5
AX_RESET = 6
AX_SYNC_WRITE = 131

# Broadcast Constant
AX_BROADCAST = 254

class SerialIO:
    """
    Lowest level IO needed for communication with AX-12 motors. Uses pyserial and a USB2Dynamixel
    to send/receive with motors.
    """

    def __init__(self, port='/dev/ttyUSB0', baud=1000000):
        """
        Constructor for SerialIO object takes optional parameters for port and baudrate.
        The port defaults to the 1st USB port on Ubuntu. The baudrate should be 1000000 by
        default on the AX-12s.
        """
        try:
            self.ser = serial.Serial(port)
            self.ser.setTimeout(0.015)
            self.ser.baudrate = baud

        except:
           raise(RIOSerialError("Cannot open port", port, str(baud)))

    def set_reg_value(self, servoId, regstart, values):
        """
        Set the value of registers. Should be called as such:
        setReg(1, AX_GOAL_POSITION_L, (0x01,0x05))
        """
        self.ser.flushInput()
        
        # packet: FF  FF  ID LENGTH INSTRUCTION PARAM ... CHECKSUM
        # length: 0 + 0 + 0 +   1  +     1     +    N    +   1
        length = 3 + len(values)
        
        # directly from AX-12 manual:
        # Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
        # If the calculated value is > 255, the lower byte is the check sum.
        checksum = 255 - ((servoId + length + AX_WRITE_DATA + regstart + sum(values)) % 256)
        
        # packet: FF  FF  ID LENGTH INSTRUCTION PARAM ... CHECKSUM
        self.ser.write(chr(0xFF) + chr(0xFF) + chr(servoId) + chr(length) + chr(AX_WRITE_DATA) + chr(regstart))
        for val in values:
            self.ser.write(chr(val))
        self.ser.write(chr(checksum))
        
        # wait for response packet from AX-12
        time.sleep(0.0005)
        response = []
        
        # read in response packet    
        while self.ser.inWaiting() > 0:
              response.append(str(b2a_hex(self.ser.read())))
        return response

    def get_reg_value(self, servoId, regstart, rlength):
        """
        Get the value of registers, should be called as such:
        getReg(1, AX_GOAL_POSITION_L, 2)
        """
        self.ser.flushInput()  
        checksum = 255 - ((6 + servoId + regstart + rlength) % 256)
        # packet: FF  FF  ID LENGTH INSTRUCTION PARAM ... CHECKSUM
        self.ser.write(chr(0xFF) + chr(0xFF) + chr(servoId) + chr(0x04) + chr(AX_READ_DATA) + chr(regstart)
                     + chr(rlength) + chr(checksum))
        time.sleep(0.0005)
        vals = list()
        self.ser.read() # read and toss 0xFF
        self.ser.read() # read and toss 0xFF
        self.ser.read() # read and toss id
        length = ord(self.ser.read()) - 2 # read length, subtract 2 for error val and check sum
        self.ser.read() # read and toss error val
        while length > 0:
            vals.append(ord(self.ser.read()))
            length = length - 1
        if rlength == 1:
            return vals[0]
        return vals

    def sync_write(self, regstart, vals):
        """
        Set the value of registers. Should be called as such:
        sync_write(reg, ((id1, val1, val2), (id2, val1, val2)))
        """
        self.ser.flushInput()
        length = 4
        valsum = 0
        
        # calc length and keep running sum for check sum
        for i in vals:
            length += len(i)    
            valsum += sum(i)
        checksum = 255 - ((254 + length + AX_SYNC_WRITE + regstart + len(vals[0]) - 1 + valsum) % 256)
        
        # packet: FF  FF  ID LENGTH INSTRUCTION PARAM ... CHECKSUM
        self.ser.write(chr(0xFF) + chr(0xFF) + chr(AX_BROADCAST) + chr(length) + chr(AX_SYNC_WRITE) + chr(regstart) + chr(len(vals[0])-1))
        for servo in vals:
            for value in servo:
                self.ser.write(chr(value))
        self.ser.write(chr(checksum))

    def ping(self, servoId):
        self.ser.flushInput()
        length = 2
        checksum = 255 - ((AX_PING + length + servoId) % 256)
        # packet: FF  FF  ID LENGTH INSTRUCTION CHECKSUM
        self.ser.write(chr(0xFF) + chr(0xFF) + chr(servoId) + chr(length) + chr(AX_PING) + chr(checksum))
        # allow response
        time.sleep(0.0005)
        resp = str(b2a_hex(self.ser.read())) # read 0xFF
        if not resp == 'ff':
            return []
        vals = list()
        vals.append(resp)
        vals.append(str(b2a_hex(self.ser.read()))) # read  0xFF
        vals.append(str(b2a_hex(self.ser.read()))) # read id
        vals.append(str(b2a_hex(self.ser.read()))) # read length
        vals.append(str(b2a_hex(self.ser.read()))) # read error val
        vals.append(str(b2a_hex(self.ser.read()))) # read check sum
        return vals

    def close(self):
        """
        Be nice, close the serial port.
        """
        self.ser.flushInput()
        self.ser.flushOutput()
        self.ser.close()

class AX12_IO:
    """
    Abstracts the SerialIO operations for the AX-12 into a simple easy to use interface.
    """
    
    def __init__(self, port='/dev/tty.usbserial-A9005MZc', baud=1000000):
        """
        Constructor allows the specification of port and baudrate.
        """
        try:
            self.__sio = SerialIO(port, baud)
        except RIOSerialError, e:
            raise(e)
        # Avoid writing facade code
        self.close = self.__sio.close
        self.set_reg_value = self.__sio.set_reg_value
        self.get_reg_value = self.__sio.get_reg_value
        self.ping = self.__sio.ping

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
        return self.__sio.set_reg_value(servoId, AX_GOAL_SPEED_L, (loVal, hiVal))

    def set_multi_servos_to_speed(self, servoIds, speed):
        """
        Method to set multiple servos to the same speed.
        Should be called as such:
        set_multi_servos_to_speed( (id1, id2, id3), speed)
        """
        # prepare value tuples for call to syncwrite
        writeableVals = []
        for sid in servoIds:
            # split speed into 2 bytes
            if speed >= 0:
                loVal = int(speed % 256)
                hiVal = int(speed >> 8)
            else:
                loVal = int((1023 - speed) % 256)
                hiVal = int((1023 - speed) >> 8)
            writeableVals.append( (sid, loVal, hiVal) )
        # use sync write to broadcast multi servo message
        self.__sio.sync_write(AX_GOAL_SPEED_L, tuple(writeableVals))

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
        self.__sio.sync_write(AX_GOAL_SPEED_L, tuple(writeableVals))

    def set_servo_position(self, servoId, position):
        """
        Set the servo with servoId to the specified goal position.
        Position value must be positive.
        """
        # split position into 2 bytes
        loVal = int(position % 256)
        hiVal = int(position >> 8)
        # set two register values with low and high byte for the position
        return self.__sio.set_reg_value(servoId, AX_GOAL_POSITION_L, (loVal, hiVal))

    def set_multi_servos_to_position(self, servoIds, position):
        """
        Method to set multiple servos to the same position.
        Should be called as such:
        set_multi_servos_to_position (id1, id2, id3), position)
        """
        # prepare value tuples for call to syncwrite
        writeableVals = []
        for sid in servoIds:
            # split position into 2 bytes
            loVal = int(position % 256)
            hiVal = int(position >> 8)
            writeableVals.append( (sid, loVal, hiVal) )
        # use sync write to broadcast multi servo message
        self.__sio.sync_write(AX_GOAL_POSITION_L, tuple(writeableVals))

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
        self.__sio.sync_write(AX_GOAL_POSITION_L, tuple(writeableVals))


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
        # use sync write to broadcast multi servo message
        return self.__sio.set_reg_value(servoId, AX_GOAL_POSITION_L, (loPositionVal, hiPositionVal, loSpeedVal, hiSpeedVal))

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
        self.__sio.sync_write(AX_GOAL_POSITION_L, tuple(writeableVals))

    def get_servo_feedback(self, servoId):
        """
        Returns the position, speed, load, voltage, and temperature values
        from the specified servo.
        """
        # read in 8 consecutive bytes starting with low value for position
        rawData = self.__sio.get_reg_value(servoId, AX_PRESENT_POSITION_L, 8)
        
        if len(rawData) == 8:
            # extract data values from the raw data
            position = rawData[0] + (rawData[1] << 8)
            speed = rawData[2] + ( rawData[3] << 8)
            load = rawData[4] + (rawData[5] << 8)
            voltage = rawData[6]
            temperature = rawData[7]

            # return the data in a dictionary
            return {'id':servoId, 'position':position, 'speed':speed, 'load':load, 'voltage':voltage, 'temperature':temperature}

    def set_min_max_angle_limits(self, servoId, minAngle, maxAngle):
        """
        Set the min and max angle of rotation limits.
        NOTE: the absolute min is 0 and the absolute max is 300
        """
        loMinVal = int(minAngle % 256)
        hiMinVal = int(minAngle >> 8)
        loMaxVal = int(maxAngle % 256)
        hiMaxVal = int(maxAngle >> 8)
        # set four register values with low and high bytes for min and max angles
        return self.__sio.set_reg_value(servoId, AX_CW_ANGLE_LIMIT_L, (loMinVal, hiMinVal, loMaxVal, hiMaxVal))

    def get_min_max_angle_limits(self, servoId):
        """
        Returns the min and max angle limits from the specified servo.
        """
        # read in 4 consecutive bytes starting with low value of clockwise angle limit
        rawData = self.__sio.get_reg_value(servoId, AX_CW_ANGLE_LIMIT_L, 4)
        
        # extract data valus from the raw data
        cwLimit = rawData[0] + (rawData[1] << 8)
        ccwLimit = rawData[2] + ( rawData[3] << 8)
        
        # return the data in a dictionary
        return {'min':cwLimit, 'max':ccwLimit}

    def set_torque_enabled(self, servoId, enabled):
        if enabled:
            value = (1,)
        else:
            value = (0,)
        return self.__sio.set_reg_value(servoId, AX_TORQUE_ENABLE, value)

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
        self.__sio.sync_write(AX_TORQUE_ENABLE, tuple(writeableVals))

class RIOSerialError(Exception):

	def __init__(self, message, port, baud):
		Exception.__init__(self)
		self.message = message
		self.port = port
		self.baudrate = baud