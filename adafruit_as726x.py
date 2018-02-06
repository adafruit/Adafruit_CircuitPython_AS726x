# The MIT License (MIT)
#
# Copyright (c) 2017 Dean Miller for Adafruit Industries
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""
`adafruit_as726x`
====================================================

Driver for the AS726x spectral sensors

* Author(s): Dean Miller
"""

# imports

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_AS726x.git"

import time
from adafruit_bus_device.i2c_device import I2CDevice
from micropython import const
import ustruct

AS726X_ADDRESS = const(0x49)

AS726X_HW_VERSION = const(0x00)
AS726X_FW_VERSION = const(0x02)
AS726X_CONTROL_SETUP = const(0x04)
AS726X_INT_T = const(0x05)
AS726X_DEVICE_TEMP = const(0x06)
AS726X_LED_CONTROL = const(0x07)

#for reading sensor data
AS7262_V_HIGH = const(0x08)
AS7262_V_LOW = const(0x09)
AS7262_B_HIGH = const(0x0A)
AS7262_B_LOW = const(0x0B)
AS7262_G_HIGH = const(0x0C)
AS7262_G_LOW = const(0x0D)
AS7262_Y_HIGH = const(0x0E)
AS7262_Y_LOW = const(0x0F)
AS7262_O_HIGH = const(0x10)
AS7262_O_LOW = const(0x11)
AS7262_R_HIGH = const(0x12)
AS7262_R_LOW = const(0x13)

AS7262_V_CAL = const(0x14)
AS7262_B_CAL = const(0x18)
AS7262_G_CAL = const(0x1C)
AS7262_Y_CAL = const(0x20)
AS7262_O_CAL = const(0x24)
AS7262_R_CAL = const(0x28)

#hardware registers
AS726X_SLAVE_STATUS_REG = const(0x00)
AS726X_SLAVE_WRITE_REG = const(0x01)
AS726X_SLAVE_READ_REG = const(0x02)
AS726X_SLAVE_TX_VALID = const(0x02)
AS726X_SLAVE_RX_VALID = const(0x01)

AS7262_VIOLET = const(0x08)
AS7262_BLUE = const(0x0A)
AS7262_GREEN = const(0x0C)
AS7262_YELLOW = const(0x0E)
AS7262_ORANGE = const(0x10)
AS7262_RED = const(0x12)
AS7262_VIOLET_CALIBRATED = const(0x14)
AS7262_BLUE_CALIBRATED = const(0x18)
AS7262_GREEN_CALIBRATED = const(0x1C)
AS7262_YELLOW_CALIBRATED = const(0x20)
AS7262_ORANGE_CALIBRATED = const(0x24)
AS7262_RED_CALIBRATED = const(0x28)

AS726X_NUM_CHANNELS = const(6)

#pylint: disable=too-many-instance-attributes
#pylint: disable=too-many-public-methods
class Adafruit_AS726x(object):
    """Driver for the AS726x spectral sensor."""

    MODE_0 = 0b00
    MODE_1 = 0b01
    MODE_2 = 0b10 #default
    ONE_SHOT = 0b11

    GAIN_1X = 0b00 #default
    GAIN_3X7 = 0b01
    GAIN_16X = 0b10
    GAIN_64X = 0b11

    LIMIT_1MA = 0b00 #default
    LIMIT_2MA = 0b01
    LIMIT_4MA = 0b10
    LIMIT_8MA = 0b11

    LIMIT_12MA5 = 0b00 #default
    LIMIT_25MA = 0b01
    LIMIT_50MA = 0b10
    LIMIT_100MA = 0b11

    def __init__(self, i2c, *, address=AS726X_ADDRESS):

        self._driver_led = False
        self._indicator_led = False
        self._driver_led_current = self.LIMIT_12MA5
        self._indicator_led_current = self.LIMIT_1MA
        self._conversion_mode = self.MODE_0
        self._integration_time = 0
        self._gain = self.GAIN_1X
        self.buf2 = bytearray(2)

        self.i2c_device = I2CDevice(i2c, address)

        #reset device
        self._virtual_write(AS726X_CONTROL_SETUP, 0x80)

        #wait for it to boot up
        time.sleep(1)

        #try to read the version reg to make sure we can connect
        version = self._virtual_read(AS726X_HW_VERSION)

        #TODO: add support for other devices
        if version != 0x40:
            raise ValueError("device could not be reached or this device is not supported!")

        self.integration_time = 140

        self.gain = self.GAIN_64X

        #self.conversion_mode = AS726x_ONE_SHOT

    @property
    def driver_led(self):
        """Get and set the state of the driver LED. Boolean value that will
        turn on the LED with a value of True and disable with a value of False.
        """
        return self._driver_led

    @driver_led.setter
    def driver_led(self, val):
        val = bool(val)
        if self._driver_led == val:
            return
        self._driver_led = val
        enable = self._virtual_read(AS726X_LED_CONTROL)
        enable &= ~(val << 3)
        self._virtual_write(AS726X_LED_CONTROL, enable | (val << 3))

    @property
    def indicator_led(self):
        """Get and set the state of the indicator LED. Boolean value that will
        turn on the LED with a value of True and disable with a value of False.
        """
        return self._indicator_led

    @indicator_led.setter
    def indicator_led(self, val):
        val = bool(val)
        if self._indicator_led == val:
            return
        self._indicator_led = val
        enable = self._virtual_read(AS726X_LED_CONTROL)
        enable &= ~(val)
        self._virtual_write(AS726X_LED_CONTROL, enable | val)

    @property
    def driver_led_current(self):
        """Get and set the current limit for the driver LED"""
        return self._driver_led_current

    @driver_led_current.setter
    def driver_led_current(self, val):
        val = int(val)
        assert self.LIMIT_12MA5 <= val <= self.LIMIT_100MA
        if self._driver_led_current == val:
            return
        self._driver_led_current = val
        state = self._virtual_read(AS726X_LED_CONTROL)
        state &= ~(val << 4)
        self._virtual_write(AS726X_LED_CONTROL, state | (val << 4))

    @property
    def indicator_led_current(self):
        """Get and set the current limit for the indicator LED"""
        return self._indicator_led_current

    @indicator_led_current.setter
    def indicator_led_current(self, val):
        val = int(val)
        assert self.LIMIT_1MA <= val <= self.LIMIT_8MA
        if self._indicator_led_current == val:
            return
        self._indicator_led_current = val
        state = self._virtual_read(AS726X_LED_CONTROL)
        state &= ~(val << 1)
        self._virtual_write(AS726X_LED_CONTROL, state | (val << 1))

    @property
    def conversion_mode(self):
        """Get and set the conversion mode"""
        return self._conversion_mode

    @conversion_mode.setter
    def conversion_mode(self, val):
        val = int(val)
        assert self.MODE_0 <= val <= self.ONE_SHOT
        if self._conversion_mode == val:
            return
        self._conversion_mode = val
        state = self._virtual_read(AS726X_CONTROL_SETUP)
        state &= ~(val << 2)
        self._virtual_write(AS726X_CONTROL_SETUP, state | (val << 2))

    @property
    def gain(self):
        """Get and set the gain for the sensor"""
        return self._gain

    @gain.setter
    def gain(self, val):
        val = int(val)
        assert self.GAIN_1X <= val <= self.GAIN_64X
        if self._gain == val:
            return
        self._gain = val
        state = self._virtual_read(AS726X_CONTROL_SETUP)
        state &= ~(val << 4)
        self._virtual_write(AS726X_CONTROL_SETUP, state | (val << 4))

    @property
    def integration_time(self):
        """Get and set the integration time in milliseconds for the sensor"""
        return self._integration_time

    @integration_time.setter
    def integration_time(self, val):
        """Get and set integration time for the sensor"""
        val = int(val)
        assert 0 <= val <= 714
        if self._integration_time == val:
            return
        self._integration_time = val
        self._virtual_write(AS726X_INT_T, int(val/2.8))

    def start_measurement(self):
        """Begin a measurement. This will set the device to One Shot mode"""
        state = self._virtual_read(AS726X_CONTROL_SETUP)
        state &= ~(0x02)
        self._virtual_write(AS726X_CONTROL_SETUP, state)

        self.conversion_mode = self.ONE_SHOT

    def read_channel(self, channel):
        """Read an individual sensor channel"""
        return (self._virtual_read(channel) << 8) | self._virtual_read(channel + 1)

    def read_calibrated_value(self, channel):
        """Read a calibrated sensor channel"""
        val = bytearray(4)
        val[0] = self._virtual_read(channel)
        val[1] = self._virtual_read(channel + 1)
        val[2] = self._virtual_read(channel + 2)
        val[3] = self._virtual_read(channel + 3)
        return ustruct.unpack('!f', val)[0]

    @property
    def data_ready(self):
        """Returns True if the sensor has data ready to be read, False otherwise"""
        return (self._virtual_read(AS726X_CONTROL_SETUP) >> 1) & 0x01

    @property
    def temperature(self):
        """Get the temperature of the device in Centigrade"""
        return self._virtual_read(AS726X_DEVICE_TEMP)

    @property
    def violet(self):
        """raw violet"""
        return self.read_channel(AS7262_VIOLET)

    @property
    def blue(self):
        """raw blue"""
        return self.read_channel(AS7262_BLUE)

    @property
    def green(self):
        """raw green"""
        return self.read_channel(AS7262_GREEN)

    @property
    def yellow(self):
        """raw yellow"""
        return self.read_channel(AS7262_YELLOW)

    @property
    def orange(self):
        """raw orange"""
        return self.read_channel(AS7262_ORANGE)

    @property
    def red(self):
        """raw red"""
        return self.read_channel(AS7262_RED)

    @property
    def violet_calibrated(self):
        """calibrated violet"""
        return self.read_calibrated_value(AS7262_VIOLET_CALIBRATED)

    @property
    def blue_calibrated(self):
        """calibrated blue"""
        return self.read_calibrated_value(AS7262_BLUE_CALIBRATED)

    @property
    def green_calibrated(self):
        """calibrated green"""
        return self.read_calibrated_value(AS7262_GREEN_CALIBRATED)

    @property
    def yellow_calibrated(self):
        """calibrated yellow"""
        return self.read_calibrated_value(AS7262_YELLOW_CALIBRATED)

    @property
    def orange_calibrated(self):
        """calibrated orange"""
        return self.read_calibrated_value(AS7262_ORANGE_CALIBRATED)

    @property
    def red_calibrated(self):
        """calibrated red"""
        return self.read_calibrated_value(AS7262_RED_CALIBRATED)

    def _read_u8(self, command):
        """read a single byte from a specified register"""
        buf = self.buf2
        buf[0] = command
        with self.i2c_device as i2c:
            i2c.write(buf, end=1)
            i2c.readinto(buf, end=1)
        return buf[0]

    def __write_u8(self, command, abyte):
        """Write a command and 1 byte of data to the I2C device"""
        buf = self.buf2
        buf[0] = command
        buf[1] = abyte
        with self.i2c_device as i2c:
            i2c.write(buf)

    def _virtual_read(self, addr):
        """read a virtual register"""
        while 1:
            # Read slave I2C status to see if the read buffer is ready.
            status = self._read_u8(AS726X_SLAVE_STATUS_REG)
            if (status & AS726X_SLAVE_TX_VALID) == 0:
                # No inbound TX pending at slave. Okay to write now.
                break
        # Send the virtual register address (setting bit 7 to indicate a pending write).
        self.__write_u8(AS726X_SLAVE_WRITE_REG, addr)
        while 1:
            # Read the slave I2C status to see if our read data is available.
            status = self._read_u8(AS726X_SLAVE_STATUS_REG)
            if (status & AS726X_SLAVE_RX_VALID) != 0:
                # Read data is ready.
                break
        # Read the data to complete the operation.
        data = self._read_u8(AS726X_SLAVE_READ_REG)
        return data


    def _virtual_write(self, addr, value):
        """write a virtual register"""
        while 1:
            # Read slave I2C status to see if the write buffer is ready.
            status = self._read_u8(AS726X_SLAVE_STATUS_REG)
            if (status & AS726X_SLAVE_TX_VALID) == 0:
                break # No inbound TX pending at slave. Okay to write now.
        # Send the virtual register address (setting bit 7 to indicate a pending write).
        self.__write_u8(AS726X_SLAVE_WRITE_REG, (addr | 0x80))
        while 1:
            # Read the slave I2C status to see if the write buffer is ready.
            status = self._read_u8(AS726X_SLAVE_STATUS_REG)
            if (status & AS726X_SLAVE_TX_VALID) == 0:
                break # No inbound TX pending at slave. Okay to write data now.

        # Send the data to complete the operation.
        self.__write_u8(AS726X_SLAVE_WRITE_REG, value)


#pylint: enable=too-many-instance-attributes
#pylint: enable=too-many-public-methods
