# SPDX-FileCopyrightText: 2020 Dean Miller for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
`adafruit_as726x`
====================================================

Driver for the AS726x spectral sensors

* Author(s): Dean Miller

Implementation Notes
--------------------

**Hardware:**

* Adafruit `AS7262 6-Channel Visible Light / Color Sensor Breakout
  <https://www.adafruit.com/product/3779>`_ (Product ID: 3779)

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice

"""
import struct
import time
from adafruit_bus_device.i2c_device import I2CDevice
from micropython import const

try:
    from typing import Tuple, Optional

    # This is only needed for typing
    import busio  # pylint: disable=unused-import
except ImportError:
    pass


__version__: str = "0.0.0-auto.0"
__repo__: str = "https://github.com/adafruit/Adafruit_CircuitPython_AS726x.git"

_AS726X_ADDRESS: int = const(0x49)

_AS726X_HW_VERSION: int = const(0x00)
_AS726X_FW_VERSION: int = const(0x02)
_AS726X_CONTROL_SETUP: int = const(0x04)
_AS726X_INT_T: int = const(0x05)
_AS726X_DEVICE_TEMP: int = const(0x06)
_AS726X_LED_CONTROL: int = const(0x07)

# for reading sensor data
_AS7262_V_HIGH: int = const(0x08)
_AS7262_V_LOW: int = const(0x09)
_AS7262_B_HIGH: int = const(0x0A)
_AS7262_B_LOW: int = const(0x0B)
_AS7262_G_HIGH: int = const(0x0C)
_AS7262_G_LOW: int = const(0x0D)
_AS7262_Y_HIGH: int = const(0x0E)
_AS7262_Y_LOW: int = const(0x0F)
_AS7262_O_HIGH: int = const(0x10)
_AS7262_O_LOW: int = const(0x11)
_AS7262_R_HIGH: int = const(0x12)
_AS7262_R_LOW: int = const(0x13)

_AS7262_V_CAL: int = const(0x14)
_AS7262_B_CAL: int = const(0x18)
_AS7262_G_CAL: int = const(0x1C)
_AS7262_Y_CAL: int = const(0x20)
_AS7262_O_CAL: int = const(0x24)
_AS7262_R_CAL: int = const(0x28)

# hardware registers
_AS726X_SLAVE_STATUS_REG: int = const(0x00)
_AS726X_SLAVE_WRITE_REG: int = const(0x01)
_AS726X_SLAVE_READ_REG: int = const(0x02)
_AS726X_SLAVE_TX_VALID: int = const(0x02)
_AS726X_SLAVE_RX_VALID: int = const(0x01)

_AS7262_VIOLET: int = const(0x08)
_AS7262_BLUE: int = const(0x0A)
_AS7262_GREEN: int = const(0x0C)
_AS7262_YELLOW: int = const(0x0E)
_AS7262_ORANGE: int = const(0x10)
_AS7262_RED: int = const(0x12)
_AS7262_VIOLET_CALIBRATED: int = const(0x14)
_AS7262_BLUE_CALIBRATED: int = const(0x18)
_AS7262_GREEN_CALIBRATED: int = const(0x1C)
_AS7262_YELLOW_CALIBRATED = const(0x20)
_AS7262_ORANGE_CALIBRATED: int = const(0x24)
_AS7262_RED_CALIBRATED: int = const(0x28)

_AS726X_NUM_CHANNELS: int = const(6)

_COLOR_REGS: Tuple[int, ...] = (
    _AS7262_VIOLET,
    _AS7262_BLUE,
    _AS7262_GREEN,
    _AS7262_YELLOW,
    _AS7262_ORANGE,
    _AS7262_RED,
)
_COLOR_REGS_CALIBRATED: Tuple[int, ...] = (
    _AS7262_VIOLET_CALIBRATED,
    _AS7262_BLUE_CALIBRATED,
    _AS7262_GREEN_CALIBRATED,
    _AS7262_YELLOW_CALIBRATED,
    _AS7262_ORANGE_CALIBRATED,
    _AS7262_RED_CALIBRATED,
)

# pylint: disable=too-many-instance-attributes
# pylint: disable=too-many-public-methods
# pylint: disable=invalid-name
# pylint: disable=no-else-return
# pylint: disable=inconsistent-return-statements


class AS726x:
    """AS726x spectral sensor base class."""

    MODE_0: int = 0b00
    """Continuously gather samples of violet, blue, green and yellow. Orange and red are skipped
       and read zero."""

    MODE_1: int = 0b01
    """Continuously gather samples of green, yellow, orange and red. Violet and blue are skipped
       and read zero."""

    MODE_2: int = 0b10  # default
    """Continuously gather samples of all colors"""

    ONE_SHOT: int = 0b11
    """Gather a single sample of all colors and then stop"""

    GAIN: Tuple[float, ...] = (1, 3.7, 16, 64)

    INDICATOR_CURRENT_LIMITS: Tuple[int, ...] = (1, 2, 4, 8)

    DRIVER_CURRENT_LIMITS: Tuple[float, ...] = (12.5, 25, 50, 100)

    def __init__(self):
        self._driver_led = False
        self._indicator_led = False
        self._driver_led_current = AS726x.DRIVER_CURRENT_LIMITS.index(12.5)
        self._indicator_led_current = AS726x.INDICATOR_CURRENT_LIMITS.index(1)
        self._conversion_mode = AS726x.MODE_2
        self._integration_time = 0
        self._gain = AS726x.GAIN.index(1)
        self.buf2 = bytearray(2)

        # reset device
        self._virtual_write(_AS726X_CONTROL_SETUP, 0x80)

        # wait for it to boot up
        time.sleep(1)

        # try to read the version reg to make sure we can connect
        version = self._virtual_read(_AS726X_HW_VERSION)

        # TODO: add support for other devices
        if version != 0x40:
            raise ValueError(
                "device could not be reached or this device is not supported!"
            )

        self.integration_time = 140
        self.conversion_mode = AS726x.MODE_2
        self.gain = 64

    @property
    def driver_led(self) -> bool:
        """True when the driver LED is on. False otherwise."""
        return self._driver_led

    @driver_led.setter
    def driver_led(self, val: bool) -> None:
        val = bool(val)
        if self._driver_led == val:
            return
        self._driver_led = val
        enable = self._virtual_read(_AS726X_LED_CONTROL)
        enable &= ~(0x1 << 3)
        self._virtual_write(_AS726X_LED_CONTROL, enable | (val << 3))

    @property
    def indicator_led(self) -> bool:
        """True when the indicator LED is on. False otherwise."""
        return self._indicator_led

    @indicator_led.setter
    def indicator_led(self, val: bool) -> None:
        val = bool(val)
        if self._indicator_led == val:
            return
        self._indicator_led = val
        enable = self._virtual_read(_AS726X_LED_CONTROL)
        enable &= ~(0x1)
        self._virtual_write(_AS726X_LED_CONTROL, enable | val)

    @property
    def driver_led_current(self) -> float:
        """The current limit for the driver LED in milliamps. One of:

        - 12.5 mA
        - 25 mA
        - 50 mA
        - 100 mA"""
        return self._driver_led_current

    @driver_led_current.setter
    def driver_led_current(self, val: float) -> None:
        if val not in AS726x.DRIVER_CURRENT_LIMITS:
            raise ValueError("Must be 12.5, 25, 50 or 100")
        if self._driver_led_current == val:
            return
        self._driver_led_current = val
        state = self._virtual_read(_AS726X_LED_CONTROL)
        state &= ~(0x3 << 4)
        state = state | (AS726x.DRIVER_CURRENT_LIMITS.index(val) << 4)
        self._virtual_write(_AS726X_LED_CONTROL, state)

    @property
    def indicator_led_current(self) -> int:
        """The current limit for the indicator LED in milliamps. One of:

        - 1 mA
        - 2 mA
        - 4 mA
        - 8 mA"""
        return self._indicator_led_current

    @indicator_led_current.setter
    def indicator_led_current(self, val: int) -> None:
        if val not in AS726x.INDICATOR_CURRENT_LIMITS:
            raise ValueError("Must be 1, 2, 4 or 8")
        if self._indicator_led_current == val:
            return
        self._indicator_led_current = val
        state = self._virtual_read(_AS726X_LED_CONTROL)
        state &= ~(0x3 << 1)
        state = state | (AS726x.INDICATOR_CURRENT_LIMITS.index(val) << 4)
        self._virtual_write(_AS726X_LED_CONTROL, state)

    @property
    def conversion_mode(self) -> int:
        """The conversion mode. One of:

        - `MODE_0`
        - `MODE_1`
        - `MODE_2`
        - `ONE_SHOT`"""
        return self._conversion_mode

    @conversion_mode.setter
    def conversion_mode(self, val: int) -> None:
        val = int(val)
        assert self.MODE_0 <= val <= self.ONE_SHOT
        if self._conversion_mode == val:
            return
        self._conversion_mode = val
        state = self._virtual_read(_AS726X_CONTROL_SETUP)
        state &= ~(val << 2)
        self._virtual_write(_AS726X_CONTROL_SETUP, state | (val << 2))

    @property
    def gain(self) -> float:
        """The gain for the sensor"""
        return self._gain

    @gain.setter
    def gain(self, val: float) -> None:
        if val not in AS726x.GAIN:
            raise ValueError("Must be 1, 3.7, 16 or 64")
        if self._gain == val:
            return
        self._gain = val
        state = self._virtual_read(_AS726X_CONTROL_SETUP)
        state &= ~(0x3 << 4)
        state |= AS726x.GAIN.index(val) << 4
        self._virtual_write(_AS726X_CONTROL_SETUP, state)

    @property
    def integration_time(self) -> float:
        """The integration time in milliseconds between 2.8 and 714 ms"""
        return self._integration_time

    @integration_time.setter
    def integration_time(self, val: float) -> None:
        val = int(val)
        if not 2.8 <= val <= 714:
            raise ValueError("Out of supported range 2.8 - 714 ms")
        if self._integration_time == val:
            return
        self._integration_time = val
        self._virtual_write(_AS726X_INT_T, int(val / 2.8))

    def start_measurement(self) -> None:
        """Begin a measurement.

        This will set the device to One Shot mode and values will
        not change after `data_ready` until :meth:`start_measurement`
        is called again or the :meth:`conversion_mode` is changed.
        """
        state = self._virtual_read(_AS726X_CONTROL_SETUP)
        state &= ~(0x02)
        self._virtual_write(_AS726X_CONTROL_SETUP, state)

        self.conversion_mode = self.ONE_SHOT

    def read_channel(self, channel: int) -> int:
        """Read an individual sensor channel"""
        return (self._virtual_read(channel) << 8) | self._virtual_read(channel + 1)

    def read_calibrated_value(self, channel: int) -> float:
        """Read a calibrated sensor channel"""
        val = bytearray(4)
        val[0] = self._virtual_read(channel)
        val[1] = self._virtual_read(channel + 1)
        val[2] = self._virtual_read(channel + 2)
        val[3] = self._virtual_read(channel + 3)
        return struct.unpack("!f", val)[0]

    @property
    def data_ready(self) -> bool:
        """True if the sensor has data ready to be read, False otherwise"""
        return (self._virtual_read(_AS726X_CONTROL_SETUP) >> 1) & 0x01

    @property
    def temperature(self) -> int:
        """The temperature of the device in Celsius"""
        return self._virtual_read(_AS726X_DEVICE_TEMP)

    @property
    def violet(self) -> float:
        """Calibrated violet (450nm) value"""
        return self.read_calibrated_value(_AS7262_VIOLET_CALIBRATED)

    @property
    def blue(self) -> float:
        """Calibrated blue (500nm) value"""
        return self.read_calibrated_value(_AS7262_BLUE_CALIBRATED)

    @property
    def green(self) -> float:
        """Calibrated green (550nm) value"""
        return self.read_calibrated_value(_AS7262_GREEN_CALIBRATED)

    @property
    def yellow(self) -> float:
        """Calibrated yellow (570nm) value"""
        return self.read_calibrated_value(_AS7262_YELLOW_CALIBRATED)

    @property
    def orange(self) -> float:
        """Calibrated orange (600nm) value"""
        return self.read_calibrated_value(_AS7262_ORANGE_CALIBRATED)

    @property
    def red(self) -> float:
        """Calibrated red (650nm) value"""
        return self.read_calibrated_value(_AS7262_RED_CALIBRATED)

    @property
    def raw_violet(self) -> int:
        """Raw violet (450nm) 16-bit value"""
        return self.read_channel(_AS7262_VIOLET)

    @property
    def raw_blue(self) -> int:
        """Raw blue (500nm) 16-bit value"""
        return self.read_channel(_AS7262_BLUE)

    @property
    def raw_green(self) -> int:
        """Raw green (550nm) 16-bit value"""
        return self.read_channel(_AS7262_GREEN)

    @property
    def raw_yellow(self) -> int:
        """Raw yellow (570nm) 16-bit value"""
        return self.read_channel(_AS7262_YELLOW)

    @property
    def raw_orange(self) -> int:
        """Raw orange (600nm) 16-bit value"""
        return self.read_channel(_AS7262_ORANGE)

    @property
    def raw_red(self) -> int:
        """Raw red (650nm) 16-bit value"""
        return self.read_channel(_AS7262_RED)

    def _virtual_read(self, addr: int) -> float:
        raise NotImplementedError("Must be implemented.")

    def _virtual_write(self, addr: int, value: float) -> None:
        raise NotImplementedError("Must be implemented.")


class AS726x_I2C(AS726x):
    """AS726x spectral sensor via I2C.

    :param ~busio.I2C i2c_bus: The I2C bus the AS726x is connected to
    :param int address: The I2C device address. Defaults to :const:`0x49`


    **Quickstart: Importing and using the AS726x**

        Here is an example of using the :class:`AS726x_I2C` class.
        First you will need to import the libraries to use the sensor

        .. code-block:: python

            import board
            from adafruit_as726x import AS726x_I2C

        Once this is done you can define your `board.I2C` object and define your sensor object

        .. code-block:: python

            i2c = board.I2C()  # uses board.SCL and board.SDA
            sensor = AS726x_I2C(i2c)

        Now you have access to the different color attributes

        .. code-block:: python

            violet = sensor.violet
            blue = sensor.blue
            green = sensor.green
            yellow = sensor.yellow
            orange = sensor.orange
            red = sensor.red


    """

    def __init__(self, i2c_bus: busio.I2C, address: int = _AS726X_ADDRESS) -> None:
        self.i2c_device = I2CDevice(i2c_bus, address)
        super().__init__()

    def _read_u8(self, command: int) -> int:
        """read a single byte from a specified register"""
        buf = self.buf2
        buf[0] = command
        with self.i2c_device as i2c:
            i2c.write(buf, end=1)
            i2c.readinto(buf, end=1)
        return buf[0]

    def __write_u8(self, command: int, abyte: int) -> None:
        """Write a command and 1 byte of data to the I2C device"""
        buf = self.buf2
        buf[0] = command
        buf[1] = abyte
        with self.i2c_device as i2c:
            i2c.write(buf)

    def _virtual_read(self, addr: int) -> float:
        """read a virtual register"""
        while True:
            # Read slave I2C status to see if the read buffer is ready.
            status = self._read_u8(_AS726X_SLAVE_STATUS_REG)
            if (status & _AS726X_SLAVE_TX_VALID) == 0:
                # No inbound TX pending at slave. Okay to write now.
                break
        # Send the virtual register address (setting bit 7 to indicate a pending write).
        self.__write_u8(_AS726X_SLAVE_WRITE_REG, addr)
        while True:
            # Read the slave I2C status to see if our read data is available.
            status = self._read_u8(_AS726X_SLAVE_STATUS_REG)
            if (status & _AS726X_SLAVE_RX_VALID) != 0:
                # Read data is ready.
                break
        # Read the data to complete the operation.
        data = self._read_u8(_AS726X_SLAVE_READ_REG)
        return data

    def _virtual_write(self, addr: int, value: float) -> None:
        """write a virtual register"""
        while True:
            # Read slave I2C status to see if the write buffer is ready.
            status = self._read_u8(_AS726X_SLAVE_STATUS_REG)
            if (status & _AS726X_SLAVE_TX_VALID) == 0:
                break  # No inbound TX pending at slave. Okay to write now.
        # Send the virtual register address (setting bit 7 to indicate a pending write).
        self.__write_u8(_AS726X_SLAVE_WRITE_REG, (addr | 0x80))
        while True:
            # Read the slave I2C status to see if the write buffer is ready.
            status = self._read_u8(_AS726X_SLAVE_STATUS_REG)
            if (status & _AS726X_SLAVE_TX_VALID) == 0:
                break  # No inbound TX pending at slave. Okay to write data now.

        # Send the data to complete the operation.
        self.__write_u8(_AS726X_SLAVE_WRITE_REG, value)


class AS726x_UART(AS726x):
    """AS726x spectral sensor via UART.

    :param ~busio.UART uart: The UART connected to the sensor


    **Quickstart: Importing and using the AS726x**

        Here is an example of using the :class:`AS726x_I2C` class.
        First you will need to import the libraries to use the sensor

        .. code-block:: python

            import board
            from adafruit_as726x import AS726x_UART

        Once this is done you can define your `board.UART` object and define your sensor object

        .. code-block:: python

            uart = board.UART()  # uses board.SCL and board.SDA
            sensor = AS726x_UART(uart)

        Now you have access to the different color attributes

        .. code-block:: python

            violet = sensor.violet
            blue = sensor.blue
            green = sensor.green
            yellow = sensor.yellow
            orange = sensor.orange
            red = sensor.red


    """

    def __init__(self, uart: busio.UART) -> None:
        self._uart = uart
        self._uart.baudrate = 115200
        super().__init__()

    def read_channel(self, channel: int) -> float:
        """Read an individual sensor channel"""
        return self._virtual_read(channel)

    def read_calibrated_value(self, channel: int) -> float:
        """Read a calibrated sensor channel"""
        return self._virtual_read(channel)

    def _uart_xfer(self, cmd: Optional[str]) -> str:
        self._uart.reset_input_buffer()
        cmd += "\n"
        self._uart.write(cmd.encode())
        time.sleep(0.1)
        if self._uart.in_waiting:
            resp = self._uart.read(self._uart.in_waiting)
            return resp.rstrip(b" OK\n")
        return None

    def _virtual_read(self, addr: int) -> float:
        if addr == _AS726X_HW_VERSION:
            # just return what is expected
            return 0x40
        elif addr == _AS726X_DEVICE_TEMP:
            return int(self._uart_xfer("ATTEMP"))
        elif addr == _AS726X_LED_CONTROL:
            LED_IND = int(self._uart_xfer("ATLED0"))
            LED_DRV = int(self._uart_xfer("ATLED1"))
            return LED_IND << 3 | LED_DRV
        elif addr == _AS726X_CONTROL_SETUP:
            GAIN = int(self._uart_xfer("ATGAIN"))
            BANK = int(self._uart_xfer("ATTCSMD"))
            return GAIN << 4 | BANK << 2 | 1 << 1
        elif addr in _COLOR_REGS:
            resp = self._uart_xfer("ATDATA")
            resp = resp.decode().split(",")
            return int(resp[_COLOR_REGS.index(addr)])
        elif addr in _COLOR_REGS_CALIBRATED:
            resp = self._uart_xfer("ATCDATA")
            resp = resp.decode().split(",")
            return float(resp[_COLOR_REGS_CALIBRATED.index(addr)])

    def _virtual_write(self, addr: int, value: float) -> None:
        if addr == _AS726X_CONTROL_SETUP:
            # check for reset
            if (value >> 7) & 0x01:
                self._uart.write(b"ATRST\n")
                return
            # otherwise proceed
            GAIN = (value >> 4) & 0x03
            BANK = (value >> 2) & 0x03
            self._uart_xfer("ATGAIN={}".format(GAIN))
            self._uart_xfer("ATTCSMD={}".format(BANK))
        elif addr == _AS726X_LED_CONTROL:
            ICL_DRV = (value >> 4) & 0x07
            LED_DRV = 100 if value & 0x08 else 0
            ICL_IND = (value >> 1) & 0x07
            LED_IND = 100 if value & 0x01 else 0
            self._uart_xfer("ATLED0={}".format(LED_IND))
            self._uart_xfer("ATLED1={}".format(LED_DRV))
            self._uart_xfer("ATLEDC={}".format(ICL_DRV << 4 | ICL_IND))
        elif addr == _AS726X_INT_T:
            value = int(value / 2.8)
            self._uart_xfer("ATINTTIME={}".format(value))


# pylint: enable=too-many-instance-attributes
# pylint: enable=too-many-public-methods
# pylint: enable=invalid-name
# pylint: enable=no-else-return
# pylint: enable=inconsistent-return-statements
