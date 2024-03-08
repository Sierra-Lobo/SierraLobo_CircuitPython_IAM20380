""" 
sierralobo_iam20380.py

    CircuitPython library for TDK MC3419 Accelerometer

* Author(s): chillis
* Affiliation(s): Sierra Lobo, Inc.

* Repo Link: https://github.com/Sierra-Lobo/SierraLobo_CircuitPython_IAM20380
* IAM20380 Datasheet Link:
    https://invensense.tdk.com/wp-content/uploads/2022/09/DS-000195-IAM-20380_v1.1-Typ.pdf

Implementation Notes:

Chip Capabilities Implemented:
    * TODO update

Unimplemented:
    * TODO update

"""

import time
from struct import unpack_from
from micropython import const
from adafruit_bus_device.i2c_device import I2CDevice
from adafruit_register.i2c_bits import RWBits
from adafruit_register.i2c_bit import RWBit
from adafruit_register.i2c_struct import UnaryStruct, ROUnaryStruct

try:
    from typing import Tuple
    from busio import I2C

except ImportError:
    pass

_DEFAULT_ADDR = const(0x68)

_WHO_AM_I = const((const(0xB5), const(0xFD)))  # IAM20380, IAM20380HT

_IAM20380_TEMP_OFFS = const(0x0)

_IAM20380_SMPLRT_DIV_REG = const(0x19)
_IAM20380_CONFIG_REG = const(0x1A)
_IAM20380_GYRO_CONFIG_REG = const(0x1B)
_IAM20380_LP_MODE_CFG_REG = const(0x1E)
_IAM20380_TEMP_OUT_H_REG = const(0x41)
_IAM20380_GYRO_XOUT_H_REG = const(0x43)
_IAM20380_PWR_MGMT_1_REG = const(0x6B)
_IAM20380_WHO_AM_I_REG = const(0x75)

_IAM20380_SENS_250DPS = 313
_IAM20380_SENS_500DPS = 65.5
_IAM20380_SENS_1000DPS = 32.8
_IAM20380_SENS_2000DPS = 16.4

_IAM20380_RANGE_250DPS = const(0x0)
_IAM20380_RANGE_500DPS = const(0x1)
_IAM20380_RANGE_1000DPS = const(0x2)
_IAM20380_RANGE_2000DPS = const(0x3)

_IAM20380_AVG_1 = const(0x0)
_IAM20380_AVG_2 = const(0x1)
_IAM20380_AVG_4 = const(0x2)
_IAM20380_AVG_8 = const(0x3)
_IAM20380_AVG_16 = const(0x4)
_IAM20380_AVG_32 = const(0x5)
_IAM20380_AVG_64 = const(0x6)
_IAM20380_AVG_128 = const(0x7)


class IAM20380:
    """Driver for the IAM20380 3-axis gyroscope."""

    _chip_id = ROUnaryStruct(_IAM20380_WHO_AM_I_REG, "<B")
    _rst_bit = RWBit(_IAM20380_PWR_MGMT_1_REG, 7)
    _fs_sel = RWBits(2, _IAM20380_GYRO_CONFIG_REG, 3)
    _pwr_mgmt_1 = UnaryStruct(_IAM20380_PWR_MGMT_1_REG, "<B")
    _gavg_cfg = RWBits(3, _IAM20380_LP_MODE_CFG_REG, 4)
    _fchoice_b = RWBits(2, _IAM20380_GYRO_CONFIG_REG, 0)
    _dlpf_cfg = RWBits(3, _IAM20380_CONFIG_REG, 0)
    _smplrt_div = UnaryStruct(_IAM20380_SMPLRT_DIV_REG, "<B")
    _gyro_cycle = RWBit(_IAM20380_LP_MODE_CFG_REG, 7)
    _sleep = RWBit(_IAM20380_PWR_MGMT_1_REG, 6)

    def __init__(self, i2c_bus: I2C, addr: int = _DEFAULT_ADDR) -> None:
        self.i2c_device = I2CDevice(i2c_bus, addr)
        self.reset()  # a soft reset is required
        if self._chip_id != _WHO_AM_I:  # ensured after soft reset
            raise RuntimeError(
                f"IAM20380 @ {addr:#x}: bad chip id '{self._chip_id:#x}' != '{_WHO_AM_I:#x}'"
            )

    def reset(self) -> None:
        """Reset the sensor to the default state set by the library
        the library default configuration sets the following sensor paramaters:
            Low Power Mode
            ODR: 3.9 Hz
            Sampling rate : 1 kHz
            Averages : 8
            Noise BW : 117.4 Hz
            Noise based on 0.008dps/sqrt(hz): 0.09 dps rms
            Current consumption: 1.3mA
        """
        self._rst_bit = True
        self._range = _IAM20380_RANGE_250DPS
        self._smplrt_div = 255
        self._fchoice_b = 0
        self._gavg_cfg = 3
        self._gyro_cycle = True

    @property
    def temperature(self) -> float:
        """The processed temperature sensor value, returned as float"""
        self._buffer[0] = _IAM20380_TEMP_OUT_H_REG
        with self.i2c_device as i2c:
            i2c.write_then_readinto(self._buffer, self._buffer, out_end=1)
        temp = self._buffer[0] << 8 | self._buffer[1]
        temp -= _IAM20380_TEMP_OFFS
        temp /= 326.8
        temp += 25  # not sure this is correct, in reference to datasheet p36
        return temp

    @property
    def rotation(self) -> Tuple[float, float, float]:
        """The processed gyroscope sensor values.
        A 3-tuple of X, Y, Z axis values in dps that are signed floats.
        """
        raw = self.raw

        # signed 16 to python int
        x = raw[0]
        if (x & 0x8000) == 0x8000:
            x -= 0xFFFF

        y = raw[1]
        if (y & 0x8000) == 0x8000:
            y -= 0xFFFF

        z = raw[2]
        if (z & 0x8000) == 0x8000:
            z -= 0xFFFF

        # scale to g by LSB in datasheet
        sens = self.sensitivity
        x /= sens
        y /= sens
        z /= sens
        return (x, y, z)

    @property
    def raw(self) -> Tuple[int, int, int]:
        self._buffer[0] = _IAM20380_GYRO_XOUT_H_REG
        with self.i2c_device as i2c:
            i2c.write_then_readinto(self._buffer, self._buffer, out_end=1)
        x = self._buffer[0] << 8 | self._buffer[1]
        y = self._buffer[2] << 8 | self._buffer[3]
        z = self._buffer[4] << 8 | self._buffer[5]
        return (x, y, z)

    @property
    def sensitivity(self) -> float:
        """The gyroscope sensitivity, based upon FS_SEL, returned as floats LSB/dps"""
        r = self._range
        if r == _IAM20380_RANGE_250DPS:
            return _IAM20380_SENS_250DPS
        elif r == _IAM20380_RANGE_500DPS:
            return _IAM20380_SENS_500DPS
        elif r == _IAM20380_RANGE_1000DPS:
            return _IAM20380_SENS_1000DPS
        elif r == _IAM20380_RANGE_2000DPS:
            return _IAM20380_SENS_2000DPS

    @property
    def sleep(self) -> bool:
        return self._sleep

    @sleep.setter
    def sleep(self, val: bool) -> None:
        self._sleep = val


#    @property
#    def range(self) -> int:
#        """The gyroscope full scale output setting"""
#        return self._fs_sel
#
#    @range.setter
#    def range(self, value: int) -> None:
#        if not (0 <= value <= 3):
#            raise ValueError("range must be 0b00 to 0b11.")
#        self._fs_sel = value
#
#
#
#    @property
#    def dlpf(self) -> int:
#        """The gyroscope dual low pass filter configure bits"""
#        return self._dlpf_cfg
#
#    @dlpf.setter
#    def dlpf(self, value : int) -> None:
#        if not (1 <= value <= 6):
#            raise ValueError("DLPF must be between 0b000 and 0b110")
#        else:
#            self._dlpf_cfg = value
#
#    @property
#    def sr_div(self) -> int:
#        """The gyroscope sample rate divider"""
#        return self._smplrt_div
#
#    @sr_div.setter
#    def sr_div(self, value : int) -> None:
#        if not (0 <= value <= 255):
#            raise ValueError("SMPLRT_DIV must be 0-255")
#        self._smplrt_div = value
#
#    @property
#    def avg(self) -> int:
#        "Number of averages the gyro takes per measurement, values 0-7, averages 2^x"
#        return self._gavg_cfg
#
#    @avg.setter
#    def avg(self, value : int) -> None:
#        if not (0 <= value <= 7):
#            raise ValueError("Averages must be 0b000 to 0b111")
#        self._gavg_cfg = value
#
#    def self_test(self):
#        pass
