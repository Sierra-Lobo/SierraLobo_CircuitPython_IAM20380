""" 
iam20380.py

    circuitpython driver lib for iam20380 gyro

    currently is just a set-it-and-forget-it dealio for FP/mainboard usage

* Author: Caden Hillis
"""

from micropython import const
from adafruit_bus_device.i2c_device import I2CDevice
from adafruit_register.i2c_bits import ROBits, RWBits
from adafruit_register.i2c_bit import ROBit, RWBit
from adafruit_register.i2c_struct import UnaryStruct, ROUnaryStruct
from typing import Tuple
import time

_IAM20380_DEFAULT_ADDRESS = const(0x00)
_IAM20380_WHO_AM_I = const(0xB5)

_IAM20380_SMPLRT_DIV_REG = const(0x19)
_IAM20380_CONFIG_REG = const(0x1A)
_IAM20380_GYRO_CONFIG_REG = const(0x1B)
_IAM20380_LP_MODE_CFG_REG = const(0x1E)
_IAM20380_TEMP_OUT_H_REG = const(0x41)
_IAM20380_GYRO_XOUT_H_REG = const(0x43)
_IAM20380_PWR_MGMT_1_REG = const(0x6B)
_IAM20380_WHO_AM_I_REG = const(0x75)

_IAM20380_sens = [
    """The sensitivity of the sensor per configured value, LSB/dps
    determined by FS_SEL value
    """
    131,
    65.5,
    32.8,
    16.4
]

_IAM20380_fs = [
    """The offset of the sensor per configured value, dps
    determined by FS_SEL value
    """
    250,
    500,
    1000,
    2000
]

_IAM20380_avgs = [
    """The number of averages of the sensor per configured value
    determined by G_AVGCFG value
    """
    1,
    2,
    4,
    8,
    16,
    32,
    64,
    128
]

class IAM20380:

    _chip_id = ROUnaryStruct(_IAM20380_WHO_AM_I_REG, "<B")
    _reset = RWBit(_IAM20380_PWR_MGMT_1_REG, 7)
    _fs_sel = RWBits(2, _IAM20380_GYRO_CONFIG_REG, 3)
    _pwr_mgmt_1 = UnaryStruct(_IAM20380_PWR_MGMT_1_REG, "<B")
    _gavg_cfg = RWBits(3, _IAM20380_LP_MODE_CFG_REG, 4)
    _fchoice_b = RWBits(2, _IAM20380_GYRO_CONFIG_REG, 0)
    _dlpf_cfg = RWBits(3, _IAM20380_CONFIG_REG, 0)
    _smplrt_div = UnaryStruct(_IAM20380_SMPLRT_DIV_REG, "<B")
    
    def __init__(self, i2c_bus, addr : int = _IAM20380_DEFAULT_ADDRESS) -> None:
        self.i2c_device = I2CDevice(i2c_bus, addr, probe=False)
        if self._chip_id != _IAM20380_WHO_AM_I:
            raise RuntimeError("Failed to find IAM-20380 - check your wiring!")
        
        self._buffer = bytearray(6)
        self.reset()

    def reset(self) -> None:
        """Reset the sensor to the default state set by the library"""
        self._reset = True
        time.sleep(0.020) #TODO: get the correct time for startup
        #default settings of 1khz sampling rate, 128x averaging, ODR
        self._pwr_mgmt_1 = 0x01
        self._fchoice_b = 0x00
        self.fs = 250
        self.dlpf = 6
        self.smplrt_div = 255
        self.avgs = 128

    @property
    def temperature(self) -> float:
        """The processed temperature sensor value, returned in floating point C"""
        self._buffer[0] = _IAM20380_TEMP_OUT_H_REG
        with self.i2c_device as i2c:
            i2c.write_then_readinto(self._buffer, self._buffer, out_end=1)
        temp = self._buffer[0] << 8 | self._buffer[1]
        temp /= 326.8
        temp += 25 # not sure this is correct, datasheet p36 appears wrong
        return temp
    
    @property
    def rotation(self) -> Tuple[float, float, float]:
        """The processed gyroscope sensor values.
        A 3-tuple of X, Y, Z axis values in dps that are signed floats.
        """
        self._buffer[0] = _IAM20380_GYRO_XOUT_H_REG
        with self.i2c_device as i2c:
            i2c.write_then_readinto(self._buffer, self._buffer, out_end=1)
        x = self._buffer[0] << 8 | self._buffer[1]
        y = self._buffer[2] << 8 | self._buffer[3]
        z = self._buffer[4] << 8 | self._buffer[5]
        #fix center offset
        x -= 1 << 15
        y -= 1 << 15
        z -= 1 << 15
        #scale to dps
        x /= self.sens
        y /= self.sens
        z /= self.sens

    @property
    def fs(self) -> int:
        """The gyroscope full scale output, returned as dps"""
        temp = self._fs_sel
        return _IAM20380_fs[temp]
    
    @fs.setter
    def fs(self, value: int) -> None:
        if value not in _IAM20380_fs:
            raise ValueError("Full scale must be 250, 500, 1000, or 2000 dps")
        self._fs_sel = _IAM20380_fs.index(value)

    @property
    def sens(self) -> float:
        """The gyroscope sensitivity, based upon FS_SEL, LSB/dps"""
        temp = self._fs_sel
        return _IAM20380_sens[temp]
        
    @property
    def dlpf(self) -> int:
        """The gyroscope dual low pass configure bits"""
        return self._dlpf_cfg

    @dlpf.setter
    def dlpf(self, value : int) -> None:
        if value <= 1 | 6 <= value:
            raise ValueError("DLPF must be acceptable value")
        else:
            self._dlpf_cfg = value
        
    @property
    def smplrt_div(self) -> int:
        """The gyroscope sample rate divider"""
        return self._smplrt_div
    
    @smplrt_div.setter
    def smplrt_div(self, value : int) -> None:
        if 0<= value <= 255:
            raise ValueError("SMPLRT_DIV must be 0-255")

    @property
    def avgs(self) -> int:
        "Number of averages the gyro takes per measurement, values 0-7, averages 2^x"
        temp = self._gavg_cfg
        return _IAM20380_avgs[temp]

    @avgs.setter
    def avgs(self, value : int) -> None:
        if value not in _IAM20380_avgs:
            raise ValueError("Averages must be 1, 2, 4, 8, 16, 32, 64, or 128")
        self._gavg_cfg = _IAM20380_avgs.index(value)

    @property
    def odr(self) -> int:
        temp =  (1000 / self._smplrt_div)
        return temp