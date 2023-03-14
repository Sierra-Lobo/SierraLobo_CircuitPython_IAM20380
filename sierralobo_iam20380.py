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

_IAM20380_SENS_250DPS = const(313)
_IAM20380_SENS_500DPS = const(65.5)
_IAM20380_SENS_1000DPS = const(32.8)
_IAM20380_SENS_2000DPS = const(16.4)

_IAM20380_RANGE_250DPS = const(0b00)
_IAM20380_RANGE_500DPS = const(0b01)
_IAM20380_RANGE_1000DPS = const(0b10)
_IAM20380_RANGE_2000DPS = const(0b11)

_IAM20380_AVG_1 = const(0b000)
_IAM20380_AVG_2 = const(0b001)
_IAM20380_AVG_4 = const(0b010)
_IAM20380_AVG_8 = const(0b011)
_IAM20380_AVG_16 = const(0b100)
_IAM20380_AVG_32 = const(0b101)
_IAM20380_AVG_64 = const(0b110)
_IAM20380_AVG_128 = const(0b111)

class IAM20380:

    _chip_id = ROUnaryStruct(_IAM20380_WHO_AM_I_REG, "<B")
    _reset = RWBit(_IAM20380_PWR_MGMT_1_REG, 7)
    _fs_sel = RWBits(2, _IAM20380_GYRO_CONFIG_REG, 3)
    _pwr_mgmt_1 = UnaryStruct(_IAM20380_PWR_MGMT_1_REG, "<B")
    _gavg_cfg = RWBits(3, _IAM20380_LP_MODE_CFG_REG, 4)
    _fchoice_b = RWBits(2, _IAM20380_GYRO_CONFIG_REG, 0)
    _dlpf_cfg = RWBits(3, _IAM20380_CONFIG_REG, 0)
    _smplrt_div = UnaryStruct(_IAM20380_SMPLRT_DIV_REG, "<B")
    _gyro_cycle = RWBit(_IAM20380_LP_MODE_CFG_REG, 7)
    
    def __init__(self, i2c_bus, address : int = _IAM20380_DEFAULT_ADDRESS) -> None:
        self.i2c_device = I2CDevice(i2c_bus, address, probe=False)
        if self._chip_id != _IAM20380_WHO_AM_I:
            raise RuntimeError("Failed to find IAM-20380 at address: {}".format(hex(address)))
        self.reset()
        self._buffer = bytearray(6)

    def reset(self) -> None:
        """Reset the sensor to the default state set by the library
        the library default configuration sets the following sensor paramaters:
            ODR: 3.9 Hz
            Sampling rate : 1 kHz
            Sampling rate div : 255
            Averages : 128
            Noise BW: 8 Hz
            Noise(dps) based on 0.008dps/sqrt(hz): 0.02
            Current consumption: 1.8mA"""
        self._reset = True #write b7 of PWR_MGMT_1 to reset
        while self._reset:
            time.sleep(0.01)
        self._pwr_mgmt_1 |= 0x01 #auto select best availiable clock source
        self._fchoice_b = 0b00 # set fchoice_b to 0
        self.range = _IAM20380_RANGE_250DPS # set range
        self.dlpf = 0b110 # set dlpf
        self.sr_div = 255
        self.avg = _IAM20380_AVG_128
        self._gyro_cycle = True # set gyroscope low power mode

    @property
    def temperature(self) -> float:
        """The processed temperature sensor value, returned in floating point C"""
        self._buffer[0] = _IAM20380_TEMP_OUT_H_REG
        with self.i2c_device as i2c:
            i2c.write_then_readinto(self._buffer, self._buffer, out_end=1)
        temp = self._buffer[0] << 8 | self._buffer[1]
        temp /= 326.8
        temp += 25 # not sure this is correct, in reference to datasheet p36
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
        sens = self.sens
        x /= sens
        y /= sens
        z /= sens

    @property
    def range(self) -> int:
        """The gyroscope full scale output setting"""
        return self._fs_sel
    
    @range.setter
    def range(self, value: int) -> None:
        if not (0 <= value <= 3):
            raise ValueError("range must be 0b00 to 0b11.")
        self._fs_sel = value

    @property
    def sens(self) -> float:
        """The gyroscope sensitivity, based upon FS_SEL, returned as floats LSB/dps"""
        rng = self.range
        if rng == _IAM20380_RANGE_250DPS:
            return _IAM20380_SENS_250DPS
        elif rng == _IAM20380_RANGE_500DPS:
            return _IAM20380_SENS_500DPS
        elif rng == _IAM20380_RANGE_1000DPS:
            return _IAM20380_SENS_1000DPS
        elif rng == _IAM20380_RANGE_2000DPS:
            return _IAM20380_SENS_2000DPS
        
    @property
    def dlpf(self) -> int:
        """The gyroscope dual low pass filter configure bits"""
        return self._dlpf_cfg

    @dlpf.setter
    def dlpf(self, value : int) -> None:
        if not (1 <= value <= 6):
            raise ValueError("DLPF must be between 0b000 and 0b110")
        else:
            self._dlpf_cfg = value
        
    @property
    def sr_div(self) -> int:
        """The gyroscope sample rate divider"""
        return self._smplrt_div
    
    @sr_div.setter
    def smplrt_div(self, value : int) -> None:
        if not (0 <= value <= 255):
            raise ValueError("SMPLRT_DIV must be 0-255")
        self._smplrt_div = value

    @property
    def avg(self) -> int:
        "Number of averages the gyro takes per measurement, values 0-7, averages 2^x"
        return self._gavg_cfg

    @avg.setter
    def avgs(self, value : int) -> None:
        if not (0 <= value <= 7):
            raise ValueError("Averages must be 0b000 to 0b111")
        self._gavg_cfg = value