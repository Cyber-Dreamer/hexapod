from smbus2 import SMBus
import time

class Gyro:
    MPU6050_ADDR = 0x68
    PWR_MGMT_1   = 0x6B
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H  = 0x43

    def __init__(self, bus_id=1):
        self.bus = SMBus(bus_id)
        self.bus.write_byte_data(self.MPU6050_ADDR, self.PWR_MGMT_1, 0)

    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(self.MPU6050_ADDR, addr)
        low = self.bus.read_byte_data(self.MPU6050_ADDR, addr+1)
        value = (high << 8) | low
        if value > 32767:
            value -= 65536
        return value

    def read(self):
        acc_x = self.read_raw_data(self.ACCEL_XOUT_H)
        acc_y = self.read_raw_data(self.ACCEL_XOUT_H + 2)
        acc_z = self.read_raw_data(self.ACCEL_XOUT_H + 4)
        gyro_x = self.read_raw_data(self.GYRO_XOUT_H)
        gyro_y = self.read_raw_data(self.GYRO_XOUT_H + 2)
        gyro_z = self.read_raw_data(self.GYRO_XOUT_H + 4)
        return {
            'accel': {'x': acc_x, 'y': acc_y, 'z': acc_z},
            'gyro': {'x': gyro_x, 'y': gyro_y, 'z': gyro_z}
        }

    def close(self):
        self.bus.close()
