import time
from smbus2 import SMBus

# MPU6050 I2C address
MPU6050_ADDR = 0x68

# MPU6050 Registers
PWR_MGMT_1   = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43

# Use only I2C bus 1
try:
    bus = SMBus(1)
    # Wake up MPU6050
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
except (FileNotFoundError, OSError):
    raise RuntimeError("MPU6050 not found on I2C bus 1")

def read_raw_data(addr):
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr+1)
    value = (high << 8) | low
    if value > 32767:
        value -= 65536
    return value

try:
    while True:
        # Read accelerometer data
        acc_x = read_raw_data(ACCEL_XOUT_H)
        acc_y = read_raw_data(ACCEL_XOUT_H + 2)
        acc_z = read_raw_data(ACCEL_XOUT_H + 4)

        # Read gyroscope data
        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_XOUT_H + 2)
        gyro_z = read_raw_data(GYRO_XOUT_H + 4)

        print(
            '\r\033[KAccel: X={0} Y={1} Z={2} | Gyro: X={3} Y={4} Z={5}'.format(
                acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z
            ),
            end='',
            flush=True
        )
        time.sleep(0.5)
except KeyboardInterrupt:
    bus.close()