from smbus2 import SMBus
import time
import struct

I2C_BUS = 7       # i2c bus for pin 5 and 6
ADDRESS = 0x6B    # accelerometer and gyroscope I2C address

bus = SMBus(I2C_BUS)

# ---- Sensor setup ----
# Check WHO_AM_I
who_am_i = bus.read_byte_data(ADDRESS, 0x0F)
print(f"WHO_AM_I: 0x{who_am_i:02X}")
if who_am_i != 0x6B:
    print("⚠️ Unexpected device ID")

# Configure accelerometer: 104 Hz, ±4g
bus.write_byte_data(ADDRESS, 0x10, 0b01010000)
# Configure gyroscope: 104 Hz, 2000 dps
bus.write_byte_data(ADDRESS, 0x11, 0b01001100)

time.sleep(0.1)

# ---- Helper to read 6 bytes and convert to signed 16-bit ----
def read_xyz(base_addr):
    data = bus.read_i2c_block_data(ADDRESS, base_addr, 6)
    x, y, z = struct.unpack('<hhh', bytes(data))
    return x, y, z

# ---- Conversion factors (from datasheet) ----
ACC_SENS = 0.122  # mg/LSB for ±4g, convert to milli-g
GYRO_SENS = 70    # mdps/LSB for 2000 dps, convert to milli-dps

# ---- Main loop ----
try:
    while True:
        gx, gy, gz = read_xyz(0x22)
        ax, ay, az = read_xyz(0x28)

        # Convert to physical units
        ax_g = ax * ACC_SENS / 1000
        ay_g = ay * ACC_SENS / 1000
        az_g = az * ACC_SENS / 1000

        gx_dps = gx * GYRO_SENS / 1000
        gy_dps = gy * GYRO_SENS / 1000
        gz_dps = gz * GYRO_SENS / 1000

        print(f"Accel [g]: X={ax_g:.3f}, Y={ay_g:.3f}, Z={az_g:.3f}")
        print(f"Gyro [dps]: X={gx_dps:.3f}, Y={gy_dps:.3f}, Z={gz_dps:.3f}")
        print("---")
        time.sleep(0.2)
except KeyboardInterrupt:
    bus.close()
    print("Stopped.")
