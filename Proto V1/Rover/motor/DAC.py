import board
import busio
from adafruit_mcp4725 import MCP4725

# Create the I2C bus interface.
i2c = busio.I2C(board.SCL, board.SDA)

# Wait until the I2C bus is ready.
while not i2c.try_lock():
    pass

try:
    # Scan for I2C devices and print their addresses.
    devices = i2c.scan()
    print("I2C devices found:", [hex(device) for device in devices])
finally:
    i2c.unlock()

# Create an instance of the MCP4725 DAC.
dac = MCP4725(i2c, address=0x60)

# Now you can use the DAC instance to set the output voltage, etc.
# For example, to set the output to half of the maximum value:
dac.normalized_value = 0.2
