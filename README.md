# RPI_EPIC_CANBUS

Raspberry Pi port of MEGA_EPIC_CANBUS - A CAN I/O Expander for epicEFI ECU.

This firmware expands epicEFI ECU input/output capabilities over CAN bus, providing:
- 16 analog inputs (via ADS1115 ADC modules)
- 16 digital inputs
- 4 wheel speed sensor (VSS) inputs
- 8 digital outputs
- 10 PWM outputs
- GPS data (position, speed, time, etc.)

## Hardware Requirements

### Required
- **Raspberry Pi 5** (or Pi 4/3 with minor modifications)
- **Waveshare 2-Channel Isolated CAN Bus Expansion HAT**
  - Uses MCP2515 CAN controller
  - Accessed via SocketCAN (can0, can1)

### Optional (depending on features needed)
- **Adafruit Ultimate GPS with USB**
  - Appears as /dev/ttyUSB0 or /dev/ttyACM0
  - 9600 baud default (configurable)
- **CQRobot ADS1115 16-bit ADC Module(s)**
  - Up to 4 modules for 16 analog channels
  - I2C addresses: 0x48, 0x49, 0x4A, 0x4B
  - Connected via I2C (/dev/i2c-1)

## Software Dependencies

Install required packages:

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install build tools
sudo apt install -y build-essential cmake git

# Install libgpiod for GPIO access
sudo apt install -y libgpiod-dev gpiod

# Install CAN utilities
sudo apt install -y can-utils

# Install I2C tools (for ADC debugging)
sudo apt install -y i2c-tools
```

## Building

```bash
cd rpi_epic_canbus

# Create build directory
mkdir build && cd build

# Configure
cmake ..

# Build
make -j$(nproc)

# Optional: Install
sudo make install
```

## CAN Bus Setup

### Enable SPI and CAN in /boot/firmware/config.txt

Add these lines to `/boot/firmware/config.txt`:

```ini
# Enable SPI
dtparam=spi=on

# For Waveshare 2-CH CAN HAT+ on SPI0 (compatible with X735 UPS):
dtoverlay=mcp2515,spi0-0,oscillator=16000000,interrupt=22
dtoverlay=mcp2515,spi0-1,oscillator=16000000,interrupt=13

# Alternative: For SPI1 (default HAT configuration, conflicts with X735):
# dtoverlay=spi1-3cs
# dtoverlay=mcp2515,spi1-1,oscillator=16000000,interrupt=22
# dtoverlay=mcp2515,spi1-2,oscillator=16000000,interrupt=13
```

**Note:** If using with Geekworm X735 UPS, you must solder the CAN HAT to use SPI0
(see X735 Compatibility section below).

Reboot after making changes.

### Bring up CAN interface manually

```bash
# Set bitrate to 500kbps and bring up interface
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# Verify interface is up
ip -details link show can0

# Monitor CAN traffic (for debugging)
candump can0
```

## I2C Setup (for ADC)

```bash
# Enable I2C
sudo raspi-config
# Navigate to: Interface Options -> I2C -> Enable

# Verify I2C devices
sudo i2cdetect -y 1
```

Expected output with ADS1115 modules:
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- 48 49 4a 4b -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
```

## GPS Setup

The Adafruit Ultimate GPS via USB typically appears as `/dev/ttyUSB0` or `/dev/ttyACM0`.

```bash
# Check for GPS device
ls -la /dev/ttyUSB* /dev/ttyACM*

# Test GPS output
cat /dev/ttyUSB0
```

You should see NMEA sentences like:
```
$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*47
```

## Running

### Basic usage
```bash
sudo ./rpi_epic_canbus
```

### With I/O mode selection
```bash
# 16 digital inputs only
sudo ./rpi_epic_canbus -m inputs

# 8 inputs + 4 outputs + 4 VSS (default)
sudo ./rpi_epic_canbus -m balanced

# 4 inputs + 8 outputs + 4 VSS
sudo ./rpi_epic_canbus -m outputs

# Use custom config file
sudo ./rpi_epic_canbus -f /etc/epic.conf -m custom
```

### Generate a config file
```bash
# Generate default balanced config
sudo ./rpi_epic_canbus --generate-config

# Generate inputs-only config
sudo ./rpi_epic_canbus -m inputs --generate-config
```

### Command-line options
| Option | Description | Default |
|--------|-------------|---------|
| `-f, --config` | Config file path | epic.conf |
| `-c, --can` | CAN interface | can0 |
| `-g, --gps` | GPS serial device | /dev/ttyUSB0 |
| `-i, --i2c` | I2C device for ADC | /dev/i2c-1 |
| `-p, --gpio` | GPIO chip | /dev/gpiochip4 (use /dev/gpiochip10 on Pi 5) |
| `-m, --mode` | I/O mode | balanced |
| `--generate-config` | Generate config and exit | - |
| `-h, --help` | Show help | - |

## I/O Modes

The application supports three preset I/O modes plus custom configuration:

### inputs (inputs_only)
All 16 available GPIOs configured as digital inputs:
- **16 digital inputs** (GPIO 4,5,6,12,13,14,15,16,17,18,19,20,21,22,23,26)
- No outputs or VSS
- Best for: Button boxes, switch panels, sensor arrays

### balanced (default)
Mix of inputs, outputs, and wheel speed sensors:
- **8 digital inputs** (GPIO 4,5,6,12,13,14,15,16)
- **4 digital outputs** (GPIO 17,18,19,20)
- **4 VSS inputs** (GPIO 21,22,23,26)
- Best for: General automotive use with wheel speed sensing

### outputs (outputs_focus)
More outputs for controlling relays and actuators:
- **4 digital inputs** (GPIO 4,5,6,12)
- **8 digital outputs** (GPIO 13,14,15,16,17,18,19,20)
- **4 VSS inputs** (GPIO 21,22,23,26)
- Best for: Relay control, actuator systems

### custom
Full control via config file:
```ini
io_mode = custom

gpio4 = input, StartButton
gpio5 = input, StopButton
gpio17 = output, MainRelay
gpio18 = output, AuxRelay
gpio21 = vss, FrontLeft
gpio22 = vss, FrontRight
```

See `configs/custom_example.conf` for a complete example.

## Running as a Service

Install and enable the systemd service:

```bash
sudo cp rpi_epic_canbus.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable rpi_epic_canbus
sudo systemctl start rpi_epic_canbus

# Check status
sudo systemctl status rpi_epic_canbus

# View logs
sudo journalctl -u rpi_epic_canbus -f
```

## GPIO Pin Mapping

GPIO pin assignments depend on the selected I/O mode. All assignments use BCM numbering.

### Available GPIOs
```
4, 5, 6, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 26, 27
```

### Reserved GPIOs (do not use)

**For SPI0 configuration (X735 compatible):**
| GPIO | Function |
|------|----------|
| 2, 3 | I2C SDA/SCL (for ADS1115 ADC) |
| 7, 8 | SPI0 CE1/CE0 (for CAN HAT) |
| 9, 10, 11 | SPI0 MISO/MOSI/SCLK (for CAN HAT) |
| 13, 22 | CAN HAT interrupt pins |
| 4, 17, 18 | X735 UPS power management |

**For SPI1 configuration (default, no X735):**
| GPIO | Function |
|------|----------|
| 2, 3 | I2C SDA/SCL (for ADS1115 ADC) |
| 16, 17 | SPI1 CE2/CE1 (for CAN HAT) |
| 19, 20, 21 | SPI1 MISO/MOSI/SCLK (for CAN HAT) |
| 13, 22 | CAN HAT interrupt pins |

### Default Pin Assignments (Balanced Mode)

### Digital Inputs (16 channels)
| Arduino Pin | RPi GPIO (BCM) |
|-------------|----------------|
| D22 | GPIO 5 |
| D23 | GPIO 6 |
| D24 | GPIO 13 |
| D25 | GPIO 19 |
| D26 | GPIO 26 |
| D27 | GPIO 16 |
| D28 | GPIO 20 |
| D29 | GPIO 21 |
| D30 | GPIO 12 |
| D31 | GPIO 7 |
| D32 | GPIO 8 |
| D33 | GPIO 25 |
| D34 | GPIO 24 |
| D35 | GPIO 23 |
| D36 | GPIO 18 |
| D37 | GPIO 15 |

### Slow GPIO Outputs (8 channels)
| Arduino Pin | RPi GPIO (BCM) |
|-------------|----------------|
| D39 | GPIO 2 |
| D40 | GPIO 3 |
| D41 | GPIO 4 |
| D42 | GPIO 17 |
| D43 | GPIO 27 |
| D47 | GPIO 22 |
| D48 | GPIO 10 |
| D49 | GPIO 9 |

### VSS Inputs (4 channels)
| Channel | RPi GPIO (BCM) |
|---------|----------------|
| Front Left | GPIO 14 |
| Front Right | GPIO 15 |
| Rear Left | GPIO 18 |
| Rear Right | GPIO 23 |

**Note:** Modify the pin arrays in `main.cpp` to match your actual wiring.

## EPIC CAN Protocol

Communication uses the EPIC_CAN_BUS protocol at 500 kbps:

| Frame Type | CAN ID | Direction | Payload |
|------------|--------|-----------|---------|
| Variable Request | 0x701 | TX | 4 bytes (hash) |
| Variable Response | 0x721 | RX | 8 bytes (hash + value) |
| Variable Set | 0x781 | TX | 8 bytes (hash + value) |

All multi-byte values use big-endian byte order.

## Performance

- **Loop frequency:** ~1000 Hz (1ms cycle time)
- **CAN bitrate:** 500 kbps
- **Input sampling:** Every 10ms
- **Fast TX (on change):** Every 25ms
- **Slow TX (heartbeat):** Every 500ms

## Troubleshooting

### CAN bus not working
```bash
# Check if CAN interface exists
ip link show can0

# Check dmesg for CAN errors
dmesg | grep -i can

# Check SPI is enabled
ls /dev/spidev*
```

### GPS not detected
```bash
# Check USB devices
lsusb

# Check serial ports
ls -la /dev/ttyUSB* /dev/ttyACM*

# Check if another process is using the port
fuser /dev/ttyUSB0
```

### ADC not working
```bash
# Scan I2C bus
sudo i2cdetect -y 1

# Check if I2C is enabled
ls /dev/i2c*
```

### GPIO permission denied
```bash
# Add user to gpio group
sudo usermod -aG gpio $USER

# Or run as root
sudo ./rpi_epic_canbus
```

## Differences from Arduino Version

| Feature | Arduino | Raspberry Pi |
|---------|---------|--------------|
| CAN Interface | MCP2515 via SPI | SocketCAN |
| GPS | Serial2 UART | USB Serial |
| ADC | Built-in 10-bit | ADS1115 16-bit via I2C |
| GPIO | Direct port access | libgpiod |
| VSS | Hardware interrupts | Polling (high-frequency) |
| PWM | Hardware PWM | Software (on/off only) |

## X735 UPS Compatibility

The Geekworm X735 UPS uses GPIO 4, 17, and 18 for power management. The Waveshare
2-CH CAN HAT+ uses GPIO 17 (SPI1 CE1) by default, causing a conflict.

### Solution: Solder CAN HAT to use SPI0

Move the 0-ohm resistor solder bridges on the CAN HAT:

| Function | FROM (SPI1) | TO (SPI0) |
|----------|-------------|-----------|
| MISO | GPIO 19 | GPIO 9 |
| MOSI | GPIO 20 | GPIO 10 |
| SCK | GPIO 21 | GPIO 11 |
| CS_0 | GPIO 17 | GPIO 8 |
| CS_1 | GPIO 16 | GPIO 7 |

Interrupt pins (GPIO 22, 13) remain unchanged.

### Stack Order

Pi 5 → X735 → CAN HAT

### config.txt for X735 + CAN HAT

```ini
dtparam=spi=on
dtoverlay=mcp2515,spi0-0,oscillator=16000000,interrupt=22
dtoverlay=mcp2515,spi0-1,oscillator=16000000,interrupt=13
```

## GPS with gpsd

To share GPS between multiple applications (e.g., TSDash and rpi_epic_canbus):

```bash
sudo apt install gpsd gpsd-clients
```

Configure `/etc/default/gpsd`:
```ini
DEVICES="/dev/ttyUSB0"
GPSD_OPTIONS="-n -G"
START_DAEMON="true"
USBAUTO="true"
```

Applications can connect via TCP to `localhost:2947`.

## License

Same as the original MEGA_EPIC_CANBUS project.

## Author

Ported by nwehle for the epicEFI project.

