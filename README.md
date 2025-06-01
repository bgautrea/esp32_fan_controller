# ESP32 Smart Fan Controller

A WiFi-enabled smart fan controller built on ESP32 that provides web-based PWM fan speed control, real-time RPM monitoring, temperature-based automation, and over-the-air (OTA) firmware updates.

![ESP32 Fan Controller](https://img.shields.io/badge/ESP32-Fan%20Controller-blue) ![Arduino](https://img.shields.io/badge/Arduino-IDE-00979D) ![License](https://img.shields.io/badge/License-MIT-green)

## ✨ Features

- 🌐 **Web-based Control Interface** - Control fans from any device on your network
- ⚡ **PWM Speed Control** - Variable speed control for 4-pin PWM fans (0-100%)
- 📊 **Real-time RPM Monitoring** - Live tachometer readings for each fan
- 🌡️ **Temperature-based Automation** - Automatic fan speed adjustment based on DHT22 sensor
- 🔄 **OTA Updates** - Upload new firmware wirelessly via Arduino IDE
- 🎛️ **Manual Override** - Direct PWM control for testing and debugging
- 🔒 **Secure Configuration** - Credentials stored in separate header file

## 🛠️ Hardware Requirements

### Components
- **ESP32 Development Board** (tested with ESP32 DevKitV1)
- **4-pin PWM Fans** (tested with Noctua NF-F12)
- **DHT22 Temperature Sensor**
- **12V Power Supply** (for fans)
- **Breadboard and Jumper Wires**

### Wiring Diagram

| Component | ESP32 Pin | Wire Color | Notes |
|-----------|-----------|------------|-------|
| **Fan 1 PWM** | D25 | Blue (Noctua) | PWM signal |
| **Fan 1 Tach** | D32 | Green (Noctua) | RPM feedback |
| **Fan 2 PWM** | D26 | Blue | PWM signal |
| **Fan 2 Tach** | D33 | Green | RPM feedback |
| **Fan 3 PWM** | D27 | Blue | PWM signal |
| **Fan 4 PWM** | D14 | Blue | PWM signal |
| **DHT22 Data** | D4 | - | Temperature sensor |
| **DHT22 VCC** | 3.3V | - | Power |
| **DHT22 GND** | GND | - | Ground |
| **Fan VCC** | External 12V | Yellow | Fan power |
| **Fan GND** | GND | Black | Common ground |

> ⚠️ **Important**: Noctua fans use non-standard wire colors! Blue = PWM, Green = Tach (opposite of most fans)

## 📋 Software Requirements

- **Arduino IDE** (1.8.x or 2.x)
- **ESP32 Arduino Core** (3.0+)

### Required Libraries
```
ESPAsyncWebServer
DHT sensor library
```

Install via Arduino IDE Library Manager:
1. `Tools` → `Manage Libraries`
2. Search and install:
   - "ESPAsyncWebServer" by Hristo Gochkov
   - "DHT sensor library" by Adafruit

## 🚀 Installation

### 1. Clone Repository
```bash
git clone https://github.com/yourusername/esp32-fan-controller.git
cd esp32-fan-controller
```

### 2. Configure Credentials
Create a `secrets.h` file in the project directory:

```cpp
// secrets.h - Keep this file out of version control!
#ifndef SECRETS_H
#define SECRETS_H

// WiFi Credentials
#define WIFI_SSID "YourWiFiName"
#define WIFI_PASSWORD "YourWiFiPassword"

// OTA Password
#define OTA_PASSWORD "YourOTAPassword"

#endif
```

### 3. Upload Firmware
1. Open `esp32-fan-controller.ino` in Arduino IDE
2. Select your ESP32 board: `Tools` → `Board` → `ESP32 Dev Module`
3. Select the correct port: `Tools` → `Port`
4. Click **Upload**

### 4. Find Your Device
Check the Serial Monitor (115200 baud) for the IP address:
```
WiFi connected!
IP address: 192.168.1.100
```

## 📱 Usage

### Web Interface
Navigate to your ESP32's IP address in a web browser:
```
http://192.168.1.100
```

### Features Available:
- **Fan Speed Control**: Adjust intake and exhaust fan speeds (0-255)
- **Auto Mode**: Enable temperature-based automatic control
- **RPM Monitoring**: Real-time display of fan speeds
- **Manual PWM Testing**: Direct PWM value input for troubleshooting

### Temperature Thresholds (Auto Mode)
| Temperature | Fan Speed |
|-------------|-----------|
| < 25°C | 20% (50 PWM) |
| 25-30°C | 39% (100 PWM) |
| 30-35°C | 71% (180 PWM) |
| > 35°C | 100% (255 PWM) |

## 🔄 OTA Updates

After initial USB upload, you can update wirelessly:

1. In Arduino IDE: `Tools` → `Port` → `Network Ports` → `ESP32-FanController`
2. Enter your OTA password when prompted
3. Upload normally - it goes over WiFi!

## 🔧 Configuration

### PWM Settings
- **Frequency**: 25 kHz (adjustable in code)
- **Resolution**: 8-bit (0-255 values)
- **Pins**: D25, D26, D27, D14

### Tachometer Settings  
- **Pins**: D32, D33, D13 (D34 skipped - input only)
- **Trigger**: Falling edge
- **Calculation**: 2 pulses per revolution (standard for PC fans)

## 🐛 Troubleshooting

### Fan Not Responding to PWM
1. **Check wiring** - Ensure PWM wire is connected to correct pin
2. **Verify fan compatibility** - Must be 4-pin PWM fan
3. **Try different frequency** - Change `PWM_FREQ` from 25000 to 1000
4. **Check power** - Fans need 12V power supply

### No RPM Readings
1. **Check tachometer wiring** - Ensure tach wire is connected
2. **Verify pin compatibility** - Avoid input-only pins (34, 35)
3. **Check pull-up resistors** - Some pins need external pull-ups

### WiFi Connection Issues
1. **Verify credentials** in `secrets.h`
2. **Check signal strength** - ESP32 might be too far from router
3. **Monitor Serial output** for connection status

### OTA Upload Fails
1. **Check network connectivity** - ESP32 and computer on same network
2. **Verify OTA password** - Must match `secrets.h`
3. **Restart ESP32** - Sometimes helps with network discovery

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **Noctua** for excellent PWM fans with clear documentation
- **ESP32 Community** for comprehensive Arduino core
- **Adafruit** for reliable sensor libraries

## 📞 Support

If you encounter issues:
1. Check the troubleshooting section above
2. Review the Serial Monitor output
3. Open an issue on GitHub with:
   - Hardware setup details
   - Serial Monitor output
   - Steps to reproduce the problem

---

**Made with ❤️ for the maker community**
