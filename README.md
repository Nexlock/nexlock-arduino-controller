# 🔧 Arduino Controller - Smart Locker Hardware Module

<div align="center">
  <img src="https://img.shields.io/badge/ESP32-000000?style=for-the-badge&logo=espressif&logoColor=white" />
  <img src="https://img.shields.io/badge/Arduino-00979D?style=for-the-badge&logo=arduino&logoColor=white" />
  <img src="https://img.shields.io/badge/C++-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white" />
  <img src="https://img.shields.io/badge/WiFi-4285F4?style=for-the-badge&logo=wifi&logoColor=white" />
  <img src="https://img.shields.io/badge/WebSocket-010101?style=for-the-badge&logo=socket.io&logoColor=white" />
  <img src="https://img.shields.io/badge/NFC-FF6B35?style=for-the-badge&logo=nfc&logoColor=white" />
  <img src="https://img.shields.io/badge/Version-1.0.0-brightgreen?style=for-the-badge" />
</div>

## 🚀 What is Arduino Controller?

Arduino Controller is the **intelligent hardware module** that powers each NexLock smart locker unit! Built on the ESP32 platform with Arduino framework, it provides real-time communication, NFC authentication, occupancy detection, and secure locker control for the complete NexLock ecosystem.

### ✨ Key Features

- 🔌 **ESP32 Powered** - Dual-core processor with WiFi & Bluetooth
- 📡 **Real-time Communication** - WebSocket connection to NexLock Server
- 💳 **NFC Authentication** - RFID/NFC card and tag support
- 🔐 **Smart Lock Control** - Servo-based locking mechanism
- 📊 **Occupancy Detection** - Ultrasonic and magnetic sensors
- 🌐 **WiFi Provisioning** - Auto-discovery and easy setup
- 🔄 **OTA Updates** - Over-the-air firmware updates
- 🛡️ **Secure Communication** - Encrypted data transmission
- ⚡ **Low Power Mode** - Battery backup and sleep modes
- 📱 **Status LEDs** - Visual feedback and diagnostics

## 🛠️ Hardware Specifications

### Core Components

- **Microcontroller**: ESP32-WROOM-32D
- **Operating Voltage**: 3.3V (5V tolerant)
- **Flash Memory**: 4MB
- **RAM**: 520KB SRAM
- **Connectivity**: WiFi 802.11 b/g/n, Bluetooth 4.2
- **GPIO Pins**: 30 digital I/O pins
- **ADC**: 12-bit, 18 channels
- **PWM**: 16 channels

### Integrated Modules

```
Arduino Controller Board:
├── 🔌 ESP32-WROOM-32D    # Main microcontroller
├── 📡 WiFi Antenna       # 2.4GHz ceramic antenna
├── 💳 PN532 NFC Module   # 13.56MHz RFID/NFC reader
├── 📏 HC-SR04 Sensor     # Ultrasonic distance sensor
├── 🧲 Reed Switch        # Magnetic door sensor
├── 🔐 SG90 Servo Motor   # Lock mechanism control
├── 💡 WS2812B LED Strip  # Status indicator LEDs
├── 🔊 Passive Buzzer     # Audio feedback
├── 🔋 LiPo Battery       # 3.7V backup power
└── ⚡ TP4056 Charger     # USB-C charging circuit
```

### Pin Configuration

| Component | ESP32 Pin | Function |
|-----------|-----------|----------|
| PN532 NFC | GPIO 21 (SDA), GPIO 22 (SCL) | I2C Communication |
| HC-SR04 | GPIO 18 (Trig), GPIO 19 (Echo) | Ultrasonic Sensor |
| Reed Switch | GPIO 23 | Door Status Detection |
| Servo Motor | GPIO 16 | Lock Control |
| WS2812B LEDs | GPIO 17 | Status Indicators |
| Buzzer | GPIO 4 | Audio Feedback |
| Battery Monitor | GPIO 35 (ADC) | Battery Level |

## 🚀 Quick Start Guide

### 1. 📥 Hardware Setup

```
Required Components:
├── 🔧 Arduino Controller PCB
├── 📱 ESP32 Development Board
├── 🔌 USB-C Cable
├── 💳 NFC Tags/Cards (for testing)
├── 🔋 3.7V LiPo Battery (optional)
└── 📦 Locker Hardware (servo, sensors)
```

### 2. 🔧 Development Environment

```bash
# Install Arduino IDE or PlatformIO
# Add ESP32 board support

# Arduino IDE - Board Manager URLs:
https://dl.espressif.com/dl/package_esp32_index.json

# PlatformIO - Install ESP32 platform:
pio platform install espressif32
```

### 3. 📚 Required Libraries

```cpp
// Library dependencies in platformio.ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps = 
    adafruit/Adafruit PN532@^1.3.1
    fastled/FastLED@^3.6.0
    bblanchon/ArduinoJson@^6.21.4
    links2004/WebSockets@^2.4.1
    arduino-libraries/WiFi@^1.2.7
    esp32servo/ESP32Servo@^1.1.1
    adafruit/Adafruit NeoPixel@^1.11.0
```

### 4. ⚡ Flash the Firmware

```bash
# Using PlatformIO
pio run --target upload

# Using Arduino IDE
# Select: Tools > Board > ESP32 Arduino > ESP32 Dev Module
# Select: Tools > Port > (your ESP32 port)
# Click Upload button

# Using esptool directly
esptool.py --port COM3 write_flash 0x1000 firmware.bin
```

### 5. 🌐 WiFi Provisioning

```
1. Power on the ESP32 module
2. Look for WiFi network "NexLock_XXXXXX"
3. Connect with password "nexlock123"
4. Open browser to 192.168.4.1
5. Enter your WiFi credentials
6. Module will connect and register with server
```

## 📁 Project Structure

```
arduino_controller/
├── 📁 src/
│   ├── 📄 main.cpp              # Main program entry
│   ├── 📁 modules/              # Hardware modules
│   │   ├── wifi_manager.cpp
│   │   ├── nfc_reader.cpp
│   │   ├── servo_control.cpp
│   │   ├── sensor_manager.cpp
│   │   └── led_controller.cpp
│   ├── 📁 communication/        # Network communication
│   │   ├── websocket_client.cpp
│   │   ├── http_client.cpp
│   │   └── message_handler.cpp
│   ├── 📁 utils/               # Utility functions
│   │   ├── config.cpp
│   │   ├── logger.cpp
│   │   └── crypto.cpp
│   └── 📁 config/              # Configuration headers
│       ├── pins.h
│       ├── constants.h
│       └── settings.h
├── 📁 lib/                     # Custom libraries
├── 📁 data/                    # SPIFFS data files
│   ├── index.html             # WiFi setup page
│   ├── style.css
│   └── script.js
├── 📁 schematics/             # Hardware schematics
│   ├── nexlock_controller.sch
│   └── pcb_layout.brd
├── 📄 platformio.ini          # PlatformIO configuration
├── 📄 arduino_secrets.h       # Credentials template
└── 📄 README.md              # You are here! 👋
```

## 🔌 API Documentation

### 📡 WebSocket Communication

#### To NexLock Server (`ws://server:3000/module`)

```cpp
// Module announces availability (WiFi provisioning)
{
  "event": "module-available",
  "data": {
    "macAddress": "AA:BB:CC:DD:EE:FF",
    "deviceInfo": "NexLock Module v1.0",
    "version": "1.0.0",
    "capabilities": 3,
    "rssi": -45
  }
}

// Module registration after configuration
{
  "event": "register",
  "data": {
    "moduleId": "DEV_12345678",
    "firmware": "1.0.0",
    "lockers": ["L01", "L02", "L03"]
  }
}

// Heartbeat ping
{
  "event": "ping",
  "data": {
    "moduleId": "DEV_12345678",
    "uptime": 3600,
    "freeHeap": 234567,
    "batteryLevel": 85
  }
}

// Locker occupancy status
{
  "event": "locker-status",
  "data": {
    "moduleId": "DEV_12345678",
    "lockerId": "L01",
    "occupied": true,
    "timestamp": 1640995200,
    "sensorReading": 15.6
  }
}

// NFC validation request
{
  "event": "validate-nfc",
  "data": {
    "nfcCode": "abcd1234ef567890",
    "moduleId": "DEV_12345678",
    "lockerId": "L01",
    "rssi": -12
  }
}
```

#### From NexLock Server

```cpp
// Configuration after pairing
{
  "event": "module-configured",
  "data": {
    "moduleId": "DEV_12345678",
    "lockerIds": ["L01", "L02", "L03"],
    "serverUrl": "ws://nexlock.server.com:3000",
    "updateInterval": 30
  }
}

// Remote unlock command
{
  "event": "unlock",
  "data": {
    "lockerId": "L01",
    "action": "unlock",
    "duration": 5000,
    "timestamp": 1640995200
  }
}

// NFC validation response
{
  "event": "nfc-validation-result",
  "data": {
    "valid": true,
    "lockerId": "L01",
    "message": "Access granted",
    "unlockDuration": 5000
  }
}

// Firmware update notification
{
  "event": "ota-update",
  "data": {
    "version": "1.1.0",
    "url": "http://updates.nexlock.com/firmware.bin",
    "checksum": "sha256:abcd1234..."
  }
}
```

### 🔧 Configuration API

#### WiFi Provisioning Endpoints

```http
GET  /                          # Setup page
POST /wifi                      # Save WiFi credentials
GET  /status                    # Connection status
POST /reset                     # Factory reset
```

## ⚙️ Hardware Configuration

### 📐 Sensor Calibration

```cpp
// config/sensor_config.h
#define OCCUPANCY_THRESHOLD_CM  20      // Distance threshold for occupancy
#define SENSOR_READ_INTERVAL    1000    // Sensor reading interval (ms)
#define DEBOUNCE_TIME          100     // Switch debounce time (ms)
#define SERVO_LOCK_ANGLE       0       // Servo angle for locked position
#define SERVO_UNLOCK_ANGLE     90      // Servo angle for unlocked position
```

### 🎨 LED Status Indicators

```cpp
// LED Color Codes
LED_OFF         // Module offline
LED_BLUE        // WiFi connecting
LED_GREEN       // Connected and ready
LED_YELLOW      // Processing NFC
LED_RED         // Error state
LED_PURPLE      // OTA update mode
LED_WHITE       // WiFi provisioning mode
```

### 🔊 Audio Feedback

```cpp
// Buzzer Patterns
BEEP_SHORT      // 100ms beep (button press)
BEEP_SUCCESS    // 3 short beeps (successful operation)
BEEP_ERROR      // Long beep (error/denied)
BEEP_WARNING    // 2 medium beeps (warning)
```

## 🔧 Development

### 🛠️ Building the Firmware

```bash
# Using PlatformIO
cd arduino_controller
pio run                    # Compile
pio run --target upload    # Upload to device
pio device monitor         # Serial monitor

# Using Arduino IDE
# File > Open > main.cpp
# Sketch > Upload
# Tools > Serial Monitor
```

### 🧪 Testing Hardware

```cpp
// Test Mode Configuration
#define TEST_MODE           1           // Enable test mode
#define TEST_NFC_CARDS      1           // Test with predefined NFC codes
#define TEST_SERVO          1           // Test servo movements
#define TEST_SENSORS        1           // Test all sensors
#define SERIAL_DEBUG        1           // Enable serial debugging
```

### 📊 Debugging & Monitoring

```cpp
// Serial Debug Output
void debugPrint(String message) {
  #if SERIAL_DEBUG
    Serial.println("[DEBUG] " + String(millis()) + ": " + message);
  #endif
}

// Performance Monitoring
void printSystemStats() {
  Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("Uptime: %d seconds\n", millis() / 1000);
  Serial.printf("WiFi RSSI: %d dBm\n", WiFi.RSSI());
  Serial.printf("Battery: %.2fV\n", readBatteryVoltage());
}
```

## 🔌 Hardware Integration

### 🔐 Lock Mechanism

```cpp
class ServoController {
  private:
    Servo lockServo;
    int lockPin;
    
  public:
    void initialize(int pin);
    void lock();
    void unlock();
    bool isLocked();
    void testMovement();
};
```

### 💳 NFC Reader Integration

```cpp
class NFCReader {
  private:
    Adafruit_PN532 nfc;
    
  public:
    bool initialize();
    String readCard();
    bool isCardPresent();
    void setScanInterval(int ms);
};
```

### 📏 Occupancy Detection

```cpp
class OccupancySensor {
  private:
    int trigPin, echoPin;
    float threshold;
    
  public:
    void initialize(int trig, int echo);
    float getDistance();
    bool isOccupied();
    void calibrate();
};
```

## 📊 Power Management

### 🔋 Battery Monitoring

```cpp
// Battery voltage monitoring
float readBatteryVoltage() {
  int adcValue = analogRead(BATTERY_PIN);
  float voltage = (adcValue * 3.3) / 4095.0;
  return voltage * 2; // Voltage divider compensation
}

// Power saving modes
void enterDeepSleep(int seconds) {
  esp_sleep_enable_timer_wakeup(seconds * 1000000);
  esp_deep_sleep_start();
}
```

### ⚡ Power Consumption

| Mode | Current Draw | Duration | Notes |
|------|-------------|----------|-------|
| Active WiFi | 160-260mA | Variable | Normal operation |
| WiFi Connected | 20-30mA | Standby | Waiting for commands |
| Deep Sleep | 10µA | Extended | Motion wake-up |
| Charging | 500mA | 2-4 hours | USB-C charging |

## 🐛 Troubleshooting

### Common Hardware Issues

**🔴 ESP32 Not Connecting to WiFi**

```cpp
// Check WiFi credentials
void debugWiFi() {
  Serial.println("SSID: " + String(ssid));
  Serial.println("Signal Strength: " + String(WiFi.RSSI()));
  Serial.println("IP Address: " + WiFi.localIP().toString());
  Serial.println("MAC Address: " + WiFi.macAddress());
}
```

**🔴 NFC Reader Not Working**

```cpp
// Test NFC module
void testNFC() {
  if (nfc.getFirmwareVersion()) {
    Serial.println("NFC Module: OK");
  } else {
    Serial.println("NFC Module: FAILED - Check I2C connections");
  }
}
```

**🔴 Servo Not Moving**

```cpp
// Test servo functionality
void testServo() {
  servo.write(0);
  delay(1000);
  servo.write(90);
  delay(1000);
  servo.write(180);
  delay(1000);
}
```

**🔴 Sensor Readings Inconsistent**

```cpp
// Calibrate distance sensor
void calibrateSensor() {
  float readings[10];
  for(int i = 0; i < 10; i++) {
    readings[i] = getDistance();
    delay(100);
  }
  // Calculate average and set threshold
}
```

### Debug Mode

Enable comprehensive debugging:

```cpp
#define DEBUG_LEVEL 3           // 0=None, 1=Error, 2=Warning, 3=Info
#define DEBUG_WEBSOCKET 1       // Enable WebSocket debugging
#define DEBUG_NFC 1             // Enable NFC debugging
#define DEBUG_SENSORS 1         // Enable sensor debugging
```

## 🧪 Testing

### Hardware Tests

```cpp
// Run comprehensive hardware test
void runHardwareTest() {
  Serial.println("Starting Hardware Test...");
  
  // Test WiFi
  testWiFiConnection();
  
  // Test NFC
  testNFCReader();
  
  // Test Servo
  testServoMovement();
  
  // Test Sensors
  testOccupancySensor();
  testReedSwitch();
  
  // Test LEDs
  testLEDStrip();
  
  // Test Buzzer
  testBuzzer();
  
  Serial.println("Hardware Test Complete!");
}
```

### Field Testing

```cpp
// Simulate real-world usage
void fieldTest() {
  // Simulate NFC scan
  simulateNFCScan("test_card_123");
  
  // Test unlock sequence
  performUnlockSequence("L01");
  
  // Monitor for 60 seconds
  monitorSensors(60000);
  
  // Test lock sequence
  performLockSequence("L01");
}
```

## 🚀 Deployment

### Production Configuration

```cpp
// production_config.h
#define PRODUCTION_MODE     1
#define SERIAL_DEBUG        0
#define TEST_MODE          0
#define OTA_ENABLED        1
#define WATCHDOG_ENABLED   1
#define ENCRYPTION_ENABLED 1
```

### OTA Updates

```cpp
// Over-the-Air update handling
void handleOTAUpdate(String url, String checksum) {
  HTTPClient http;
  http.begin(url);
  
  int httpCode = http.GET();
  if(httpCode == 200) {
    Update.begin();
    // Write firmware and reboot
  }
}
```

### Factory Reset

```cpp
// Complete factory reset
void factoryReset() {
  // Clear EEPROM
  for(int i = 0; i < 512; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
  
  // Clear WiFi credentials
  WiFi.disconnect(true);
  
  // Restart in provisioning mode
  ESP.restart();
}
```

## 🤝 Contributing

We welcome hardware and firmware contributions!

### Development Guidelines

- Follow Arduino coding standards
- Use meaningful variable names
- Comment complex hardware interactions
- Test on actual hardware before submitting
- Update schematic diagrams for hardware changes

### Hardware Modifications

```
Before modifying hardware:
1. 📋 Document current pin assignments
2. 🔧 Test on breadboard first
3. 📐 Update PCB schematics
4. 🧪 Validate with existing firmware
5. 📝 Update documentation
```

## 📄 License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.

## 🏆 Credits

Created with ❤️ by the NexLock hardware team

### Hardware Components

- **Espressif ESP32** - Powerful WiFi/Bluetooth SoC
- **NXP PN532** - NFC/RFID communication
- **HC-SR04** - Ultrasonic distance sensor
- **WS2812B** - Addressable RGB LEDs
- **SG90** - Micro servo motor

## 📞 Support

Need help with hardware or firmware?

- 📧 **Email**: hardware@nexlock.com
- 💬 **Discord**: [Join our community](https://discord.gg/nexlock)
- 🐛 **Issues**: [GitHub Issues](https://github.com/your-username/arduino_controller/issues)
- 📖 **Documentation**: [Hardware Docs](https://docs.nexlock.com/hardware)

---

<div align="center">
  <h3>🌟 Star this repository if you found it helpful! 🌟</h3>
  <p>Made with 💖 and lots of ⚡</p>
  <p><strong>Building smart hardware for the future! 🔧</strong></p>
</div>
