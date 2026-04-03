# 🖥️ Smart Home Monitor — HPS C Supervisor

Part of the **IoT Smart Home Monitor** project built on a DE10-Nano (Intel Cyclone V SoC) for the *Electronics for Embedded Systems* course at Politecnico di Torino (A.Y. 2025–2026).

This component runs on the ARM Cortex-A9 HPS (Hard Processor System) under embedded Linux. It polls sensor data from the FPGA fabric via the Lightweight AXI Bridge, compensates raw readings using hardware calibration coefficients, publishes to an MQTT broker over WiFi (through an ESP32), and sends SMS alerts via a SIM800L GSM module.

---

## 📋 Table of Contents

- [Architecture Overview](#architecture-overview)
- [File Descriptions](#file-descriptions)
- [Hardware Connections](#hardware-connections)
- [Configuration](#configuration)
- [Build & Deploy](#build--deploy)
- [Runtime Behavior](#runtime-behavior)
- [Dependencies](#dependencies)

---

## 🏗️ Architecture Overview

```
FPGA Fabric (Avalon-MM @ 0xFF200000)
        │  LW AXI Bridge (/dev/mem)
        ▼
  main_supervisor.c  ──► esp32_handler.c ──► ESP32 (AT-MQTT) ──► 🌐 Broker
        │
        └──────────────► sim800l_handler.c ──► SIM800L UART ──► 📱 SMS
        │
        └──────────────► ufm_storage.c ──► On-chip RAM (event log)
```

All peripheral access is done via direct memory-mapped I/O through `/dev/mem`. No kernel drivers are required.

---

## 📁 File Descriptions

| File | Purpose |
|------|---------|
| `main_supervisor.c` | 🔄 Main loop: sensor polling (1 Hz), BME280 compensation, alarm detection, MQTT publish (every 5 s), SMS on alarm |
| `smart_home.h` | ⚙️ Central configuration: register offsets, threshold defaults, MQTT/WiFi settings, pre-loaded calibration coefficients |
| `esp32_handler.c` | 📡 Drives ESP32 over HPS UART1 (`/dev/ttyS1`, 115200 baud) using Espressif AT-command firmware for WiFi and MQTT |
| `sim800l_handler.c` | 📱 Drives SIM800L via direct MMIO on the Altera UART at offset 0x80, 19200 baud — no Linux tty driver needed |
| `ufm_storage.c` | 💾 Appends 32-byte alarm records to FPGA on-chip RAM (8 KB / 256 records). Magic header `0xCAFEBABE` marks valid entries |
| `ufm_erase.c` | 🗑️ Standalone utility to clear the on-chip event log |
| `Makefile` | 🔨 Cross-compilation for ARM Linux (`arm-linux-gnueabihf-gcc -std=c99 -O2`) |

---

## 🔌 Hardware Connections

All peripherals connect through the **JP1** (GPIO_0) 40-pin expansion header:

| Function | JP1 Pins | FPGA Pins | Notes |
|----------|----------|-----------|-------|
| 🌡️ BME280 SDA/SCL | 1 / 2 | PIN_V12 / PIN_E8 | 4.7 kΩ pull-ups on Waveshare module |
| 🔊 MCP3008 CLK/MOSI/MISO/CS | 3–6 | PIN_W12, D11, D8, AH13 | 10-bit 8-ch SPI ADC |
| 🚶 PIR sensor 1 / 2 | 7 / 8 | PIN_AF7 / AH14 | Active-high, 5V powered |
| 📱 UART SIM800L TX/RX | 19 / 20 | PIN_D12 / AD20 | 3.3 V compatible, 19200 baud |
| 📡 UART ESP32 TX/RX | 21 / 22 | PIN_C12 / AD17 | HPS UART1, 115200 baud |
| ⚡ 3.3 V / GND | 29 / 30 | — | Sensor supply |

> ⚠️ **Note:** The BME280 SDA/SCL lines go to FPGA GPIO pins, **not** to HPS I²C. Calibration coefficients are therefore pre-loaded in `smart_home.h` (use the tool in `tools/` to extract them once per chip).

---

## ⚙️ Configuration

All project-wide settings live in `smart_home.h`. Before compiling, fill in your own values:

```c
/* 🌐 WiFi / MQTT */
#define WIFI_SSID           "YOUR_WIFI_SSID"
#define WIFI_PASSWORD       "YOUR_WIFI_PASSWORD"
#define MQTT_BROKER         "YOUR_BROKER_IP_OR_HOST"
#define MQTT_PORT           1883
#define MQTT_USER           "YOUR_MQTT_USER"
#define MQTT_PASS           "YOUR_MQTT_PASS"
#define MQTT_CLIENT_ID      "DE10Nano_SmartHome"

/* 📱 SMS */
#define ALERT_PHONE_NUMBER  "+XXXXXXXXXXX"   /* E.164 format */
#define SMS_COOLDOWN_SEC    300              /* Min seconds between SMS bursts */

/* 🗺️ FPGA peripheral base (LW Bridge) */
#define LW_BRIDGE_BASE      0xFF200000UL
#define LW_BRIDGE_SPAN      0x00200000UL

/* 🌡️ Alarm thresholds (raw ADC space — recalculate if chip changes) */
#define DEFAULT_TEMP_LOW_RAW    0x7800U   /* ≈ 15 °C */
#define DEFAULT_TEMP_HIGH_RAW   0x8800U   /* ≈ 38 °C */
#define DEFAULT_LIGHT_THRESH    100U      /* 0–1023 ADC counts */

/* 🔬 BME280 factory calibration (run tools/read_bme280_calib to get these) */
#define BME280_PRECAL_T1    28267U
#define BME280_PRECAL_T2    26539
#define BME280_PRECAL_T3    50
/* ... (pressure and humidity coefficients follow) */
```

> 🔒 **Security:** Do not commit real credentials to a public repository. Consider moving secrets to a separate `config.h` excluded via `.gitignore`.

---

## 🚀 Build & Deploy

### Prerequisites

- ARM cross-compiler: `arm-linux-gnueabihf-gcc`
- Target: DE10-Nano running embedded Linux with `/dev/mem` access

### 🔨 Build

```bash
cd "c code/"
make
# Produces: smarthome
```

### 📤 Deploy

```bash
scp smarthome root@<board-ip>:/home/root/
ssh root@<board-ip>
```

### ▶️ Run

```bash
# Must run as root (required for /dev/mem)
sudo ./smarthome
```

To clear the on-chip event log:

```bash
make ufm_erase
sudo ./ufm_erase
```

---

## 🔄 Runtime Behavior

1. **🟢 Startup**: Maps LW bridge, reads BME280 calibration, back-solves temperature thresholds to raw ADC values, writes thresholds to FPGA alarm comparator, initializes ESP32 (WiFi + MQTT) and SIM800L (GSM).
2. **⏱️ Every 1 second**: Reads all sensor registers, compensates BME280 raw values (Bosch datasheet Annex 4.2.3), checks alarm flags.
3. **📡 Every 5 seconds**: Publishes full sensor JSON to `smarthome/sensors` via MQTT.
4. **🚨 On alarm edge**: Publishes to `smarthome/alarms`, sends SMS (subject to 5-minute cooldown), appends record to on-chip log.

### 📨 MQTT Payload Format

**`smarthome/sensors`** (published every 5 s):
```json
{
  "temperature": 23.45,
  "pressure": 1012.3,
  "humidity": 55.2,
  "light": 512,
  "heating": 256,
  "sound": 128,
  "pir1": 0,
  "pir2": 1,
  "alarms": 0
}
```

**`smarthome/alarms`** (published on alarm event):
```json
{
  "alarm_flags": 4,
  "description": "MOTION_DETECTED"
}
```

---

## 📦 Dependencies

- Standard C library (`<stdio.h>`, `<stdlib.h>`, `<fcntl.h>`, `<sys/mman.h>`, `<unistd.h>`, `<time.h>`)
- No external libraries — all sensor protocols implemented in-house over MMIO
- Target OS: Embedded Linux (tested on DE10-Nano Angstrom / Yocto image)
