
[README.md](https://github.com/user-attachments/files/21709752/README.md)

# ESPHome Garage Controller (Single & Double Door)

This project is a **complete ESPHome replacement** for the original Arduino-based garage door controller firmware.
It supports **both single** and **double** garage door setups, using a single YAML file with `substitutions` to switch between configurations.

---

## 🚀 Features
- **Single or Double Door Support** – set in `substitutions`.
- **Direct Home Assistant API Integration** – no MQTT broker required.
- **Temperature Offset** – matches original Arduino (-0.5°C for single, +0.5°C for double).
- **BME280 Sensor** – temperature, humidity, and pressure readings.
- **Light Level Detection** – via ADC input (0–100% scaled).
- **Motion-Triggered Garage Light** – light turns on when motion detected below brightness threshold; auto-off after 30s.
- **Relay-Controlled Garage Door** – active LOW, 600ms pulse for open/close/stop.
- **Door Open/Closed Sensors** – reed switches with combined door state sensor.
- **External Hardware Watchdog Support** – pulses GPIO0 to keep external watchdog happy.
  - Fast heartbeat during boot
  - Normal heartbeat when running
  - Fast heartbeat during Wi-Fi reconnects
- **Wi-Fi Status Feedback** – blinks garage light on connect.
- **Online Status Binary Sensor** – shows if controller is connected to HA.
- **Uptime Sensor** – reports in hours since last reboot.
- **Manual Restart Button** – restart ESP from HA.

---

## 🛠 Hardware Pinout

| GPIO  | Name              | Function |
|-------|------------------|----------|
| GPIO0 | `WATCHDOG_PIN`   | External hardware watchdog heartbeat |
| GPIO12| `DOOR_CLOSE_PIN` | Reed switch – door closed detection |
| GPIO13| `DOOR_OPEN_PIN`  | Reed switch – door open detection |
| GPIO14| `DOOR_RELAY_PIN` | Relay for garage door opener (active LOW) |
| GPIO15| `LIGHT_RELAY_PIN`| Relay for garage light (active LOW) |
| GPIO16| `MOTION_PIN`     | PIR motion sensor |
| A0    | `LIGHT_DETECTION_PIN` | Analog light level sensor |
| GPIO4 | I²C SDA          | BME280 sensor |
| GPIO5 | I²C SCL          | BME280 sensor |

---

## 🔌 Wiring Diagram

```
        +-------------------+
        |   ESP8266 Board   |
        |                   |
 SDA ---| GPIO4         3V3 |--- VCC (BME280, PIR, Reed)
 SCL ---| GPIO5         GND |--- GND (all sensors, relays)
 WD  ---| GPIO0         VIN |--- +5V (Relays, PIR)
 D-OPEN-| GPIO13            |
 D-CLOS-| GPIO12            |
 RELAY1-| GPIO14            |
 RELAY2-| GPIO15            |
 MOTION-| GPIO16            |
 ADC ---| A0                |
        +-------------------+
```

---

## ⚙️ Substitutions

You can easily switch between single and double door by changing:

```yaml
substitutions:
  garage_type: "single"          # "single" or "double"
  garage_name: "Garage Single"   # Friendly name
  temp_offset: "-0.5"            # "-0.5" for single, "0.5" for double
  light_on_threshold: "75"       # Light % below which motion triggers light
```

---

## 📦 Included Files
- `garage-single.yaml` – ready to flash for single-door setup.
- `garage-double.yaml` – ready to flash for double-door setup.

---

## 🔄 Behavior Summary
- Door relay pulses for 600ms on open/close/stop commands.
- Light turns on when motion detected if brightness < `light_on_threshold`.
- Light turns off after 30 seconds with no motion.
- External watchdog pin pulses to prevent reset unless firmware stops responding.
- Garage light blinks briefly when Wi-Fi connects.
- Binary sensor shows online/offline status in Home Assistant.
- Uptime (in hours) resets after watchdog or manual restart.

---

## 🏠 Home Assistant Entities
| Entity Example                  | Description |
|----------------------------------|-------------|
| `cover.garage_single_door`       | Garage door control |
| `binary_sensor.garage_single_door_state` | Combined open/closed door status |
| `binary_sensor.garage_single_online`     | Online/offline status |
| `sensor.garage_single_temperature`       | Temperature (°C, offset applied) |
| `sensor.garage_single_humidity`          | Humidity (%) |
| `sensor.garage_single_pressure`          | Pressure (hPa) |
| `sensor.garage_single_light_level`       | Light level (%) |
| `binary_sensor.garage_single_motion`     | Motion detected |
| `switch.garage_single_light`             | Garage light control |
| `sensor.garage_single_uptime`            | Uptime in hours |
| `button.garage_single_restart`           | Restart ESP |

---

## 🖥 How to Flash
1. Copy the YAML file (`garage-single.yaml` or `garage-double.yaml`) into your ESPHome configuration folder.
2. Update `YOUR_WIFI`, `YOUR_WIFI_PASSWORD`, and `YOUR_API_KEY` in the YAML.
3. From ESPHome Dashboard, click **Install** and flash your ESP8266.
4. Add the device to Home Assistant via ESPHome integration.
