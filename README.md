# CubeSat Telemetry Model

A CubeSat scale model built around an ESP32 that collects sensor data and shows it live on a ground-station web dashboard and an onboard OLED display.

## Features
- Live telemetry from IMU, pressure, GPS and gas sensors
- Ground-station web dashboard for real-time monitoring
- Onboard OLED display for local readout
- Custom frame, designed and 3D-printed in-house

## Hardware
| Component | Purpose |
|-----------|---------|
| ESP32 | Main controller and Wi-Fi telemetry link |
| IMU [model, e.g. MPU6050] | Orientation and motion |
| Pressure sensor [model, e.g. BMP280] | Pressure / altitude |
| GPS module [model, e.g. NEO-6M] | Position data |
| Gas sensor [model, e.g. MQ-series] | Air quality / gas detection |
| OLED display [e.g. 0.96" I2C SSD1306] | Local data display |
| 3D-printed frame | CubeSat-scale structure |

## How It Works
1. The ESP32 reads all sensors over I2C, UART and analog (ADC) interfaces.
2. Readings are shown on the OLED display.
3. The ESP32 sends the data to the ground-station dashboard, which updates in real time in the browser.

## Getting Started
1. Clone the repository:
   ```
   git clone https://github.com/adityarai95369/[repo-name].git
   ```
2. Open the firmware in Arduino IDE and install the required libraries: [list libraries].
3. Select the ESP32 board, set your Wi-Fi credentials in the code, and upload.
4. Open the dashboard to see live data.


## Project Structure
```
/firmware    ESP32 code
/dashboard   Ground-station web dashboard
/cad         3D model files for the frame
/images      Photos and screenshots
```

