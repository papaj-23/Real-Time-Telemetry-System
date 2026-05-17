# RTOS Telemetry Project

A minimal telemetry system built around two microcontrollers, a 433 MHz radio link, and a browser-based 3D visualization.

The project collects data from an MPU6050 sensor connected to an STM32L4, transmits it over radio using an nRF905 transceiver, receives it on an ESP32, forwards it over Wi-Fi as UDP, and exposes it to the browser through a Python WebSocket bridge. The visualization conditions the IMU data and renders the orientation of a 3D object. With level zeroing, it can also work as a simple digital level.

## Data Flow

```text
MPU6050 -> STM32L4 + FreeRTOS -> nRF905 433 MHz
        -> ESP32 + FreeRTOS -> UDP/Wi-Fi
        -> Python UDP server -> WebSocket
        -> JavaScript + Three.js -> 3D visualization
```

The radio payload is 18 bytes:

- 14 bytes of raw MPU6050 data: accelerometer, temperature, gyroscope
- 4 bytes of timestamp added by the STM32

On the ESP32 side, the RF frame is parsed into `int16_t` sensor values and a timestamp. The Python server converts the raw MPU6050 readings into physical units before broadcasting them to WebSocket clients.

## Repository Structure

```text
.
+-- STM32L4/              # transmitter firmware: STM32L432KC + MPU6050 + nRF905
+-- ESP32/telemetry_gateway/
|   +-- main/             # ESP-IDF application entry point
|   +-- components/       # Wi-Fi, UDP, and nRF905 components
+-- server/               # Python UDP -> WebSocket bridge
+-- visualization/        # browser-based 3D visualization
```

### `STM32L4/`

STM32L4 firmware built with CMake. It contains startup code, STM32 HAL, FreeRTOS, the MPU6050 driver, and the nRF905 integration.

Main directories:

- `Core/main/` - system, peripheral, and FreeRTOS scheduler initialization
- `Core/mpu6050/` - MPU6050 data acquisition task
- `Core/nrf905/` - nRF905 initialization and radio transmission logic
- `Lib/mpu6050_lib/` - custom MPU6050 library written from scratch: [library README](STM32L4/Lib/mpu6050_lib/README.md)
- `Lib/nrf905_lib/` - integrated `libdriver`-based nRF905 driver
- `FreeRTOS/` - local FreeRTOS sources
- `Drivers/` - STM32 HAL/CMSIS
- `cmake/` - toolchain configuration

In this part of the project, MPU6050 data is acquired over I2C/DMA, queued, extended with a timestamp, and sent through the nRF905 radio link.

### `ESP32/telemetry_gateway/`

ESP-IDF application acting as a radio-to-network gateway.

Main elements:

- `main/telemetry_gateway.c` - starts Wi-Fi, the nRF905 task, and the UDP client
- `components/nrf905/` - nRF905 receiver-side handling
- `components/wifi/wifi_connect.c` - Wi-Fi station connection logic
- `components/wifi/udp_client.c` - RF frame parsing and UDP transmission to the PC

The ESP32 receives RF packets, parses the payload, and forwards the data to a configured UDP host on the local network.

### `server/`

Python server that receives UDP packets and exposes the data through WebSocket.

- `udp_server.py` - asynchronous `asyncio` server, UDP `0.0.0.0:16161`, WebSocket `0.0.0.0:8765`

The server validates frame length, unpacks the `<7hI` payload format, converts raw MPU6050 values into physical units, and broadcasts samples to all connected WebSocket clients.

### `visualization/`

Browser frontend for live telemetry visualization.

- `index.html` - view structure
- `styles.css` - interface styling
- `app.js` - WebSocket client, data conditioning, and Three.js scene

The visualization uses Three.js, estimates roll/pitch/yaw, compensates gyroscope bias, and displays the object orientation in real time.

## RTOS Mechanisms

FreeRTOS is used on both microcontrollers.

On STM32L4:

- statically created tasks for LED, MPU6050, and nRF905 handling
- static memory allocation for the idle task
- `mpu_queue_handle` queue between the MPU6050 task and the nRF905 task
- task notifications for interrupt-driven synchronization: MPU6050 INT, I2C RX complete, nRF905 DR
- dynamically created FreeRTOS queue for MPU6050 frames

On ESP32:

- nRF905 task receiving `DR`, `AM`, and `CD` events
- UDP client task forwarding samples to the PC
- task notifications from GPIO ISRs to the nRF905 task
- `rf_queue_handle` queue between the RF receiver and UDP client
- event group for Wi-Fi connection state: connected or failed

## Technologies

- STM32L432KC, STM32 HAL, CMake, OpenOCD
- ESP32, ESP-IDF
- FreeRTOS
- MPU6050: accelerometer, gyroscope, thermometer
- nRF905 433 MHz
- Python, `asyncio`, UDP, WebSocket
- JavaScript, Three.js

## Status

The project is currently fully working and functional. Further development is planned for the future.
