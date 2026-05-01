# a-Dual-Mode-Intelligent-Bristlebot

## 1. Intelligent Bristlebot: Dual-Mode Control Firmware using ESP-IDF

## 2. Introduction
This repository contains the core firmware for an Intelligent Bristlebot, developed as part of a 3rd Year Individual Project (EEEN30330) at The University of Manchester. The system is built on the Espressif ESP32 microcontroller using the native ESP-IDF framework, allowing for robust, low-level hardware control and FreeRTOS task management.

The robot operates utilizing a Shared Control Architecture (SCA). It seamlessly integrates two modes: a manual remote-control mode via Bluetooth Low Energy (BLE) GATT server, and an autonomous navigation mode utilizing an I2C-based TCS34725 colour sensor. Unique to this design, propulsion is achieved through a custom 5-layer tape-wrapped ERM motor (propeller), while steering is dynamically controlled via a vertically mounted servo arm.

## 3. Context

*   **Figure 1:** [System Architecture Diagram](image/Figure 1.png)
*   **Figure 2:** [Completed Bristlebot Prototype](image/Figure 2.png)


The firmware handles real-time priority switching. For instance, manual BLE inputs drive the robot under normal conditions, but if the sensor detects a predefined environmental boundary (e.g., a green or red line), the autonomous logic pre-empts the manual control to execute evasive manoeuvres.

## 4. User Installation Instructions
Since this project is developed in pure C using the ESP-IDF, you must set up the Espressif development environment.

1.  **Install ESP-IDF**: Follow the official [Espressif ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/) to install the framework (v4.4 or v5.x recommended).
2.  **Clone the Repository**:
    ```bash
    git clone [https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git](https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git)
    cd YOUR_REPO_NAME
    ```
3.  **Hardware Wiring**:
    *   **I2C Colour Sensor (TCS34725)**: SDA -> GPIO 21, SCL -> GPIO 19.
    *   **Servo Motor (PWM)**: Control pin -> GPIO 4.
    *   **Propulsion Motors**: GPIO 22 & GPIO 23 (ensure logic signals are routed through the BJT/MOSFET driver board, not directly to the motors).

## 5. How to Run the Code
1.  **Build the Project**: Open your ESP-IDF command prompt/terminal, navigate to the project directory, and run:
    ```bash
    idf.py build
    ```
2.  **Flash to ESP32**: Connect the ESP32 via USB and flash the compiled binary:
    ```bash
    idf.py -p (YOUR_PORT) flash monitor
    ```
3.  **Operation via BLE**:
    *   Download a BLE scanner application on your smartphone (e.g., *nRF Connect* or *LightBlue*).
    *   Scan and connect to the device named **`my_esp32`**.
    *   Locate the characteristic with write properties (UUID ends in `FF02` or `FF03`) and send the following UTF-8 string commands:
        *   `f`: Drive Forward
        *   `l`: Turn Left
        *   `r`: Turn Right
        *   `s`: Stop completely
        *   `e`: Enable propulsion fan only

## 6. More Technical Details
*   **SCA Priority Logic**: The `updateColor()` function continuously polls the TCS34725 via I2C at 100kHz. If a `RED` color is classified, the robot immediately executes a right turn (`turn_right_action()`). If a `GREEN` color is detected, the firmware initiates a 2-second vibration pause (`COLOR_OVERRIDE_GREEN_PAUSE`) using `esp_timer_get_time()` before executing a turn, ensuring physical momentum is neutralized before changing direction.
*   **Hardware PWM (LEDC)**: To minimize CPU overhead, the steering servo is controlled using the ESP32's built-in LEDC peripheral (`ledc_timer_config_t`). It outputs a stable 50Hz frequency on Timer 0, with duty cycles dynamically switching between 563, 614, and 665 to dictate the servo's physical angle.
*   **BLE GATT Server**: The Bluetooth stack is configured using `esp_gatts_api.h`. It utilizes a custom profile table (`heart_rate_profile_tab` architecture adapted for motor control) to handle asynchronous write events without blocking the main sensing loop.

## 7. Known Issues / Future Improvements
*   **FreeRTOS Task Separation**: Currently, the I2C sensor polling and the BLE command evaluation run sequentially within the `app_main` while-loop. A future iteration should separate the BLE GATT event handler and the I2C sensor logic into distinct FreeRTOS tasks (`xTaskCreate`), utilizing semaphores or event groups to pass state data. This would improve real-time responsiveness.
*   **Propulsion Voltage Drop**: High current draw from the tape-wrapped propeller motor occasionally causes minor voltage sags. Implementing an ADC-based battery voltage monitor to dynamically adjust the PWM duty cycle would stabilize steering at low battery levels.
*   **Sensor False Positives**: Rapid changes in ambient laboratory lighting can cause the colour thresholding (`classify_color()`) to misread boundaries. A temporal moving-average filter for the RGB values should be implemented.
