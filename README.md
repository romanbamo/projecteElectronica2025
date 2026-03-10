# Physiological Monitoring System (ECG & Respiration)

A dual-platform bio-signal processing system that utilizes an ESP32 and an ADS1292R analog front-end to monitor ECG and Respiration in real-time. The system calculates Heart Rate Variability (HRV) and stress levels using FFT analysis and transmits data via Bluetooth Low Energy (BLE) to a custom Python dashboard.

## Key Features

* **High-Resolution Sampling:** 250 Hz sampling rate for precise QRS detection.
* **HRV & Stress Analysis:** On-board FFT calculation for LF/HF ratios every 90 seconds.
* **Dual-Channel Processing:** Simultaneous monitoring of Cardiac (ECG) and Respiratory signals.
* **Wireless Connectivity:** Data transmission via Bluetooth Low Energy (BLE) with optimized MTU chunking.
* **Python Dashboard:** Real-time GUI with Seaborn/Matplotlib integration for physiological monitoring.

---

## Hardware Architecture

The system utilizes the **ADS1292R** (24-bit ADC) to capture biopotentials.


### Pin Configuration (ESP32)

| ESP32 Pin | ADS1292R Function | Description |
| :--- | :--- | :--- |
| **GPIO 32** | CS | Chip Select (SPI) |
| **GPIO 18** | SCLK | Serial Clock |
| **GPIO 19** | MISO | Master In Slave Out |
| **GPIO 23** | MOSI | Master Out Slave In |
| **GPIO 21** | DRDY | Data Ready Interrupt (Active Low) |
| **GPIO 33** | START | Start Conversion |
| **GPIO 27** | RESET | Hardware Reset |

---

## Signal Processing Workflow

### 1. Data Acquisition
* **Frequency:** 250 Hz ($f_s$).
* **Interrupt-Driven:** The ESP32 triggers a read every 4ms upon the `DRDY` falling edge.
* **Data Packet:** 9 bytes total (3 Status, 3 CH1/Resp, 3 CH2/ECG).

### 2. HRV Analysis (Every 90s)
The system extracts stress metrics through the following steps:
1.  **R-Peak Detection:** Identifies peaks in the ECG buffer.
2.  **RR Interval Calculation:** Determines time between beats.
3.  **Linear Interpolation:** Resamples the RR series to **4 Hz** to create a uniform time series.

4.  **FFT Analysis:** Uses a Hamming window to compute power spectral density.
5.  **Bands:**
    * **LF (Low Frequency):** 0.04 – 0.15 Hz.
    * **HF (High Frequency):** 0.15 – 0.4 Hz.
6.  **Stress Index:** Calculated as the $LF/HF$ Ratio.

---

## Software Components

### Firmware (`main.cpp`)
* **SPI Management:** Configures ADS1292R registers (Gain, Sampling Rate, Respiration Modulation).
* **BLE Service:** UUID `4cbd1aa2-2e2b-4a51-9be0-a577dcf27eec`.
* **Memory Management:** Implements circular buffers and queues to handle high-speed data without blocking BLE notifications.

### Dashboard (`interface.py`)
* **Async BLE:** Uses `Bleak` for asynchronous notification handling.
* **Multi-plot GUI:** * **Top-Left:** Real-time ECG (30s window).
    * **Bottom-Left:** Respiration wave (60s window).
    * **Right:** SNS/PNS balance bars and Stress % indicator.


---

## Setup & Installation

### Firmware
1.  Open the project in **PlatformIO** or **Arduino IDE**.
2.  Install the following libraries:
    * `arduinoFFT`
    * `BLEDevice`
3.  Flash the code to your ESP32.

### Python Interface
1.  Install the required Python packages:
    ```bash
    pip install bleak matplotlib seaborn asyncio
    ```
2.  Find your ESP32's MAC address (displayed in the Serial Monitor) and update the `ADDRESS` variable in `interface.py`.
3.  Run the dashboard:
    ```bash
    python interface.py
    ```

---

## Project Structure
* `/src/main.cpp`: ESP32 Firmware.
* `/scripts/interface.py`: Python Visualization tool.
* `README.md`: Project documentation.

---

## Requirements
* **ESP32** (WROOM-32 recommended).
* **ADS1292R** Frontend module.
* **Python 3.8+** for the interface.
