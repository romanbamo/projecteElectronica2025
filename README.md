# Physiological Monitoring System (ECG & Respiration)

A dual-platform bio-signal processing system that utilizes an ESP32 and an ADS1292R analog front-end to monitor ECG and Respiration in real-time. The system calculates Heart Rate Variability (HRV) and stress levels using FFT analysis and transmits data via Bluetooth Low Energy (BLE) to a custom Python dashboard.

## Key Features

* **High-Resolution Sampling:** 250 Hz sampling rate for precise QRS detection.
* **HRV & Stress Analysis:** On-board FFT calculation for LF/HF ratios every 120 seconds using a larger 5000-sample buffer for better frequency resolution.
* **Voltage Scaling:** Raw 24-bit ADC data is converted to Millivolts (mV) on-board using a 2.42V reference.
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
* **Sensitivity:** Data is processed in 24-bit signed integers and converted via:
    $$Voltage (mV) = \frac{Raw}{2^{23}-1} \times \frac{V_{ref}}{Gain} \times 1000$$
* **Smoothing:** The respiration signal includes a delta-check to prevent sudden spikes from corrupting the real-time feed.
* **Hybrid Acquisition:** * **Interrupt-Flagged:** The `DRDY` falling edge (every 4ms) sets a hardware flag (`dataReadyFlag`).
    * **Software-Timed:** The main loop now checks this flag inside a `samplingTime` window (250Hz) to ensure the SPI read coincides with your internal clock cycles.
* **Data Packet:** 9 bytes total (3 bytes Status, 3 bytes CH1/Respiration, 3 bytes CH2/ECG).

### 2. HRV Analysis (Every 120s)
1.  **Peak Detection:** Uses a fixed threshold (`LLINDAR = 5872025`) to identify R-peaks.
2.  **RR Interpolation:** Intervals are resampled at **4 Hz** using linear interpolation.
3.  **Spectral Analysis (FFT):**
    * **LF Band:** 0.04 – 0.15 Hz (Sympathetic/Parasympathetic).
    * **HF Band:** 0.15 – 0.4 Hz (Vagal/Parasympathetic).
    * **Stress Index:** Calculated as the $LF/HF$ Ratio.
---

## BLE Transmission Protocol

* **Service UUID:** `4cbd1aa2-2e2b-4a51-9be0-a577dcf27eec`
* **Characteristic UUID:** `7a6ffc80-ef27-4a4d-a8a6-56d93f8feff3`
* **Data Format:** Strings are sent in CSV format, terminated by a semicolon:
    `ECG_mV, Resp_mV, LF, HF, Stress;`
* **Chunking:** Large data strings are split into smaller packets with a 20ms delay to ensure compatibility with mobile BLE stacks.

---

## Configuration & Calibration (`main.cpp`)

### Gain Adjustment
Registers `CH1SET` and `CH2SET` are configured for **Gain 6** (view ADS1292R Datasheet). To change sensitivity, modify the `setup()` function:
* `0b00000000` -> Gain 6
* `0b00100000` -> Gain 2
* `0b01100000` -> Gain 12

### Peak Threshold
If R-peaks are not being detected in the Python dashboard, adjust the `LLINDAR` constant in `main.cpp`. This value represents the raw ADC amplitude required to trigger a "peak" event.

---

## Dashboard (`interface.py`)
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
