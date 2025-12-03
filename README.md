# ESP32-Transmitter-TX-RX-for-a-Long-Range-Drone-RC-System
This sketch transforms an ESP32 into a high-frequency, long-range remote control transmitter for custom drone or RC applications. It leverages the ESP-NOW protocol along with the Long Range (LR) mode capability of the ESP32's Wi-Fi radio to achieve extended communication range with improved packet success rates compared to standard Wi-Fi.
The core functionality involves reading four analog joystick/potentiometer inputs, performing a one-time calibration to center the stick values, and continuously sending the processed flight data packet to a target receiver at a fixed rate of ~67 Hz (every 15 milliseconds) using a dedicated FreeRTOS task.

The transmitter also includes basic telemetry reception to monitor the connection status and the receiver's battery voltage.
⚙️ Key Technical Details
1. Hardware Pin Assignments
The code is set up to read four analog inputs, which are typically connected to the potentiometers of an RC controller's gimbals.
Constant,GPIO Pin,Control Axis,Type
throttlr_pin,34,Throttle (Thrust),Analog In
pich_pin,32,Pitch,Analog In
roll_pin,35,Roll,Analog In
yaw_pin,33,Yaw,Analog In
mode_pin,25,Mode (Unused in current logic),Digital In
Status LED,2,Status/Safety Indicator,Digital Out
2. Communication Protocol (ESP-NOW LR)
Protocol: ESP-NOW Long Range (LR) mode is enabled using esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR).

Mode: The ESP32 is set to Station (STA) mode (WiFi.mode(WIFI_STA)), which is required for ESP-NOW.

Broadcast Address: Communication is directed to the specific MAC address: 0xB4:3A:45:89:76:88.

Transmission Rate: The data is sent every 15 ms (SEND_INTERVAL_MS), resulting in a target rate of approximately 67 frames per second (FPS).
3. Data Structures
The system uses two defined structures for transmitting and receiving data:
🚀 Operation and Workflow
1. Stick Calibration (stickcalibration)
Upon startup, the device enters a calibration routine:
It reads the analog stick values 500 times to find the average center point for Pitch, Roll, and Yaw.
These baseline values (stick_Base_Value_pi, _ro, _ya) are used to ensure the sticks register 0 (zero command) when centered, compensating for potentiometers drift.
Note: The Throttle input is designed to be a non-centering stick (always reading $0$ at the lowest position), so its base value calculation is commented out.
2. Safety Check (Throttle Lock)
The main setup() function includes a safety lock requiring the user to push the Throttle stick to its lowest position (0) before initializing the system. If the throttle is not at zero, the Status LED (GPIO 2) blinks, preventing the device from starting transmission.
3. Analog Reading and Value Conversion (getAngValue)
This function converts the raw analog input (typically 0–4095 on the ESP32) into the final floating-point command values:
Read: Reads the raw analog values.
Offset: Subtracts the calibrated base values from the Pitch and Roll readings.
Deadband: Applies a +-15 deadband to Pitch and Roll to ignore small stick movements near the center.
Scaling & Constraining: Converts the stick outputs into the desired float range:Pitch & Roll: Values are scaled to range from approximately $-40.0$ to $+40.0$ using different multipliers for positive and negative deflection (e.g., $0.02$ for positive, $1/31.25$ for negative). This suggests the sticks might have asymmetric scaling or the intent was to create a specific non-linear response.Throttle: The raw analog value is used directly for s_control_data.thrust.
Transmission Task (SendTask)
The core transmission logic runs within a FreeRTOS task to ensure highly stable timing, independent of the main loop() function.

Every 15 ms, the task calls getAngValue(), increments the packet ID, loads the calculated Roll, Pitch, Yaw (0.00), and Thrust into the flight_control_packet_t structure, and calls esp_now_send().
Monitoring (Main Loop)
The loop() function runs infrequently (delayed by 500ms) and is mainly used for debugging and monitoring, printing the current stick values and the received telemetry data (packet rate and battery voltage).
