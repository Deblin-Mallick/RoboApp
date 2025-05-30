# RoboApp: Soccer Robot Controller

A real-time control system for a soccer robot using a Raspberry Pi Pico backend and a Python/Tkinter frontend.  
The system features low-latency TCP communication with CBOR-encoded messages, a responsive GUI, and robust safety features.

---

## Features

- **Real-time control** of left and right motors via Xbox/compatible game controller.
- **Compact binary communication** using CBOR for efficient, low-memory messaging.
- **Python GUI frontend** with live visualization of controller inputs and robot status.
- **Multithreaded architecture** for smooth GUI, responsive control, and reliable networking.
- **Safety watchdog** on backend to stop motors if commands are lost.
- **Emergency stop** button in GUI (always visible).
- **Restart Backend** and **Shutdown Pico** buttons in GUI.
- **Connect/Disconnect** via GUI or Xbox "A" button.
- **Detailed debug logging** in the GUI event log.
- **Configurable deadzone** for joystick inputs.
- **Non-commercial license:** You may fork and modify, but **commercial use is prohibited** (see License section).

---

## Hardware Requirements

- Raspberry Pi Pico (or compatible MicroPython board)
- L298N motor driver (or equivalent)
- DC motors for left and right wheels
- Xbox controller or compatible gamepad
- WiFi network for TCP communication

---

## Software Requirements

- **Frontend (PC):**
  - Python 3.7+
  - `pygame`
  - `numpy`
  - `scipy`
  - `cbor2`
  - `tkinter` (usually included with Python)

- **Backend (Pico):**
  - MicroPython firmware
  - `micropython-cbor` library (upload `cbor.py` to Pico)

---

## Installation

### Frontend (PC)

1. Install Python dependencies:

pip install pygame numpy scipy cbor2

2. Run the GUI:

python robot_controller.py


### Backend (Pico)

1. Upload your MicroPython firmware to the Pico.
2. Download [`cbor.py`](https://github.com/alexmrqt/micropython-cbor/blob/master/cbor.py) and upload it to the Pico using Thonny or ampy.
3. Upload the backend script (`robot_driver.py`) to the Pico.
4. Update WiFi credentials in `robot_driver.py`:
ssid = 'YourSSID'
password = 'YourPassword'

5. Run the backend script on the Pico.

---

## Usage

1. Connect your Xbox controller to the PC.
2. Launch the frontend GUI.
3. Enter the robot's IP address and click **Connect** (or use the Xbox "A" button).
4. Use the right joystick for steering and triggers for throttle control.
5. Monitor the live input visualization and debug logs.
6. Use the **Emergency Stop** button to immediately stop the robot.
7. Use **Restart Backend** and **Shutdown Pico** buttons for backend management.

---

## Communication Protocol

- TCP socket connection on port `65432`.
- Messages are CBOR-encoded dictionaries with:
- `cmd_id`: int (for latency tracking)
- `lf`, `lr`, `rf`, `rr`: float (-1.0 to 1.0) for motor speeds
- Each message is prefixed with a 4-byte big-endian length header.

---

## Safety Features

- Backend watchdog stops motors if no commands received for 2 seconds.
- Frontend prevents command spamming by sending only on significant input changes.
- Emergency stop button immediately sends zero speed commands.

---

## License

**MIT License + Commons Clause — Non-commercial use only**

This project is licensed under the MIT License with the Commons Clause restriction.  
You may use, fork, and modify this code for **non-commercial purposes only**.  
Commercial use (including selling, SaaS, or paid support) is strictly prohibited without a separate commercial license.

See the [LICENSE](LICENSE) file for full details.

---

## Acknowledgments

- [MicroPython](https://micropython.org/)
- [pygame](https://www.pygame.org/)
- [CBOR](https://cbor.io/)
- [Raspberry Pi Pico](https://www.raspberrypi.com/products/raspberry-pi-pico/)

---

## Contact

For questions or contributions, please open an issue or pull request on GitHub.

---

*Happy coding and good luck with your soccer robot!*