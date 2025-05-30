# RoboApp – Soccer Robot Controller

**RoboApp** is a real-time control system for a soccer robot, featuring a Raspberry Pi Pico backend and a Python/Tkinter frontend. It supports low-latency TCP communication with CBOR-encoded messages, a responsive GUI, and robust safety features to ensure reliable operation on the field.

---

## Features

- 🎮 **Real-time Xbox/Gamepad control** of left and right motors.
- ⚡ **Low-latency TCP communication** with compact CBOR-encoded messages.
- 🖥️ **Python GUI frontend** with live controller input visualization and robot status.
- 🧵 **Multithreaded architecture** for smooth GUI performance and reliable networking.
- 🛡️ **Backend watchdog** stops motors if control commands are lost.
- 🆘 **Emergency Stop button** built into the GUI.
- 🧾 **Event log** for detailed debugging and status tracking.
- 🎯 **Configurable deadzone** for joystick input filtering.
- 🚫 **Non-commercial license** (see [License](#license)).

---

## Hardware Requirements

- Raspberry Pi Pico (or compatible MicroPython board)
- L298N motor driver (or equivalent)
- Two DC motors (left and right wheel)
- Xbox or compatible game controller
- WiFi-enabled network for TCP communication

---

## Software Requirements

### Frontend (PC)
- Python 3.7+
- `pygame`
- `numpy`
- `scipy`
- `cbor2`
- `tkinter` (usually included with Python)

### Backend (Pico)
- MicroPython firmware
- `micropython-cbor` (upload `cbor.py` to Pico)

---

## Installation

### Frontend (PC)
Install dependencies:
```bash
pip install pygame numpy scipy cbor2

Run the GUI:

bash
Copy
Edit
python robot_controller.py
Backend (Pico)
Flash the MicroPython firmware to the Pico.

Upload cbor.py and robot_driver.py using Thonny or ampy.

Update your WiFi credentials in robot_driver.py:

python
Copy
Edit
sta_if.connect('YourSSID', 'YourPassword')
Run robot_driver.py on the Pico.

Usage
Connect your Xbox controller to the PC.

Launch the frontend GUI.

Enter the robot's IP address and click Connect, or press the Xbox A button.

Use:

Right joystick for steering

Triggers for throttle control

Monitor:

Live input visualization

Event log for debug messages

Use Emergency Stop to instantly halt the robot.

Use Restart Backend or Shutdown Pico for backend control.

Communication Protocol
Transport: TCP socket on port 65432

Encoding: CBOR

Message format: Dictionary with the following keys:

json
Copy
Edit
{
  "timestamp": float,     // Unix time
  "left": float,          // Left motor speed (-1.0 to 1.0)
  "right": float          // Right motor speed (-1.0 to 1.0)
}
Framing: Each message is prefixed with a 4-byte big-endian length header.

Safety Features
⏱️ Backend watchdog halts motors if no commands received for 1 second.

🚫 Frontend prevents command spamming, sending only on significant input changes.

🆘 Emergency Stop immediately sends zero-speed commands to both motors.

License
MIT License + Commons Clause

This project is licensed under the MIT License with the Commons Clause restriction.

✅ You may use, fork, and modify this code for non-commercial purposes.

❌ Commercial use (including resale, SaaS, paid support) is prohibited without a separate commercial license.

See the LICENSE file for full terms.

Acknowledgments
MicroPython

pygame

CBOR

Raspberry Pi Pico

Contact
For questions, suggestions, or contributions, please open an issue or submit a pull request.

Happy coding and good luck with your soccer robot!
