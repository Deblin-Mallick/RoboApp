RoboApp
Soccer Robot Controller
A real-time control system for a soccer robot using a Raspberry Pi Pico backend and a Python/Tkinter frontend.
The system features low-latency TCP communication with CBOR-encoded messages, a responsive GUI, and robust safety features.

Features
Real-time control of left and right motors via Xbox/compatible game controller.

Compact binary communication using CBOR for efficient, low-memory messaging.

Python GUI frontend with live visualization of controller inputs and robot status.

Multithreaded architecture for smooth GUI, responsive control, and reliable networking.

Safety watchdog on backend to stop motors if commands are lost.

Emergency stop button in GUI.

Detailed debug logging in the GUI event log.

Configurable deadzone for joystick inputs.

Non-commercial license: You may fork and modify, but commercial use is prohibited (see License section).

Hardware Requirements
Raspberry Pi Pico (or compatible MicroPython board)

L298N motor driver (or equivalent)

DC motors for left and right wheels

Xbox controller or compatible gamepad

WiFi network for TCP communication

Software Requirements
Frontend (PC):

Python 3.7+

pygame

numpy

scipy

cbor2

tkinter (usually included with Python)

Backend (Pico):

MicroPython firmware

micropython-cbor library (upload cbor.py to Pico)

Installation
Frontend (PC)
Install Python dependencies:

bash
pip install pygame numpy scipy cbor2
Run the GUI:

bash
python robot_controller.py
Backend (Pico)
Upload your MicroPython firmware to the Pico.

Download cbor.py and upload it to the Pico using Thonny or ampy.

Upload the backend script (robot_driver.py) to the Pico.

Update WiFi credentials in robot_driver.py:

python
sta_if.connect('YourSSID', 'YourPassword')
Run the backend script on the Pico.

Usage
Connect your Xbox controller to the PC.

Launch the frontend GUI.

Enter the robot's IP address and click Connect (or use the Xbox "A" button).

Use the right joystick for steering and triggers for throttle control.

Monitor the live input visualization and debug logs.

Use the Emergency Stop button to immediately stop the robot.

Use Restart Backend and Shutdown Pico buttons for backend management.

Communication Protocol
TCP socket connection on port 65432.

Messages are CBOR-encoded dictionaries with:

timestamp: float (Unix time)

left: float (-1.0 to 1.0) left motor speed

right: float (-1.0 to 1.0) right motor speed

Each message is prefixed with a 4-byte big-endian length header.

Safety Features
Backend watchdog stops motors if no commands received for 1 second.

Frontend prevents command spamming by sending only on significant input changes.

Emergency stop button immediately sends zero speed commands.

License
MIT License + Commons Clause — Non-commercial use only

This project is licensed under the MIT License with the Commons Clause restriction.
You may use, fork, and modify this code for non-commercial purposes only.
Commercial use (including selling, SaaS, or paid support) is strictly prohibited without a separate commercial license.

See the LICENSE file for full details.

Acknowledgments
MicroPython

pygame

CBOR

Raspberry Pi Pico

Contact
For questions or contributions, please open an issue or pull request on GitHub.

Happy coding and good luck with your soccer robot!