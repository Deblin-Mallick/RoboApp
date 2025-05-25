import network
import usocket as socket
import _thread
from machine import Pin, PWM, reset
import utime
import math
from cbor import loads as cbor_loads

class MotorController:
    """Motor control with instant response and minimum duty for reliable start."""
    def __init__(self, in1_pin, in2_pin, pwm_pin):
        # Initialize motor direction and PWM pins
        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin.OUT)
        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(1000)  # Set PWM frequency to 1kHz
        self._prev_state = (0, 0, 0)  # Track previous state to avoid redundant updates

    def set_speed(self, target):
        # Deadzone: treat very small commands as zero
        if abs(target) < 0.02:
            target = 0.0
        min_duty = 0.35  # Minimum duty cycle (35%) to overcome motor deadband

        if target == 0.0:
            # Stop motor: both direction pins low, PWM zero
            in1_val = 0
            in2_val = 0
            duty = 0
        else:
            # Set direction based on sign of target
            in1_val = 1 if target > 0 else 0
            in2_val = 0 if target > 0 else 1
            # Ensure minimum duty for movement, scale up to max
            duty = int((min_duty + (1 - min_duty) * abs(target)) * 65535)
        # Only update hardware if something changed
        if (in1_val, in2_val, duty) != self._prev_state:
            self.in1.value(in1_val)
            self.in2.value(in2_val)
            self.pwm.duty_u16(duty)
            self._prev_state = (in1_val, in2_val, duty)

class SoccerRobot:
    """4-wheel independent control backend."""
    def __init__(self):
        # Assign GPIO pins for each motor (adjust as per your wiring)
        self.lf_motor = MotorController(16, 17, 18)   # Left Front
        self.lr_motor = MotorController(19, 20, 21)   # Left Rear
        self.rf_motor = MotorController(13, 14, 15)   # Right Front
        self.rr_motor = MotorController(10, 11, 12)   # Right Rear

        # Watchdog and safety state
        self.watchdog_timer = utime.ticks_ms()
        self.safety_active = False
        self.client = None
        # Store last command for all motors
        self.last_cmd = {'lf': 0.0, 'lr': 0.0, 'rf': 0.0, 'rr': 0.0}

        # Start WiFi, safety monitor, and server
        self._connect_wifi()
        _thread.start_new_thread(self._safety_monitor, ())
        self._start_server()

    def _connect_wifi(self):
        # Connect to WiFi network
        sta_if = network.WLAN(network.STA_IF)
        sta_if.active(True)
        ssid = 'OpenWrt'      # CHANGE as needed
        password = 'iitmadras'
        max_retries = 10
        for _ in range(max_retries):
            sta_if.connect(ssid, password)
            utime.sleep(2)
            if sta_if.isconnected():
                print("Connected. IP:", sta_if.ifconfig()[0])
                return
            print("Retrying WiFi...")
        print("WiFi failed! Rebooting...")
        reset()  # If WiFi fails, reboot the Pico

    def _start_server(self):
        # Start TCP server to accept frontend connections
        s = socket.socket()
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('0.0.0.0', 65432))
        s.listen(1)
        s.setblocking(False)
        print("Server ready")
        while True:
            try:
                # Accept a new client if none is connected
                if not self.client:
                    try:
                        self.client, addr = s.accept()
                        self.client.setblocking(False)
                        print("Client connected:", addr)
                    except OSError:
                        pass  # No connection yet
                if self.client:
                    try:
                        # Read message header (4 bytes: message length)
                        header = self.client.recv(4)
                        if header:
                            msg_len = int.from_bytes(header, 'big')
                            # Read the actual message
                            if 0 < msg_len <= 1024:
                                data = self.client.recv(msg_len)
                                if len(data) == msg_len:
                                    self._process_command(data)
                        else:
                            # Client disconnected
                            self.client.close()
                            self.client = None
                            print("Client disconnected")
                    except OSError as e:
                        if e.args[0] == 11:  # EAGAIN, nothing to read
                            pass
                        elif e.args[0] == 104:  # Connection reset
                            self.client.close()
                            self.client = None
                        else:
                            print("OSError in client handling:", e)
                            self.client.close()
                            self.client = None
            except Exception as e:
                print("Server error:", e)
                self.client = None
            utime.sleep_ms(10)

    def _process_command(self, data):
        """Process CBOR-encoded commands and update watchdog timer."""
        try:
            cmd = cbor_loads(data)
            self.watchdog_timer = utime.ticks_ms()  # Reset watchdog on valid command
            # Get each wheel's speed, default to 0 if not present
            lf = cmd.get('lf', 0)
            lr = cmd.get('lr', 0)
            rf = cmd.get('rf', 0)
            rr = cmd.get('rr', 0)
            self.last_cmd = {'lf': lf, 'lr': lr, 'rf': rf, 'rr': rr}
            # Set each motor's speed
            self.lf_motor.set_speed(lf)
            self.lr_motor.set_speed(lr)
            self.rf_motor.set_speed(rf)
            self.rr_motor.set_speed(rr)
        except Exception as e:
            print("Command error:", e)

    def _safety_monitor(self):
        """Emergency stop if no commands received for 2 seconds and all speeds are zero."""
        last_safety_state = None
        while True:
            diff = utime.ticks_diff(utime.ticks_ms(), self.watchdog_timer)
            # Only stop if all speeds are zero and no command for 2s
            in_safety = diff > 2000 and all(abs(v) < 0.02 for v in self.last_cmd.values())
            if in_safety != last_safety_state:
                if in_safety:
                    print(f"EMERGENCY STOP (diff={diff})")
                    # Stop all motors immediately
                    for motor in [self.lf_motor, self.lr_motor, self.rf_motor, self.rr_motor]:
                        motor.in1.value(0)
                        motor.in2.value(0)
                        motor.pwm.duty_u16(0)
                else:
                    print(f"Safety cleared (diff={diff})")
                last_safety_state = in_safety
            self.safety_active = in_safety
            utime.sleep(0.1)

if __name__ == "__main__":
    SoccerRobot()

