import network
import usocket as socket
import _thread
from machine import Pin, PWM, reset
import utime
import math
from cbor import loads as cbor_loads

class MotorController:
    """Motor control with instant response and minimum duty."""
    def __init__(self, in1_pin, in2_pin, pwm_pin):
        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin.OUT)
        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(1000)
        self._prev_state = (0, 0, 0)  # (IN1, IN2, Duty)

    def set_speed(self, target):
        # Treat very small values as zero (deadzone)
        if abs(target) < 0.02:
            target = 0.0

        # Minimum duty for movement (e.g., 0.35 = 35%)
        min_duty = 0.35

        if target == 0.0:
            in1_val = 0
            in2_val = 0
            duty = 0
        else:
            in1_val = 1 if target > 0 else 0
            in2_val = 0 if target > 0 else 1
            # Add minimum duty to overcome deadband
            duty = int((min_duty + (1 - min_duty) * abs(target)) * 65535)

        # Only update hardware if changed
        if (in1_val, in2_val, duty) != self._prev_state:
            self.in1.value(in1_val)
            self.in2.value(in2_val)
            self.pwm.duty_u16(duty)
            self._prev_state = (in1_val, in2_val, duty)

class SoccerRobot:
    """Robust, competition-ready remote-controlled soccer robot backend."""
    def __init__(self):
        self.left_motor = MotorController(16, 17, 18)
        self.right_motor = MotorController(19, 20, 21)
        self.watchdog_timer = utime.ticks_ms()
        self.safety_active = False
        self.client = None
        self.last_left = 0.0
        self.last_right = 0.0

        self._connect_wifi()
        _thread.start_new_thread(self._safety_monitor, ())
        self._start_server()

    # --- WiFi connection ---
    def _connect_wifi(self):
        sta_if = network.WLAN(network.STA_IF)
        sta_if.active(True)
        ssid = 'OpenWrt'      # CHANGE to your event WiFi SSID
        password = 'iitmadras' # CHANGE to your event WiFi password
        max_retries = 10
        for _ in range(max_retries):
            sta_if.connect(ssid, password)
            utime.sleep(2)
            if sta_if.isconnected():
                print("Connected. IP:", sta_if.ifconfig()[0])
                return
            print("Retrying WiFi...")
        print("WiFi failed! Rebooting...")
        reset()

    # --- Networking/server ---
    def _start_server(self):
        s = socket.socket()
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('0.0.0.0', 65432))
        s.listen(1)
        s.setblocking(False)
        print("Server ready")
        while True:
            try:
                if not self.client:
                    try:
                        self.client, addr = s.accept()
                        self.client.setblocking(False)
                        print("Client connected:", addr)
                    except OSError:
                        pass
                if self.client:
                    try:
                        header = self.client.recv(4)
                        if header:
                            msg_len = int.from_bytes(header, 'big')
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

    # --- Command processing ---
    def _process_command(self, data):
        """Process CBOR-encoded commands and update watchdog timer."""
        try:
            cmd = cbor_loads(data)
            self.watchdog_timer = utime.ticks_ms()  # Always update watchdog on valid command!
            self.last_left = cmd.get('left', 0)
            self.last_right = cmd.get('right', 0)
            self.left_motor.set_speed(self.last_left)
            self.right_motor.set_speed(self.last_right)
        except Exception as e:
            print("Command error:", e)

    # --- Safety monitor ---
    def _safety_monitor(self):
        """Emergency stop if no commands received for 2 seconds and both speeds are zero."""
        last_safety_state = None
        while True:
            diff = utime.ticks_diff(utime.ticks_ms(), self.watchdog_timer)
            # Only stop if both speeds are zero
            in_safety = diff > 2000 and abs(self.last_left) < 0.02 and abs(self.last_right) < 0.02
            if in_safety != last_safety_state:
                if in_safety:
                    print(f"EMERGENCY STOP (diff={diff})")
                    # Stop motors immediately
                    self.left_motor.in1.value(0)
                    self.left_motor.in2.value(0)
                    self.right_motor.in1.value(0)
                    self.right_motor.in2.value(0)
                    self.left_motor.pwm.duty_u16(0)
                    self.right_motor.pwm.duty_u16(0)
                else:
                    print(f"Safety cleared (diff={diff})")
                last_safety_state = in_safety
            self.safety_active = in_safety
            utime.sleep(0.1)

if __name__ == "__main__":
    SoccerRobot()

