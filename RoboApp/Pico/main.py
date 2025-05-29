import network
import usocket as socket
import select
import _thread
from machine import Pin, PWM, reset, WDT
import utime
import gc
from cbor import loads as cbor_loads, dumps as cbor_dumps

class DebugLogger:
    def __init__(self, enabled=False, max_lines=10):
        self.enabled = enabled
        self.max_lines = max_lines
        self.lines = []

    def log(self, msg):
        if self.enabled:
            now = utime.localtime()
            timestamp = f"{now[3]:02}:{now[4]:02}:{now[5]:02}"
            line = f"[{timestamp}] {msg}"
            print(line)
            self.lines.append(line)
            if len(self.lines) > self.max_lines:
                self.lines.pop(0)

class MotorController:
    def __init__(self, name, in1_pin, in2_pin, pwm_pin, debug=None):
        self.name = name
        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin.OUT)
        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(1000)
        self._prev_state = (0, 0, 0)
        self.debug = debug

    def set_speed(self, target):
        if abs(target) < 0.02:
            target = 0.0
        min_duty = 0.35
        if target == 0.0:
            in1_val = 0
            in2_val = 0
            duty = 0
        else:
            in1_val = 1 if target > 0 else 0
            in2_val = 0 if target > 0 else 1
            duty = int((min_duty + (1 - min_duty) * abs(target)) * 65535)
        if (in1_val, in2_val, duty) != self._prev_state:
            self.in1.value(in1_val)
            self.in2.value(in2_val)
            self.pwm.duty_u16(duty)
            self._prev_state = (in1_val, in2_val, duty)

class SoccerRobot:
    def __init__(self):
        # --- Hardware and state setup ---
        self.debug = DebugLogger(enabled=False, max_lines=10)
        self.lf_motor = MotorController('LF', 16, 17, 18, debug=self.debug)
        self.lr_motor = MotorController('LR', 19, 20, 21, debug=self.debug)
        self.rf_motor = MotorController('RF', 13, 14, 15, debug=self.debug)
        self.rr_motor = MotorController('RR', 10, 11, 12, debug=self.debug)
        self.watchdog_timer = utime.ticks_ms()
        self.safety_active = False
        self.last_cmd = {'lf': 0.0, 'lr': 0.0, 'rf': 0.0, 'rr': 0.0}
        self.led = Pin("LED", Pin.OUT)
        self.led.value(0)
        self.server_running = True

        # --- Start WiFi, safety monitor, and server ---
        self._connect_wifi()
        _thread.start_new_thread(self._safety_monitor, ())
        self._start_server()

    def _connect_wifi(self):
        """Connect to WiFi and set LED status. Retries forever on failure."""
        sta_if = network.WLAN(network.STA_IF)
        sta_if.active(True)
        sta_if.config(pm=0xa11140)
        ssid = 'OpenWrt'
        password = 'iitmadras'
        print("Connecting to WiFi...")
        while True:
            try:
                sta_if.connect(ssid, password)
                for _ in range(20):
                    self.led.toggle()
                    utime.sleep(0.2)
                    if sta_if.isconnected():
                        self.led.value(1)
                        print("WiFi connected.")
                        ip = sta_if.ifconfig()[0]
                        print(f"Connected to. IP: {ip}")
                        return
                print("WiFi not connected, retrying...")
                self._blink_led(0.1, 5)
            except Exception as e:
                print(f"WiFi error: {e}")
                self._blink_led(0.1, 5)
            utime.sleep(2)

    def _start_server(self):
        """Main server loop with watchdog and error recovery."""
        wdt = WDT(timeout=8000)
        while True:
            try:
                self._run_server(wdt)
            except Exception as e:
                print(f"Server error: {e}")
                self._blink_led(0.05, 10)
                utime.sleep(2)  # Wait before retrying
                # WiFi may have dropped, so reconnect
                self._connect_wifi()

    def _run_server(self, wdt):
        """TCP server for robot control, feeds watchdog regularly."""
        s = socket.socket()
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('0.0.0.0', 65432))
        s.listen(1)
        s.setblocking(False)
        poller = select.poll()
        poller.register(s, select.POLLIN)
        client_sock = None
        last_activity = utime.ticks_ms()
        buffer = b""
        expected_len = None

        while True:
            wdt.feed()  # Feed watchdog to prevent reboot

            # Accept new client if none connected
            if not client_sock:
                self.led.value(1)  # Solid LED: ready for client
                events = poller.poll(10)
                for sock, event in events:
                    if sock is s and event & select.POLLIN:
                        client_sock, addr = s.accept()
                        client_sock.setblocking(False)
                        poller.register(client_sock, select.POLLIN)
                        print(f"Client connected: {addr}")
                        self._blink_led(0.05, 2)
                        last_activity = utime.ticks_ms()
                        buffer = b""
                        expected_len = None

            # Handle client communication
            if client_sock:
                self.led.value(1)  # Solid LED: client connected
                try:
                    events = poller.poll(0)
                    for sock, event in events:
                        if sock is client_sock and event & select.POLLIN:
                            data = client_sock.recv(1024)
                            if data:
                                buffer += data
                                last_activity = utime.ticks_ms()
                                while True:
                                    if expected_len is None and len(buffer) >= 4:
                                        expected_len = int.from_bytes(buffer[:4], 'big')
                                        buffer = buffer[4:]
                                    if expected_len is not None and len(buffer) >= expected_len:
                                        msg = buffer[:expected_len]
                                        buffer = buffer[expected_len:]
                                        expected_len = None
                                        if self._process_command(msg, client_sock):
                                            # If True, means restart/shutdown requested
                                            client_sock.close()
                                            return
                                    else:
                                        break
                            else:
                                client_sock.close()
                                client_sock = None
                                self.led.value(1)
                    # Timeout client after 30s inactivity
                    if utime.ticks_diff(utime.ticks_ms(), last_activity) > 30000:
                        print("Client timeout")
                        client_sock.close()
                        client_sock = None
                        self.led.value(1)
                except Exception as e:
                    print(f"Client error: {e}")
                    if client_sock:
                        client_sock.close()
                        client_sock = None
                    self.led.value(1)
            utime.sleep_ms(10)
            gc.collect()

    def _process_command(self, data, sock):
        """Process incoming commands, handle restart/shutdown."""
        try:
            cmd = cbor_loads(data)
            # Special commands from frontend
            if 'command' in cmd:
                if cmd['command'] == 'restart':
                    print("Soft restarting server...")
                    self._blink_led(0.1, 4)
                    return True  # Return True to trigger server restart
                elif cmd['command'] == 'shutdown':
                    print("Performing safe shutdown...")
                    self._blink_led(0.3, 5)
                    utime.sleep(1)
                    reset()
            # Normal motor command
            self.watchdog_timer = utime.ticks_ms()
            lf = cmd.get('lf', 0)
            lr = cmd.get('lr', 0)
            rf = cmd.get('rf', 0)
            rr = cmd.get('rr', 0)
            self.last_cmd = {'lf': lf, 'lr': lr, 'rf': rf, 'rr': rr}
            self.lf_motor.set_speed(lf)
            self.lr_motor.set_speed(lr)
            self.rf_motor.set_speed(rf)
            self.rr_motor.set_speed(rr)
            # Latency/acknowledge
            if sock and 'cmd_id' in cmd:
                resp = {'cmd_id': cmd['cmd_id']}
                resp_data = cbor_dumps(resp)
                try:
                    sock.sendall(len(resp_data).to_bytes(4, 'big') + resp_data)
                except:
                    pass
        except Exception as e:
            print(f"Command error: {str(e)}")
            self._blink_led(0.05, 5)
        return False

    def _safety_monitor(self):
        """Emergency stop monitoring with LED feedback and DebugLogger."""
        last_safety_state = False
        while True:
            diff = utime.ticks_diff(utime.ticks_ms(), self.watchdog_timer)
            in_safety = diff > 2000 and all(abs(v) < 0.02 for v in self.last_cmd.values())
            if in_safety != last_safety_state:
                if in_safety:
                    self.debug.log("EMERGENCY STOP")  # Only logs if enabled
                    for motor in [self.lf_motor, self.lr_motor, self.rf_motor, self.rr_motor]:
                        motor.in1.value(0)
                        motor.in2.value(0)
                        motor.pwm.duty_u16(0)
                    self._blink_led(0.1, 10)
                else:
                    self.debug.log("Safety cleared")  # Only logs if enabled
                    self.led.value(1)
                last_safety_state = in_safety
                self.safety_active = in_safety
            utime.sleep(0.1)

    def _blink_led(self, interval, count):
        """Blink the onboard LED for status feedback."""
        for _ in range(count):
            self.led.value(1)
            utime.sleep(interval)
            self.led.value(0)
            utime.sleep(interval)
        self.led.value(1)  # Restore to normal state (solid on)

if __name__ == "__main__":
    SoccerRobot()

