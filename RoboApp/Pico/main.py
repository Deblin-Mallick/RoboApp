import network
import usocket as socket
import select
import _thread
from machine import Pin, PWM, reset
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
        self.debug = DebugLogger(enabled=False, max_lines=10)
        self.lf_motor = MotorController('LF', 16, 17, 18, debug=self.debug)
        self.lr_motor = MotorController('LR', 19, 20, 21, debug=self.debug)
        self.rf_motor = MotorController('RF', 13, 14, 15, debug=self.debug)
        self.rr_motor = MotorController('RR', 10, 11, 12, debug=self.debug)
        self.watchdog_timer = utime.ticks_ms()
        self.safety_active = False
        self.last_cmd = {'lf': 0.0, 'lr': 0.0, 'rf': 0.0, 'rr': 0.0}
        self._connect_wifi()
        _thread.start_new_thread(self._safety_monitor, ())
        self._start_server()

    def _connect_wifi(self):
        sta_if = network.WLAN(network.STA_IF)
        sta_if.active(True)
        # CRITICAL: Disable WiFi power management
        sta_if.config(pm=0xa11140)
        ssid = 'OpenWrt'
        password = 'iitmadras'
        max_retries = 10
        connected = False
        for _ in range(max_retries):
            print("Trying to connect to WiFi...")
            sta_if.connect(ssid, password)
            for _ in range(10):
                if sta_if.isconnected():
                    connected = True
                    break
                utime.sleep(0.5)
            if connected:
                break
        if not connected:
            print("WiFi failed! Rebooting...")
            reset()
        ip = sta_if.ifconfig()[0]
        print(f"Connected. IP: {ip}")

    def _start_server(self):
        s = socket.socket()
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('0.0.0.0', 65432))
        s.listen(1)
        s.setblocking(False)
        print("Server ready")
        buffer = b""
        expected_len = None
        last_heartbeat = utime.ticks_ms()

        poller = select.poll()
        poller.register(s, select.POLLIN)
        client_sock = None

        while True:
            loop_start = utime.ticks_ms()
            try:
                # Connection timeout check
                if client_sock and utime.ticks_diff(utime.ticks_ms(), last_heartbeat) > 30000:
                    poller.unregister(client_sock)
                    client_sock.close()
                    client_sock = None
                    buffer = b""
                    expected_len = None
                    print("Client connection timeout")
                # Accept new client
                if not client_sock:
                    events = poller.poll(10)
                    for sock, event in events:
                        if sock is s and event & select.POLLIN:
                            client_sock, addr = s.accept()
                            client_sock.setblocking(False)
                            poller.register(client_sock, select.POLLIN)
                            print(f"Client connected: {addr}")
                            buffer = b""
                            expected_len = None
                            last_heartbeat = utime.ticks_ms()
                # Process client data
                if client_sock:
                    events = poller.poll(0)
                    for sock, event in events:
                        if sock is client_sock and event & select.POLLIN:
                            try:
                                data = client_sock.recv(1024)
                                if data:
                                    buffer += data
                                    last_heartbeat = utime.ticks_ms()
                                    while True:
                                        if expected_len is None:
                                            if len(buffer) >= 4:
                                                expected_len = int.from_bytes(buffer[:4], 'big')
                                                buffer = buffer[4:]
                                            else:
                                                break
                                        if expected_len is not None and len(buffer) >= expected_len:
                                            msg = buffer[:expected_len]
                                            buffer = buffer[expected_len:]
                                            expected_len = None
                                            self._process_command(msg, client_sock)
                                        else:
                                            break
                                else:
                                    poller.unregister(client_sock)
                                    client_sock.close()
                                    client_sock = None
                                    print("Client disconnected")
                            except OSError as e:
                                if e.args[0] not in (11, 104):  # EAGAIN, ECONNRESET
                                    poller.unregister(client_sock)
                                    client_sock.close()
                                    client_sock = None
                                    print(f"Client error: {e}")
            except Exception as e:
                print(f"Server error: {e}")
                if client_sock:
                    poller.unregister(client_sock)
                    client_sock.close()
                    client_sock = None

            # Memory and performance monitoring
            if utime.ticks_diff(utime.ticks_ms(), loop_start) > 100:
                print(f"!!! CPU WARNING: Server loop took {utime.ticks_diff(utime.ticks_ms(), loop_start)} ms !!!")
            gc.collect()
            utime.sleep_ms(1)

    def _process_command(self, data, sock):
        try:
            cmd = cbor_loads(data)
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
            # Echo cmd_id for latency measurement
            if sock and 'cmd_id' in cmd:
                resp = {'cmd_id': cmd['cmd_id']}
                resp_data = cbor_dumps(resp)
                try:
                    sock.sendall(len(resp_data).to_bytes(4, 'big') + resp_data)
                except OSError:
                    pass  # Client disconnected during send
        except Exception as e:
            print(f"Command error: {repr(e)}")

    def _safety_monitor(self):
        last_safety_state = None
        while True:
            diff = utime.ticks_diff(utime.ticks_ms(), self.watchdog_timer)
            in_safety = diff > 2000 and all(abs(v) < 0.02 for v in self.last_cmd.values())
            if in_safety != last_safety_state:
                if in_safety:
                    self.debug.log(f"EMERGENCY STOP (diff={diff})")
                    for motor in [self.lf_motor, self.lr_motor, self.rf_motor, self.rr_motor]:
                        motor.in1.value(0)
                        motor.in2.value(0)
                        motor.pwm.duty_u16(0)
                else:
                    self.debug.log(f"Safety cleared (diff={diff})")
                last_safety_state = in_safety
            self.safety_active = in_safety
            utime.sleep(0.1)

if __name__ == "__main__":
    SoccerRobot()

