import tkinter as tk
from tkinter import ttk
import pygame
import socket
import threading
import cbor2
import time
import numpy as np
from scipy.interpolate import CubicSpline
from queue import Queue, Empty

class SoccerRobotController:
    CONTROL_RATE_HZ = 50  # Control loop frequency (Hz)

    def __init__(self):
        self.root = tk.Tk()
        self.invert_x = False
        self.root.title("Soccer Robot Controller")
        self.root.geometry("1000x800")

        # Queues for thread-safe communication
        self.data_queue = Queue()
        self.log_queue = Queue()
        self.gui_update_queue = Queue()

        # State variables
        self.sock = None
        self.connected = False
        self.stop_flag = threading.Event()
        self.last_cmd_id = 0
        self.cmd_send_times = {}
        self.deadzone = 0.15
        self.response_curve = CubicSpline([0, 0.2, 0.5, 0.8, 1], [0, 0.1, 0.4, 0.9, 1])
        self.controller_mapping = {'right_x': 2, 'rt': 5, 'lt': 4, 'a_button': 0}
        self.current_x = 150
        self.logging_enabled = True
        self.a_button_prev_state = False

        self._setup_gui()
        self._log("Event log test: If you see this, logging works!")
        self._init_controller()
        self._start_threads()
        self.root.protocol("WM_DELETE_WINDOW", self._shutdown)
        self.root.mainloop()

    def _setup_gui(self):
        # --- Emergency Stop Button (always at the top) ---
        button_frame = ttk.Frame(self.root)
        button_frame.pack(fill='x', padx=10, pady=10)
        self.estop_btn = ttk.Button(button_frame, text="EMERGENCY STOP", command=self._emergency_stop)
        self.estop_btn.pack(side='left', padx=5, pady=5)

        # --- Connection Panel ---
        conn_frame = ttk.LabelFrame(self.root, text="Connection")
        conn_frame.pack(fill='x', padx=10, pady=5)
        ttk.Label(conn_frame, text="Robot IP:").grid(row=0, column=0, padx=5)
        self.ip_entry = ttk.Entry(conn_frame, width=20)
        self.ip_entry.grid(row=0, column=1, padx=5)
        self.ip_entry.insert(0, "192.168.2.179")
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self._handle_connection)
        self.connect_btn.grid(row=0, column=2, padx=5)
        self.status_led = tk.Canvas(conn_frame, width=20, height=20, bg='red')
        self.status_led.grid(row=0, column=3, padx=5)

        # --- Control Settings Panel ---
        config_frame = ttk.LabelFrame(self.root, text="Control Settings")
        config_frame.pack(fill='x', padx=10, pady=5)
        ttk.Label(config_frame, text="Deadzone:").grid(row=0, column=0, padx=5)
        self.deadzone_label = ttk.Label(config_frame, text="0.15")
        self.deadzone_label.grid(row=0, column=2, padx=5)
        self.deadzone_slider = ttk.Scale(config_frame, from_=0, to=30, command=self._update_deadzone)
        self.deadzone_slider.set(15)
        self.deadzone_slider.grid(row=0, column=1, padx=5)

        # --- Inverst Toggle ---
        self.invert_x_var = tk.BooleanVar(value=self.invert_x)
        self.invert_x_checkbox = ttk.Checkbutton(
            config_frame,
            text="Invert X Axis",
            variable=self.invert_x_var,
            command=self._toggle_invert_x
        )
        self.invert_x_checkbox.grid(row=0, column=3, padx=10)

        # --- Restart and Shutdown Buttons (Top Right of Control Settings) ---
        self.restart_btn = ttk.Button(config_frame, text="Restart Backend", command=lambda: self._send_special_command('restart'))
        self.restart_btn.place(relx=0.77, rely=0.0001, relwidth=0.1, anchor='nw')
        self.shutdown_btn = ttk.Button(config_frame, text="Shutdown Pico", command=lambda: self._send_special_command('shutdown'))
        self.shutdown_btn.place(relx=0.98, rely=0.0001, relwidth=0.1, anchor='ne')

        # --- Visualization Panel ---
        vis_frame = ttk.LabelFrame(self.root, text="Controller Input")
        vis_frame.pack(fill='both', expand=True, padx=10, pady=5)
        self.steering_canvas = tk.Canvas(vis_frame, width=300, height=100, bg='#f0f0f0')
        self.steering_canvas.pack(pady=10)
        self.steering_indicator = self.steering_canvas.create_oval(145, 45, 155, 55, fill='blue')
        self.rt_progress = ttk.Progressbar(vis_frame, length=200, mode='determinate')
        self.rt_progress.pack(pady=5)
        self.lt_progress = ttk.Progressbar(vis_frame, length=200, mode='determinate')
        self.lt_progress.pack(pady=5)

        # --- System Status Panel ---
        debug_frame = ttk.LabelFrame(self.root, text="System Status")
        debug_frame.pack(fill='both', expand=True, padx=10, pady=5)
        self.debug_vars = {
            'steering': tk.StringVar(value="0.00"),
            'throttle': tk.StringVar(value="0.00"),
            'packet_rate': tk.StringVar(value="0.0/s"),
            'latency': tk.StringVar(value="0 ms")
        }
        ttk.Label(debug_frame, text="Steering:").pack(anchor='w', padx=10)
        ttk.Label(debug_frame, textvariable=self.debug_vars['steering']).pack(anchor='w', padx=20)
        ttk.Label(debug_frame, text="Throttle:").pack(anchor='w', padx=10)
        ttk.Label(debug_frame, textvariable=self.debug_vars['throttle']).pack(anchor='w', padx=20)
        ttk.Label(debug_frame, text="Packet Rate:").pack(anchor='w', padx=10)
        ttk.Label(debug_frame, textvariable=self.debug_vars['packet_rate']).pack(anchor='w', padx=20)
        ttk.Label(debug_frame, text="Latency:").pack(anchor='w', padx=10)
        ttk.Label(debug_frame, textvariable=self.debug_vars['latency']).pack(anchor='w', padx=20)

        # --- Event Log Panel ---
        log_frame = ttk.LabelFrame(self.root, text="Event Log")
        log_frame.pack(fill='both', expand=True, padx=10, pady=5)
        self.log_text = tk.Text(log_frame, wrap=tk.WORD, state='disabled')
        scroll = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scroll.set)
        self.log_text.pack(side='left', fill='both', expand=True)
        scroll.pack(side='right', fill='y')

        # --- Logging Toggle Button ---
        self.toggle_log_btn = ttk.Button(self.root, text="Disable Logging", command=self._toggle_logging)
        self.toggle_log_btn.place(relx=0.97, rely=0.93, anchor='ne')
    
    def _toggle_invert_x(self):
        self.invert_x = self.invert_x_var.get()
        self._log(f"Invert X Axis set to {self.invert_x}")

    def _toggle_logging(self):
        self.logging_enabled = not self.logging_enabled
        if self.logging_enabled:
            self.toggle_log_btn.config(text="Disable Logging")
            self._log("Event logging enabled.")
        else:
            self.toggle_log_btn.config(text="Enable Logging")

    def _send_special_command(self, command):
        if self.connected:
            try:
                cmd = {'command': command}
                data = cbor2.dumps(cmd)
                self.data_queue.put(data)
                self._log(f"Sent {command} command")
            except Exception as e:
                self._log(f"Failed to send {command}: {str(e)}", "ERROR")
        else:
            self._log("Not connected to backend!", "ERROR")

    def _init_controller(self):
        pygame.init()
        pygame.joystick.init()
        self.joystick = None
        self._try_connect_controller()

    def _try_connect_controller(self):
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self._log("Controller connected: " + self.joystick.get_name())
        else:
            self._log("No controller detected!", "ERROR")

    def _start_threads(self):
        threading.Thread(target=self._control_loop, daemon=True).start()
        threading.Thread(target=self._network_loop, daemon=True).start()
        threading.Thread(target=self._receive_loop, daemon=True).start()
        threading.Thread(target=self._connection_monitor, daemon=True).start()
        self._start_gui_update_loop()

    def _control_loop(self):
        last_check = time.time()
        while not self.stop_flag.is_set():
            self._process_controls()
            # Controller reconnection logic
            if not hasattr(self, 'joystick') or self.joystick is None:
                if time.time() - last_check > 2:
                    pygame.joystick.quit()
                    pygame.joystick.init()
                    self._try_connect_controller()
                    last_check = time.time()
            time.sleep(1.0 / self.CONTROL_RATE_HZ)

    def _network_loop(self):
        packet_count = 0
        last_update = time.time()
        while not self.stop_flag.is_set():
            if not self.connected:
                time.sleep(0.2)
                continue
            try:
                data = self.data_queue.get(timeout=0.1)
                header = len(data).to_bytes(4, 'big')
                try:
                    self.sock.sendall(header + data)
                except Exception as e:
                    self._log(f"Network error: {str(e)}", "ERROR")
                    self._disconnect()
                    continue
                packet_count += 1
                if time.time() - last_update >= 1:
                    self.debug_vars['packet_rate'].set(f"{packet_count}/s")
                    packet_count = 0
                    last_update = time.time()
            except Empty:
                continue
            except Exception as e:
                self._log(f"Network error: {str(e)}", "ERROR")
                self._disconnect()

    def _receive_loop(self):
        while not self.stop_flag.is_set():
            if not self.connected:
                time.sleep(0.2)
                continue
            try:
                if self.sock:
                    header = self.sock.recv(4)
                    if not header:
                        self._disconnect()
                        continue
                    msg_len = int.from_bytes(header, 'big')
                    if 0 < msg_len <= 1024:
                        data = self.sock.recv(msg_len)
                        if len(data) == msg_len:
                            resp = cbor2.loads(data)
                            self._handle_backend_response(resp)
            except Exception:
                time.sleep(0.01)

    def _connection_monitor(self):
        while not self.stop_flag.is_set():
            if self.connected and self._is_socket_closed():
                self._log("Lost connection to backend.", "ERROR")
                self._disconnect()
            time.sleep(0.5)

    def _start_gui_update_loop(self):
        def update_gui():
            for _ in range(20):
                try:
                    update_type, *args = self.gui_update_queue.get_nowait()
                    if update_type == 'steering':
                        self._update_display(*args)
                    elif update_type == 'log':
                        self._write_log(*args)
                    elif update_type == 'connection_status':
                        is_connected = args[0]
                        self.connect_btn.config(text="Disconnect" if is_connected else "Connect")
                        self.status_led.config(bg='green' if is_connected else 'red')
                except Empty:
                    break
            while self.log_queue.qsize() > 20:
                self.log_queue.get_nowait()
            while not self.log_queue.empty():
                message, level = self.log_queue.get_nowait()
                self._write_log(message, level)
            self.root.after(15, update_gui)
        update_gui()

    def _update_display(self, steering, throttle, left, right):
        x = 150 + steering * 100
        if abs(x - self.current_x) > 2:
            self.steering_canvas.coords(self.steering_indicator, x-5, 45, x+5, 55)
            self.current_x = x
        self.rt_progress['value'] = max(0, throttle) * 100
        self.lt_progress['value'] = max(0, -throttle) * 100
        self.debug_vars['steering'].set(f"{steering:.2f}")
        self.debug_vars['throttle'].set(f"{throttle:.2f}")

    def _process_controls(self):
        if hasattr(self, 'joystick') and self.joystick is not None:
            try:
                pygame.event.pump()
                # Read and invert X axis if needed
                steering_raw = self.joystick.get_axis(self.controller_mapping['right_x'])
                if self.invert_x:
                    steering_raw = -steering_raw
                
                rt = (self.joystick.get_axis(5) + 1) / 2
                lt = (self.joystick.get_axis(4) + 1) / 2
                steering = self._process_axis(steering_raw)
                throttle = rt - lt
                left = throttle + steering
                right = throttle - steering
                scale = 1.0 / max(abs(left), abs(right), 1.0)
                left = left * scale
                right = right * scale
                lf = left
                lr = left
                rf = right
                rr = right
                self.gui_update_queue.put(('steering', steering, throttle, left, right))
                if self.connected and (
                    not hasattr(self, 'last_lf') or
                    abs(lf - getattr(self, 'last_lf', 0.0)) > 0.01 or
                    abs(lr - getattr(self, 'last_lr', 0.0)) > 0.01 or
                    abs(rf - getattr(self, 'last_rf', 0.0)) > 0.01 or
                    abs(rr - getattr(self, 'last_rr', 0.0)) > 0.01
                ):
                    self._send_command(lf, lr, rf, rr)
                    self.last_lf, self.last_lr, self.last_rf, self.last_rr = lf, lr, rf, rr

                # --- Xbox A button connect/disconnect toggle ---
                a_button_current = self.joystick.get_button(self.controller_mapping['a_button'])
                if a_button_current and not self.a_button_prev_state:
                    self._handle_connection()
                self.a_button_prev_state = a_button_current

            except pygame.error:
                self._log("Controller disconnected!", "ERROR")
                self.joystick = None

    def _process_axis(self, value):
        abs_val = abs(value)
        if abs_val < self.deadzone:
            return 0.0
        return self.response_curve((abs_val - self.deadzone) / (1 - self.deadzone)) * np.sign(value)

    def _send_command(self, lf, lr, rf, rr):
        try:
            now = time.time()
            self.last_cmd_id += 1
            cmd_id = self.last_cmd_id
            cmd = {
                'cmd_id': cmd_id,
                'lf': float(np.clip(lf, -1.0, 1.0)),
                'lr': float(np.clip(lr, -1.0, 1.0)),
                'rf': float(np.clip(rf, -1.0, 1.0)),
                'rr': float(np.clip(rr, -1.0, 1.0))
            }
            self.cmd_send_times[cmd_id] = now
            data = cbor2.dumps(cmd)
            self.data_queue.put_nowait(data)
            self._log(
                f"Sent command: id={cmd_id} lf={lf:.2f}, lr={lr:.2f}, rf={rf:.2f}, rr={rr:.2f}"
            )
        except Exception as e:
            self._log(f"Command error: {str(e)}", "ERROR")

    def _handle_backend_response(self, resp):
        if 'cmd_id' in resp:
            cmd_id = resp['cmd_id']
            now = time.time()
            if cmd_id in self.cmd_send_times:
                latency_ms = int((now - self.cmd_send_times[cmd_id]) * 1000)
                self.debug_vars['latency'].set(f"{latency_ms} ms")
                del self.cmd_send_times[cmd_id]

    def _handle_connection(self):
        if self.connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        ip = self.ip_entry.get()
        try:
            self.sock = socket.create_connection((ip, 65432), timeout=2)
            self.connected = True
            self.gui_update_queue.put(('connection_status', True))
            self._log(f"Connected to {ip}")
        except Exception as e:
            self._log(f"Connection failed: {str(e)}", "ERROR")
            self.connected = False
            self.gui_update_queue.put(('connection_status', False))

    def _is_socket_closed(self):
        if not self.sock:
            return True
        try:
            data = self.sock.recv(16, socket.MSG_DONTWAIT | socket.MSG_PEEK)
            if len(data) == 0:
                return True
        except BlockingIOError:
            return False
        except ConnectionResetError:
            return True
        except Exception:
            return False
        return False

    def _disconnect(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
        self.connected = False
        with self.data_queue.mutex:
            self.data_queue.queue.clear()
        with self.gui_update_queue.mutex:
            self.gui_update_queue.queue.clear()
        self.gui_update_queue.put(('connection_status', False))
        self._log("Disconnected from robot")

    def _emergency_stop(self):
        if self.connected:
            self._send_command(0, 0, 0, 0)
        self._log("EMERGENCY STOP ACTIVATED", "CRITICAL")
        self.status_led.config(bg='red')
        self.connected = False

    def _update_deadzone(self, value):
        self.deadzone = float(value)/100
        self.deadzone_label.config(text=f"{self.deadzone:.2f}")

    def _log(self, message, level="INFO"):
        if self.logging_enabled:
            self.log_queue.put((message, level))

    def _write_log(self, message, level):
        timestamp = time.strftime("%H:%M:%S")
        color_map = {
            "INFO": "black",
            "ERROR": "red",
            "CRITICAL": "darkred"
        }
        self.log_text.config(state='normal')
        self.log_text.insert('end', f"[{timestamp}] {message}\n")
        self.log_text.tag_config(level, foreground=color_map.get(level, "black"))
        self.log_text.tag_add(level, "end-2l", "end-1l")
        self.log_text.see('end')
        self.log_text.config(state='disabled')

    def _shutdown(self):
        self.stop_flag.set()
        if self.sock:
            self.sock.close()
        pygame.quit()
        self.root.destroy()

if __name__ == "__main__":
    SoccerRobotController()
