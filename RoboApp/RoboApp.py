import tkinter as tk
from tkinter import ttk
import pygame
import socket
import threading
import json
import time
import numpy as np
from scipy.interpolate import CubicSpline
from queue import Queue, Empty

class SoccerRobotController:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Soccer Robot Controller")
        self.root.geometry("1000x800")
        
        # Thread-safe communication
        self.data_queue = Queue()
        self.log_queue = Queue()
        self.gui_update_queue = Queue()
        
        # State variables
        self.last_left = 0.0
        self.last_right = 0.0
        self.current_x = 150
        self.frame_time = 0.015  # ~66Hz refresh rate
        self.deadzone = 0.15
        self.response_curve = CubicSpline([0, 0.2, 0.5, 0.8, 1], [0, 0.1, 0.4, 0.9, 1])
        self.controller_mapping = {'right_x': 2, 'rt': 5, 'lt': 4}
        
        # Network setup
        self.sock = None
        self.connected = False
        self.stop_flag = False
        
        self._setup_gui()
        self._init_controller()
        self._start_control_thread()
        self._start_network_thread()
        self._start_gui_update_loop()
        
        self.root.protocol("WM_DELETE_WINDOW", self._shutdown)
        self.root.mainloop()

    def _setup_gui(self):
        # Connection Panel
        conn_frame = ttk.LabelFrame(self.root, text="Connection")
        conn_frame.pack(fill='x', padx=10, pady=5)
        
        ttk.Label(conn_frame, text="Robot IP:").grid(row=0, column=0, padx=5)
        self.ip_entry = ttk.Entry(conn_frame, width=20)
        self.ip_entry.grid(row=0, column=1, padx=5)
        self.ip_entry.insert(0, "192.168.4.1")
        
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self._handle_connection)
        self.connect_btn.grid(row=0, column=2, padx=5)
        
        self.status_led = tk.Canvas(conn_frame, width=20, height=20, bg='red')
        self.status_led.grid(row=0, column=3, padx=5)
        
        # Control Configuration
        config_frame = ttk.LabelFrame(self.root, text="Control Settings")
        config_frame.pack(fill='x', padx=10, pady=5)
        
        ttk.Label(config_frame, text="Deadzone:").grid(row=0, column=0, padx=5)
        self.deadzone_label = ttk.Label(config_frame, text="0.15")
        self.deadzone_label.grid(row=0, column=2, padx=5)
        self.deadzone_slider = ttk.Scale(config_frame, from_=0, to=50, command=self._update_deadzone)
        self.deadzone_slider.set(15)
        self.deadzone_slider.grid(row=0, column=1, padx=5)
        
        # Visualization Panel
        vis_frame = ttk.LabelFrame(self.root, text="Controller Input")
        vis_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        self.steering_canvas = tk.Canvas(vis_frame, width=300, height=100, bg='#f0f0f0')
        self.steering_canvas.pack(pady=10)
        self.steering_indicator = self.steering_canvas.create_oval(145, 45, 155, 55, fill='blue')
        
        self.rt_progress = ttk.Progressbar(vis_frame, length=200, mode='determinate')
        self.rt_progress.pack(pady=5)
        self.lt_progress = ttk.Progressbar(vis_frame, length=200, mode='determinate')
        self.lt_progress.pack(pady=5)
        
        # Debug Information
        debug_frame = ttk.LabelFrame(self.root, text="System Status")
        debug_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        self.debug_vars = {
            'steering': tk.StringVar(value="0.00"),
            'throttle': tk.StringVar(value="0.00"),
            'packet_rate': tk.StringVar(value="0.0/s")
        }
        
        ttk.Label(debug_frame, text="Steering:").pack(anchor='w', padx=10)
        ttk.Label(debug_frame, textvariable=self.debug_vars['steering']).pack(anchor='w', padx=20)
        ttk.Label(debug_frame, text="Throttle:").pack(anchor='w', padx=10)
        ttk.Label(debug_frame, textvariable=self.debug_vars['throttle']).pack(anchor='w', padx=20)
        ttk.Label(debug_frame, text="Packet Rate:").pack(anchor='w', padx=10)
        ttk.Label(debug_frame, textvariable=self.debug_vars['packet_rate']).pack(anchor='w', padx=20)
        
        # System Log
        log_frame = ttk.LabelFrame(self.root, text="Event Log")
        log_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        self.log_text = tk.Text(log_frame, wrap=tk.WORD, state='disabled')
        scroll = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scroll.set)
        self.log_text.pack(side='left', fill='both', expand=True)
        scroll.pack(side='right', fill='y')
        
        # Emergency Stop
        self.estop_btn = ttk.Button(self.root, text="EMERGENCY STOP", command=self._emergency_stop)
        self.estop_btn.pack(pady=10)

    def _init_controller(self):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self._log("Controller connected: " + self.joystick.get_name())
        else:
            self._log("No controller detected!", "ERROR")

    def _start_control_thread(self):
        def control_loop():
            while not self.stop_flag:
                self._process_controls()
                time.sleep(0.02)  # 50Hz control loop
        threading.Thread(target=control_loop, daemon=True).start()

    def _process_controls(self):
        if hasattr(self, 'joystick'):
            try:
                pygame.event.pump()
                
                # Read controller inputs
                steering_raw = self.joystick.get_axis(self.controller_mapping['right_x'])
                rt = (self.joystick.get_axis(5) + 1) / 2  # Right trigger
                lt = (self.joystick.get_axis(4) + 1) / 2  # Left trigger
                
                # Process inputs
                steering = self._process_axis(steering_raw)
                throttle = rt - lt
                left, right = self._differential_mix(throttle, steering)
                
                # Queue GUI updates
                self.gui_update_queue.put(('steering', steering, throttle, left, right))
                
                # Send commands
                if self.connected and (abs(left - self.last_left) > 0.01 or abs(right - self.last_right) > 0.01):
                    self._send_command(left, right)
                    self.last_left = left
                    self.last_right = right
                    
            except pygame.error:
                self._log("Controller disconnected!", "ERROR")
                del self.joystick

    def _start_gui_update_loop(self):
        def update_gui():
            while not self.gui_update_queue.empty():
                update_type, *args = self.gui_update_queue.get_nowait()
                if update_type == 'steering':
                    self._update_display(*args)
                elif update_type == 'log':
                    self._write_log(*args)
            
            # Process logs
            while not self.log_queue.empty():
                message, level = self.log_queue.get_nowait()
                self._write_log(message, level)
            
            self.root.after(15, update_gui)
        update_gui()

    def _update_display(self, steering, throttle, left, right):
        """Update GUI elements with current values (main thread only)"""
        # Steering indicator
        x = 150 + steering * 100
        if abs(x - self.current_x) > 2:
            self.steering_canvas.coords(self.steering_indicator, x-5, 45, x+5, 55)
            self.current_x = x
        
        # Progress bars
        self.rt_progress['value'] = max(0, throttle) * 100
        self.lt_progress['value'] = max(0, -throttle) * 100
        
        # Debug labels
        self.debug_vars['steering'].set(f"{steering:.2f}")
        self.debug_vars['throttle'].set(f"{throttle:.2f}")

    def _differential_mix(self, throttle, steering):
        left = throttle + steering
        right = throttle - steering
        scale = 1.0 / max(abs(left), abs(right), 1.0)
        return left * scale, right * scale

    def _process_axis(self, value):
        abs_val = abs(value)
        if abs_val < self.deadzone:
            return 0.0
        return self.response_curve((abs_val - self.deadzone) / (1 - self.deadzone)) * np.sign(value)

    def _send_command(self, left, right):
        try:
            cmd = {
                'timestamp': time.time(),
                'left': np.clip(left, -1.0, 1.0),
                'right': np.clip(right, -1.0, 1.0)
            }
            data = json.dumps(cmd, separators=(',', ':')).encode()
            self.data_queue.put_nowait(data)
        except Exception as e:
            self._log(f"Command error: {str(e)}", "ERROR")

    def _start_network_thread(self):
        def network_loop():
            packet_count = 0
            last_update = time.time()
            
            while not self.stop_flag:
                try:
                    data = self.data_queue.get(timeout=0.1)
                    if self.connected:
                        header = len(data).to_bytes(4, 'big')
                        self.sock.sendall(header + data)
                        packet_count += 1
                        
                        # Update packet rate every second
                        if time.time() - last_update >= 1:
                            self.debug_vars['packet_rate'].set(f"{packet_count}/s")
                            packet_count = 0
                            last_update = time.time()
                            
                except Empty:
                    continue
                except Exception as e:
                    self._log(f"Network error: {str(e)}", "ERROR")
                    self.connected = False
        threading.Thread(target=network_loop, daemon=True).start()

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
            self.connect_btn.config(text="Disconnect")
            self.status_led.config(bg='green')
            self._log(f"Connected to {ip}")
        except Exception as e:
            self._log(f"Connection failed: {str(e)}", "ERROR")
            self.connected = False
            self.status_led.config(bg='red')

    def _disconnect(self):
        if self.sock:
            self.sock.close()
        self.connected = False
        self.connect_btn.config(text="Connect")
        self.status_led.config(bg='red')
        self._log("Disconnected from robot")

    def _emergency_stop(self):
        if self.connected:
            self._send_command(0, 0)
        self._log("EMERGENCY STOP ACTIVATED", "CRITICAL")
        self.status_led.config(bg='red')
        self.connected = False

    def _update_deadzone(self, value):
        self.deadzone = float(value)/100
        self.deadzone_label.config(text=f"{self.deadzone:.2f}")

    def _log(self, message, level="INFO"):
        self.log_queue.put((message, level))

    def _write_log(self, message, level):
        timestamp = time.strftime("%H:%M:%S")
        color_map = {
            "INFO": "black",
            "ERROR": "red",
            "CRITICAL": "darkred"
        }
        self.log_text.config(state='normal')
        self.log_text.insert('end', f"[{timestamp}] {message}\n", level)
        self.log_text.tag_config(level, foreground=color_map.get(level, "black"))
        self.log_text.see('end')
        self.log_text.config(state='disabled')

    def _shutdown(self):
        self.stop_flag = True
        if self.sock:
            self.sock.close()
        self.root.destroy()

if __name__ == "__main__":
    SoccerRobotController()
