#frontend Code
import tkinter as tk
from tkinter import SEL, ttk
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
        self.data_queue = Queue()

        # Controller state
        self.sock = None
        self.connected = False
        self.stop_flag = False
        self.deadzone = 0.15
        self.response_curve = CubicSpline([0, 0.2, 0.5, 0.8, 1], [0, 0.1, 0.4, 0.9, 1])
        self.controller_mapping = {
            'right_x': 2,  # Verified Xbox controller mapping
            'rt': 5,
            'lt': 4
        }
        self.packet_count = 0
        self.last_packet_time = time.time()

        self._setup_gui()
        self._init_controller()
        
        threading.Thread(target=self._control_loop, daemon=True).start()
        threading.Thread(target=self._network_loop, daemon=True).start()
        
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
        self.deadzone_slider = ttk.Scale(config_frame, from_=0, to=50,
                                      command=lambda v: self._update_deadzone(v))
        self.deadzone_slider.grid(row=0, column=1, padx=5)
        self.deadzone_slider.set(15)
        
        # Visualization Panel
        vis_frame = ttk.LabelFrame(self.root, text="Controller Input")
        vis_frame.pack(fill='both', expand=True, padx=10, pady=5)
        self.steering_canvas = tk.Canvas(vis_frame, width=300, height=100, bg='#f0f0f0')
        self.steering_canvas.pack(pady=10)
        self.steering_indicator = self.steering_canvas.create_rectangle(145, 20, 155, 80, fill='blue')
        self.rt_progress = ttk.Progressbar(vis_frame, length=200, mode='determinate')
        self.rt_progress.pack(pady=5)
        self.lt_progress = ttk.Progressbar(vis_frame, length=200, mode='determinate')
        self.lt_progress.pack(pady=5)
        
        # Debug Information
        debug_frame = ttk.LabelFrame(self.root, text="System Status")
        debug_frame.pack(fill='both', expand=True, padx=10, pady=5)
        self.debug_labels = {
            'latency': ttk.Label(debug_frame, text="Latency: -- ms"),
            'packet_rate': ttk.Label(debug_frame, text="Packet Rate: --/s"),
            'raw_steering': ttk.Label(debug_frame, text="Raw Steering: --"),
            'raw_throttle': ttk.Label(debug_frame, text="Raw Throttle: --")
        }
        for label in self.debug_labels.values():
            label.pack(anchor='w', padx=10, pady=2)
        
        # System Log
        log_frame = ttk.LabelFrame(self.root, text="Event Log")
        log_frame.pack(fill='both', expand=True, padx=10, pady=5)
        self.log_text = tk.Text(log_frame, wrap=tk.WORD, state='disabled')
        scroll = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scroll.set)
        self.log_text.pack(side='left', fill='both', expand=True)
        scroll.pack(side='right', fill='y')
        
        # Emergency Stop
        self.estop_btn = ttk.Button(self.root, text="EMERGENCY STOP", 
                                  style='Emergency.TButton', command=self._emergency_stop)
        self.estop_btn.pack(pady=10)
    def _update_display(self, steering, throttle, left, right):
        """Update all GUI elements with current control values"""
        try:
            # Update steering visualization
            x = 150 + steering * 100
            self.steering_canvas.coords(self.steering_indicator, x-5, 20, x+5, 80)
            
            # Update trigger progress bars
            self.rt_progress['value'] = max(0, throttle) * 100
            self.lt_progress['value'] = max(0, -throttle) * 100
            
            # Update debug labels
            self.debug_labels['raw_steering'].config(text=f"Steering: {steering:.2f}")
            self.debug_labels['raw_throttle'].config(text=f"Throttle: {throttle:.2f}")
            self.debug_labels['latency'].config(text=f"Motors: L={left:.2f}, R={right:.2f}")
            
        except Exception as e:
            self._log(f"Display update error: {str(e)}", "ERROR")

    def _init_controller(self):
        pygame.init()
        pygame.joystick.init()
        try:
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self._log(f"Controller connected: {self.joystick.get_name()}", "INFO")
                self._log(f"Detected {self.joystick.get_numaxes()} axes", "DEBUG")
            else:
                self._log("No controller detected!", "ERROR")
        except Exception as e:
            self._log(f"Controller init error: {str(e)}", "ERROR")

    def _handle_connection(self):
        if self.connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        ip = self.ip_entry.get()
        try:
            if self.sock:
                self.sock.close()
            self.sock = socket.socket()
            self.sock.settimeout(2)
            self.sock.connect((ip, 65432))
            self.connected = True
            self.connect_btn.config(text="Disconnect")
            self.status_led.config(bg='green')
            self._log(f"Connected to {ip}")
        except Exception as e:
            self.connected = False
            self.connect_btn.config(text="Connect")
            self.status_led.config(bg='red')
            self._log(f"Connection failed: {str(e)}", "ERROR")

    def _disconnect(self):
        if self.sock:
            self.sock.close()
        self.connected = False
        self.connect_btn.config(text="Connect")
        self.status_led.config(bg='red')
        self._log("Disconnected from robot")

    def _differential_mix(self, throttle, steering):
        """
        Convert throttle (forward/backward) and steering (left/right) 
        into left/right motor speeds for tank drive.
        """
        left = throttle + steering
        right = throttle - steering
    
        # Normalize to prevent exceeding ±1.0
        max_speed = max(abs(left), abs(right), 1.0)
        return left / max_speed, right / max_speed

    def _control_loop(self):
        """Send commands only when values change significantly"""
        last_left = 0.0
        last_right = 0.0
        threshold = 0.01  # 1% change threshold

        while not self.stop_flag:
            if hasattr(self, 'joystick'):
                try:
                    pygame.event.pump()
                
                    # Get raw controller inputs
                    steering_raw = self.joystick.get_axis(self.controller_mapping['right_x'])
                    rt = (self.joystick.get_axis(self.controller_mapping['rt']) + 1) / 2
                    lt = (self.joystick.get_axis(self.controller_mapping['lt']) + 1) / 2
                
                    # Process inputs
                    steering = self._process_axis(steering_raw)
                    throttle = rt - lt
                    # Calculate motor speeds
                    left, right = self._differential_mix(throttle, steering)
                
                    # Only send if values changed significantly
                    if (abs(left - last_left) > threshold or 
                        abs(right - last_right) > threshold):
                        self._send_command(left, right)
                        last_left = left
                        last_right = right
                
                    # Update UI
                    self._update_display(steering, throttle, left, right)
                
                except pygame.error:
                    self._log("Controller disconnected!", "ERROR")
                    del self.joystick
            time.sleep(0.02)  # 50Hz update rate (was 0.02)

    def _process_axis(self, value):
        abs_val = abs(value)
        if abs_val < self.deadzone:
            return 0.0
        return self.response_curve((abs_val - self.deadzone) / (1 - self.deadzone)) * np.sign(value)

    def _calculate_speeds(self, throttle, steering):
        left = throttle + steering
        right = throttle - steering
        max_speed = max(abs(left), abs(right), 1.0)
        return left/max_speed, right/max_speed

    def _send_command(self, left, right):
        """Validate and queue motor commands"""
        # Deadzone for zero values
        if abs(left) < 0.01 and abs(right) < 0.01:
            left = right = 0.0
        try:
            # Validate input types and ranges
            if not (isinstance(left, (int, float)) and isinstance(right, (int, float))):
                raise TypeError("Invalid command types")
                
            cmd = {
                'timestamp': time.time(),
                'left': np.clip(float(left), -1.0, 1.0),
                'right': np.clip(float(right), -1.0, 1.0)
            }
            
            # Optimized JSON serialization
            json_data = json.dumps(cmd, ensure_ascii=False, separators=(',', ':')).encode()
            
            # Size validation
            if len(json_data) > 1024:
                raise ValueError(f"Command too large ({len(json_data)} bytes)")
            
            # Log actual JSON being sent
            self._log(f"Sending: {json_data.decode()}", "DEBUG")
            
            self.data_queue.put(json_data)
            
        except Exception as e:
            self._log(f"Command error: {str(e)}", "ERROR")
            self.connected = False
            self.status_led.config(bg='red')

    def _network_loop(self):
        """Optimized network handler with rate limiting"""
        while not self.stop_flag:
            if self.connected and hasattr(self, 'joystick'):
                try:
                    # Get data with timeout
                    json_data = self.data_queue.get(timeout=0.01)
                    
                    # Send with header
                    header = len(json_data).to_bytes(4, 'big')
                    self.sock.sendall(header + json_data)
                    
                    # Update network stats
                    self.packet_count += 1
                    now = time.time()
                    if now - self.last_packet_time >= 1:
                        rate = self.packet_count / (now - self.last_packet_time)
                        self.debug_labels['packet_rate'].config(text=f"Packet Rate: {rate:.1f}/s")
                        self.packet_count = 0
                        self.last_packet_time = now
                        
                except Empty:
                    pass  # Normal queue timeout
                except socket.error as e:
                    self._log(f"Socket error: {str(e)}", "ERROR")
                    self.connected = False
                except Exception as e:
                    self._log(f"Network error: {str(e)}", "ERROR")
                    self.connected = False
                    self.status_led.config(bg='red')

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
