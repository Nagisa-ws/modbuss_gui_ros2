from std_msgs.msg import Float32MultiArray
import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient
import tkinter as tk
from tkinter import ttk, messagebox
import struct
from datetime import datetime

class ModbusPollGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Modbus Poll Tool")
        self.root.geometry("1050x750")
        
        self.client = None
        self.polling = False
        self.timer_id = None
        
        self.setup_ui()
        
        # Inisialisasi ROS2 node untuk Publisher
        rclpy.init(args=[])
        self.ros_node = rclpy.create_node('modbus_publisher')
        
        # Publisher untuk AI dan AO
        self.ai_pub = self.ros_node.create_publisher(Float32MultiArray, '/modbus/ai', 10)
        self.ao_pub = self.ros_node.create_publisher(Float32MultiArray, '/modbus/ao', 10)
        
        # Timer untuk spin ROS2 di background
        self.ros_timer = self.root.after(100, self.spin_ros)
    
    def setup_ui(self):
        # Main container
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # === LEFT PANEL: Connection ===
        left_frame = ttk.LabelFrame(main_frame, text="Connection", padding="10")
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0,10))
        
        ttk.Label(left_frame, text="Port:").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.port_var = tk.StringVar(value="/dev/ttyUSB0")
        ttk.Entry(left_frame, textvariable=self.port_var, width=15).grid(row=0, column=1, pady=2)
        
        ttk.Label(left_frame, text="Baud rate:").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.baud_var = tk.StringVar(value="9600")
        ttk.Combobox(left_frame, textvariable=self.baud_var, 
                     values=["1200","2400","4800","9600","19200","38400","57600","115200"], 
                     width=13).grid(row=1, column=1, pady=2)
        
        ttk.Label(left_frame, text="Parity:").grid(row=2, column=0, sticky=tk.W, pady=2)
        self.parity_var = tk.StringVar(value="N")
        ttk.Combobox(left_frame, textvariable=self.parity_var, values=["N", "E", "O"], width=13).grid(row=2, column=1, pady=2)
        
        ttk.Label(left_frame, text="Stop Bits:").grid(row=3, column=0, sticky=tk.W, pady=2)
        self.stopbits_var = tk.StringVar(value="1")
        ttk.Combobox(left_frame, textvariable=self.stopbits_var, values=["1", "2"], width=13).grid(row=3, column=1, pady=2)
        
        ttk.Label(left_frame, text="Time out:").grid(row=4, column=0, sticky=tk.W, pady=2)
        self.timeout_var = tk.StringVar(value="3")
        ttk.Entry(left_frame, textvariable=self.timeout_var, width=15).grid(row=4, column=1, pady=2)
        
        ttk.Label(left_frame, text="Data Bits:").grid(row=5, column=0, sticky=tk.W, pady=2)
        self.databits_var = tk.StringVar(value="8")
        ttk.Combobox(left_frame, textvariable=self.databits_var, values=["7", "8"], width=13).grid(row=5, column=1, pady=2)
        
        ttk.Label(left_frame, text="Mode:").grid(row=6, column=0, sticky=tk.W, pady=2)
        self.mode_var = tk.StringVar(value="RTU")
        ttk.Combobox(left_frame, textvariable=self.mode_var, values=["RTU", "ASCII"], width=13).grid(row=6, column=1, pady=2)
        
        self.connect_btn = ttk.Button(left_frame, text="Connect", command=self.toggle_connect)
        self.connect_btn.grid(row=7, column=0, columnspan=2, pady=20)
        
        self.status_label = ttk.Label(left_frame, text="● Disconnected", foreground="red")
        self.status_label.grid(row=8, column=0, columnspan=2)
        
        ttk.Label(left_frame, text="Poll (ms):").grid(row=9, column=0, sticky=tk.W, pady=(20,2))
        self.poll_var = tk.StringVar(value="1000")
        ttk.Entry(left_frame, textvariable=self.poll_var, width=15).grid(row=9, column=1, pady=(20,2))
        
        self.poll_btn = ttk.Button(left_frame, text="Start Poll", command=self.toggle_poll)
        self.poll_btn.grid(row=10, column=0, columnspan=2, pady=5)
        
        # === RIGHT PANEL: R/W Definition ===
        right_frame = ttk.LabelFrame(main_frame, text="R/W Definition", padding="10")
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # Device ID
        row0 = ttk.Frame(right_frame)
        row0.pack(fill=tk.X, pady=2)
        ttk.Label(row0, text="Device_id:", width=12).pack(side=tk.LEFT)
        self.device_id_var = tk.StringVar(value="1")
        ttk.Entry(row0, textvariable=self.device_id_var, width=15).pack(side=tk.LEFT)
        
        # Address Mode
        row1 = ttk.Frame(right_frame)
        row1.pack(fill=tk.X, pady=2)
        ttk.Label(row1, text="Address Mode:", width=12).pack(side=tk.LEFT)
        self.addr_mode_var = tk.StringVar(value="Dec")
        ttk.Combobox(row1, textvariable=self.addr_mode_var, values=["Dec", "Hex"], width=13).pack(side=tk.LEFT)
        
        # Data Format (BARU: untuk Float/Integer)
        row_format = ttk.Frame(right_frame)
        row_format.pack(fill=tk.X, pady=2)
        ttk.Label(row_format, text="Data Format:", width=12).pack(side=tk.LEFT)
        self.format_var = tk.StringVar(value="Float (mA)")
        format_combo = ttk.Combobox(row_format, textvariable=self.format_var,
                                    values=["Float (mA)", "Integer (uA/1000)"],
                                    width=15, state="readonly")
        format_combo.pack(side=tk.LEFT)
        
        # Function code
        row2 = ttk.Frame(right_frame)
        row2.pack(fill=tk.X, pady=2)
        ttk.Label(row2, text="Function:", width=12).pack(side=tk.LEFT)
        self.func_var = tk.StringVar(value="03 Read Holding Registers")
        func_combo = ttk.Combobox(row2, textvariable=self.func_var, width=25)
        func_combo.pack(side=tk.LEFT)
        func_combo['values'] = [
            "01 Read Coils",
            "02 Read Discrete Inputs",
            "03 Read Holding Registers",
            "04 Read Input Registers",
            "05 Write Single Coil",
            "06 Write Single Register",
            "15 Write Multiple Coils",
            "16 Write Multiple Registers"
        ]
        
        
        # Address register
        row3 = ttk.Frame(right_frame)
        row3.pack(fill=tk.X, pady=2)
        ttk.Label(row3, text="Address register:", width=12).pack(side=tk.LEFT)
        self.start_addr_var = tk.StringVar(value="0")
        ttk.Entry(row3, textvariable=self.start_addr_var, width=15).pack(side=tk.LEFT)
        
        # Quantity/Count
        row4 = ttk.Frame(right_frame)
        row4.pack(fill=tk.X, pady=2)
        ttk.Label(row4, text="Quantity/Count:", width=12).pack(side=tk.LEFT)
        self.quantity_var = tk.StringVar(value="4")
        ttk.Entry(row4, textvariable=self.quantity_var, width=15).pack(side=tk.LEFT)
        
        # Apply button
        self.apply_btn = ttk.Button(right_frame, text="Apply", command=self.apply_read)
        self.apply_btn.pack(pady=10)
        
        # === TABLE ===
        table_frame = ttk.LabelFrame(right_frame, text="Register Table (Double click to write)", padding="5")
        table_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        
        scroll_y = ttk.Scrollbar(table_frame)
        scroll_y.pack(side=tk.RIGHT, fill=tk.Y)
        scroll_x = ttk.Scrollbar(table_frame, orient=tk.HORIZONTAL)
        scroll_x.pack(side=tk.BOTTOM, fill=tk.X)
        
        columns = ("NO", "Register", "Value", "Mode")
        self.tree = ttk.Treeview(table_frame, columns=columns, show="headings",
                                  yscrollcommand=scroll_y.set, xscrollcommand=scroll_x.set)
        
        self.tree.heading("NO", text="NO")
        self.tree.column("NO", width=50)
        self.tree.heading("Register", text="Register")
        self.tree.column("Register", width=120)
        self.tree.heading("Value", text="Value")
        self.tree.column("Value", width=200)
        self.tree.heading("Mode", text="Mode")
        self.tree.column("Mode", width=100)
        
        self.tree.pack(fill=tk.BOTH, expand=True)
        scroll_y.config(command=self.tree.yview)
        scroll_x.config(command=self.tree.xview)
        
        self.tree.bind("<Double-1>", self.on_double_click)
        
        # Log
        log_frame = ttk.LabelFrame(right_frame, text="Log", padding="5")
        log_frame.pack(fill=tk.X, pady=5)
        self.log_text = tk.Text(log_frame, height=5)
        self.log_text.pack(fill=tk.X)
    
    def log(self, msg):
        self.log_text.insert(tk.END, f"[{datetime.now().strftime('%H:%M:%S')}] {msg}\n")
        self.log_text.see(tk.END)
    
    def get_addr_value(self, addr_str):
        if self.addr_mode_var.get() == "Hex":
            return int(addr_str, 16)
        return int(addr_str)
    
    def get_ao_mode(self, address):
        if not self.client:
            return None
        try:
            device_id = int(self.device_id_var.get())
            if 0x0514 <= address <= 0x0517:
                result = self.client.read_holding_registers(
                    address=address, count=1, device_id=device_id
                )
                if not result.isError():
                    return result.registers[0]
        except:
            pass
        return None

    def spin_ros(self):
        """Spin ROS2 di background"""
        if hasattr(self, 'ros_node'):
            rclpy.spin_once(self.ros_node, timeout_sec=0)
        self.root.after(100, self.spin_ros)
    
    def publish_ai(self, ai_values):
        """Publish AI values ke topic ROS"""
        if hasattr(self, 'ai_pub'):
            msg = Float32MultiArray()
            msg.data = ai_values
            self.ai_pub.publish(msg)
    
    def publish_ao(self, ao_values):
        """Publish AO values ke topic ROS"""
        if hasattr(self, 'ao_pub'):
            msg = Float32MultiArray()
            msg.data = ao_values
            self.ao_pub.publish(msg)
    
    def read_ai_float(self, channel):
        """Baca AI float dari 0x00C8 (langsung mA)"""
        if not self.client:
            return None
        try:
            device_id = int(self.device_id_var.get())
            addr = 0x00C8 + (channel * 2)
            result = self.client.read_input_registers(
                address=addr, count=2, device_id=device_id
            )
            if not result.isError() and len(result.registers) == 2:
                # Coba byte order: little-endian (CDAB)
                bytes_data = result.registers[1].to_bytes(2, 'big') + result.registers[0].to_bytes(2, 'big')
                value = struct.unpack('>f', bytes_data)[0]
                
                # Validasi nilai wajar (0-25 mA)
                if 0 <= value <= 25:
                    return value
                else:
                    # Coba big-endian (ABCD)
                    bytes_data = result.registers[0].to_bytes(2, 'big') + result.registers[1].to_bytes(2, 'big')
                    value = struct.unpack('>f', bytes_data)[0]
                    if 0 <= value <= 25:
                        return value
                    return None
            return None
        except Exception as e:
            self.log(f"Read AI float error: {e}")
            return None
    
    def read_ao_float(self, channel):
        """Baca AO float dari 0x0000 (langsung mA)"""
        if not self.client:
            return None
        try:
            device_id = int(self.device_id_var.get())
            addr = 0x0000 + (channel * 2)
            result = self.client.read_holding_registers(
                address=addr, count=2, device_id=device_id
            )
            if not result.isError() and len(result.registers) == 2:
                bytes_data = result.registers[0].to_bytes(2, 'big') + result.registers[1].to_bytes(2, 'big')
                return struct.unpack('>f', bytes_data)[0]
        except Exception as e:
            self.log(f"Read AO float error: {e}")
        return None
    
    def write_ao_float(self, channel, value_ma):
        """Tulis AO float ke 0x0000 (langsung mA)"""
        if not self.client:
            return False
        try:
            device_id = int(self.device_id_var.get())
            addr = 0x0000 + (channel * 2)
            bytes_data = struct.pack('>f', value_ma)
            regs = [
                int.from_bytes(bytes_data[0:2], 'big'),
                int.from_bytes(bytes_data[2:4], 'big')
            ]
            result = self.client.write_registers(
                address=addr, values=regs, device_id=device_id
            )
            if not result.isError():
                self.log(f"Write AO{channel+1} = {value_ma} mA")
                return True
        except Exception as e:
            self.log(f"Write AO float error: {e}")
        return False
    
    def toggle_connect(self):
        if self.client and self.client.is_socket_open():
            self.client.close()
            self.client = None
            self.status_label.config(text="● Disconnected", foreground="red")
            self.connect_btn.config(text="Connect")
            self.log("Disconnected")
        else:
            try:
                self.client = ModbusSerialClient(
                    port=self.port_var.get(),
                    baudrate=int(self.baud_var.get()),
                    parity=self.parity_var.get(),
                    stopbits=int(self.stopbits_var.get()),
                    timeout=float(self.timeout_var.get()),
                    bytesize=int(self.databits_var.get())
                )
                if self.client.connect():
                    self.status_label.config(text="● Connected", foreground="green")
                    self.connect_btn.config(text="Disconnect")
                    self.log("Connected")
                else:
                    self.log("Connection failed")
            except Exception as e:
                self.log(f"Error: {e}")
        
    def apply_read(self):
        if not self.client or not self.client.is_socket_open():
            messagebox.showwarning("Warning", "Not connected")
            return
        
        try:
            device_id = int(self.device_id_var.get())
            start_addr = self.get_addr_value(self.start_addr_var.get())
            quantity = int(self.quantity_var.get())
            func = self.func_var.get()
            data_format = self.format_var.get()
            
            # ========== VALIDASI KONFIGURASI ==========
            is_write = any(x in func for x in ["Write", "05", "06", "15", "16"])
            
            is_valid = False
            
            if is_write:
                is_valid = True
            elif "03 Read Holding Registers" in func and 0 <= start_addr <= 7 and data_format == "Float (mA)":
                is_valid = True
            elif "03 Read Holding Registers" in func and 100 <= start_addr <= 103 and data_format == "Integer (uA/1000)":
                is_valid = True
            elif "04 Read Input Registers" in func and 100 <= start_addr <= 103 and data_format == "Integer (uA/1000)":
                is_valid = True
            elif "04 Read Input Registers" in func and 200 <= start_addr <= 207 and data_format == "Float (mA)":
                is_valid = True
            
            if not is_valid:
                for item in self.tree.get_children():
                    self.tree.delete(item)
                # Kosongkan tabel, tanpa pesan
                self.log(f"Konfigurasi tidak valid: Func={func}, Addr={start_addr}, Qty={quantity}, Format={data_format}")
                return
            
            # Clear table
            for item in self.tree.get_children():
                self.tree.delete(item)
            
            # Handle Write functions
            if "05 Write Single Coil" in func:
                result = self.client.write_coil(
                    address=start_addr, value=bool(quantity), device_id=device_id
                )
                self.log(f"Write Coil {start_addr} = {bool(quantity)}")
                result = self.client.read_coils(address=start_addr, count=1, device_id=device_id)
                
            elif "06 Write Single Register" in func:
                result = self.client.write_register(
                    address=start_addr, value=quantity, device_id=device_id
                )
                self.log(f"Write Register {start_addr} = {quantity}")
                result = self.client.read_holding_registers(address=start_addr, count=1, device_id=device_id)
                if not result.isError():
                    self.publish_ao(result.registers)
                    
            elif "15 Write Multiple Coils" in func:
                values = [bool(int(x)) for x in str(quantity).split(',')] if ',' in str(quantity) else [bool(quantity)]
                result = self.client.write_coils(
                    address=start_addr, values=values, device_id=device_id
                )
                if not result.isError():
                    self.log(f"Write Multiple Coils {start_addr} = {values}")
                else:
                    self.log(f"Write Multiple Coils failed")
                result = self.client.read_coils(
                    address=start_addr, count=len(values), device_id=device_id
                )

            elif "16 Write Multiple Registers" in func:
                if ',' in str(quantity):
                    values = [int(x.strip()) for x in str(quantity).split(',')]
                else:
                    values = [quantity]
                result = self.client.write_registers(
                    address=start_addr, values=values, device_id=device_id
                )
                if not result.isError():
                    self.log(f"Write Multiple Registers {start_addr} = {values}")
                else:
                    self.log(f"Write Multiple Registers failed")
                result = self.client.read_holding_registers(
                    address=start_addr, count=len(values), device_id=device_id
                )
                if not result.isError():
                    self.publish_ao(result.registers)
            
            # Handle Read functions
            elif "04 Read Input Registers" in func and data_format == "Float (mA)":
                # Baca AI Float dari 0x00C8 (address 200-207)
                ai_values = []
                for i in range(4):  # 4 channel, bukan quantity
                    value = self.read_ai_float(i)
                    if value is not None:
                        ai_values.append(value)
                        addr_display = f"0x{0x00C8 + (i*2):04X}" if self.addr_mode_var.get() == "Hex" else str(0x00C8 + (i*2))
                        self.tree.insert("", tk.END, values=(i+1, addr_display, f"{value:.3f} mA", ""))
                    else:
                        ai_values.append(0)
                        addr_display = f"0x{0x00C8 + (i*2):04X}" if self.addr_mode_var.get() == "Hex" else str(0x00C8 + (i*2))
                        self.tree.insert("", tk.END, values=(i+1, addr_display, "Error", ""))
                
                self.publish_ai(ai_values)
                self.log("Read 4 AI float channels")
                return
                
            elif "03 Read Holding Registers" in func and data_format == "Float (mA)":
                # Baca AO Float dari 0x0000 (address 0-7)
                ao_values = []
                for i in range(4):  # 4 channel, bukan quantity
                    value = self.read_ao_float(i)
                    ao_values.append(value if value else 0)
                    addr_display = f"0x{0x0000 + (i*2):04X}" if self.addr_mode_var.get() == "Hex" else str(0x0000 + (i*2))
                    self.tree.insert("", tk.END, values=(i+1, addr_display, f"{value:.3f} mA" if value else "Error", ""))
                
                self.publish_ao(ao_values)
                self.log("Read 4 AO float channels")
                return
            
            elif "03 Read Holding Registers" in func:
                result = self.client.read_holding_registers(
                    address=start_addr, count=quantity, device_id=device_id
                )
                if not result.isError():
                    # Konversi uA ke mA
                    ao_ma_values = [val / 1000.0 for val in result.registers]
                    self.publish_ao(ao_ma_values)

            elif "04 Read Input Registers" in func:
                result = self.client.read_input_registers(
                    address=start_addr, count=quantity, device_id=device_id
                )
                if not result.isError():
                    # Konversi uA ke mA
                    ai_ma_values = [val / 1000.0 for val in result.registers]
                    self.publish_ai(ai_ma_values)
                    
            elif "01 Read Coils" in func:
                result = self.client.read_coils(
                    address=start_addr, count=quantity, device_id=device_id
                )
            elif "02 Read Discrete Inputs" in func:
                result = self.client.read_discrete_inputs(
                    address=start_addr, count=quantity, device_id=device_id
                )
            else:
                self.log("Unknown function")
                return
            
            # Tampilkan hasil untuk integer/standard read
            if not result.isError():
                values = result.registers if hasattr(result, 'registers') else result.bits
                for i, val in enumerate(values):
                    addr = start_addr + i
                    addr_display = f"0x{addr:04X}" if self.addr_mode_var.get() == "Hex" else str(addr)
                    
                    if data_format == "Integer (uA/1000)":
                        ma_value = val / 1000.0
                        display_val = f"{ma_value:.3f} mA"
                    else:
                        display_val = str(val)
                    
                    mode_val = self.get_ao_mode(addr)
                    mode_display = "4-20mA" if mode_val == 1 else ("0-20mA" if mode_val == 0 else "")
                    
                    self.tree.insert("", tk.END, values=(i+1, addr_display, display_val, mode_display))
                self.log(f"Read {len(values)} registers from {start_addr}")
            else:
                self.log(f"Failed to read: {result}")
                
        except Exception as e:
            self.log(f"Error: {e}")
    
    def on_double_click(self, event):
        selection = self.tree.selection()
        if not selection:
            return
        
        item = selection[0]
        values = self.tree.item(item, 'values')
        if not values:
            return
        
        register = values[1]
        current_value = values[2]
        
        if self.addr_mode_var.get() == "Hex":
            addr_int = int(register, 16)
        else:
            addr_int = int(register)
        
        is_ao_float_register = 0x0000 <= addr_int <= 0x0006
        
        dialog = tk.Toplevel(self.root)
        dialog.title("Write Register")
        dialog.geometry("400x250")
        dialog.transient(self.root)
        dialog.resizable(False, False)
        
        main_frame = ttk.Frame(dialog, padding="15")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        ttk.Label(main_frame, text=f"Register: {register}").pack(anchor=tk.W)
        ttk.Label(main_frame, text=f"Current Value: {current_value}").pack(anchor=tk.W, pady=(5,0))
        
        ttk.Label(main_frame, text="Device ID:").pack(anchor=tk.W, pady=(10,0))
        device_entry = ttk.Entry(main_frame)
        device_entry.insert(0, self.device_id_var.get())
        device_entry.pack(fill=tk.X, pady=2)
        
        ttk.Label(main_frame, text="New Value:").pack(anchor=tk.W, pady=(5,0))
        value_entry = ttk.Entry(main_frame)
        value_entry.pack(fill=tk.X, pady=2)
        
        button_frame = ttk.Frame(main_frame)
        button_frame.pack(fill=tk.X, pady=(15,0))
        
        def do_write():
            try:
                device_id = int(device_entry.get())
                new_value = value_entry.get()
                
                if is_ao_float_register:
                    # Write AO float
                    channel = (addr_int - 0x0000) // 2
                    success = self.write_ao_float(channel, float(new_value))
                    if success:
                        self.log(f"Written AO{channel+1} = {new_value} mA")
                        self.apply_read()
                        dialog.destroy()
                    else:
                        messagebox.showerror("Error", "Write failed", parent=dialog)
                else:
                    # Write integer
                    result = self.client.write_register(
                        address=addr_int, value=int(new_value), device_id=device_id
                    )
                    if not result.isError():
                        self.log(f"Written register {register} = {new_value}")
                        self.apply_read()
                        dialog.destroy()
                    else:
                        messagebox.showerror("Error", "Write failed", parent=dialog)
            except Exception as e:
                self.log(f"Write error: {e}")
                messagebox.showerror("Error", f"Write error: {e}", parent=dialog)
        
        ttk.Button(button_frame, text="Write", command=do_write).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Cancel", command=dialog.destroy).pack(side=tk.LEFT, padx=5)
        
        dialog.update_idletasks()
        x = self.root.winfo_x() + (self.root.winfo_width() // 2) - (dialog.winfo_width() // 2)
        y = self.root.winfo_y() + (self.root.winfo_height() // 2) - (dialog.winfo_height() // 2)
        dialog.geometry(f"+{x}+{y}")
        dialog.grab_set()
        dialog.focus_set()
    
    def toggle_poll(self):
        if not self.client or not self.client.is_socket_open():
            messagebox.showwarning("Warning", "Not connected")
            return
        
        self.polling = not self.polling
        if self.polling:
            self.poll_btn.config(text="Stop Poll")
            self.start_poll()
        else:
            self.poll_btn.config(text="Start Poll")
            if self.timer_id:
                self.root.after_cancel(self.timer_id)
    
    def start_poll(self):
        if self.polling and self.client:
            self.apply_read()
            interval = int(self.poll_var.get())
            self.timer_id = self.root.after(interval, self.start_poll)

def main():
    root = tk.Tk()
    app = ModbusPollGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()