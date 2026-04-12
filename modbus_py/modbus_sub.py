#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from datetime import datetime

class ModbusSubscriber(Node):
    def __init__(self):
        super().__init__('modbus_subscriber')
        
        # Subscribe ke topic AI
        self.ai_sub = self.create_subscription(
            Float32MultiArray,
            '/modbus/ai',
            self.ai_callback,
            10
        )
        
        # Subscribe ke topic AO
        self.ao_sub = self.create_subscription(
            Float32MultiArray,
            '/modbus/ao',
            self.ao_callback,
            10
        )
        
        self.get_logger().info("Modbus Subscriber Started")
        
        # === UNCOMMENT BARIS DI BAWAH INI JIKA INGIN SIMPAN KE FILE ===
        # self.file = open('modbus_data.csv', 'a')
        # self.file.write('Timestamp,Type,CH1,CH2,CH3,CH4\n')
    
    def ai_callback(self, msg):
        """Callback untuk data AI"""
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        
        # Tampilkan di terminal
        print(f"\n[AI DATA] {timestamp}")
        print(f"  AI1: {msg.data[0]:.3f} mA")
        print(f"  AI2: {msg.data[1]:.3f} mA")
        print(f"  AI3: {msg.data[2]:.3f} mA")
        print(f"  AI4: {msg.data[3]:.3f} mA")
        
        # === UNCOMMENT UNTUK SIMPAN KE FILE ===
        # self.file.write(f"{timestamp},AI,{msg.data[0]},{msg.data[1]},{msg.data[2]},{msg.data[3]}\n")
        # self.file.flush()
    
    def ao_callback(self, msg):
        """Callback untuk data AO"""
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        
        # Tampilkan di terminal
        print(f"\n[AO DATA] {timestamp}")
        print(f"  AO1: {msg.data[0]:.3f} mA")
        print(f"  AO2: {msg.data[1]:.3f} mA")
        print(f"  AO3: {msg.data[2]:.3f} mA")
        print(f"  AO4: {msg.data[3]:.3f} mA")
        
        # === UNCOMMENT UNTUK SIMPAN KE FILE ===
        # self.file.write(f"{timestamp},AO,{msg.data[0]},{msg.data[1]},{msg.data[2]},{msg.data[3]}\n")
        # self.file.flush()
    
    def __del__(self):
        """Tutup file saat program berakhir"""
        # === UNCOMMENT UNTUK SIMPAN KE FILE ===
        # if hasattr(self, 'file'):
        #     self.file.close()

def main(args=None):
    rclpy.init(args=args)
    node = ModbusSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()