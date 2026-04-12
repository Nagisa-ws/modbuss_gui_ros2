import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient

class ModbusNode(Node):
    def __init__(self):
        super().__init__('modbus_node')
        self.client = ModbusSerialClient(
            port='/dev/ttyUSB1',
            baudrate=9600, 
            parity='N', 
            stopbits=1, 
            timeout=3
        )
        self.client.connect()
        self.create_timer(1.0, self.read_AI)  # Ganti ke read_AI
    
    def read_AI(self):
        # Baca 4 AI channel (function 04 - input register)
        result = self.client.read_input_registers(address=100, count=4, device_id=1)
        
        if not result.isError():
            for i, value in enumerate(result.registers):
                mA = value / 1000
                self.get_logger().info(f'AI{i+1}: {mA} mA')
        else:
            self.get_logger().warn('Gagal baca AI')

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ModbusNode())
    rclpy.shutdown()

