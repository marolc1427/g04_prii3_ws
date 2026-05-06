import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Bool, Int32MultiArray

class ArduinoDriverNode(Node):
    def __init__(self):
        super().__init__('arduino_driver')
       
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
            self.get_logger().info(f"Driver conectado a {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Fallo al abrir puerto: {e}")
            raise SystemExit

        # SUSCRIPTORES: Escuchan las órdenes del exterior
        self.sub_ventosa = self.create_subscription(Bool, '/cmd_ventosa', self.callback_ventosa, 10)
        self.sub_motores = self.create_subscription(Int32MultiArray, '/cmd_motores', self.callback_motores, 10)

        # TIMER: Para mantener el buffer serie vacío y leer los datos del Arduino
        self.timer = self.create_timer(0.05, self.leer_arduino)

    def callback_ventosa(self, msg):
        estado = 1 if msg.data else 0
        comando = f"V{estado}\n"
        self.ser.write(comando.encode())
        self.ser.flush()
        self.get_logger().info(f"Enviado a Arduino: {comando.strip()}")

    def callback_motores(self, msg):
        if len(msg.data) == 2:
            comando = f"M{msg.data[0]},{msg.data[1]}\n"
            self.ser.write(comando.encode())
            self.ser.flush()
            self.get_logger().info(f"Enviado a Arduino: {comando.strip()}")
        else:
            self.get_logger().warning("El topic /cmd_motores requiere exactamente 2 enteros.")

    def leer_arduino(self):
        while self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    # Opcional: Aquí podrías PUBLICAR las RPM en otro topic
                    self.get_logger().debug(f"Datos recibidos: {line}")
            except Exception:
                pass

def main(args=None):
    rclpy.init(args=args)
    try:
        node = ArduinoDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals() and hasattr(node, 'ser') and node.ser.is_open:
            node.ser.write(b'V0\n')
            node.ser.write(b'M0,0\n')
            node.ser.close()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()