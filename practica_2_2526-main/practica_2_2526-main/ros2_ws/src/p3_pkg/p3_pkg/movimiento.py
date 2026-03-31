import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class MovimientoNode(Node):
    def __init__(self) -> None:
        super().__init__('movimiento')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _send(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def _stop(self) -> None:
        self._send(0.0, 0.0)
        self._send(0.0, 0.0)

    def move_for(self, linear_x: float, angular_z: float, duration_s: float, hz: float = 20.0) -> None:
        period = 1.0 / hz
        end_time = time.time() + max(0.0, duration_s)
        while rclpy.ok() and time.time() < end_time:
            self._send(linear_x, angular_z)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)
        self._stop()

    def move_distance(self, distance_m: float, linear_speed_mps: float = 0.2) -> None:
        if linear_speed_mps == 0.0:
            return
        sign = 1.0 if distance_m >= 0.0 else -1.0
        v = sign * abs(linear_speed_mps)
        duration = abs(distance_m) / abs(linear_speed_mps)
        self.get_logger().info(f'Movimiento lineal: {distance_m:.2f} m a {v:.2f} m/s')
        self.move_for(v, 0.0, duration)

    def rotate_angle(self, angle_rad: float, angular_speed_rps: float = 0.6) -> None:
        if angular_speed_rps == 0.0:
            return
        sign = 1.0 if angle_rad >= 0.0 else -1.0
        w = sign * abs(angular_speed_rps)
        duration = abs(angle_rad) / abs(angular_speed_rps)
        self.get_logger().info(f'Giro: {math.degrees(angle_rad):.1f} grados a {w:.2f} rad/s')
        self.move_for(0.0, w, duration)

    def run_mode(self, mode: int) -> None:
        if mode == 0:
            self.mode_0_lineal_2m()
        elif mode == 1:
            self.mode_1_triangulo_equilatero()
        elif mode == 2:
            self.mode_2_cuadrado()
        elif mode == 3:
            self.mode_3_infinito()
        else:
            raise ValueError('Modo invalido. Usa 0, 1, 2 o 3.')

    def mode_0_lineal_2m(self) -> None:
        self.get_logger().info('Modo 0: avanzar 2 metros')
        self.move_distance(2.0, linear_speed_mps=0.2)

    def mode_1_triangulo_equilatero(self) -> None:
        self.get_logger().info('Modo 1: triangulo equilatero (lado 3 m)')
        for i in range(3):
            self.get_logger().info(f'Lado {i + 1}/3')
            self.move_distance(3.0, linear_speed_mps=0.22)
            self.rotate_angle(2.0 * math.pi / 3.0, angular_speed_rps=0.6)

    def mode_2_cuadrado(self) -> None:
        self.get_logger().info('Modo 2: cuadrado (lado 1 m)')
        for i in range(4):
            self.get_logger().info(f'Lado {i + 1}/4')
            self.move_distance(1.0, linear_speed_mps=0.18)
            self.rotate_angle(math.pi / 2.0, angular_speed_rps=0.6)

    def mode_3_infinito(self) -> None:
        self.get_logger().info('Modo 3: infinito')
        self.move_distance(0.5, linear_speed_mps=0.15)
        self.rotate_angle(math.pi / 3.0, angular_speed_rps=0.5)

        # Dos bucles en sentidos opuestos para dibujar un "8" .
        radius = 0.4
        linear = 0.14
        angular = linear / radius
        loop_time = (2.0 * math.pi) / angular

        self.get_logger().info('Primer bucle')
        self.move_for(linear, angular, loop_time)

        self.get_logger().info('Segundo bucle')
        self.move_for(linear, -angular, loop_time)


def _parse_mode(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description='Nodo de movimientos para la practica 2')
    parser.add_argument('modo', type=int, help='0: 2m, 1: triangulo, 2: cuadrado, 3: infinito')
    args, _ = parser.parse_known_args(argv)
    return args.modo


def main() -> None:
    rclpy.init(args=sys.argv)
    node = MovimientoNode()
    try:
        mode = _parse_mode(sys.argv[1:])
        node.run_mode(mode)
    except Exception as exc:
        node.get_logger().error(str(exc))
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
