import argparse
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class DibujaMovNode(Node):
    def __init__(self) -> None:
        super().__init__('dibuja_mov')
        self.xs: list[float] = []
        self.ys: list[float] = []
        self.create_subscription(Odometry, '/odom', self._odom_cb, 50)

    def _odom_cb(self, msg: Odometry) -> None:
        self.xs.append(msg.pose.pose.position.x)
        self.ys.append(msg.pose.pose.position.y)

    def collect(self, duration_s: float) -> None:
        self.get_logger().info(f'Recogiendo odometria durante {duration_s:.1f} s...')
        end_time = time.time() + duration_s
        while rclpy.ok() and time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)

    def save_plot(self, out_file: str) -> None:
        if not self.xs:
            self.get_logger().warning('No se recibieron datos de /odom')
            return

        try:
            import matplotlib.pyplot as plt
        except Exception as exc:
            self.get_logger().error(f'No se pudo importar matplotlib: {exc}')
            return

        plt.figure(figsize=(7, 7))
        plt.plot(self.xs, self.ys, linewidth=1.5, label='trayectoria')
        plt.scatter(self.xs[0], self.ys[0], c='green', s=50, label='inicio')
        plt.scatter(self.xs[-1], self.ys[-1], c='red', s=50, label='fin')
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.title('Trayectoria del robot (odom)')
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(out_file, dpi=140)

        dx = self.xs[-1] - self.xs[0]
        dy = self.ys[-1] - self.ys[0]
        err = math.hypot(dx, dy)
        self.get_logger().info(f'Grafica guardada en: {out_file}')
        self.get_logger().info(f'Error final respecto al inicio: {err:.4f} m')


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Dibuja trayectoria de /odom')
    parser.add_argument('--duracion', type=float, default=40.0, help='Segundos de captura (default: 40)')
    parser.add_argument('--salida', type=str, default='/workspace/practica_2/ros2_ws/trayectoria.png', help='Ruta PNG de salida')
    return parser.parse_args(argv)


def main() -> None:
    args = _parse_args(sys.argv[1:])
    rclpy.init(args=sys.argv)
    node = DibujaMovNode()
    try:
        node.collect(max(1.0, args.duracion))
        node.save_plot(args.salida)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
