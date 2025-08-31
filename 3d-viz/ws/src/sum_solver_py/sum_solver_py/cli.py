import rclpy

from sum_solver_py.solver import TwoSumNode

def main():
    rclpy.init()
    node = TwoSumNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
