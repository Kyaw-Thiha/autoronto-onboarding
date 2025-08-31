from typing import List
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int8MultiArray, Int8

class TwoSumNode(Node):
    input: List[int]
    target: int

    def __init__(self):
        super().__init__("sum_solver_py")
        quality_of_service = QoSProfile(depth=10)   # keep last 10 messages
        self.input_sub = self.create_subscription(Int8MultiArray, "/input", self.on_input, quality_of_service)
        self.target_sub = self.create_subscription(Int8, "/target", self.on_target, quality_of_service)
        self.pub = self.create_publisher(Int8MultiArray, "/solution", quality_of_service)

    def on_input(self, msg: Int8MultiArray):
        self.input = list(msg.data)
        self.solve()

    def on_target(self, msg: Int8):
        self.target = int(msg.data)
        self.solve()

    def solve(self):
        if self.input is None or self.target is None:
            return
        pairs = {}
        for i, num in enumerate(self.input):
            other_pair = self.target - num
            if other_pair in pairs:
                result = Int8MultiArray()
                result.data = [pairs[other_pair], i]
                return self.pub.publish(result)
            else:
                pairs[other_pair] = i

