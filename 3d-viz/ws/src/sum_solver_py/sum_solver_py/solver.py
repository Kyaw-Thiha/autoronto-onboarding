from typing import List, Optional
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Int8MultiArray, Int8

class TwoSumNode(Node):
    input: Optional[List[int]] 
    target: Optional[int] 

    def __init__(self):
        super().__init__("sum_solver_py")
         # Subscribers: normal reliable/volatile
        sub_qos = QoSProfile(depth=10)  # keep last 10 messages

        # Publisher: keep last message for late subscribers (latched-like)
        pub_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.input_sub = self.create_subscription(Int8MultiArray, "/input", self.on_input, sub_qos)
        self.target_sub = self.create_subscription(Int8, "/target", self.on_target, sub_qos)
        self.pub = self.create_publisher(Int8MultiArray, "/solution", pub_qos)

        self.input = None
        self.target = None

        self.get_logger().info("TwoSumNode started and ready ✅")


    def on_input(self, msg: Int8MultiArray):
        self.input = list(msg.data)
        self.get_logger().info(f"[INFO]: received /input: {self.input}")
        self.solve()

    def on_target(self, msg: Int8):
        self.target = int(msg.data)
        self.get_logger().info(f"[INFO]: received /target: {self.target}")
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
            pairs[num] = i

