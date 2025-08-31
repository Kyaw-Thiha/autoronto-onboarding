#include "std_msgs/msg/int8_multi_array.hpp"
#include <memory>
#include <optional>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <unordered_map>

class TwoSumNode : public rclcpp::Node {
  using Int8 = std_msgs::msg::Int8;
  using OptionalInt8 = std::optional<Int8>;
  using Int8MultiArray = std_msgs::msg::Int8MultiArray;
  using OptionalInt8MultiArray = std::optional<Int8MultiArray>;

  OptionalInt8MultiArray input;
  OptionalInt8 target;

  rclcpp::Subscription<Int8MultiArray>::SharedPtr input_sub;
  rclcpp::Subscription<Int8>::SharedPtr target_sub;
  rclcpp::Publisher<Int8MultiArray>::SharedPtr pub;

public:
  TwoSumNode() : rclcpp::Node("sum_solver_cpp") {

    auto qos = rclcpp::QoS(10);
    this->input_sub = create_subscription<Int8MultiArray>(
        "/input", qos,
        [this](Int8MultiArray::ConstSharedPtr msg) { on_input(msg); });
    this->target_sub = create_subscription<Int8>(
        "/target", qos, [this](Int8::ConstSharedPtr msg) { on_target(msg); });
    this->pub = create_publisher<Int8MultiArray>("/solution", qos);

    this->input = std::nullopt;
    this->target = std::nullopt;
  };

private:
  void on_input(Int8MultiArray::ConstSharedPtr msg) {
    this->input.reset();
    this->input = *msg;
    this->solve();
  }
  void on_target(Int8::ConstSharedPtr msg) {
    this->target.reset();
    this->target = *msg;
    this->solve();
  }

  void solve() {
    if (!this->input.has_value() || !this->target.has_value()) {
      return;
    }
    std::unordered_map<int, int> pairs;
    for (int i = 0; i < this->input.value().data.size(); ++i) {
      int num = this->input.value().data[i];
      int other_num = this->target.value().data - num;

      auto pair_found = pairs.find(other_num);
      if (pair_found != pairs.end()) {
        Int8MultiArray result;
        result.data = {static_cast<int8_t>(pair_found->second),
                       static_cast<int8_t>(i)};
        this->pub->publish(result);
        return;
      }

      pairs[num] = i;
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwoSumNode>());
  rclcpp::shutdown();

  // auto node = std::make_shared<TwoSumNode>();
  // rclcpp::executors::SingleThreadedExecutor exec;
  // exec.add_node(node);
  // exec.spin();

  return 0;
}
