#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <unordered_map>
#include <vector>

class TwoSumNode : public rclcpp::Node {
public:
  TwoSumNode() : rclcpp::Node("two_sum_node") {
    auto qos = rclcpp::QoS(10);
    input_sub_ = create_subscription<std_msgs::msg::Int8MultiArray>(
        "/input", qos, [this](std_msgs::msg::Int8MultiArray::SharedPtr msg) {
          input_.assign(msg->data.begin(), msg->data.end());
          have_input_ = true;
          try_solve_and_publish();
        });
    target_sub_ = create_subscription<std_msgs::msg::Int8>(
        "/target", qos, [this](std_msgs::msg::Int8::SharedPtr msg) {
          target_ = msg->data;
          have_target_ = true;
          try_solve_and_publish();
        });
    pub_ = create_publisher<std_msgs::msg::Int8MultiArray>("/solution", qos);
  }

private:
  void try_solve_and_publish() {
    if (!(have_input_ && have_target_))
      return;

    std::unordered_map<int, int> seen;
    for (int i = 0; i < static_cast<int>(input_.size()); ++i) {
      int v = static_cast<int>(input_[i]);
      int need = static_cast<int>(target_) - v;
      auto it = seen.find(need);
      if (it != seen.end()) {
        // publish indices
        std_msgs::msg::Int8MultiArray out;
        out.data = {static_cast<int8_t>(it->second), static_cast<int8_t>(i)};
        pub_->publish(out);
        return;
      }
      seen[v] = i;
    }
  }

  rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr input_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr target_sub_;
  rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr pub_;
  std::vector<int8_t> input_;
  int8_t target_{0};
  bool have_input_{false}, have_target_{false};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwoSumNode>());
  rclcpp::shutdown();
  return 0;
}
