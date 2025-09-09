#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

class PubControlCmd : public rclcpp::Node {
public:
  PubControlCmd() : Node("pub_control_cmd") {
    pub_ = this->create_publisher<std_msgs::msg::String>("/car_control_cmd", 10);
  }

    // 读取单个键盘字符
    char getKey()
    {
        struct termios oldt, newt;
        char ch;
        // 获取终端设置
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        // 禁用缓冲和回显
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        // 读取一个字符
        ch = getchar();
        // 恢复终端设置
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return ch;
    }
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PubControlCmd>();

    // 主循环
    while (rclcpp::ok())
    {
        // 获取键盘输入
        char key = node->getKey();
        
        // 创建消息并发布
        auto msg = std_msgs::msg::String();
        msg.data = std::string(1, key);
        RCLCPP_INFO(node->get_logger(), "发布: '%s'", msg.data.c_str());
        node->pub_->publish(msg);

        // 按'q'退出
        if (key == 'q')
        {
            break;
        }
        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return 0;
}