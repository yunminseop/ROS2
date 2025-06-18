#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
    public:
    MinimalSubscriber()
    : Node("minimal_subscriber") // 왜 public으로 subscriber 정의하고 private으로 callback 정의하는가?
    {                           // main함수나 외부 코드에서 std::make_shared<MinimalSubscriber>을 호출하기 위해서는 subscriber 노드를 public으로 선언해야 함.
        subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1)); // callback 함수 바인딩
        // What is binding? 메시지를 받았을 때 호출할 멤버 함수인 topic::callback()을 지정해주는 부분.
    }

    private: // callback 함수는 노드 내부에서만 호출되는 함수지, 외부에서 호출되지 않음. -> 캡슐화의 기본 원칙에 따라 private으로 선언.
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const // 마지막 const의 의미: 이 함수가 MinimalSubscriber의 클래스 멤버 함수인데 이 함수 내에서 this 객체의 멤버 변수나 상태를 수정하지 않겠다
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[]){ //argc: argument count , argv: argument vector
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();   
    return 0;
}