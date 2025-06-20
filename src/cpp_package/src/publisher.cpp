#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals; // publisher callback timer 시간 500ms를 정의하기 위해서는 std::chrono_literals 네임스페이스가 필요함

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this)); // create_wall_timer = rclcpp::Node로부터 상속받은 멤버 함수
      // 위 코드의 의미: "본 클래스의 timer_callback 함수의 주소와 객체의 주소를 가리키는 this 포인터를 묶어서 콜백함수로 대기시키겠다."
      // &MinimalPublisher::timer_callback = timer_callback함수의 주소.
      // Q. 그럼 timer_callback 함수의 주소와 this 포인터를 묶어서 객체로 전달하지 말고, 처음부터 this->timer_callback으로 작성하면 코드도 간단해지고 가독성도 좋아지는 것 아닌가?
      // A. this->timer_callback()은 콜백함수의 '실행 결과'를 반환한다. 하지만 여기서 create_wall_timer함수의 의도는 실행 가능한 함수의 주소를 '미리' 받아 놓는 것이기 때문에 this->callback을 사용하면 제대로 동작하지 않는다.
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      // python의 self.get_logger().info("") = cpp의 RCLCPP_INFO(this->get_logger(), ). RCLCPP_INFO()는 ROS2 Cpp의 로그를 출력하는 매크로 함수임.
      // Q. Cpp 스타일처럼 cout << message.data 방식으로 쓸 수는 없을까?
      // A. RCLCPP_INFO() 매크로 자체가 C언어 스타일의 %s 포맷에 대응하는 printf 스타일 문자열을 받으므로 string을 char * 타입으로 변환하는 .c_str() 함수를 뒤에 붙여야 함.
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::TimerBase::SharedPtr는 갑자기 어디서 튀어나온 걸까?
    /*template <typename CallbackT>
    rclcpp::TimerBase::SharedPtr create_wall_timer(std::Chrono::duration<...> period, CallbackT callback)
    {
    auto timer_ = std::make_shared<rclcpp::GenericTimer<callback T>> // 원래 create_wall_timer 함수는 GenericTimer 클래스 객체임.
    return std::static_pointer_cast<rclcpp::TimerBase>(timer) // 해당 GenericTimer을 TimerBase 클래스로 업캐스팅하여 반환
    }*/
    // 그래서 create_wall_timer 함수의 반환형은 rclcpp::TimerBase::SharedPtr 타입임.
    
    //SharedPtr이 여러 클래스의 멤버로 사용될 수 있는 이유: 클래스마다 SharedPtr이라는 이름의 타입 별칭이 존재하기 때문임.
    /*class TimerBase{
    public:
    using SharedPtr = std::shared_ptr<TimerBase>; // Cpp 별칭 지정 문법 -> 'using 새 이름 = 기존 이름'
    }*/

    /*template <typename MessageT>
    class Publisher{
    public:
    using SharedPtr = std::shared_ptr<Publisher<MessageT>>;
    }*/
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; // rclcpp::Publisher<MessageT, Alloc> // Generic Class
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}