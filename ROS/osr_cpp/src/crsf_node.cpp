// STL
#include <cstdio>
#include <vector>
#include <array>
#include <string>
#include <chrono>

using namespace std::chrono_literals; 

// ROS2 + OSR
#include <rclcpp/rclcpp.hpp>
#include "osr_interfaces/msg/crsf_channels.hpp"

// 3rd-party
#include "xcrsf/crossfire.h"

class CRSFNode : public rclcpp::Node
{
  public:
    CRSFNode()
    : Node("crsf_node")
    {

      this->declare_parameter<std::string>("serial_device", "/dev/ttyUSB0");
      this->declare_parameter<int>("baud", 420000);

      _crsf_pub = this->create_publisher<osr_interfaces::msg::CRSFChannels>("/crsf", rclcpp::QoS(rclcpp::KeepLast(10)));
      _timer = this->create_wall_timer(
                    100us, 
                    std::bind(&CRSFNode::crsf_callback, this)
                  );
    
      _crsf_msg = std::make_shared<osr_interfaces::msg::CRSFChannels>();
      
      init_param();
      setup_crossfire();

    }

  private:

    void init_param()
    {

      this->get_parameter_or<std::string>("serial_device", _uart_path, "/dev/ttyUSB0");
      this->get_parameter_or<int>("baud", _baud_rate, 420000);

    }

    void setup_crossfire()
    {
      // reset socket if already initialized
      if(_crsf_sock != nullptr)
      {

        _crsf_sock->close_port();
        _crsf_sock = nullptr;
      }

      RCLCPP_INFO(this->get_logger(), "Opening CRSF port on %s", _uart_path.c_str());

      _crsf_sock = std::unique_ptr<crossfire::XCrossfire>(new crossfire::XCrossfire(_uart_path, _baud_rate));
      
      bool ret = _crsf_sock->open_port();
      
      if(!ret)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to open CRSF port on %s",_uart_path.c_str());
      }

    }

    void crsf_callback()
    {
      // check if CRSF socket is initialized or paired
      if(_crsf_sock == nullptr || !_crsf_sock->is_paired())
      { 
        setup_crossfire();
        return;
      }

      _crsf_channels = _crsf_sock->get_channel_state();

      std::transform(_crsf_channels.begin(), 
                     _crsf_channels.end(), 
                     _crsf_msg->channels.begin(),
                    [](uint16_t x) { return static_cast<int32_t>(x); });

      _crsf_pub->publish(*_crsf_msg);

    }

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<osr_interfaces::msg::CRSFChannels>::SharedPtr _crsf_pub;
    
    std::shared_ptr<osr_interfaces::msg::CRSFChannels> _crsf_msg;

    std::array<uint16_t, 16> _crsf_channels; 

    std::string _uart_path;
    int _baud_rate;

    std::unique_ptr<crossfire::XCrossfire> _crsf_sock = nullptr;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);  
  auto crsf_node_ptr = std::make_shared<CRSFNode>();
  rclcpp::spin(crsf_node_ptr);
  rclcpp::shutdown();
  return 0;
}
