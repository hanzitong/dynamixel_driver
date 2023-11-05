
#include <rclcpp/rclcpp.hpp>
#include <dynamixel_driver/DxlXDriver.hpp>


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DxlXDriver>());
    rclcpp::shutdown();

    return 0;
}
