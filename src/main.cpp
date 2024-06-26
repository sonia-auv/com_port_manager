#include "com_port_manager/ComProvider.hpp"

int main(int argc, char *argv[]){
    
    rclcpp::init(argc, argv);
    auto sub_com = std::make_shared<com_provider::ComProvider>();
    if (!sub_com->OpenPort())
    {   
        std::cout << "Could not open port..." << std::endl;
        return EXIT_FAILURE;
    }

    rclcpp::spin(sub_com);
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
 