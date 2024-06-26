#pragma once

#include <mutex>
#include <stdio.h>
#include <thread>
#include <condition_variable>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sonia_common_cpp/SerialConn.h"
#include "sonia_common_ros2/msg/underwater_com.hpp"

namespace com_provider
{
    struct Register
    {
        std::string str="";
        std::mutex mutex;
        std::condition_variable cond;            
    };

    enum _Cmd: char
    {
        CMD_GET_VERSION = 'v',
        CMD_GET_PAYLOAD_SIZE= 'n',
        CMD_GET_BUFFER_LENGTH='l',
        CMD_GET_DIAGNOSTIC='d',
        CMD_GET_SETTINGS='c',
        CMD_SET_SETTINGS='s',
        CMD_QUEUE_PACKET='q',
        CMD_FLUSH='f'
    };

    class ComProvider:public rclcpp::Node
    {
        public:
            ComProvider();
            ~ComProvider();
            bool OpenPort();
        private:

            static const int BUFFER_SIZE=256;
            static const int MODEM_M64_PAYLOAD= 8;
            const char SOP= 'w';
            const char EOP= '\n';
            const char DIR_CMD= 'c';
            const char DIR_RESP= 'r';
            const char CHECKSUM= '*';
            const char RESP_GOT_PACKET= 'p';
            const char RETURN_ERROR= '?';
            const char MALFORMED= '!';
            const char ACK='a';
            const char NAK = 'n';
            const std::vector<char> all_CMD={'v','n','l','d','c','s','q','f'};

            uint8_t calculeCheckSum(const std::string& data);
            void appendCheckSum(std::string& data);
            bool confirmCheckSum(std::string& data);
            std::string resize_data(std::string &data, const size_t size);
            bool check_CMD(const char &cmd);

            void Append_packet(std::string& buffer, const char (&packet)[MODEM_M64_PAYLOAD]);
            void Queue_packet(const char cmd, const char (&packet)[MODEM_M64_PAYLOAD]);
            bool Transmit_Packet(bool pop_packet);
            bool Send_cmd_to_sensor(std::string &buffer,char cmd,const char (&packet)[MODEM_M64_PAYLOAD]={});
            void Manage_write();
            void Read_packet();
            void Publish_on_ROS2(std::string &buffer);
            
            void UnderwaterComCallback(const sonia_common_ros2::msg::UnderwaterCom &msg);

            void Init_function();
            bool Get_payload_load();
            bool Check_version();
            bool Set_config(const char role, const uint8_t channel);
            bool Flush_queue();
            void Set_sensor();

            template <typename TType>
            void Find_parameter(const std::string &parameter, TType &attribute);

            sonia_common_cpp::SerialConn _serialConn;
            std::thread init_function_thread;
            std::thread read_packet_thread;
            std::thread manage_write_thread;
            std::promise<bool> init_complete;

            Register _write;
            Register _parse;

            rclcpp::Subscription<sonia_common_ros2::msg::UnderwaterCom>::SharedPtr underwaterComSub;
            rclcpp::Publisher<sonia_common_ros2::msg::UnderwaterCom>::SharedPtr underwaterComPub;

            std::string writeBuffer;
            std::atomic_bool stop_write_thread = {false};
            std::atomic_bool stop_read_thread = {false};
            uint8_t payload; 
            bool init_err= {true};

            std::string ttyport;
            char role;
            uint8_t channel;

    };
    
}