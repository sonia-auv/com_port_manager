#include <sstream>
#include <iostream>

#include "boost/log/trivial.hpp"
#include "com_port_manager/ComProvider.hpp"

using std::placeholders::_1;
namespace com_provider
{
    ComProvider::ComProvider()
    : Node("underwater_com_provider_node"),_serialConn("/dev/MODEM",B115200)
    {
        //BOOST_LOG_TRIVIAL(info)<<"Deserializing parameters....";
        //Find_parameter("/connection/ttyport", ttyport);
        //Find_parameter("/settings/role", role);
        //Find_parameter("/settings/channel", channel);
        //BOOST_LOG_TRIVIAL(info)<<"Deserialization done!";
        role= 'a';
        channel= 6;

        writeBuffer.reserve(BUFFER_SIZE);

        underwaterComPub=this->create_publisher<sonia_common_ros2::msg::UnderwaterCom>("/provider_underwater_com/recieve_msgs",100);
        underwaterComSub=this->create_subscription<sonia_common_ros2::msg::UnderwaterCom>("/proc_underwater_com/send_msgs",100,std::bind(&ComProvider::UnderwaterComCallback,this,_1));
        
        manage_write_thread = std::thread(std::bind(&ComProvider::Manage_write,this));
        read_packet_thread = std::thread(std::bind(&ComProvider::Read_packet,this));

        BOOST_LOG_TRIVIAL(info)<<"Setting the sensor";
        
        Set_sensor();    
    }

    ComProvider::~ComProvider()
    {
        stop_read_thread= true;
        stop_write_thread=true;
    }

    bool ComProvider::OpenPort()
    {
        bool res = _serialConn.OpenPort();
        if (res)
        {
            _serialConn.Flush();
        }
        return res;
    }
    void ComProvider::UnderwaterComCallback(const sonia_common_ros2::msg::UnderwaterCom &msg){
        char packet_array[MODEM_M64_PAYLOAD];
        std::memcpy(packet_array, &(msg.data), sizeof(packet_array));
        Queue_packet(_Cmd::CMD_QUEUE_PACKET, packet_array);
    }
    uint8_t ComProvider::calculeCheckSum(const std::string& data)
    {
        uint8_t check = 0;

        for(unsigned int i = 1; i < data.size(); i++)
            check ^= data[i];
        
        return check;
    }
    
    void ComProvider::appendCheckSum(std::string &data)
    {
        std::stringstream ss;

        char buffer[3];
        uint8_t checksum = calculeCheckSum(data);
        std::snprintf(buffer, 3, "%02x", checksum);
        ss << resize_data(data,BUFFER_SIZE) << std::string("*") <<std::hex <<buffer<<std::showbase <<EOP;
        data = ss.str();

    }

    bool ComProvider::confirmCheckSum(std::string &data)
    {
        try
        {
            std::string checksumData = data.substr(0, data.find("*", 0));
            uint8_t calculatedChecksum = calculeCheckSum(checksumData);
            uint8_t originalChecksum = std::stoi(data.substr(data.find("*", 0)+1, 2), nullptr, 16);
            return originalChecksum == calculatedChecksum;
        }
        catch(...)
        {
            BOOST_LOG_TRIVIAL(warning)<<"IMU : Bad packet checksum";
            return false;
        }
    }
    bool ComProvider::Transmit_Packet(bool pop_packet)
    {
        if(!writeBuffer.empty()){
            _serialConn.Transmit(writeBuffer);
            if(pop_packet)
            {
                writeBuffer.clear();
            }
            return true;   
        }
        else
            {
                BOOST_LOG_TRIVIAL(warning)<<"Packet isnt queue";
                return false;
            }
    }
    bool ComProvider::Send_cmd_to_sensor(std::string &buffer, char cmd, const char (&packet)[MODEM_M64_PAYLOAD])
    {
        Queue_packet(cmd,packet);
        std::unique_lock<std::mutex> mlock(_parse.mutex);
        _parse.cond.wait(mlock);

        if(!_parse.str.empty()){
            std::string tmp=_parse.str;
            _parse.str.erase();
            buffer.assign(tmp);
            return false;
        }
        return true;
    }
    std::string ComProvider::resize_data(std::string &data, const size_t size)
    {
        std::string resized_data(size, '\0');
        resized_data.assign(data.substr(0, size));
        return resized_data;  
    }
    bool ComProvider::check_CMD(const char &cmd)
    {
        for(char cm: all_CMD){
            if(cm==cmd){
                return true;
            }
        }   
        return false;
    }
    void ComProvider::Append_packet(std::string& buffer, const char (&packet)[MODEM_M64_PAYLOAD])
    {
        std::stringstream ss;
        ss << buffer << packet;
        buffer = ss.str();
    }
    void ComProvider::Queue_packet(const char cmd, const char (&packet)[MODEM_M64_PAYLOAD])
    {
        std::stringstream ss;
        std::string data;
        
        if(!check_CMD(cmd)){
            ss <<SOP<<DIR_CMD<< cmd;

            if(cmd == _Cmd::CMD_QUEUE_PACKET){
                ss<< ',' << std::to_string(payload)<<',';
                data=ss.str();
                Append_packet(data,packet);
            }
            else if(cmd == _Cmd::CMD_SET_SETTINGS){
                ss<<',';
                data=ss.str();
                Append_packet(data,packet);
            }
            appendCheckSum(data);
            std::unique_lock<std::mutex> mlock(_write.mutex);
            writeBuffer.assign(data);
            resize_data(writeBuffer,BUFFER_SIZE);
            _write.cond.notify_one();
            BOOST_LOG_TRIVIAL(debug)<<"Packet sent to Modem";
        }
        else
            BOOST_LOG_TRIVIAL(warning)<<"CMD unknown, Packet was not queued";
            
    }
    void ComProvider::Manage_write()
    {
        rclcpp::Rate(1);
        BOOST_LOG_TRIVIAL(info)<<"Packet isnt queue";
        while(!stop_write_thread){
            std::unique_lock<std::mutex> mlock(_write.mutex);
            _write.cond.wait(mlock);
            Transmit_Packet(true);
        }

    }
    void ComProvider::Read_packet()
    {
        uint8_t i;
        char buffer[BUFFER_SIZE];

        BOOST_LOG_TRIVIAL(info)<<"Payload is set";
        while(!stop_read_thread){
            do{
                _serialConn.ReadOnce((uint8_t*)buffer,0);
                if(stop_read_thread) return;
            }while(buffer[0]!=SOP);

            for(i = 1; buffer[i-1] != EOP && i < BUFFER_SIZE; ++i)
            {
                _serialConn.ReadOnce((uint8_t*)buffer, i);
            }
            buffer[i]=0;
            if (i >= BUFFER_SIZE)
            {
                continue;
            }
            std::string data(buffer);
            if(confirmCheckSum(data)){
                if(data.at(2) == RESP_GOT_PACKET)
                {   
                    Publish_on_ROS2(data);
                }
                else if(data.at(2) == _Cmd::CMD_QUEUE_PACKET && data.at(4) == ACK)
                {
                    BOOST_LOG_TRIVIAL(debug)<<"Packet queue";
                }
                else if(data.at(2) == RETURN_ERROR || data.at(2) == MALFORMED)
                {
                    BOOST_LOG_TRIVIAL(error)<<"Request not made properly";
                    std::unique_lock<std::mutex> mlock(_parse.mutex);
                    _parse.cond.notify_one();
                }
                else
                {
                    std::unique_lock<std::mutex> mlock(_parse.mutex);
                    _parse.str = std::string(buffer);
                    _parse.cond.notify_one();
                }
            }

        }
    }
    void ComProvider::Publish_on_ROS2(std::string &buffer)
    {
        sonia_common_ros2::msg::UnderwaterCom msg;
        std::string temp;
        uint8_t prep_msg[MODEM_M64_PAYLOAD];
        temp.assign(buffer.substr(6));
        for(uint8_t i=0; i<MODEM_M64_PAYLOAD; i++){
            prep_msg[i]= (uint8_t)temp[i];
        }
        std::memcpy(&(msg.data),prep_msg, sizeof(msg.data));
        underwaterComPub->publish(msg);
    }
    
    void ComProvider::Init_function()
    {
        uint8_t i=0;
        bool err= true;
        while(i < 3 && err == true)
        {
            err = false;
            err= Check_version() | Get_payload_load() | Set_config(role, channel) | Flush_queue();  
            ++i;
        }
        init_complete.set_value_at_thread_exit(err);
        std::move(init_complete);
    }
    bool ComProvider::Get_payload_load()
    {
        std::string load="";
        std::string buffer;
        buffer=resize_data(buffer,BUFFER_SIZE);

        if(Send_cmd_to_sensor(buffer,_Cmd::CMD_GET_PAYLOAD_SIZE)) return true;

        std::stringstream ss(buffer);
        std::getline(ss,load, ',');
        std::getline(ss,load, '*');

        if((load>="0" || load <= "9")){
            payload=std::stoi(load);
            BOOST_LOG_TRIVIAL(info)<<"Payload is set";
            return false;
        }
        else{
            BOOST_LOG_TRIVIAL(info)<<"Payload isn't an integer";
            return true;
        }

    }
    bool ComProvider::Check_version()
    {
        std::string version="";
        std::string buffer;
        buffer=resize_data(buffer,BUFFER_SIZE);

        if(Send_cmd_to_sensor(buffer,_Cmd::CMD_GET_VERSION)) return true;

        std::stringstream ss(buffer);
        std::getline(ss,version, ',');
        std::getline(ss,version, ',');

        if(version != "1"){
            BOOST_LOG_TRIVIAL(error)<<"Major Version is not 1";
            return true;
        }
        else{
            BOOST_LOG_TRIVIAL(info)<<"Major Version is 1";
            return false;
        }
    }
    bool ComProvider::Set_config(const char role, const uint8_t channel)
    {
        std::string acknowledge="";
        std::string buffer;
        buffer=resize_data(buffer,BUFFER_SIZE);
        char packet[MODEM_M64_PAYLOAD]={role, ','};
        buffer.assign(std::to_string(channel));
        packet[2]=buffer.front();

        if(Send_cmd_to_sensor(buffer,_Cmd::CMD_SET_SETTINGS,packet)) return true;

        std::stringstream ss(buffer);
        std::getline(ss,acknowledge, ',');
        std::getline(ss,acknowledge, '*');

        if(acknowledge != std::string(1, ACK)){
            BOOST_LOG_TRIVIAL(error)<<"Configuration is not set";
            return true;
        }
        else{
            BOOST_LOG_TRIVIAL(info)<<"Configuration set";
            return false;
        }
    }
    bool ComProvider::Flush_queue()
    {
        std::string acknowledge;
        std::string buffer;
        buffer=resize_data(buffer,BUFFER_SIZE);

        if(Send_cmd_to_sensor(buffer,_Cmd::CMD_FLUSH)) return true;

        std::stringstream ss(buffer);
        std::getline(ss,acknowledge, ',');
        std::getline(ss,acknowledge, '*');

        if(acknowledge != std::string(1, ACK)){
            BOOST_LOG_TRIVIAL(error)<<"Couldn't flush the queue";
            return true;
            
        }
        else{
            BOOST_LOG_TRIVIAL(info)<<"Queue flushed";
            return false;
        }
    }
    void ComProvider::Set_sensor()
    {
        std::chrono::system_clock::time_point thirty_seconds_passed
            = std::chrono::system_clock::now()+ std::chrono::seconds(30);

        std::future<bool>init_complete_future = init_complete.get_future();
        init_function_thread=std::thread(std::bind(&ComProvider::Init_function,this));
        init_function_thread.detach();

        if(std::future_status::ready ==init_complete_future.wait_until(thirty_seconds_passed)){
            init_err=false;
            BOOST_LOG_TRIVIAL(info)<<"Initialisation complete";
        }
        else{
            init_err=true;
            BOOST_LOG_TRIVIAL(error)<<"Initialisation Failed. Node shutdown";
        }
    }
    template <typename TType>
    inline void ComProvider::Find_parameter(const std::string &parameter, TType &attribute)
    {
        if(this->has_parameter("com_port_manager"+parameter)){
            this->get_parameter("com_port_manager"+parameter,attribute);
        }
        else{
            BOOST_LOG_TRIVIAL(warning)<<"Could not find /com_port_manager"+parameter<<" using default.";
            
        }
    }
}
