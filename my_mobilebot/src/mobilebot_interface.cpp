#include <my_mobilebot/mobilebot_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace my_mobilebot
{
mobilebotInterface::mobilebotInterface()
{

}

mobilebotInterface::~mobilebotInterface()
{
    if(arduino.IsOpen())
    {
        try
        {
            arduino.Close();
            
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("mobilebotInterface"),"Something went wrong while closing the connection with port "<< port);
        }
        
    }
}
CallbackReturn mobilebotInterface::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
    CallbackReturn result =  hardware_interface::SystemInterface::on_init(hardware_info);
    if(result != CallbackReturn::SUCCESS)
    {
        return result;
    }
    try
    {
        port = info_.hardware_parameters.at("port");
    }
    catch(const std::out_of_range &e)
    {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("mobilebotInterface"),"No serial port provided! Aborting ");
        return CallbackReturn::FAILURE;
    }
    velocity_commands_.reserve(info_.joints.size());
    position_states_.reserve(info_.joints.size());
    velocity_states_.reserve(info_.joints.size());

    return CallbackReturn::SUCCESS;
    

}
std::vector<hardware_interface::StateInterface> mobilebotInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for(size_t i=0; i < info_.joints.size();i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, 
            hardware_interface::HW_IF_POSITION,&position_states_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, 
            hardware_interface::HW_IF_VELOCITY,&velocity_states_[i]));

    }
    return state_interfaces;

}
std::vector<hardware_interface::CommandInterface> mobilebotInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for(size_t i =0; i < info_.joints.size();i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
    }
    return command_interfaces;

}
CallbackReturn mobilebotInterface::on_activate(const rclcpp_lifecycle::State & previous_state) 
{
    RCLCPP_INFO(rclcpp::get_logger("mobilebotInterface"), "Starting robot hardware... ");
    velocity_commands_= {0.0, 0.0};
    position_states_= {0.0, 0.0};
    velocity_states_ ={0.0, 0.0};

    try
    {
        arduino.Open(port);
        arduino.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

    }
    catch(...)
    {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("mobilebotInterface"),"Something went wrong while ineracting with port "<< port);
        return CallbackReturn::FAILURE;
        
    }
    RCLCPP_INFO(rclcpp::get_logger("mobilebotInterface"), "Hardware started, ready to take commands");
    return CallbackReturn::SUCCESS;
    
    

}

CallbackReturn mobilebotInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("mobilebotInterface"), "Stoping robot hardware... ");
    if (arduino.IsOpen())
    {
        try
        {
            arduino.Close();
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("mobilebotInterface"),"Something went wrong while closing the port "<< port);
            return CallbackReturn::FAILURE;
            
        }
        
        
    }
    

}
hardware_interface::return_type mobilebotInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if (arduino.IsDataAvailable())
    {
        std::string message;
        arduino.ReadLine(message);
        std::stringstream ss(message);
        std::string res;
        int multiplier = 1;
        while (std::getline(ss, res,','))
        {
            multiplier = res.at(1) == 'p' ? 1 : -1;
            if(res.at(0) == 'r')
            {
                velocity_states_.at(0) =multiplier * std::stod(res.substr(2, res.size()));
            }
            else if(res.at(0) == 'l')
            {
                velocity_states_.at(1) =multiplier * std::stod(res.substr(2, res.size()));

            }
         
        }
        
        
    }
    return hardware_interface::return_type::OK;

    

}

hardware_interface::return_type mobilebotInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    std::stringstream message_stream;
    char right_wheel_sign = velocity_commands_.at(0) >= 0 ? 'p' : 'n';
    char left_wheel_sign = velocity_commands_.at(1) >= 0 ? 'p' : 'n';
    std::string compansate_zeros_right = "";
    std::string compansate_zeros_left = "";

    if(std::abs(velocity_commands_.at(0)) < 10.0)
    {
        compansate_zeros_right = "0";
    }
    else
    {
        compansate_zeros_right = "";        

    }

    if(std::abs(velocity_commands_.at(1)) < 10.0)
    {
        compansate_zeros_left = "0";
    }
    else
    {
        compansate_zeros_left = "";        

    }

    message_stream << std::fixed << std::setprecision(2) << "r" << right_wheel_sign << compansate_zeros_right <<std::abs(velocity_commands_.at(0)) <<
        ",l" << left_wheel_sign << compansate_zeros_left << std::abs(velocity_commands_.at(1)) <<",";

    try
    {
        arduino.Write(message_stream.str());
    }
    catch(...)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("mobilebotInterface"),"Something went wrong while sending the message "<< 
            message_stream.str() << "on the port " << port);
        return hardware_interface::return_type::ERROR;
            
    }
    return hardware_interface::return_type::OK;
        

}

}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(my_mobilebot::mobilebotInterface, hardware_interface::SystemInterface)