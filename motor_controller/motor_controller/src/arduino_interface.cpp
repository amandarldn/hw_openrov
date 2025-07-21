#include "motor_controller/arduino_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <std_msgs/msg/int32_multi_array.hpp>  // Include the proper message type
#include <cmath>
#include <std_msgs/msg/float32.hpp>







namespace arduino_controller
{
ArduinoInterface::ArduinoInterface() 
{
}



ArduinoInterface::~ArduinoInterface()
{
    if (SerialPort != -1)
    {
        close(SerialPort);
    }
}


CallbackReturn ArduinoInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

// Declare the node_ variable
rclcpp::Node::SharedPtr node_;

// Initialize the publisher here

  node_ = std::make_shared<rclcpp::Node>("velocity_publisher_node");
  publisher_ = node_->create_publisher<std_msgs::msg::Float32>("w_velocity", 10);


  try
  {
  

    std::string port = "/dev/ttyACM0";
    SerialPort = open(port.c_str(), O_RDWR);
    if (SerialPort < 0)
    {

        RCLCPP_WARN(rclcpp::get_logger("arduino_controller_interface"), 
                    "Unable to open serial port %s. Error: %s", port.c_str(), strerror(errno));
        RCLCPP_WARN(rclcpp::get_logger("arduino_controller_interface"), 
                    "Controller will run in simulation mode.");
        return CallbackReturn::SUCCESS;  // Continue even if Arduino is not connected
    }

    if (tcgetattr(SerialPort, &tty) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("arduino_controller_interface"), "Error %i from tcgetattr: %s", errno, strerror(errno));
        close(SerialPort);
        return CallbackReturn::ERROR;
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 0;

    speed_t speed = B115200;
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tcflush(SerialPort, TCIFLUSH);
    if (tcsetattr(SerialPort, TCSANOW, &tty) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("CustomHardware"), "Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("CustomHardware"), "SERIAL PORT OPENED: %d! WAITING...", SerialPort);

    auto t_start = std::chrono::high_resolution_clock::now();
    while(true)
    {
        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        if(elapsed_time_ms > 3000)
        {
            break;
        }
    }


   
  }


  catch(std::exception &e)
  {

    RCLCPP_WARN(
      rclcpp::get_logger("arduino_actuator_interface"),
      "Error during initialization: %s. Running in simulation mode.", e.what()
    );
    return CallbackReturn::SUCCESS;  // Continue even if there's an error
  }

  velocity_commands_.reserve(info_.joints.size());
  velocity_states_.reserve(info_.joints.size());
  prev_velocity_commands_.reserve(info_.joints.size());
  position_states_.reserve(info_.joints.size());

  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> ArduinoInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;


  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
  }

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
  }

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> ArduinoInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
  }

  return command_interfaces;
}


CallbackReturn ArduinoInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ArduinoInterface"), "Starting robot hardware ...");

  // Reset commands and states
  // velocity_commands_ = { 0.0, 0.0};
  // prev_velocity_commands_ = { 0.0, 0.0};
  // velocity_states_ = { 0.0, 0.0};
  // position_states_ = { 0.0, 0.0};

  velocity_commands_.resize(info_.joints.size(), 0.0);
  prev_velocity_commands_.resize(info_.joints.size(), 0.0);
  velocity_states_.resize(info_.joints.size(), 0.0);
  position_states_.resize(info_.joints.size(), 0.0);



  RCLCPP_INFO(rclcpp::get_logger("ArduinoInterface"),
              "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}


CallbackReturn ArduinoInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    if(SerialPort == -1)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    tcflush(SerialPort, TCIFLUSH);
    close(SerialPort);
    return hardware_interface::CallbackReturn::SUCCESS;
}


int ArduinoInterface::WriteToSerial(const unsigned char* buf, int nBytes)
{
    return ::write(SerialPort, const_cast<unsigned char*>(buf), nBytes);
}

int ArduinoInterface::ReadSerial(unsigned char* buf, int nBytes)
{
    auto t_start = std::chrono::high_resolution_clock::now();
    int n = 0;
    while(n < nBytes)
    {
        int ret = ::read(SerialPort, &buf[n], 1);
        if(ret < 0)
        {
            return ret;
        }

        n+=ret;
        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
        if(elapsed_time_ms > 10000)
        {
            break;
        }
    }
    return n;
}

// This function is responsible for writing commands to the Arduino
hardware_interface::return_type ArduinoInterface::write(const rclcpp::Time &, const rclcpp::Duration &) {
  try {
    // Clamp PWM values to 1000-2000µs (valid range for ESCs)
    int pwm1 = static_cast<int>(velocity_commands_[0]);
    int pwm2 = static_cast<int>(velocity_commands_[1]);
    int pwm3 = static_cast<int>(velocity_commands_[2]);

    pwm1 = std::clamp(pwm1, 1000, 2000);
    pwm2 = std::clamp(pwm2, 1000, 2000);
    pwm3 = std::clamp(pwm3, 1000, 2000);

    // Format: "PWM1 PWM2\n" (e.g., "1500 1600\n")
    std::string data = std::to_string(pwm1) + " " + std::to_string(pwm2) + " " + std::to_string(pwm3) + "\n";

    // Send to Arduino
    WriteToSerial(reinterpret_cast<const unsigned char*>(data.c_str()), data.length());

    RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), 
                "PWM1: %d, PWM2: %d, PWM3: %d", pwm1, pwm2, pwm3);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("arduino_actuator_interface"), 
                 "Erro no write: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArduinoInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    (void) period;
    return hardware_interface::return_type::OK;
}

rclcpp::Node::SharedPtr node_;  // Node for publisher

}  // namespace arduinobot_controller

PLUGINLIB_EXPORT_CLASS(arduino_controller::ArduinoInterface, hardware_interface::SystemInterface)
