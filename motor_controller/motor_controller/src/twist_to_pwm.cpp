#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using std::placeholders::_1;

class TwistToPWM : public rclcpp::Node
{
public:
    TwistToPWM() : Node("twist_to_pwm")
    {
        // Parâmetros configuráveis
        this->declare_parameter<double>("max_speed", 1900.0);
        this->declare_parameter<double>("min_speed", 1100.0);
        this->declare_parameter<double>("neutral", 1500.0);
        this->declare_parameter<int>("thruster_count", 3);

        // Subscriber para comandos Twist
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/openrov/cmd_vel", 10,
            std::bind(&TwistToPWM::twistCallback, this, _1));

        // Publisher para comandos PWM
        pwm_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/motor_controller/commands", 10);

        RCLCPP_INFO(this->get_logger(), "Twist to PWM converter ready");
    }

private:
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Obter parâmetros
        double max_speed = this->get_parameter("max_speed").as_double();
        double min_speed = this->get_parameter("min_speed").as_double();
        double neutral = this->get_parameter("neutral").as_double();
        int thruster_count = this->get_parameter("thruster_count").as_int();

        // Criar mensagem PWM
        auto pwm_msg = std_msgs::msg::Float64MultiArray();
        pwm_msg.data.resize(thruster_count);

        // Lógica de conversão (exemplo para 3 thrusters)
        double linear = msg->linear.x;
        double angular = msg->angular.z;

        // Configuração padrão para ROV com 3 thrusters:
        // - Thruster 0: Frontal/Traseiro
        // - Thruster 1: Lateral Esquerdo
        // - Thruster 2: Lateral Direito
        pwm_msg.data[0] = neutral + linear * (max_speed - neutral); // Frontal
        pwm_msg.data[1] = neutral + (linear - angular) * (max_speed - neutral); // Esquerdo
        pwm_msg.data[2] = neutral + (linear + angular) * (max_speed - neutral); // Direito

        // Garantir limites de PWM
        for (auto& value : pwm_msg.data) {
            value = std::clamp(value, min_speed, max_speed);
        }

        // Publicar comandos
        pwm_pub_->publish(pwm_msg);

        RCLCPP_DEBUG(this->get_logger(), "PWM Commands: [%.1f, %.1f, %.1f]",
                    pwm_msg.data[0], pwm_msg.data[1], pwm_msg.data[2]);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pwm_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwistToPWM>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}