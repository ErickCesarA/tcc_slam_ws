#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>

#include <stdexcept>

class GroundTruthOdom : public rclcpp::Node
{
public:
    GroundTruthOdom()
    : Node("ground_truth_odom")
    {
        // Parametros
        this->declare_parameter<std::string>(
            "gazebo_topic",
            "/world/tcc_world/pose/info"
        );

        this->declare_parameter<std::string>(
            "model_name",
            "tcc_robot"
        );

        this->declare_parameter<std::string>(
            "world_frame",
            "world"
        );

        this->declare_parameter<std::string>(
            "child_frame",
            "tcc_robot"
        );

        gazebo_topic_ = this->get_parameter("gazebo_topic").as_string();
        model_name_ = this->get_parameter("model_name").as_string();
        world_frame_ = this->get_parameter("world_frame").as_string();
        child_frame_ = this->get_parameter("child_frame").as_string();

        // Publicador ROS 2
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/ground_truth/odom",
            10
        );

        // Assinante direto do Gazebo Transport
        if (!gazebo_node_.Subscribe(
                gazebo_topic_,
                &GroundTruthOdom::gazeboCallback,
                this))
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Falha ao assinar o topico Gazebo: %s",
                gazebo_topic_.c_str()
            );

            throw std::runtime_error("Falha na assinatura Gazebo");
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Assinado em: %s",
            gazebo_topic_.c_str()
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Modelo monitorado: %s",
            model_name_.c_str()
        );
    }

private:

    void gazeboCallback(const gz::msgs::Pose_V &_msg)
    {
        for (int i = 0; i < _msg.pose_size(); ++i)
        {
            const auto &pose = _msg.pose(i);

            // Seleciona exclusivamente o modelo desejado
            if (pose.name() != model_name_)
            {
                continue;
            }

            nav_msgs::msg::Odometry odom;

            // Timestamp ROS 2 (usar_sim_time deve estar habilitado)
            const auto timestamp = this->get_clock()->now();

            odom.header.stamp.sec = static_cast<int32_t>(timestamp.seconds());
            odom.header.stamp.nanosec =
   	    static_cast<uint32_t>(timestamp.nanoseconds() % 1000000000LL);

            odom.header.frame_id = world_frame_;
            odom.child_frame_id = child_frame_;

            // Posicao
            odom.pose.pose.position.x = pose.position().x();
            odom.pose.pose.position.y = pose.position().y();
            odom.pose.pose.position.z = pose.position().z();

            // Orientacao
            odom.pose.pose.orientation.x = pose.orientation().x();
            odom.pose.pose.orientation.y = pose.orientation().y();
            odom.pose.pose.orientation.z = pose.orientation().z();
            odom.pose.pose.orientation.w = pose.orientation().w();

            // Velocidades não são fornecidas por esta pose.
            // Permanecem zeradas e não devem ser usadas como ground truth.

            publisher_->publish(odom);

            return;
        }
    }

    gz::transport::Node gazebo_node_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;

    std::string gazebo_topic_;
    std::string model_name_;
    std::string world_frame_;
    std::string child_frame_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GroundTruthOdom>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
