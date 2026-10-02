#include <chrono>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class G1CameraBridge : public rclcpp::Node
{
public:
    G1CameraBridge()
        : Node("g1_camera_bridge")
    {
        publisher_ =
            this->create_publisher<sensor_msgs::msg::Image>(
                "/camera/image_raw",
                rclcpp::SensorDataQoS()
                );

        buffer_.resize(WIDTH * HEIGHT * CHANNELS);

        timer_ =
            this->create_wall_timer(
                100ms,
                std::bind(
                    &G1CameraBridge::publish_image,
                    this
                    )
                );

        RCLCPP_INFO(
            this->get_logger(),
            "G1 camera bridge started"
            );
    }

private:

    static constexpr int WIDTH = 640;
    static constexpr int HEIGHT = 480;
    static constexpr int CHANNELS = 3;

    const std::string camera_path_ =
        "/dev/shm/g1_head_camera.rgb";

    void publish_image()
    {
        std::ifstream file(
            camera_path_,
            std::ios::binary
            );

        if (!file.is_open())
            return;

        file.read(
            reinterpret_cast<char*>(buffer_.data()),
            buffer_.size()
            );

        if (
            file.gcount() !=
            static_cast<std::streamsize>(buffer_.size())
            )
            return;

        sensor_msgs::msg::Image msg;

        msg.header.stamp = this->now();
        msg.header.frame_id = "head_camera";

        msg.height = HEIGHT;
        msg.width = WIDTH;

        msg.encoding = "rgb8";
        msg.is_bigendian = false;

        msg.step = WIDTH * CHANNELS;

        msg.data = buffer_;

        publisher_->publish(msg);

        if (!first_frame_)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Publishing /camera/image_raw 640x480 RGB"
                );

            first_frame_ = true;
        }
    }

    std::vector<uint8_t> buffer_;

    bool first_frame_ = false;

    rclcpp::Publisher<
        sensor_msgs::msg::Image
        >::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<G1CameraBridge>()
        );

    rclcpp::shutdown();

    return 0;
}