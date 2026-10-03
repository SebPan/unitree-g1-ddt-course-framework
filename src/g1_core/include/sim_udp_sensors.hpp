#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>


class SimUdpSensors
{
public:

    static constexpr uint32_t MAGIC = 0x47315354;
    static constexpr uint32_t VERSION = 1;
    static constexpr int NUM_JOINTS = 23;


#pragma pack(push, 1)

    struct Packet
    {
        uint32_t magic;
        uint32_t version;

        uint64_t sequence;
        double sim_time;

        double q[NUM_JOINTS];
        double dq[NUM_JOINTS];
        double tau[NUM_JOINTS];

        double imu_quat[4];
        double imu_gyro[3];
        double imu_accel[3];
    };

#pragma pack(pop)


    explicit SimUdpSensors(rclcpp::Node *node)
        : node_(node)
    {
        socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);

        if (socket_ < 0)
        {
            throw std::runtime_error(
                "No se pudo crear socket UDP SIM"
            );
        }


        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(15010);
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);


        if (::bind(
                socket_,
                reinterpret_cast<sockaddr *>(&address),
                sizeof(address)) < 0)
        {
            const std::string error =
                std::strerror(errno);

            ::close(socket_);
            socket_ = -1;

            throw std::runtime_error(
                "No se pudo bindear UDP SIM 127.0.0.1:15010: "
                + error
            );
        }


        joint_pub_ =
            node_->create_publisher<sensor_msgs::msg::JointState>(
                "/g1/joint_states",
                10
            );

        imu_pub_ =
            node_->create_publisher<sensor_msgs::msg::Imu>(
                "/g1/imu",
                10
            );


        timer_ =
            node_->create_wall_timer(
                std::chrono::milliseconds(5),
                std::bind(
                    &SimUdpSensors::poll,
                    this
                )
            );


        RCLCPP_INFO(
            node_->get_logger(),
            "SIM Sensors UDP escuchando en 127.0.0.1:15010"
        );
    }


    ~SimUdpSensors()
    {
        if (socket_ >= 0)
        {
            ::close(socket_);
        }
    }


private:

    void poll()
    {
        Packet packet{};
        Packet newest{};

        bool received = false;


        // Drenamos la cola y usamos únicamente el paquete más reciente.
        while (true)
        {
            const ssize_t bytes =
                ::recvfrom(
                    socket_,
                    &packet,
                    sizeof(packet),
                    MSG_DONTWAIT,
                    nullptr,
                    nullptr
                );

            if (bytes < 0)
            {
                break;
            }


            if (bytes != static_cast<ssize_t>(sizeof(Packet)))
            {
                continue;
            }


            if (packet.magic != MAGIC ||
                packet.version != VERSION)
            {
                continue;
            }


            newest = packet;
            received = true;
        }


        if (!received)
        {
            return;
        }


        if (!first_packet_received_)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "Primer estado SIM recibido. seq=%lu",
                static_cast<unsigned long>(newest.sequence)
            );

            first_packet_received_ = true;
        }


        publish_joint_state(newest);
        publish_imu(newest);
    }


    void publish_joint_state(const Packet &packet)
    {
        sensor_msgs::msg::JointState msg;

        msg.header.stamp = node_->now();

        msg.name = joint_names_;

        msg.position.resize(NUM_JOINTS);
        msg.velocity.resize(NUM_JOINTS);
        msg.effort.resize(NUM_JOINTS);


        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            msg.position[i] = packet.q[i];
            msg.velocity[i] = packet.dq[i];
            msg.effort[i] = packet.tau[i];
        }


        joint_pub_->publish(msg);
    }


    void publish_imu(const Packet &packet)
    {
        sensor_msgs::msg::Imu msg;

        msg.header.stamp = node_->now();


        // MuJoCo / Unitree: [w, x, y, z]
        // ROS: x, y, z, w
        msg.orientation.w = packet.imu_quat[0];
        msg.orientation.x = packet.imu_quat[1];
        msg.orientation.y = packet.imu_quat[2];
        msg.orientation.z = packet.imu_quat[3];


        msg.angular_velocity.x = packet.imu_gyro[0];
        msg.angular_velocity.y = packet.imu_gyro[1];
        msg.angular_velocity.z = packet.imu_gyro[2];


        msg.linear_acceleration.x = packet.imu_accel[0];
        msg.linear_acceleration.y = packet.imu_accel[1];
        msg.linear_acceleration.z = packet.imu_accel[2];


        imu_pub_->publish(msg);
    }


private:

    rclcpp::Node *node_;

    int socket_ = -1;

    bool first_packet_received_ = false;


    rclcpp::Publisher<
        sensor_msgs::msg::JointState
    >::SharedPtr joint_pub_;

    rclcpp::Publisher<
        sensor_msgs::msg::Imu
    >::SharedPtr imu_pub_;

    rclcpp::TimerBase::SharedPtr timer_;


    const std::vector<std::string> joint_names_ =
    {
        "left_hip_pitch",
        "left_hip_roll",
        "left_hip_yaw",
        "left_knee",
        "left_ankle_pitch",
        "left_ankle_roll",

        "right_hip_pitch",
        "right_hip_roll",
        "right_hip_yaw",
        "right_knee",
        "right_ankle_pitch",
        "right_ankle_roll",

        "waist_yaw",

        "left_shoulder_pitch",
        "left_shoulder_roll",
        "left_shoulder_yaw",
        "left_elbow",
        "left_wrist_roll",

        "right_shoulder_pitch",
        "right_shoulder_roll",
        "right_shoulder_yaw",
        "right_elbow",
        "right_wrist_roll"
    };
};
