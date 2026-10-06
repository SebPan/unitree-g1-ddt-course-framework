#pragma once

#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/dds_wrapper/common/unitree_joystick.hpp>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>


class SimLowState
{
public:
    using SharedPtr = std::shared_ptr<SimLowState>;

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


    SimLowState()
    {
        last_update_time_ =
            std::chrono::steady_clock::now()
            - std::chrono::seconds(10);

        socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);

        if (socket_ < 0)
        {
            throw std::runtime_error(
                "SimLowState: no se pudo crear socket UDP"
            );
        }

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(15011);
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

        if (
            ::bind(
                socket_,
                reinterpret_cast<sockaddr *>(&address),
                sizeof(address)
            ) < 0
        )
        {
            const std::string error = std::strerror(errno);

            ::close(socket_);
            socket_ = -1;

            throw std::runtime_error(
                "SimLowState: bind 127.0.0.1:15011 fallo: "
                + error
            );
        }

        // Permite que recvfrom despierte periódicamente.
        timeval timeout{};
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;

        setsockopt(
            socket_,
            SOL_SOCKET,
            SO_RCVTIMEO,
            &timeout,
            sizeof(timeout)
        );

        running_ = true;

        receiver_thread_ =
            std::thread(
                &SimLowState::receiver_loop,
                this
            );
    }


    ~SimLowState()
    {
        running_ = false;

        if (receiver_thread_.joinable())
        {
            receiver_thread_.join();
        }

        if (socket_ >= 0)
        {
            ::close(socket_);
        }
    }


    void update()
    {
        // En SIM el estado se actualiza continuamente
        // desde el hilo receptor UDP.
        //
        // La implementación Unitree original usa update()
        // principalmente para actualizar el joystick desde
        // wireless_remote. Nuestro HighLevel SIM utiliza
        // SimCommand, por lo que aquí no necesitamos hacer nada.
    }


    bool isTimeout()
    {
        const auto now =
            std::chrono::steady_clock::now();

        return (
            now - last_update_time_
            >
            std::chrono::milliseconds(timeout_ms_)
        );
    }


    void wait_for_connection()
    {
        bool warning_printed = false;
        const auto start =
            std::chrono::steady_clock::now();

        while (isTimeout())
        {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(100)
            );

            if (
                !warning_printed &&
                std::chrono::steady_clock::now() - start
                    > std::chrono::seconds(2)
            )
            {
                std::cout
                    << "Waiting for SIM state UDP 127.0.0.1:15011"
                    << std::endl;

                warning_printed = true;
            }
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(100)
        );

        std::cout
            << "Connected to SIM state UDP"
            << std::endl;
    }


    unitree_hg::msg::dds_::LowState_ msg_;
    std::mutex mutex_;

    unitree::common::UnitreeJoystick joystick;


private:

    void receiver_loop()
    {
        while (running_)
        {
            Packet packet{};

            const ssize_t bytes =
                ::recvfrom(
                    socket_,
                    &packet,
                    sizeof(packet),
                    0,
                    nullptr,
                    nullptr
                );

            if (bytes != static_cast<ssize_t>(sizeof(Packet)))
            {
                continue;
            }

            if (
                packet.magic != MAGIC ||
                packet.version != VERSION
            )
            {
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(mutex_);

                // El controlador G1 23DOF espera mode_machine = 4.
                msg_.mode_machine() = 4;

                for (int i = 0; i < NUM_JOINTS; ++i)
                {
                    const int motor = MOTOR_ID[i];

                    msg_.motor_state()[motor].q() =
                        static_cast<float>(packet.q[i]);

                    msg_.motor_state()[motor].dq() =
                        static_cast<float>(packet.dq[i]);

                    msg_.motor_state()[motor].tau_est() =
                        static_cast<float>(packet.tau[i]);
                }

                for (int i = 0; i < 4; ++i)
                {
                    msg_.imu_state().quaternion()[i] =
                        static_cast<float>(
                            packet.imu_quat[i]
                        );
                }

                for (int i = 0; i < 3; ++i)
                {
                    msg_.imu_state().gyroscope()[i] =
                        static_cast<float>(
                            packet.imu_gyro[i]
                        );

                    msg_.imu_state().accelerometer()[i] =
                        static_cast<float>(
                            packet.imu_accel[i]
                        );
                }
            }

            last_update_time_ =
                std::chrono::steady_clock::now();

            if (!first_packet_received_)
            {
                first_packet_received_ = true;

                std::cout
                    << "First SIM LowState UDP packet received"
                    << std::endl;
            }
        }
    }


    static constexpr std::array<int, NUM_JOINTS> MOTOR_ID =
    {
        0, 1, 2, 3, 4, 5,
        6, 7, 8, 9, 10, 11,
        12,
        15, 16, 17, 18, 19,
        22, 23, 24, 25, 26
    };


    int socket_ = -1;

    std::atomic<bool> running_{false};
    std::thread receiver_thread_;

    uint32_t timeout_ms_ = 1000;

    std::chrono::steady_clock::time_point
        last_update_time_;

    bool first_packet_received_ = false;
};
