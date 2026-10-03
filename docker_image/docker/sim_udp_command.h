#pragma once

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

#include <mujoco/mujoco.h>

class SimUdpCommandReceiver
{
public:
    static constexpr uint32_t MAGIC = 0x4731434D;
    static constexpr uint32_t VERSION = 1;
    static constexpr int NUM_JOINTS = 23;

#pragma pack(push, 1)

    struct Packet
    {
        uint32_t magic;
        uint32_t version;

        uint64_t sequence;

        float q[NUM_JOINTS];
        float dq[NUM_JOINTS];
        float tau[NUM_JOINTS];
        float kp[NUM_JOINTS];
        float kd[NUM_JOINTS];
    };

#pragma pack(pop)

    SimUdpCommandReceiver()
    {
        socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);

        if (socket_ < 0)
        {
            throw std::runtime_error(
                "No se pudo crear UDP comando SIM"
            );
        }

        sockaddr_in address{};

        address.sin_family = AF_INET;
        address.sin_port = htons(15020);
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
                "No se pudo bindear UDP SIM 15020: " +
                error
            );
        }
    }

    ~SimUdpCommandReceiver()
    {
        if (socket_ >= 0)
        {
            ::close(socket_);
        }
    }

    void apply(
        mjData *data,
        int num_motor)
    {
        if (!data || socket_ < 0)
        {
            return;
        }

        Packet packet{};
        Packet newest{};

        bool received = false;

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

            if (bytes !=
                static_cast<ssize_t>(sizeof(Packet)))
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

        if (received)
        {
            latest_ = newest;
            has_command_ = true;
        }

        if (!has_command_)
        {
            return;
        }

        static constexpr int motor_map[NUM_JOINTS] =
        {
            0, 1, 2, 3, 4, 5,
            6, 7, 8, 9, 10, 11,
            12,
            15, 16, 17, 18, 19,
            22, 23, 24, 25, 26
        };

        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            const int motor = motor_map[i];

            if (motor >= num_motor)
            {
                continue;
            }

            const double q_actual =
                data->sensordata[motor];

            const double dq_actual =
                data->sensordata[
                    motor + num_motor
                ];

            data->ctrl[motor] =
                latest_.tau[i]
                +
                latest_.kp[i] *
                (
                    latest_.q[i] -
                    q_actual
                )
                +
                latest_.kd[i] *
                (
                    latest_.dq[i] -
                    dq_actual
                );
        }
    }

private:
    int socket_ = -1;

    Packet latest_{};

    bool has_command_ = false;
};
