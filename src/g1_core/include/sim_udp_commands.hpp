#pragma once

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <stdexcept>

class SimUdpCommands
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

    SimUdpCommands()
    {
        socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);

        if (socket_ < 0)
        {
            throw std::runtime_error(
                "No se pudo crear socket UDP de comandos SIM"
            );
        }

        std::memset(&address_, 0, sizeof(address_));

        address_.sin_family = AF_INET;
        address_.sin_port = htons(15020);
        address_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    }

    ~SimUdpCommands()
    {
        if (socket_ >= 0)
        {
            ::close(socket_);
        }
    }

    void send(
        const std::array<float, NUM_JOINTS> &q,
        const std::array<float, NUM_JOINTS> &dq,
        const std::array<float, NUM_JOINTS> &tau,
        const std::array<float, NUM_JOINTS> &kp,
        const std::array<float, NUM_JOINTS> &kd)
    {
        Packet packet{};

        packet.magic = MAGIC;
        packet.version = VERSION;
        packet.sequence = sequence_++;

        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            packet.q[i] = q[i];
            packet.dq[i] = dq[i];
            packet.tau[i] = tau[i];
            packet.kp[i] = kp[i];
            packet.kd[i] = kd[i];
        }

        ::sendto(
            socket_,
            &packet,
            sizeof(packet),
            MSG_DONTWAIT,
            reinterpret_cast<sockaddr *>(&address_),
            sizeof(address_)
        );
    }

private:
    int socket_ = -1;
    sockaddr_in address_{};
    uint64_t sequence_ = 0;
};
