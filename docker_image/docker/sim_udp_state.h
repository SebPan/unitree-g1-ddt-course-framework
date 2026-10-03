#pragma once

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>

#include <mujoco/mujoco.h>

static constexpr uint32_t G1_SIM_STATE_MAGIC = 0x47315354;
static constexpr uint32_t G1_SIM_STATE_VERSION = 1;
static constexpr int G1_SIM_NUM_JOINTS = 23;

#pragma pack(push, 1)

struct G1SimStatePacket
{
    uint32_t magic;
    uint32_t version;

    uint64_t sequence;
    double sim_time;

    double q[G1_SIM_NUM_JOINTS];
    double dq[G1_SIM_NUM_JOINTS];
    double tau[G1_SIM_NUM_JOINTS];

    double imu_quat[4];
    double imu_gyro[3];
    double imu_accel[3];
};

#pragma pack(pop)


class SimUdpStatePublisher
{
public:
    SimUdpStatePublisher()
    {
        socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);

        std::memset(&address_, 0, sizeof(address_));

        address_.sin_family = AF_INET;
        address_.sin_port = htons(15010);
        address_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    }

    ~SimUdpStatePublisher()
    {
        if (socket_ >= 0)
            ::close(socket_);
    }

    void publish(
        mjData *data,
        int num_motor,
        int imu_quat_adr,
        int imu_gyro_adr,
        int imu_acc_adr)
    {
        if (socket_ < 0 || data == nullptr)
            return;

        G1SimStatePacket packet{};

        packet.magic = G1_SIM_STATE_MAGIC;
        packet.version = G1_SIM_STATE_VERSION;
        packet.sequence = sequence_++;
        packet.sim_time = data->time;

        // G1 EDU 23DOF dentro de los 29 slots del modelo.
        static constexpr int motor_map[G1_SIM_NUM_JOINTS] =
        {
            0, 1, 2, 3, 4, 5,
            6, 7, 8, 9, 10, 11,
            12,
            15, 16, 17, 18, 19,
            22, 23, 24, 25, 26
        };

        for (int i = 0; i < G1_SIM_NUM_JOINTS; ++i)
        {
            const int m = motor_map[i];

            packet.q[i] =
                data->sensordata[m];

            packet.dq[i] =
                data->sensordata[m + num_motor];

            packet.tau[i] =
                data->sensordata[m + 2 * num_motor];
        }

        for (int i = 0; i < 4; ++i)
            packet.imu_quat[i] =
                data->sensordata[imu_quat_adr + i];

        for (int i = 0; i < 3; ++i)
        {
            packet.imu_gyro[i] =
                data->sensordata[imu_gyro_adr + i];

            packet.imu_accel[i] =
                data->sensordata[imu_acc_adr + i];
        }

        ::sendto(
            socket_,
            &packet,
            sizeof(packet),
            MSG_DONTWAIT,
            reinterpret_cast<sockaddr *>(&address_),
            sizeof(address_));
    }

private:
    int socket_ = -1;
    sockaddr_in address_{};
    uint64_t sequence_ = 0;
};
