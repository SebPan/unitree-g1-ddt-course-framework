#pragma once

#include <atomic>
#include <thread>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <mutex>
#include <string>
#include <unordered_map>
#include <sstream>

#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include <spdlog/spdlog.h>


namespace simcmd
{

enum Mode
{
    PASSIVE  = 0,
    STAND    = 1,
    VELOCITY = 2
};


// ============================================================
// Locomocion
// ============================================================

inline std::atomic<int>& mode()
{
    static std::atomic<int> value{PASSIVE};
    return value;
}


inline std::atomic<float>& vx()
{
    static std::atomic<float> value{0.0f};
    return value;
}


inline std::atomic<float>& vy()
{
    static std::atomic<float> value{0.0f};
    return value;
}


inline std::atomic<float>& wz()
{
    static std::atomic<float> value{0.0f};
    return value;
}


// ============================================================
// Overrides articulares genericos
//
// La clave es el nombre logico del joint:
//
// right_elbow
// right_shoulder_roll
// ...
//
// Aqui NO existen IDs de motores ni indices de la policy.
// ============================================================

using JointOverrideMap =
    std::unordered_map<std::string, float>;


inline std::mutex& joint_override_mutex()
{
    static std::mutex mutex;
    return mutex;
}


inline JointOverrideMap& joint_override_storage()
{
    static JointOverrideMap overrides;
    return overrides;
}


inline void set_joint_override(
    const std::string& name,
    float position)
{
    std::lock_guard<std::mutex> lock(
        joint_override_mutex()
    );

    joint_override_storage()[name] =
        position;
}


inline void release_joint_override(
    const std::string& name)
{
    std::lock_guard<std::mutex> lock(
        joint_override_mutex()
    );

    joint_override_storage().erase(
        name
    );
}


inline void release_all_joint_overrides()
{
    std::lock_guard<std::mutex> lock(
        joint_override_mutex()
    );

    joint_override_storage().clear();
}


inline JointOverrideMap joint_overrides()
{
    std::lock_guard<std::mutex> lock(
        joint_override_mutex()
    );

    return joint_override_storage();
}


// ============================================================
// Receptor UDP interno del backend SIM
//
// LOCOMOCION:
//
//   mode vx vy wz
//
// Ejemplo:
//
//   2 0.10 0.0 0.0
//
//
// OVERRIDE:
//
//   J joint_name position
//
// Ejemplo:
//
//   J right_elbow 0.70
//
//
// RELEASE:
//
//   R joint_name
//
// Ejemplo:
//
//   R right_elbow
//
//
// RELEASE TODOS:
//
//   R ALL
//
// ============================================================

inline void start_udp_receiver(int port = 15000)
{
    static std::atomic<bool> started{false};

    if(started.exchange(true))
    {
        return;
    }


    std::thread(
        [port]()
        {
            int sock =
                socket(
                    AF_INET,
                    SOCK_DGRAM,
                    0
                );


            if(sock < 0)
            {
                spdlog::error(
                    "SimCommand: socket() failed"
                );

                return;
            }


            sockaddr_in addr{};

            addr.sin_family =
                AF_INET;

            addr.sin_port =
                htons(port);

            addr.sin_addr.s_addr =
                htonl(INADDR_LOOPBACK);


            if(
                bind(
                    sock,
                    reinterpret_cast<sockaddr*>(&addr),
                    sizeof(addr)
                ) < 0
            )
            {
                spdlog::error(
                    "SimCommand: bind failed: {}",
                    std::strerror(errno)
                );

                close(sock);

                return;
            }


            spdlog::info(
                "SimCommand listening on 127.0.0.1:{}",
                port
            );


            char buffer[256];


            while(true)
            {
                ssize_t n =
                    recvfrom(
                        sock,
                        buffer,
                        sizeof(buffer) - 1,
                        0,
                        nullptr,
                        nullptr
                    );


                if(n <= 0)
                {
                    continue;
                }


                buffer[n] = '\0';


                std::istringstream stream(
                    buffer
                );

                std::string command;

                stream >> command;


                if(command.empty())
                {
                    continue;
                }


                // ================================================
                // J <joint_name> <position>
                // ================================================

                if(command == "J")
                {
                    std::string joint_name;
                    float position;


                    if(
                        !(stream >> joint_name >> position)
                    )
                    {
                        spdlog::warn(
                            "SimCommand: invalid joint command: {}",
                            buffer
                        );

                        continue;
                    }


                    set_joint_override(
                        joint_name,
                        position
                    );


                    spdlog::info(
                        "Joint override: {} -> {:.3f}",
                        joint_name,
                        position
                    );


                    continue;
                }


                // ================================================
                // R <joint_name>
                //
                // R ALL
                // ================================================

                if(command == "R")
                {
                    std::string joint_name;


                    if(!(stream >> joint_name))
                    {
                        spdlog::warn(
                            "SimCommand: invalid release command: {}",
                            buffer
                        );

                        continue;
                    }


                    if(joint_name == "ALL")
                    {
                        release_all_joint_overrides();

                        spdlog::info(
                            "Joint overrides: RELEASE ALL"
                        );
                    }
                    else
                    {
                        release_joint_override(
                            joint_name
                        );

                        spdlog::info(
                            "Joint override released: {}",
                            joint_name
                        );
                    }


                    continue;
                }


                // ================================================
                // Locomocion
                //
                // mode vx vy wz
                // ================================================

                int new_mode = PASSIVE;

                float new_vx = 0.0f;
                float new_vy = 0.0f;
                float new_wz = 0.0f;


                // ------------------------------------------------
                // Compatibilidad TEMPORAL con el formato anterior:
                //
                // mode vx vy wz arm shoulder elbow
                //
                // Esto se eliminara cuando g1_core ya use
                // completamente joint_override.
                // ------------------------------------------------

                int arm_enable = 0;

                float shoulder_roll = -0.25f;
                float elbow = 0.97f;


                int fields =
                    std::sscanf(
                        buffer,
                        "%d %f %f %f %d %f %f",
                        &new_mode,
                        &new_vx,
                        &new_vy,
                        &new_wz,
                        &arm_enable,
                        &shoulder_roll,
                        &elbow
                    );


                if(fields != 4 && fields != 7)
                {
                    spdlog::warn(
                        "SimCommand: invalid command: {}",
                        buffer
                    );

                    continue;
                }


                if(
                    new_mode < PASSIVE
                    ||
                    new_mode > VELOCITY
                )
                {
                    spdlog::warn(
                        "SimCommand: invalid mode {}",
                        new_mode
                    );

                    continue;
                }


                mode().store(
                    new_mode
                );

                vx().store(
                    new_vx
                );

                vy().store(
                    new_vy
                );

                wz().store(
                    new_wz
                );


                // ------------------------------------------------
                // Migracion del protocolo antiguo hacia el
                // nuevo sistema generico.
                // ------------------------------------------------

                if(fields == 7)
                {
                    if(arm_enable != 0)
                    {
                        set_joint_override(
                            "right_shoulder_roll",
                            shoulder_roll
                        );

                        set_joint_override(
                            "right_elbow",
                            elbow
                        );
                    }
                    else
                    {
                        release_joint_override(
                            "right_shoulder_roll"
                        );

                        release_joint_override(
                            "right_elbow"
                        );
                    }


                    spdlog::warn(
                        "Legacy arm command received. "
                        "Use generic joint overrides."
                    );
                }


                spdlog::info(
                    "SimCommand: mode={} "
                    "vx={:.3f} vy={:.3f} wz={:.3f}",
                    new_mode,
                    new_vx,
                    new_vy,
                    new_wz
                );
            }
        }
    ).detach();
}


}

