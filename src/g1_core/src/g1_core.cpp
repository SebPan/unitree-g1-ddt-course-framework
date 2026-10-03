#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <stdexcept>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "unitree_api/msg/request.hpp"

#include "nlohmann/json.hpp"

#include "g1_core/msg/joint_command.hpp"

#include "motor_crc_hg.h"


constexpr int G1_NUM_JOINTS = 23;


class G1Core : public rclcpp::Node
{
public:

    G1Core()
        : Node("g1_core")
    {
        // ====================================================
        // Backend SIM / REAL
        // ====================================================

        simulation_ =
            this->declare_parameter<bool>(
                "simulation",
                false
            );

        RCLCPP_INFO(
            this->get_logger(),
            "Backend: %s",
            simulation_ ? "SIM" : "REAL"
        );


        // ====================================================
        // Estado Unitree
        // ====================================================

        lowstate_sub_ =
            this->create_subscription<unitree_hg::msg::LowState>(
                "/lowstate",
                10,
                std::bind(
                    &G1Core::lowstate_callback,
                    this,
                    std::placeholders::_1
                )
            );


        // ====================================================
        // Estado normalizado para la API Python
        // ====================================================

        joint_state_pub_ =
            this->create_publisher<sensor_msgs::msg::JointState>(
                "/g1/joint_states",
                10
            );

        imu_pub_ =
            this->create_publisher<sensor_msgs::msg::Imu>(
                "/g1/imu",
                10
            );


        // ====================================================
        // LowLevel - siempre disponible
        // ====================================================

        joint_command_sub_ =
            this->create_subscription<g1_core::msg::JointCommand>(
                "/g1/joint_command",
                10,
                std::bind(
                    &G1Core::joint_command_callback,
                    this,
                    std::placeholders::_1
                )
            );

        lowcmd_pub_ =
            this->create_publisher<unitree_hg::msg::LowCmd>(
                "/lowcmd",
                10
            );

        control_timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(2),
                std::bind(
                    &G1Core::control_loop,
                    this
                )
            );


        // ====================================================
        // HighLevel SIM bridge
        //
        // REAL utilizará posteriormente su backend Unitree.
        // ====================================================

        if (simulation_)
        {
            udp_socket_ =
                ::socket(
                    AF_INET,
                    SOCK_DGRAM,
                    0
                );

            if (udp_socket_ < 0)
            {
                throw std::runtime_error(
                    "No se pudo crear socket UDP"
                );
            }

            udp_address_.sin_family = AF_INET;
            udp_address_.sin_port = htons(15000);

            if (
                ::inet_pton(
                    AF_INET,
                    "127.0.0.1",
                    &udp_address_.sin_addr
                ) != 1
            )
            {
                throw std::runtime_error(
                    "Direccion UDP invalida"
                );
            }

            sport_request_sub_ =
                this->create_subscription<
                    unitree_api::msg::Request
                >(
                    "/api/sport/request",
                    10,
                    std::bind(
                        &G1Core::sport_request_callback,
                        this,
                        std::placeholders::_1
                    )
                );

            arm_sdk_sub_ =
                this->create_subscription<
                    unitree_hg::msg::LowCmd
                >(
                    "/arm_sdk",
                    10,
                    std::bind(
                        &G1Core::arm_sdk_callback,
                        this,
                        std::placeholders::_1
                    )
                );
        }


        // ====================================================
        // Valores iniciales LowLevel
        // ====================================================

        command_velocity_.fill(0.0f);
        command_torque_.fill(0.0f);

        command_kp_ = default_kp_;
        command_kd_ = default_kd_;


        RCLCPP_INFO(
            this->get_logger(),
            "G1 Core iniciado"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Esperando /lowstate..."
        );
    }

    ~G1Core() override
    {
        if (udp_socket_ >= 0)
        {
            ::close(udp_socket_);
        }
    }


private:

    // ========================================================
    // Articulaciones lógicas
    // ========================================================

    const std::array<std::string, G1_NUM_JOINTS> joint_names_ = {

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

    static constexpr int G1_NUM_ARM_JOINTS = 10;
    static constexpr int ARM_WEIGHT_MOTOR = 29;


    const std::array<std::string, G1_NUM_ARM_JOINTS>
        arm_joint_names_ = {

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


const std::array<int, G1_NUM_ARM_JOINTS>
    arm_motor_id_ = {

15, 16, 17, 18, 19,
    22, 23, 24, 25, 26
};


// ========================================================
// Motor IDs reales Unitree G1 23DOF
// ========================================================

const std::array<int, G1_NUM_JOINTS> motor_id_ = {

0, 1, 2, 3, 4, 5,

    6, 7, 8, 9, 10, 11,

    12,

    15, 16, 17, 18, 19,

    22, 23, 24, 25, 26
};


// ========================================================
// Limites articulares G1 23DOF
//
// IMPORTANTE:
// El orden debe coincidir exactamente con joint_names_
// ========================================================

const std::array<float, G1_NUM_JOINTS> joint_min_ = {

    // ----------------------------------------------------
    // Left leg
    // indices 0 - 5
    // ----------------------------------------------------

    -2.5307f,    // 0  left_hip_pitch
    -0.5236f,    // 1  left_hip_roll
    -2.7576f,    // 2  left_hip_yaw
    -0.087267f,  // 3  left_knee
    -0.87267f,   // 4  left_ankle_pitch
    -0.2618f,    // 5  left_ankle_roll


    // ----------------------------------------------------
    // Right leg
    // indices 6 - 11
    // ----------------------------------------------------

    -2.5307f,    // 6  right_hip_pitch
    -2.9671f,    // 7  right_hip_roll
    -2.7576f,    // 8  right_hip_yaw
    -0.087267f,  // 9  right_knee
    -0.87267f,   // 10 right_ankle_pitch
    -0.2618f,    // 11 right_ankle_roll


    // ----------------------------------------------------
    // Waist
    // index 12
    // ----------------------------------------------------

    -2.6180f,    // 12 waist_yaw


    // ----------------------------------------------------
    // Left arm
    // indices 13 - 17
    // ----------------------------------------------------

    -3.0892f,    // 13 left_shoulder_pitch
    -1.5882f,    // 14 left_shoulder_roll
    -2.6180f,    // 15 left_shoulder_yaw
    -1.0472f,    // 16 left_elbow
    -1.97222f,   // 17 left_wrist_roll


    // ----------------------------------------------------
    // Right arm
    // indices 18 - 22
    // ----------------------------------------------------

    -3.0892f,    // 18 right_shoulder_pitch
    -2.2515f,    // 19 right_shoulder_roll
    -2.6180f,    // 20 right_shoulder_yaw
    -1.0472f,    // 21 right_elbow
    -1.97222f    // 22 right_wrist_roll
};


const std::array<float, G1_NUM_JOINTS> joint_max_ = {

    // ----------------------------------------------------
    // Left leg
    // ----------------------------------------------------

     2.8798f,   // 0
     2.9671f,   // 1
     2.7576f,   // 2
     2.8798f,   // 3
     0.5236f,   // 4
     0.2618f,   // 5


    // ----------------------------------------------------
    // Right leg
    // ----------------------------------------------------

     2.8798f,   // 6
     0.5236f,   // 7
     2.7576f,   // 8
     2.8798f,   // 9
     0.5236f,   // 10
     0.2618f,   // 11


    // ----------------------------------------------------
    // Waist
    // ----------------------------------------------------

     2.6180f,   // 12


    // ----------------------------------------------------
    // Left arm
    // ----------------------------------------------------

     2.6704f,   // 13
     2.2515f,   // 14
     2.6180f,   // 15
     2.0944f,   // 16
     1.97222f,  // 17


    // ----------------------------------------------------
    // Right arm
    // ----------------------------------------------------

     2.6704f,   // 18
     1.5882f,   // 19
     2.6180f,   // 20
     2.0944f,   // 21
     1.97222f   // 22
};


// No permitiremos llegar exactamente al limite mecanico
static constexpr float JOINT_LIMIT_MARGIN = 0.02f;


// ========================================================
// Ganancias por defecto
// ========================================================

const std::array<float, G1_NUM_JOINTS> default_kp_ = {

60, 60, 60, 100, 40, 40,

    60, 60, 60, 100, 40, 40,

    60,

    40, 40, 40, 40, 40,

    40, 40, 40, 40, 40
};


const std::array<float, G1_NUM_JOINTS> default_kd_ = {

1.5, 1.5, 1.5, 2.0, 1.0, 1.0,

    1.5, 1.5, 1.5, 2.0, 1.0, 1.0,

    1.5,

    1.0, 1.0, 1.0, 1.0, 1.0,

    1.0, 1.0, 1.0, 1.0, 1.0
};

void arm_sdk_callback(
    const unitree_hg::msg::LowCmd::SharedPtr msg)
{
    if (!simulation_)
    {
        return;
    }


    const float weight =
        msg->motor_cmd[ARM_WEIGHT_MOTOR].q;


    // Por ahora solo procesamos control adquirido.
    // release_joints() lo implementaremos después.
    if (weight <= 0.001f)
    {
        if (arm_sdk_initialized_){
            send_udp("R ALL");
            arm_sdk_initialized_ = false;
            RCLCPP_INFO(this->get_logger(),"ARM SDK -> control liberado");
        }
        return;
    }


    // ====================================================
    // Primera recepción activa:
    // enviamos los 10 joints para sincronizar g1_ctrl
    // ====================================================

    if (!arm_sdk_initialized_)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "ARM SDK -> control activo"
            );


        for (int i = 0; i < G1_NUM_ARM_JOINTS; i++)
        {
            const int motor =
                arm_motor_id_[i];

            const float position =
                msg->motor_cmd[motor].q;


            std::ostringstream command;

            command
                << "J "
                << arm_joint_names_[i]
                << " "
                << position;


            send_udp(
                command.str()
                );


            last_arm_position_[i] =
                position;
        }


        arm_sdk_initialized_ = true;

        return;
    }


    // ====================================================
    // Después solo enviamos joints que cambiaron
    // ====================================================

    for (int i = 0; i < G1_NUM_ARM_JOINTS; i++)
    {
        const int motor =
            arm_motor_id_[i];

        const float position =
            msg->motor_cmd[motor].q;


        if (
            std::fabs(
                position -
                last_arm_position_[i]
                ) < arm_change_epsilon_
            )
        {
            continue;
        }


        std::ostringstream command;

        command
            << "J "
            << arm_joint_names_[i]
            << " "
            << position;


        send_udp(
            command.str()
            );


        last_arm_position_[i] =
            position;


        RCLCPP_INFO(
            this->get_logger(),
            "ARM -> %s = %.3f",
            arm_joint_names_[i].c_str(),
            position
            );
    }
}

// ========================================================
// UDP -> g1_ctrl
// ========================================================

bool send_udp(const std::string &command)
{
    if (!simulation_ || udp_socket_ < 0)
    {
        return false;
    }


    const ssize_t sent =
        ::sendto(
            udp_socket_,
            command.c_str(),
            command.size(),
            0,
            reinterpret_cast<sockaddr *>(
                &udp_address_
                ),
            sizeof(udp_address_)
            );


    if (sent < 0)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Error enviando UDP"
            );

        return false;
    }


    RCLCPP_INFO(
        this->get_logger(),
        "UDP -> %s",
        command.c_str()
        );


    return true;
}

// ========================================================
// High-level Sport API
// Solo utilizado en SIM
// ========================================================

void sport_request_callback(
    const unitree_api::msg::Request::SharedPtr msg)
{
    if (!simulation_)
    {
        return;
    }


    const int64_t api_id =
        msg->header.identity.api_id;


    nlohmann::json parameter;


    try
    {
        if (msg->parameter.empty())
        {
            parameter =
                nlohmann::json::object();
        }
        else
        {
            parameter =
                nlohmann::json::parse(
                    msg->parameter
                    );
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "JSON invalido: %s",
            e.what()
            );

        return;
    }


    // ====================================================
    // 7101 -> SetFsmId
    // ====================================================

    if (api_id == 7101)
    {
        const int fsm_id =
            parameter.value(
                "data",
                -1
                );


        // StandUp()
        if (fsm_id == 4)
        {
            // El high-level toma control.
            // Este core deja de publicar LowCmd propio.
            low_level_active_ = false;
            moving_ = false;


            send_udp(
                "1 0 0 0"
                );


            RCLCPP_INFO(
                this->get_logger(),
                "SPORT -> STAND"
                );
        }
        else
        {
            RCLCPP_WARN(
                this->get_logger(),
                "FSM %d aun no implementado en SIM",
                fsm_id
                );
        }


        return;
    }


    // ====================================================
    // 7105 -> SetVelocity
    // ====================================================

    if (api_id == 7105)
    {
        if (
            !parameter.contains("velocity") ||
            !parameter["velocity"].is_array() ||
            parameter["velocity"].size() < 3
            )
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Comando velocity invalido"
                );

            return;
        }


        const float vx =
            parameter["velocity"][0]
                .get<float>();

        const float vy =
            parameter["velocity"][1]
                .get<float>();

        const float wz =
            parameter["velocity"][2]
                .get<float>();


        low_level_active_ = false;
        moving_ = false;


        std::ostringstream command;

        command
            << "2 "
            << vx << " "
            << vy << " "
            << wz;


        send_udp(
            command.str()
            );


        RCLCPP_INFO(
            this->get_logger(),
            "SPORT -> VELOCITY "
            "vx=%.3f vy=%.3f wz=%.3f",
            vx,
            vy,
            wz
            );


        return;
    }


    RCLCPP_WARN(
        this->get_logger(),
        "Sport API no implementada en SIM: %ld",
        static_cast<long>(api_id)
        );
}

// ========================================================
// LowState
// ========================================================

void lowstate_callback(
    const unitree_hg::msg::LowState::SharedPtr msg)
{
    mode_machine_ = msg->mode_machine;


    for (int i = 0; i < G1_NUM_JOINTS; i++)
    {
        const int motor = motor_id_[i];

        current_position_[i] =
            msg->motor_state[motor].q;

        current_velocity_[i] =
            msg->motor_state[motor].dq;

        current_torque_[i] =
            msg->motor_state[motor].tau_est;
    }


    // ====================================================
    // Publicar estado articular normalizado
    // ====================================================

    sensor_msgs::msg::JointState joint_state;

    joint_state.header.stamp =
        this->now();

    joint_state.name.assign(
        joint_names_.begin(),
        joint_names_.end()
    );

    joint_state.position.resize(G1_NUM_JOINTS);
    joint_state.velocity.resize(G1_NUM_JOINTS);
    joint_state.effort.resize(G1_NUM_JOINTS);

    for (int i = 0; i < G1_NUM_JOINTS; i++)
    {
        joint_state.position[i] =
            current_position_[i];

        joint_state.velocity[i] =
            current_velocity_[i];

        joint_state.effort[i] =
            current_torque_[i];
    }

    joint_state_pub_->publish(joint_state);


    // ====================================================
    // Publicar IMU normalizada
    //
    // Unitree quaternion:
    // [w, x, y, z]
    //
    // sensor_msgs/Imu:
    // x, y, z, w
    // ====================================================

    sensor_msgs::msg::Imu imu;

    imu.header.stamp =
        joint_state.header.stamp;

    imu.orientation.w =
        msg->imu_state.quaternion[0];

    imu.orientation.x =
        msg->imu_state.quaternion[1];

    imu.orientation.y =
        msg->imu_state.quaternion[2];

    imu.orientation.z =
        msg->imu_state.quaternion[3];


    imu.angular_velocity.x =
        msg->imu_state.gyroscope[0];

    imu.angular_velocity.y =
        msg->imu_state.gyroscope[1];

    imu.angular_velocity.z =
        msg->imu_state.gyroscope[2];


    imu.linear_acceleration.x =
        msg->imu_state.accelerometer[0];

    imu.linear_acceleration.y =
        msg->imu_state.accelerometer[1];

    imu.linear_acceleration.z =
        msg->imu_state.accelerometer[2];


    imu_pub_->publish(imu);


    if (!state_received_)
    {
        command_position_ = current_position_;
        start_position_ = current_position_;
        target_position_ = current_position_;

        state_received_ = true;

        RCLCPP_INFO(
            this->get_logger(),
            "LowState recibido"
            );
    }
}


// ========================================================
// Permisos de articulaciones en robot REAL
// ========================================================



// ========================================================
// Buscar articulación
// ========================================================

int find_joint(const std::string &name)
{
    for (int i = 0; i < G1_NUM_JOINTS; i++)
    {
        if (joint_names_[i] == name)
        {
            return i;
        }
    }

    return -1;
}


// ========================================================
// Validar una articulacion solicitada
// ========================================================

bool validate_joint_request(
    const g1_core::msg::JointCommand::SharedPtr &msg,
    size_t n,
    int index
)
{
    if (index < 0)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Joint desconocido: %s",
            msg->name[n].c_str()
        );

        return false;
    }


    if (!std::isfinite(msg->position[n]))
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Posicion invalida para %s",
            msg->name[n].c_str()
        );

        return false;
    }


    const float requested_position =
        static_cast<float>(
            msg->position[n]
        );


    // ----------------------------------------------------
    // Limites articulares
    // ----------------------------------------------------

    const float safe_min =
        joint_min_[index] + JOINT_LIMIT_MARGIN;

    const float safe_max =
        joint_max_[index] - JOINT_LIMIT_MARGIN;


    if (
        requested_position < safe_min ||
        requested_position > safe_max
    )
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "%s rechazado: %.3f rad fuera de [%.3f, %.3f]",
            msg->name[n].c_str(),
            requested_position,
            safe_min,
            safe_max
        );

        return false;
    }


    // ----------------------------------------------------
    // Calcular desplazamiento solicitado
    // ----------------------------------------------------

    const float delta =
        std::fabs(
            requested_position -
            current_position_[index]
            );


    const bool trajectory_command =
        msg->duration > 0.0;


    // ----------------------------------------------------
    // Seguridad de trayectoria
    // Aplica tanto en SIM como en REAL
    // ----------------------------------------------------

    if (trajectory_command)
    {
        const double duration =
            msg->duration;




    }


    // ----------------------------------------------------
    // Protecciones exclusivas del robot REAL
    // ----------------------------------------------------

    if (!simulation_)
    {


        // set_joint(): impedir saltos directos grandes.
        // move_joint(): usa las protecciones de trayectoria anteriores.




    }


    return true;
}


// ========================================================
// Comando desde Python
// ========================================================

void joint_command_callback(
    const g1_core::msg::JointCommand::SharedPtr msg)
{
    if (!state_received_)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Todavia no se recibio /lowstate"
            );

        return;
    }

    const size_t lowcmd_publishers =
        this->count_publishers("/lowcmd");


    if (simulation_ && lowcmd_publishers > 1)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Otro publisher detectado en /lowcmd. "
            "Comando LOW LEVEL rechazado."
            );

        low_level_active_ = false;

        return;
    }

    const size_t count = msg->name.size();

    if (!std::isfinite(msg->duration) || msg->duration < 0.0)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "duration invalido"
            );

        return;
    }



    if (msg->position.size() != count)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "position debe tener el mismo tamano que name"
            );

        return;
    }


    // Los parámetros opcionales pueden estar:
    // - vacíos
    // - con un elemento por articulación

    if (
        (!msg->velocity.empty() && msg->velocity.size() != count) ||
        (!msg->torque.empty()   && msg->torque.size()   != count) ||
        (!msg->kp.empty()       && msg->kp.size()       != count) ||
        (!msg->kd.empty()       && msg->kd.size()       != count)
        )
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "velocity, torque, kp y kd deben estar vacios "
            "o tener el mismo tamano que name"
            );

        return;
    }


    // Si es el primer comando low-level, partimos
    // desde la posición REAL actual del robot.
    std::array<float, G1_NUM_JOINTS> new_target;

    if (!low_level_active_)
    {
        new_target = current_position_;

        command_position_ = current_position_;
        start_position_ = current_position_;

        command_velocity_.fill(0.0f);
        command_torque_.fill(0.0f);

        command_kp_ = default_kp_;
        command_kd_ = default_kd_;
    }
    else
    {
        new_target = command_position_;
    }


    // ====================================================
    // Primera pasada: validar TODO el comando
    //
    // Si un solo joint falla, se rechaza el mensaje
    // completo antes de modificar el estado de control.
    // ====================================================

    for (size_t n = 0; n < count; n++)
    {
        const int index =
            find_joint(msg->name[n]);

        if (!validate_joint_request(msg, n, index))
        {
            return;
        }
    }


    // Todos los joints fueron validados.

    bool valid_command = false;


    // ====================================================
    // Segunda pasada: aplicar el comando
    //
    // En este punto TODO el JointCommand ya fue validado.
    // ====================================================

    for (size_t n = 0; n < count; n++)
    {
        const int index =
            find_joint(msg->name[n]);


        const float requested_position =
            static_cast<float>(
                msg->position[n]
            );


        new_target[index] =
            requested_position;


        // -----------------------------------------------
        // Velocity
        // -----------------------------------------------

        if (
            !msg->velocity.empty() &&
            std::isfinite(msg->velocity[n])
        )
        {
            command_velocity_[index] =
                static_cast<float>(
                    msg->velocity[n]
                );
        }
        else
        {
            command_velocity_[index] = 0.0f;
        }


        // -----------------------------------------------
        // Torque
        // -----------------------------------------------

        if (
            !msg->torque.empty() &&
            std::isfinite(msg->torque[n])
        )
        {
            command_torque_[index] =
                static_cast<float>(
                    msg->torque[n]
                );
        }
        else
        {
            command_torque_[index] = 0.0f;
        }


        // -----------------------------------------------
        // KP
        // -----------------------------------------------

        if (
            !msg->kp.empty() &&
            std::isfinite(msg->kp[n])
        )
        {
            command_kp_[index] =
                static_cast<float>(
                    msg->kp[n]
                );
        }
        else
        {
            command_kp_[index] =
                default_kp_[index];
        }


        // -----------------------------------------------
        // KD
        // -----------------------------------------------

        if (
            !msg->kd.empty() &&
            std::isfinite(msg->kd[n])
        )
        {
            command_kd_[index] =
                static_cast<float>(
                    msg->kd[n]
                );
        }
        else
        {
            command_kd_[index] =
                default_kd_[index];
        }


        RCLCPP_INFO(
            this->get_logger(),
            "%s -> q=%.3f kp=%.2f kd=%.2f",
            msg->name[n].c_str(),
            new_target[index],
            command_kp_[index],
            command_kd_[index]
        );


        valid_command = true;
    }


    if (!valid_command)
    {
        return;
    }


    start_position_ = command_position_;
    target_position_ = new_target;

    if (msg->duration > 0.0)
    {
        // move_joint / move_joints:
        // trayectoria interpolada durante el tiempo solicitado.
        active_transition_duration_ =
            msg->duration;

        trajectory_start_time_ =
            std::chrono::steady_clock::now();

        elapsed_time_ = 0.0;
        moving_ = true;
    }
    else
    {
        // set_joint / set_joints:
        // referencia directa, sin trayectoria oculta.
        command_position_ =
            target_position_;

        elapsed_time_ = 0.0;
        moving_ = false;
    }

    low_level_active_ = true;

    RCLCPP_INFO(
        this->get_logger(),
        "LOW LEVEL activo"
        );
}


// ========================================================
// Loop de control
// ========================================================

void control_loop()
{
    if (!state_received_)
    {
        return;
    }


    // Hasta recibir el primer comando Python,
    // NO publicamos /lowcmd.
    if (!low_level_active_)
    {
        return;
    }

    const size_t lowcmd_publishers =
        this->count_publishers("/lowcmd");


    if (simulation_ && lowcmd_publishers > 1)
    {
        low_level_active_ = false;
        moving_ = false;


        RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Otro publisher detectado en /lowcmd. "
            "LOW LEVEL desactivado."
            );


        return;
    }

    // ----------------------------------------------------
    // Interpolación suave de posición
    // ----------------------------------------------------

    if (moving_)
    {
        elapsed_time_ =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now()
                -
                trajectory_start_time_
            ).count();


        double ratio =
            elapsed_time_
            /
            active_transition_duration_;


        if (ratio > 1.0)
        {
            ratio = 1.0;
        }


        const double smooth =
            3.0 * ratio * ratio
            -
            2.0 * ratio * ratio * ratio;


        // Derivada temporal del smoothstep.
        // Esto genera dq deseada coherente con q deseada.
        const double smooth_velocity =
            (
                6.0 * ratio
                -
                6.0 * ratio * ratio
                )
            /
            active_transition_duration_;


        for (int i = 0; i < G1_NUM_JOINTS; i++)
        {
            const double delta =
                target_position_[i]
                -
                start_position_[i];


            command_position_[i] =
                static_cast<float>(
                    start_position_[i]
                    +
                    delta * smooth
                    );


            command_velocity_[i] =
                static_cast<float>(
                    delta * smooth_velocity
                    );
        }


        if (ratio >= 1.0)
        {
            command_position_ =
                target_position_;

            command_velocity_.fill(0.0f);

            moving_ = false;

}
    }

    // ----------------------------------------------------
    // Watchdog
    // ----------------------------------------------------


    // ----------------------------------------------------
    // Crear LowCmd
    // ----------------------------------------------------

    unitree_hg::msg::LowCmd cmd;


    cmd.mode_pr = 0;

    cmd.mode_machine =
        mode_machine_;


    // Solo configuramos los 23 motores usados
    // por nuestro G1 23DOF.
    for (int i = 0; i < G1_NUM_JOINTS; i++)
    {
        const int motor =
            motor_id_[i];

        // En REAL podemos deshabilitar grupos completos.

        // Motor habilitado.
        cmd.motor_cmd[motor].mode = 1;

        cmd.motor_cmd[motor].q =
            command_position_[i];

        cmd.motor_cmd[motor].dq =
            command_velocity_[i];

        cmd.motor_cmd[motor].tau =
            command_torque_[i];

        cmd.motor_cmd[motor].kp =
            command_kp_[i];

        cmd.motor_cmd[motor].kd =
            command_kd_[i];
    }


    // CRC requerido por Unitree
    get_crc(cmd);


    lowcmd_pub_->publish(cmd);
}


// ========================================================
// ROS
// ========================================================

rclcpp::Subscription<
    unitree_hg::msg::LowState
    >::SharedPtr lowstate_sub_;


rclcpp::Subscription<
    g1_core::msg::JointCommand
    >::SharedPtr joint_command_sub_;


rclcpp::Publisher<
    unitree_hg::msg::LowCmd
    >::SharedPtr lowcmd_pub_;


rclcpp::TimerBase::SharedPtr
    control_timer_;


rclcpp::Subscription<
    unitree_api::msg::Request
    >::SharedPtr sport_request_sub_;

rclcpp::Subscription<
    unitree_hg::msg::LowCmd
    >::SharedPtr arm_sdk_sub_;


// ========================================================
// Estado de control
// ========================================================

rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_pub_;

rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr
    imu_pub_;


std::array<float, G1_NUM_JOINTS>
    current_velocity_{};

std::array<float, G1_NUM_JOINTS>
    current_torque_{};


std::array<float, G1_NUM_JOINTS>
    current_position_{};

std::array<float, G1_NUM_JOINTS>
    command_position_{};

std::array<float, G1_NUM_JOINTS>
    start_position_{};

std::array<float, G1_NUM_JOINTS>
    target_position_{};


std::array<float, G1_NUM_JOINTS>
    command_velocity_{};

std::array<float, G1_NUM_JOINTS>
    command_torque_{};

std::array<float, G1_NUM_JOINTS>
    command_kp_{};

std::array<float, G1_NUM_JOINTS>
    command_kd_{};

std::array<float, G1_NUM_ARM_JOINTS>
    last_arm_position_{};

bool arm_sdk_initialized_ = false;

const float arm_change_epsilon_ = 0.0005f;


bool state_received_ = false;
bool low_level_active_ = false;
bool moving_ = false;

// ========================================================
// SIMULATION
// ========================================================

bool simulation_ = false;

int udp_socket_ = -1;

sockaddr_in udp_address_{};



uint8_t mode_machine_ = 0;


double elapsed_time_ = 0.0;

std::chrono::steady_clock::time_point
    trajectory_start_time_;


double active_transition_duration_ = 2.0;

// ========================================================
// Seguridad para robot REAL
// ========================================================



};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<G1Core>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}