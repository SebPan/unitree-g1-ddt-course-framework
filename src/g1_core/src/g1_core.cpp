 #include <algorithm>
#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdio>
#include <cstring>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "std_srvs/srv/trigger.hpp"

#include "motor_crc_hg.h"


constexpr int G1_NUM_JOINTS = 23;
constexpr double CONTROL_DT = 0.002;   // 500 Hz


class G1Core : public rclcpp::Node
{
public:

    G1Core()
        : Node("g1_core")
    {
        // ====================================================
        // Unitree LowState
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
        // Comando ROS estándar
        // ====================================================

        joint_command_sub_ =
            this->create_subscription<sensor_msgs::msg::JointState>(
                "/g1/joint_command",
                10,
                std::bind(
                    &G1Core::joint_command_callback,
                    this,
                    std::placeholders::_1
                )
            );

        // ====================================================
        // Comando de velocidad high-level
        // ====================================================

        cmd_vel_sub_ =
            this->create_subscription<geometry_msgs::msg::Twist>(
                "/g1/cmd_vel",
                10,
                std::bind(
                    &G1Core::cmd_vel_callback,
                    this,
                    std::placeholders::_1
                )
            );


        // ====================================================
        // Joint override high-level
        // ====================================================

        joint_override_sub_ =
            this->create_subscription<sensor_msgs::msg::JointState>(
                "/g1/joint_override",
                10,
                std::bind(
                    &G1Core::joint_override_callback,
                    this,
                    std::placeholders::_1
                    )
                );

        // ====================================================
        // Servicios high-level
        // ====================================================

        stand_srv_ =
            this->create_service<std_srvs::srv::Trigger>(
                "/g1/stand",
                std::bind(
                    &G1Core::stand_callback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2
                )
            );

        stop_srv_ =
            this->create_service<std_srvs::srv::Trigger>(
                "/g1/stop",
                std::bind(
                    &G1Core::stop_callback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2
                )
            );


        joint_override_release_srv_ =
            this->create_service<std_srvs::srv::Trigger>(
                "/g1/joint_override_release",
                std::bind(
                    &G1Core::joint_override_release_callback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2
                    )
                );


        // ====================================================
        // Sensores ROS estándar
        // ====================================================

        joint_pub_ =
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
        // Unitree LowCmd
        // ====================================================

        lowcmd_pub_ =
            this->create_publisher<unitree_hg::msg::LowCmd>(
                "/lowcmd",
                10
            );


        // ====================================================
        // Control a 500 Hz
        // ====================================================

        control_timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(2),
                std::bind(
                    &G1Core::control_loop,
                    this
                )
            );


        RCLCPP_INFO(
            this->get_logger(),
            "G1 Core iniciado."
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Esperando /lowstate..."
        );
    }


private:

    // ========================================================
    // Nombres ROS de las 23 articulaciones
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


    // ========================================================
    // Ganancias PD
    // ========================================================

    const std::array<float, G1_NUM_JOINTS> kp_ = {

        60, 60, 60, 100, 40, 40,

        60, 60, 60, 100, 40, 40,

        60,

        40, 40, 40, 40, 40,

        40, 40, 40, 40, 40
    };


    const std::array<float, G1_NUM_JOINTS> kd_ = {

        1, 1, 1, 2, 1, 1,

        1, 1, 1, 2, 1, 1,

        1,

        1, 1, 1, 1, 1,

        1, 1, 1, 1, 1
    };


    // ========================================================
    // Enviar comando al controlador de simulacion
    // ========================================================

    void send_sim_command(
        int mode,
        float vx,
        float vy,
        float wz)
    {
        int sock =
            socket(
                AF_INET,
                SOCK_DGRAM,
                0
            );

        if (sock < 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "No se pudo crear socket UDP."
            );

            return;
        }


        sockaddr_in addr{};

        addr.sin_family = AF_INET;

        addr.sin_port =
            htons(15000);

        inet_pton(
            AF_INET,
            "127.0.0.1",
            &addr.sin_addr
        );


        char buffer[128];

        std::snprintf(
            buffer,
            sizeof(buffer),
            "%d %.4f %.4f %.4f",
            mode,
            vx,
            vy,
            wz
        );


        sendto(
            sock,
            buffer,
            std::strlen(buffer),
            0,
            reinterpret_cast<sockaddr*>(&addr),
            sizeof(addr)
        );


        close(sock);
    }

    void send_sim_text_command(
        const std::string& command)
    {
        int sock =
            socket(
                AF_INET,
                SOCK_DGRAM,
                0
                );

        if (sock < 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "No se pudo crear socket UDP."
                );

            return;
        }


        sockaddr_in addr{};

        addr.sin_family = AF_INET;
        addr.sin_port = htons(15000);

        inet_pton(
            AF_INET,
            "127.0.0.1",
            &addr.sin_addr
            );


        sendto(
            sock,
            command.c_str(),
            command.size(),
            0,
            reinterpret_cast<sockaddr*>(&addr),
            sizeof(addr)
            );


        close(sock);
    }


    // ========================================================
    // LowState callback
    // ========================================================

    void lowstate_callback(
        const unitree_hg::msg::LowState::SharedPtr msg)
    {
        mode_machine_ = msg->mode_machine;


        for (int i = 0; i < G1_NUM_JOINTS; i++)
        {
            current_position_[i] =
                msg->motor_state[i].q;
        }


        if (!state_received_)
        {
            for (int i = 0; i < G1_NUM_JOINTS; i++)
            {
                command_position_[i] =
                    current_position_[i];

                start_position_[i] =
                    current_position_[i];

                target_position_[i] =
                    current_position_[i];
            }

            state_received_ = true;

            RCLCPP_INFO(
                this->get_logger(),
                "LowState recibido."
            );
        }


        publish_joint_states(msg);

        publish_imu(msg);
    }


    // ========================================================
    // Joint command
    // ========================================================

    void joint_command_callback(
        const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!state_received_)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Comando ignorado: no hay LowState."
            );

            return;
        }


        if (msg->name.size() != msg->position.size())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Joint command invalido."
            );

            return;
        }


        // Empezamos desde la posición actual
        for (int i = 0; i < G1_NUM_JOINTS; i++)
        {
            start_position_[i] =
                current_position_[i];

            target_position_[i] =
                current_position_[i];
        }


        // Cambiamos solo las articulaciones solicitadas
        for (size_t j = 0; j < msg->name.size(); j++)
        {
            auto it =
                std::find(
                    joint_names_.begin(),
                    joint_names_.end(),
                    msg->name[j]
                );


            if (it == joint_names_.end())
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Articulacion desconocida: %s",
                    msg->name[j].c_str()
                );

                continue;
            }


            int index =
                static_cast<int>(
                    std::distance(
                        joint_names_.begin(),
                        it
                    )
                    );


            target_position_[index] =
                static_cast<float>(
                    msg->position[j]
                    );


            RCLCPP_INFO(
                this->get_logger(),
                "%s -> %.3f rad",
                msg->name[j].c_str(),
                msg->position[j]
            );
        }


        elapsed_time_ = 0.0;

        moving_ = true;

        low_level_active_ = true;
    }


    // ========================================================
    // High-level velocity command
    // ========================================================

    void cmd_vel_callback(
        const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // g1_ctrl toma el control de LowCmd
        low_level_active_ = false;
        moving_ = false;

        const float vx =
            static_cast<float>(msg->linear.x);

        const float vy =
            static_cast<float>(msg->linear.y);

        const float wz =
            static_cast<float>(msg->angular.z);


        high_level_vx_ = vx;
        high_level_vy_ = vy;
        high_level_wz_ = wz;


        send_sim_command(
            2,
            vx,
            vy,
            wz
        );

        RCLCPP_INFO(
            this->get_logger(),
            "HIGH LEVEL -> vx=%.3f vy=%.3f wz=%.3f",
            vx,
            vy,
            wz
        );
    }


    void joint_override_callback(
        const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->name.size() != msg->position.size())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Joint override invalido: name y position "
                "deben tener la misma longitud."
                );

            return;
        }


        if (msg->name.empty())
        {
            return;
        }


        // El control directo low-level debe quedar desactivado.
        // g1_ctrl conserva el LowCmd.
        low_level_active_ = false;
        moving_ = false;


        for (size_t i = 0; i < msg->name.size(); i++)
        {
            const std::string& joint_name =
                msg->name[i];

            const float position =
                static_cast<float>(
                    msg->position[i]
                    );


            char command[256];

            std::snprintf(
                command,
                sizeof(command),
                "J %s %.4f",
                joint_name.c_str(),
                position
                );


            send_sim_text_command(
                command
                );


            RCLCPP_INFO(
                this->get_logger(),
                "JOINT OVERRIDE -> %s = %.3f",
                joint_name.c_str(),
                position
                );
        }
    }

    void joint_override_release_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;


        send_sim_text_command(
            "R ALL"
            );


        response->success = true;

        response->message =
            "Joint overrides liberados.";


        RCLCPP_INFO(
            this->get_logger(),
            "JOINT OVERRIDE -> RELEASE ALL"
            );
    }

    // ========================================================
    // Stand
    // ========================================================

    void stand_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        // g1_ctrl toma el control de LowCmd
        low_level_active_ = false;
        moving_ = false;

        high_level_vx_ = 0.0f;
        high_level_vy_ = 0.0f;
        high_level_wz_ = 0.0f;


        // Al volver a Stand limpiamos cualquier override
        // anterior para empezar desde un estado conocido.
        send_sim_text_command(
            "R ALL"
            );


        // FixStand
        send_sim_command(
            1,
            0.0f,
            0.0f,
            0.0f
            );


        response->success = true;
        response->message = "Comando STAND enviado.";

        RCLCPP_INFO(
            this->get_logger(),
            "HIGH LEVEL -> STAND"
        );
    }


    // ========================================================
    // Stop
    // ========================================================

    void stop_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        low_level_active_ = false;
        moving_ = false;

        high_level_vx_ = 0.0f;
        high_level_vy_ = 0.0f;
        high_level_wz_ = 0.0f;


        // Seguimos en Velocity, pero con velocidad cero.
        // El estado actual del brazo NO cambia.
        send_sim_command(
            2,
            0.0f,
            0.0f,
            0.0f
        );

        response->success = true;
        response->message = "Robot detenido.";

        RCLCPP_INFO(
            this->get_logger(),
            "HIGH LEVEL -> STOP"
        );
    }


    // ========================================================
    // Control 500 Hz
    // ========================================================

    void control_loop()
    {
        if (!state_received_)
        {
            return;
        }


        // No publicamos LowCmd hasta recibir
        // el primer comando low-level.
        if (!low_level_active_)
        {
            return;
        }


        if (moving_)
        {
            elapsed_time_ += CONTROL_DT;


            double ratio =
                elapsed_time_
                /
                transition_duration_;


            if (ratio > 1.0)
            {
                ratio = 1.0;
            }


            // Smoothstep
            double smooth =
                3.0 * ratio * ratio
                -
                2.0 * ratio * ratio * ratio;


            for (int i = 0; i < G1_NUM_JOINTS; i++)
            {
                command_position_[i] =
                    static_cast<float>(
                        start_position_[i]
                        +
                        (
                            target_position_[i]
                            -
                            start_position_[i]
                            )
                        *
                        smooth
                        );
            }


            if (ratio >= 1.0)
            {
                moving_ = false;

                command_position_ =
                    target_position_;

                RCLCPP_INFO(
                    this->get_logger(),
                    "Movimiento terminado. HOLD."
                );
            }
        }


        unitree_hg::msg::LowCmd cmd;


        cmd.mode_pr = 0;

        cmd.mode_machine =
            mode_machine_;


        for (int i = 0; i < G1_NUM_JOINTS; i++)
        {
            cmd.motor_cmd[i].mode = 1;

            cmd.motor_cmd[i].q =
                command_position_[i];

            cmd.motor_cmd[i].dq = 0.0;

            cmd.motor_cmd[i].kp =
                kp_[i];

            cmd.motor_cmd[i].kd =
                kd_[i];

            cmd.motor_cmd[i].tau = 0.0;
        }


        get_crc(cmd);

        lowcmd_pub_->publish(cmd);
    }


    // ========================================================
    // Publicar JointState
    // ========================================================

    void publish_joint_states(
        const unitree_hg::msg::LowState::SharedPtr msg)
    {
        sensor_msgs::msg::JointState joints;

        joints.header.stamp =
            this->now();


        joints.name.resize(G1_NUM_JOINTS);

        joints.position.resize(G1_NUM_JOINTS);

        joints.velocity.resize(G1_NUM_JOINTS);

        joints.effort.resize(G1_NUM_JOINTS);


        for (int i = 0; i < G1_NUM_JOINTS; i++)
        {
            joints.name[i] =
                joint_names_[i];

            joints.position[i] =
                msg->motor_state[i].q;

            joints.velocity[i] =
                msg->motor_state[i].dq;

            joints.effort[i] =
                msg->motor_state[i].tau_est;
        }


        joint_pub_->publish(joints);
    }


    // ========================================================
    // Publicar IMU
    // ========================================================

    void publish_imu(
        const unitree_hg::msg::LowState::SharedPtr msg)
    {
        sensor_msgs::msg::Imu imu;


        imu.header.stamp =
            this->now();

        imu.header.frame_id =
            "imu_link";


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
    }


    // ========================================================
    // ROS
    // ========================================================

    rclcpp::Subscription<
        unitree_hg::msg::LowState
    >::SharedPtr lowstate_sub_;


    rclcpp::Subscription<
        sensor_msgs::msg::JointState
    >::SharedPtr joint_command_sub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
        cmd_vel_sub_;


    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        joint_override_sub_;

    rclcpp::Publisher<
        sensor_msgs::msg::JointState
    >::SharedPtr joint_pub_;


    rclcpp::Publisher<
        sensor_msgs::msg::Imu
    >::SharedPtr imu_pub_;


    rclcpp::Publisher<
        unitree_hg::msg::LowCmd
    >::SharedPtr lowcmd_pub_;


    rclcpp::TimerBase::SharedPtr
        control_timer_;


    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
        stand_srv_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
        stop_srv_;


    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
        joint_override_release_srv_;


    // ========================================================
    // Estado interno
    // ========================================================

    std::array<float, G1_NUM_JOINTS>
        current_position_{};

    std::array<float, G1_NUM_JOINTS>
        command_position_{};

    std::array<float, G1_NUM_JOINTS>
        start_position_{};

    std::array<float, G1_NUM_JOINTS>
        target_position_{};


    bool state_received_ = false;

    bool low_level_active_ = false;

    bool moving_ = false;


    uint8_t mode_machine_ = 0;


    double elapsed_time_ = 0.0;

    double transition_duration_ = 2.0;


    // ========================================================
    // Estado high-level
    // ========================================================

    float high_level_vx_ = 0.0f;
    float high_level_vy_ = 0.0f;
    float high_level_wz_ = 0.0f;

};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<G1Core>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}