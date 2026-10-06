#include "FSM/State_RLBase.h"

#include "SimCommand.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"

#include <algorithm>
#include <chrono>
#include <unordered_map>
#include <array>

namespace isaaclab
{

// ============================================================
// Keyboard velocity commands example
// ============================================================

REGISTER_OBSERVATION(keyboard_velocity_commands)
{
    return {0.0f, 0.0f, 0.0f};

    /*
    if (!FSMState::keyboard) {
        return {0.0f, 0.0f, 0.0f};
    }

    std::string key = FSMState::keyboard->key();
    static auto cfg = env->cfg["commands"]["base_velocity"]["ranges"];

    static std::string last_logged_key = "";

    if(key != last_logged_key && !key.empty())
    {
        spdlog::info(
            "Key detected: '{}' -> Command will be generated",
            key
        );

        last_logged_key = key;
    }

    static std::unordered_map<
        std::string,
        std::vector<float>
    > key_commands =
    {
        {"w", {0.4f, 0.0f, 0.0f}},
        {"s", {-0.3f, 0.0f, 0.0f}},
        {"a", {0.0f, 0.05f, 0.0f}},
        {"d", {0.0f, -0.05f, 0.0f}},
        {"q", {0.0f, 0.0f, 0.05f}},
        {"e", {0.0f, 0.0f, -0.05f}}
    };

    static std::vector<float> current_cmd =
        {0.0f, 0.0f, 0.0f};

    static std::vector<float> target_cmd =
        {0.0f, 0.0f, 0.0f};

    const float smoothing = 0.15f;

    if(key_commands.find(key) != key_commands.end())
    {
        target_cmd = key_commands[key];

        spdlog::info(
            "Command: [{:.3f}, {:.3f}, {:.3f}]",
            target_cmd[0],
            target_cmd[1],
            target_cmd[2]
        );
    }
    else
    {
        target_cmd = {0.0f, 0.0f, 0.0f};
    }

    for(size_t i = 0; i < 3; i++)
    {
        current_cmd[i] +=
            (target_cmd[i] - current_cmd[i])
            * smoothing;

        if(std::abs(current_cmd[i]) < 0.01f)
        {
            current_cmd[i] = 0.0f;
        }
    }

    return current_cmd;
    */
}

}


// ============================================================
// Constructor
// ============================================================

State_RLBase::State_RLBase(
    int state_mode,
    std::string state_string
)
: FSMState(state_mode, state_string)
{
    auto cfg =
        param::config["FSM"][state_string];

    auto policy_dir =
        param::parser_policy_dir(
            cfg["policy_dir"].as<std::string>()
        );


    spdlog::info(
        "========================================"
    );

    spdlog::info(
        "Loading RL Policy from:"
    );

    spdlog::info(
        "  Policy Directory: {}",
        policy_dir.string()
    );


    auto deploy_yaml =
        policy_dir / "params" / "deploy.yaml";

    auto policy_onnx =
        policy_dir / "exported" / "policy.onnx";


    spdlog::info(
        "  Deploy Config: {}",
        deploy_yaml.string()
    );

    spdlog::info(
        "  Policy ONNX: {}",
        policy_onnx.string()
    );


    // --------------------------------------------------------
    // Comprobar archivos
    // --------------------------------------------------------

    if(!std::filesystem::exists(deploy_yaml))
    {
        spdlog::critical(
            "Deploy YAML not found: {}",
            deploy_yaml.string()
        );

        throw std::runtime_error(
            "Deploy YAML file missing!"
        );
    }


    if(!std::filesystem::exists(policy_onnx))
    {
        spdlog::critical(
            "Policy ONNX not found: {}",
            policy_onnx.string()
        );

        throw std::runtime_error(
            "Policy ONNX file missing!"
        );
    }


    auto onnx_size =
        std::filesystem::file_size(
            policy_onnx
        );


    spdlog::info(
        "  ONNX File Size: {} bytes ({:.2f} MB)",
        onnx_size,
        onnx_size / (1024.0 * 1024.0)
    );


    spdlog::info(
        "========================================"
    );


    // --------------------------------------------------------
    // Crear entorno RL
    // --------------------------------------------------------

    env =
        std::make_unique<
            isaaclab::ManagerBasedRLEnv
        >(
            YAML::LoadFile(
                deploy_yaml
            ),

            std::make_shared<
                unitree::BaseArticulation<
                    LowState_t::SharedPtr
                >
            >(
                FSMState::lowstate
            )
        );


    env->alg =
        std::make_unique<
            isaaclab::OrtRunner
        >(
            policy_onnx
        );


    spdlog::info(
        "Policy loaded successfully!"
    );


    // --------------------------------------------------------
    // Protección por mala orientación
    // --------------------------------------------------------

    this->registered_checks.emplace_back(
        std::make_pair(

            [&]()->bool
            {
                return isaaclab::mdp::bad_orientation(
                    env.get(),
                    1.0
                );
            },

            FSMStringMap.right.at(
                "Passive"
            )
        )
    );
}


// ============================================================
// Ejecución de la policy
// ============================================================

void State_RLBase::run()
{
    // ========================================================
    // 1. Salida normal de la policy
    // ========================================================

    auto action =
        env->action_manager->processed_actions();


    // ========================================================
    // 2. Traduccion interna:
    //
    // nombre logico del framework
    //          ↓
    // indice interno de ESTA policy de simulacion
    //
    // Estos indices nunca salen de este backend.
    //
    // Por ahora SOLO incluimos joints ya validados.
    // ========================================================

    static const std::unordered_map<
        std::string,
        size_t
        > policy_joint_map =
        {
            {"left_shoulder_pitch",   5},
            {"right_shoulder_pitch",  6},

            {"left_shoulder_roll",    9},
            {"right_shoulder_roll",  10},

            {"left_shoulder_yaw",    13},
            {"right_shoulder_yaw",   14},

            {"left_elbow",           17},
            {"right_elbow",          18},

            {"left_wrist_roll",      21},
            {"right_wrist_roll",     22}
        };


    // ========================================================
    // 3. Estado de mezcla por joint
    // ========================================================

    static std::array<float, 23>
        override_blend{};

    static std::array<float, 23>
        override_target{};


    static auto last_time =
        std::chrono::steady_clock::now();


    const auto now =
        std::chrono::steady_clock::now();


    float dt =
        std::chrono::duration<float>(
            now - last_time
            ).count();


    last_time = now;


    dt =
        std::clamp(
            dt,
            0.0f,
            0.05f
            );


    constexpr float blend_rate =
        2.0f;


    // Copia thread-safe de los overrides activos.
    const auto overrides =
        simcmd::joint_overrides();


    // ========================================================
    // 4. Aplicar cada joint soportado
    // ========================================================

    for(
        const auto& mapping :
        policy_joint_map
        )
    {
        const std::string& joint_name =
            mapping.first;

        const size_t action_index =
            mapping.second;


        if(action_index >= action.size())
        {
            continue;
        }


        auto override_it =
            overrides.find(
                joint_name
                );


        const bool active =
            override_it
            !=
            overrides.end();


        if(active)
        {
            override_target[action_index] =
                override_it->second;


            override_blend[action_index] +=
                blend_rate * dt;
        }
        else
        {
            override_blend[action_index] -=
                blend_rate * dt;
        }


        override_blend[action_index] =
            std::clamp(
                override_blend[action_index],
                0.0f,
                1.0f
                );


        // ----------------------------------------------------
        // Mezcla suave entre policy y override.
        //
        // 0.0 -> policy
        // 1.0 -> usuario
        // ----------------------------------------------------

        const float blend =
            override_blend[action_index];


        if(blend > 0.0f)
        {
            const float policy_position =
                action[action_index];


            action[action_index] =
                (1.0f - blend)
                    *
                    policy_position
                +
                blend
                    *
                    override_target[action_index];
        }
    }


    // ========================================================
    // 5. Un unico LowCmd final
    // ========================================================

    for(
        int i = 0;
        i < env->robot->data.joint_ids_map.size();
        i++
        )
    {
        lowcmd->msg_.motor_cmd()[
                        env->robot->data.joint_ids_map[i]
        ].q() =
            action[i];
    }
}