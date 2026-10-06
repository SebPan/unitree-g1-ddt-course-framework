#include "State_Mimic.h"

#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "unitree_articulation.h"

#include <spdlog/spdlog.h>

static Eigen::Quaternionf init_quat = Eigen::Quaternionf::Identity();
std::shared_ptr<State_Mimic::MotionLoader_> State_Mimic::motion = nullptr;

// ================================
// 23DoF torso / anchor orientation
// ================================
// 29DoF 官方版使用 root + waist_yaw + waist_roll + waist_pitch。
// 23DoF 中 waist_roll / waist_pitch 已删除，因此这里严格只保留 waist_yaw。
// 同时 real/ref 两侧使用完全一致的定义，避免静止阶段出现持续偏航补偿。

namespace
{

constexpr int kWaistYawIdx = 12;   // in 23DoF deploy order
constexpr int kWaistYawMotor = 12; // in lowstate motor_state order / deploy mapping

Eigen::Quaternionf torso_quat_w(isaaclab::ManagerBasedRLEnv* env)
{
    using G1Type = unitree::BaseArticulation<LowState_t::SharedPtr>;
    G1Type* robot = dynamic_cast<G1Type*>(env->robot.get());

    const auto root_quat = env->robot->data.root_quat_w;
    const auto& motors = robot->lowstate->msg_.motor_state();

    Eigen::Quaternionf torso_quat =
        root_quat *
        Eigen::AngleAxisf(motors[kWaistYawMotor].q(), Eigen::Vector3f::UnitZ());

    return torso_quat;
}

Eigen::Quaternionf anchor_quat_w(const std::shared_ptr<State_Mimic::MotionLoader_>& loader)
{
    const auto root_quat = loader->root_quaternion();
    const auto joint_pos = loader->joint_pos();

    Eigen::Quaternionf torso_quat =
        root_quat *
        Eigen::AngleAxisf(joint_pos[kWaistYawIdx], Eigen::Vector3f::UnitZ());

    return torso_quat;
}

} // namespace

namespace isaaclab
{
namespace mdp
{

REGISTER_OBSERVATION(motion_joint_pos)
{
    auto& loader = State_Mimic::motion;

    auto data_dfs = loader->joint_pos();
    return std::vector<float>(data_dfs.data(), data_dfs.data() + data_dfs.size());
}

REGISTER_OBSERVATION(motion_joint_vel)
{
    auto& loader = State_Mimic::motion;

    auto data_dfs = loader->joint_vel();
    return std::vector<float>(data_dfs.data(), data_dfs.data() + data_dfs.size());
}

REGISTER_OBSERVATION(motion_command)
{
    auto& loader = State_Mimic::motion;

    auto pos_dfs = loader->joint_pos();
    auto vel_dfs = loader->joint_vel();

    std::vector<float> data;
    data.reserve(pos_dfs.size() + vel_dfs.size());
    data.insert(data.end(), pos_dfs.data(), pos_dfs.data() + pos_dfs.size());
    data.insert(data.end(), vel_dfs.data(), vel_dfs.data() + vel_dfs.size());
    return data;
}

REGISTER_OBSERVATION(motion_anchor_ori_b)
{
    // real/ref 两侧都使用同一套 torso(anchor) 定义
    auto real_quat_w = torso_quat_w(env);
    auto ref_quat_w = anchor_quat_w(State_Mimic::motion);

    auto rot_ = (init_quat * ref_quat_w).conjugate() * real_quat_w;
    auto rot = rot_.toRotationMatrix().transpose();

    Eigen::Matrix<float, 6, 1> data;
    data << rot(0, 0), rot(0, 1),
            rot(1, 0), rot(1, 1),
            rot(2, 0), rot(2, 1);

    return std::vector<float>(data.data(), data.data() + data.size());
}

} // namespace mdp
} // namespace isaaclab

State_Mimic::State_Mimic(int state_mode, std::string state_string)
: FSMState(state_mode, state_string)
{
    auto cfg = param::config["FSM"][state_string];
    auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());

    auto articulation =
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate);

    std::filesystem::path motion_file = cfg["motion_file"].as<std::string>();
    if (!motion_file.is_absolute()) {
        motion_file = param::proj_dir / motion_file;
    }

    motion_ = std::make_shared<MotionLoader_>(motion_file.string(), cfg["fps"].as<float>());
    spdlog::info(
        "[23DOF] Loaded motion file '{}' with duration {:.2f}s",
        motion_file.stem().string(),
        motion_->duration
    );

    motion = motion_;

    if (cfg["time_start"]) {
        float time_start = cfg["time_start"].as<float>();
        time_range_[0] = std::clamp(time_start, 0.0f, motion_->duration);
    } else {
        time_range_[0] = 0.0f;
    }

    if (cfg["time_end"]) {
        float time_end = cfg["time_end"].as<float>();
        time_range_[1] = std::clamp(time_end, 0.0f, motion_->duration);
    } else {
        time_range_[1] = motion_->duration;
    }

    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile(policy_dir / "params" / "deploy.yaml"),
        articulation
    );
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "exported" / "policy.onnx");

    this->registered_checks.emplace_back(
        std::make_pair(
            [&]() -> bool { return (env->episode_length * env->step_dt) > time_range_[1]; },
            FSMStringMap.right.at("Velocity")
        )
    );

    this->registered_checks.emplace_back(
        std::make_pair(
            [&]() -> bool { return isaaclab::mdp::bad_orientation(env.get(), 1.0); },
            FSMStringMap.right.at("Passive")
        )
    );
}

void State_Mimic::enter()
{
    // set gain
    for (int i = 0; i < env->robot->data.joint_stiffness.size(); ++i)
    {
        lowcmd->msg_.motor_cmd()[i].kp() = env->robot->data.joint_stiffness[i];
        lowcmd->msg_.motor_cmd()[i].kd() = env->robot->data.joint_damping[i];
        lowcmd->msg_.motor_cmd()[i].dq() = 0.0f;
        lowcmd->msg_.motor_cmd()[i].tau() = 0.0f;
    }

    motion = motion_;
    env->reset();

    policy_thread_running = true;
    policy_thread = std::thread([this] {
        using clock = std::chrono::high_resolution_clock;
        const std::chrono::duration<double> desiredDuration(env->step_dt);
        const auto dt = std::chrono::duration_cast<clock::duration>(desiredDuration);

        auto sleepTill = clock::now() + dt;

        // 先把 motion 对齐到起始时刻
        motion->reset(env->robot->data, time_range_[0]);

        // 关键修正：
        // init_quat 两侧都使用 anchor/torso 的定义，而不是 ref 用 root、real 用 torso
        auto ref_yaw = isaaclab::yawQuaternion(anchor_quat_w(motion)).toRotationMatrix();
        auto robot_yaw = isaaclab::yawQuaternion(torso_quat_w(env.get())).toRotationMatrix();
        init_quat = robot_yaw * ref_yaw.transpose();

        // 不再第二次 env->reset()，避免 init_quat 与当前状态脱节
        while (policy_thread_running)
        {
            env->robot->update();
            motion->update(env->episode_length * env->step_dt + time_range_[0]);
            env->step();

            std::this_thread::sleep_until(sleepTill);
            sleepTill += dt;
        }
    });
}

void State_Mimic::run()
{
    auto action = env->action_manager->processed_actions();

    // 与官方部署逻辑一致：action[i] 写回 joint_ids_map[i]
    for (int i = 0; i < env->robot->data.joint_ids_map.size(); ++i) {
        lowcmd->msg_.motor_cmd()[env->robot->data.joint_ids_map[i]].q() = action[i];
    }
}