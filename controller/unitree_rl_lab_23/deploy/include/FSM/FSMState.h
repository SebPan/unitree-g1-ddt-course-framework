#pragma once

#include "SimCommand.h"
#include "Types.h"
#include "param.h"
#include "FSM/BaseState.h"
#include "isaaclab/devices/keyboard/keyboard.h"
#include "unitree_joystick_dsl.hpp"
#include "UnitreeJoystickBridge.h"
#include "memory"
#include <chrono>

namespace test {
    inline std::unique_ptr<UnitreeJoystickBridge> remote_joystick;
    inline std::optional<std::chrono::steady_clock::time_point> last_combo_log_time_;
}


class FSMState : public BaseState
{
public:
    FSMState(int state, std::string state_string) 
    : BaseState(state, state_string) 
    {
        spdlog::info("Initializing State_{} ...", state_string);

        auto transitions = param::config["FSM"][state_string]["transitions"];

        if(transitions)
        {
            auto transition_map = transitions.as<std::map<std::string, std::string>>();

            for(auto it = transition_map.begin(); it != transition_map.end(); ++it)
            {
                std::string target_fsm = it->first;
                if(!FSMStringMap.right.count(target_fsm))
                {
                    spdlog::warn("FSM State_'{}' not found in FSMStringMap!", target_fsm);
                    continue;
                }

                int fsm_id = FSMStringMap.right.at(target_fsm);

                std::string condition = it->second;
                unitree::common::dsl::Parser p(condition);
                auto ast = p.Parse();
                auto func = unitree::common::dsl::Compile(*ast);
                registered_checks.emplace_back(
                    std::make_pair(
                        [func]()->bool{ return func(FSMState::lowstate->joystick); },
                        fsm_id
                    )
                );
            }
        }

	// ============================================================
// Comando directo desde g1_core / simulacion
// ============================================================

if(state_string == "Passive")
{
    registered_checks.emplace_back(
        std::make_pair(
            []()->bool
            {
                return simcmd::mode().load() == simcmd::STAND;
            },
            FSMStringMap.right.at("FixStand")
        )
    );
}

if(state_string == "FixStand")
{
    registered_checks.emplace_back(
        std::make_pair(
            []()->bool
            {
                return simcmd::mode().load() == simcmd::VELOCITY;
            },
            FSMStringMap.right.at("Velocity")
        )
    );
}

        // register for all states
        registered_checks.emplace_back(
            std::make_pair(
                []()->bool{ return lowstate->isTimeout(); },
                FSMStringMap.right.at("Passive")
            )
        );
    }

    void pre_run()
    {
        lowstate->update();
        if(keyboard) keyboard->update();
        if(test::remote_joystick) {//
            test::remote_joystick->apply(lowstate->joystick);
        }

        auto now = std::chrono::steady_clock::now();
        if(lowstate->joystick.LT.pressed && lowstate->joystick.up.on_pressed) {
            spdlog::info("Remote joystick combo detected: LT + up.on_pressed -> request FixStand");
            test::last_combo_log_time_ = now;
        }
        if(lowstate->joystick.RB.pressed && lowstate->joystick.X.on_pressed) {
            spdlog::info("Remote joystick combo detected: RB + X.on_pressed -> request Velocity");
            test::last_combo_log_time_ = now;
        }
        if(lowstate->joystick.LT.pressed && lowstate->joystick.B.on_pressed) {
            spdlog::info("Remote joystick combo detected: LT + B.on_pressed -> request Passive");
            test::last_combo_log_time_ = now;
        }//
    }

    void post_run()
    {
        lowcmd->unlockAndPublish();
    }

    static std::unique_ptr<LowCmd_t> lowcmd;
    static std::shared_ptr<LowState_t> lowstate;
    static std::shared_ptr<Keyboard> keyboard;
};
