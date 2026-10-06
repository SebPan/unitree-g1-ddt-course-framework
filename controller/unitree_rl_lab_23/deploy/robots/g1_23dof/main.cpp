#include "SimCommand.h"
#include "FSM/CtrlFSM.h"
#include "FSM/FSMState.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_RLBase.h"
#include "State_Mimic.h"
#include "UnitreeJoystickBridge.h"
#include <memory>
//#include "keyboard.h" 

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = nullptr;

void init_fsm_state()
{
    auto lowcmd_sub =
        std::make_shared<
            unitree::robot::g1::subscription::LowCmd
        >();

    usleep(0.2 * 1e6);

    if (!lowcmd_sub->isTimeout())
    {
        spdlog::critical(
            "The other process is using the lowcmd channel, "
            "please close it first."
        );

        unitree::robot::go2::shutdown();
    }

    FSMState::lowcmd =
        std::make_unique<LowCmd_t>();

    FSMState::lowstate =
        std::make_shared<LowState_t>();

    FSMState::keyboard =
        std::make_shared<Keyboard>();

    spdlog::info(
        "Waiting for connection to robot..."
    );

    FSMState::lowstate->wait_for_connection();

    spdlog::info(
        "Connected to robot."
    );
}

int main(int argc, char** argv)
{
    // Load parameters
    auto vm = param::helper(argc, argv);

    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     G1-23dof Controller \n";

    // Unitree DDS Config
    unitree::robot::ChannelFactory::Instance()->Init(0, vm["network"].as<std::string>());

    init_fsm_state();

    FSMState::lowcmd->msg_.mode_machine() = 4; // 23dof

    if (
        FSMState::lowstate->msg_.mode_machine()
        !=
        FSMState::lowcmd->msg_.mode_machine()
    )
    {
        spdlog::critical(
            "Unmatched robot type. lowstate={} lowcmd={}",
            FSMState::lowstate->msg_.mode_machine(),
            FSMState::lowcmd->msg_.mode_machine()
        );

        exit(-1);
    }

    simcmd::start_udp_receiver();
    
    // Initialize FSM
    auto fsm = std::make_unique<CtrlFSM>(param::config["FSM"]);
    fsm->start();

    std::cout << "Press [L2 + Up] to enter FixStand mode.\n";
    std::cout << "And then press [R1 + X] to start controlling the robot.\n";

    test::remote_joystick = std::make_unique<UnitreeJoystickBridge>("/dev/shm/unitree_joystick_bridge.json", 0.2);//

    while (true)
    {

        /*if(FSMState::keyboard) {
            FSMState::keyboard->update();
        }*/
        sleep(1);
    }
    
    return 0;
}

