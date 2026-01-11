#include <thread>
#include <atomic>
#include <chrono>
#include "rokae/robot.h"
#include "print_helper.hpp"

using namespace rokae;
using namespace std;
using namespace rokae::RtSupportedFields;
error_code ec;

int main() {
    string remoteIP = "192.168.2.160";
    string localIP = "192.168.2.100";

    xMateRobot robot(remoteIP, localIP);

    std::string robot_name = robot.robotInfo(ec).type;
    robot.setRtNetworkTolerance(20, ec);
    robot.setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
    robot.setOperateMode(rokae::OperateMode::automatic, ec);
    robot.setPowerState(true, ec);

    // 接收状态数据，周期 1ms
    robot.startReceiveRobotState(std::chrono::milliseconds(1), {jointPos_m});

    auto rtCon = robot.getRtMotionController().lock();

    // 必须先调用 startMove，指定控制模式
    rtCon->startMove(rokae::RtControllerMode::jointPosition);

    // 获取初始角度（返回 std::array<double,6>）
    std::array<double, 6> initJoints = robot.jointPos(ec);
    double axis6 = initJoints[5];   // 第6轴初始角度
    bool increasing = true;         // 当前方向：增加还是减少
    int counter = 0;                 // 用于实现2ms逻辑

    // 设置回调函数
    rtCon->setControlLoop<rokae::JointPosition>(
        [&]() -> rokae::JointPosition {
            counter++;
            rokae::JointPosition cmd;
            cmd.joints.resize(6);

            // 每次回调都读取当前角度
            std::array<double, 6> current = robot.jointPos(ec);

            // 打印6个轴角度
            cout << "Joint angles: ";
            for (int i = 0; i < 6; ++i) {
                cout << current[i] << " ";
                cmd.joints[i] = current[i]; // 默认保持当前位置
            }
            cout << endl;

            // 每2次回调（即2ms）更新一次第6轴
            if (counter % 2 == 0) {
                if (increasing) {
                    axis6 += 0.001;
                    if (axis6 >= 2.8) increasing = false;
                } else {
                    axis6 -= 0.001;
                    if (axis6 <= 0.0) increasing = true;
                }
            }

            // 更新第6轴目标角度
            cmd.joints[5] = axis6;

            return cmd;
        },
        0,
        true
    );

    // 启动循环（阻塞）
    rtCon->startLoop(false);

    while(1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 停止循环和运动（可选，通常在退出时调用）
    rtCon->stopLoop();
    rtCon->stopMove();

    return 0;
}
