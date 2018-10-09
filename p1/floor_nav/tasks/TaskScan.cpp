#include "TaskScan.h"
#include <cmath>
#include "floor_nav/TaskScanConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

namespace {
    constexpr auto EPSILON = 0.1;
    using std::abs;
}

TaskIndicator TaskScan::initialise() {
    ROS_INFO("Rotating with angular velocity %.2f", cfg.angular_velocity);
    auto tpose = env->getPose2D();
    initial_angle = tpose.theta;
    return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskScan::iterate() {
    auto tpose = env->getPose2D();
    double d_theta = tpose.theta - initial_angle;
    if (d_theta < -EPSILON / 10) {
        d_theta += 2 * M_PI;
    }
    // ROS_INFO("Difference: %.2f", d_theta);
    if (2 * M_PI - d_theta < EPSILON) {
        return TaskStatus::TASK_COMPLETED;
    }
    env->publishVelocity(0, cfg.angular_velocity);
    return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskScan::terminate() {
    env->publishVelocity(0, 0);
    return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryScan);
