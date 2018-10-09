#include "TaskScan.h"
#include <math.h>
#include "floor_nav/TaskScanConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

TaskIndicator TaskScan::initialise() {
    ROS_INFO("Rotating with angular velocity %.2f", cfg.angular_velocity);
    const geometry_msgs::Pose2D& tpose = env->getPose2D();
    initial_angle = tpose.theta;
    return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskScan::iterate() {
    const geometry_msgs::Pose2D& tpose = env->getPose2D();
    double d_theta = (tpose.theta >= initial_angle)
                         ? tpose.theta - initial_angle
                         : 2 * M_PI + tpose.theta - initial_angle;
    // ROS_INFO("Difference: %.2f", d_theta);
    if (d_theta > 2 * M_PI - 0.2) {
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
