#ifndef TASK_SCAN_H
#define TASK_SCAN_H

#include "task_manager_lib/TaskDefinition.h"
#include "floor_nav/SimTasksEnv.h"
#include "floor_nav/TaskScanConfig.h"

using namespace task_manager_lib;

namespace floor_nav {
    class TaskScan : public TaskInstance<TaskScanConfig,SimTasksEnv>
    {
        private:
            double initial_angle;
        public:
            TaskScan(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskScan() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactoryScan : public TaskDefinition<TaskScanConfig, SimTasksEnv, TaskScan>
    {

        public:
            TaskFactoryScan(TaskEnvironmentPtr env) : 
                Parent("Scan","Rotate 360 degrees",true,env) {}
            virtual ~TaskFactoryScan() {};
    };
};

#endif // TASK_GOTO_H
