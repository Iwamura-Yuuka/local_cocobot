#include "target_path_planner/target_path_planner.h"

//メイン関数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "target_path_planner");
    TargetPathPlanner targetpathplanner;
    targetpathplanner.process();

    return 0;
}