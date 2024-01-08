#include "local_goal_maker/local_goal_maker.h"

//メイン関数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_goal_maker");
    LocalGoalMaker localgoalmaker;
    localgoalmaker.process();

    return 0;
}