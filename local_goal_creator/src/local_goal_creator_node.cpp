#include "local_goal_creator/local_goal_creator.h"

//メイン関数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_goal_creator");
    LocalGoalCreator localgoalcreator;
    localgoalcreator.process();

    return 0;
}