#include "cost_map_creator/cost_map_creator.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "cost_map_creator");
    CostMapCreator costmapcreator;
    costmapcreator.process();

    return 0;
}