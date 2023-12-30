#include "straight_line_creator/straight_line_creator.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "straight_line_creator");
    StraightLineCreator straightlinecreator;
    straightlinecreator.process();

    return 0;
}