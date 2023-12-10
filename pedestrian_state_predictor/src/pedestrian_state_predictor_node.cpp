#include "pedestrian_state_predictor/pedestrian_state_predictor.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pedestrian_state_predictor");
    PedestrianStatePredictor pedestrianstatepredictor;
    pedestrianstatepredictor.process();

    return 0;
}