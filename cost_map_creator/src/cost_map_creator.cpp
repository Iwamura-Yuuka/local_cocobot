#include "cost_map_creator/cost_map_creator.h"

CostMapCreator::CostMapCreator():private_nh_("~")
{
    // param
    private_nh_.param("hz", hz_, {30});

    // subscriber
    sub_ped_states_ = nh_.subscribe("/pedsim_simulator/simulated_agents", 1, &CostMapCreator::pedestrian_data_callback, this);

    // publisher

}

void CostMapCreator::pedestrian_data_callback(const pedsim_msgs::AgentStatesConstPtr& agents)
{
    ped_states_.emplace(agents);
    flag_ped_states_ = true;
}

// デバック用
void CostMapCreator::id_veiwer()
{
    const auto people_states = ped_states_.front();

    for(const auto& person : people_states->agent_states)
    {
        current_state_.id = person.id;
        current_state_.pose.position.x = person.pose.position.x;
        current_state_.pose.position.y = person.pose.position.y;
        current_state_.twist.linear.x = person.twist.linear.x;
        current_state_.twist.linear.y = person.twist.linear.y;

        ROS_INFO_STREAM("id is" << current_state_.id);  // デバック用
        ROS_INFO_STREAM("position is x : " << current_state_.pose.position.x << "y : " << current_state_.pose.position.y);  // デバック用
        ROS_INFO_STREAM("speed is x : " << current_state_.twist.linear.x << "y : " << current_state_.twist.linear.y);  // デバック用
    }
}

//メイン文で実行する関数
void CostMapCreator::process()
{
    ros::Rate loop_rate(hz_);

    while(ros::ok())
    {
        if(flag_ped_states_ == true)
        {
            ROS_INFO_STREAM("get ped_states!");  // デバック用
            id_veiwer();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}