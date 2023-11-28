#include "cost_map_creator/cost_map_creator.h"

CostMapCreator::CostMapCreator():private_nh_("~")
{
    // param
    private_nh_.param("hz", hz_, {30});

    // subscriber
    sub_ped_states_ = nh_.subscribe("/pedsim_simulator/simulated_agents", 1, &CostMapCreator::pedestrian_data_callback, this);
    sub_robot_odom_ = nh_.subscribe("/pedsim_simulator/robot_position", 1, &CostMapCreator::robot_odom_callback, this);

    // publisher

    // debug
    pub_current_ped_position_ = nh_.advertise<geometry_msgs::PoseArray>("/current_ped_position", 1);

}

void CostMapCreator::pedestrian_data_callback(const pedsim_msgs::AgentStatesConstPtr& agents)
{
    ped_states_.emplace(agents);
    // ROS_INFO_STREAM("id[1]'s x is " << agents->agent_states[1].pose.position.x);  // デバック用
    flag_ped_states_ = true;
}

void CostMapCreator::robot_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    robot_odom_ = *msg;
    ROS_INFO_STREAM("robot x is " << robot_odom_.pose.pose.position.x);  // デバック用
    flag_robot_odom_ = true;
}

// デバック用
void CostMapCreator::id_veiwer()
{
    geometry_msgs::PoseArray current_ped_poses;  // 歩行者の現在位置格納用
    current_ped_poses.header.frame_id  = "odom";
    geometry_msgs::Pose current_ped_pose;

    pedestrian_msgs::PersonState current_state;
    
    const auto people_states = ped_states_.front();

    for(const auto& person : people_states->agent_states)
    {
        current_state.id = person.id;
        current_state.pose.position.x = person.pose.position.x;
        current_state.pose.position.y = person.pose.position.y;
        current_state.twist.linear.x = person.twist.linear.x;
        current_state.twist.linear.y = person.twist.linear.y;

        current_ped_pose.position.x = current_state.pose.position.x;
        current_ped_pose.position.y = current_state.pose.position.y;

        current_ped_poses.poses.push_back(current_ped_pose);

        if(current_state.id == 1)
        {
            // ROS_INFO_STREAM("id is" << current_state.id);  // デバック用
            // ROS_INFO_STREAM("position is x : " << current_state.pose.position.x << "  y : " << current_state.pose.position.y);  // デバック用
            // ROS_INFO_STREAM("speed is x : " << current_state.twist.linear.x << "  y : " << current_state.twist.linear.y);  // デバック用
        }
    }

    pub_current_ped_position_.publish(current_ped_poses);
    ped_states_.pop();
}

//メイン文で実行する関数
void CostMapCreator::process()
{
    ros::Rate loop_rate(hz_);

    while(ros::ok())
    {
        if((flag_ped_states_ == true) && (flag_robot_odom_ == true))
        {
            ROS_INFO_STREAM("get ped_states!");  // デバック用
            id_veiwer();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}