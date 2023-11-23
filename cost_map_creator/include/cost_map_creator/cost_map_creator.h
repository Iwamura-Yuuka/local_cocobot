
#ifndef COST_MAP_CREATOR_H
#define COST_MAP_CREATOR_H

#include <ros/ros.h>
#include <queue>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

#include <pedsim_msgs/AgentState.h>
#include <pedsim_msgs/AgentStates.h>

class CostMapCreator
{
public:
    CostMapCreator();
    void process();

private:
    // コールバック関数
    void pedestrian_data_callback(const pedsim_msgs::AgentStatesConstPtr& agents);

    // 引数あり関数

    // 引数なし関数

    // yamlファイルで設定可能な変数
    int hz_;
    double local_map_size_;
    double local_map_resolution_;

    // その他の変数
    
    
    // msgの受け取り判定用
    bool flag_ped_states_ = false;

    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber
    ros::Subscriber sub_ped_states_;

    // Publisher
    // ros::Publisher pub_local_map_;

    std::queue<pedsim_msgs::AgentStatesConstPtr> ped_states_;  // 歩行者情報
    // nav_msgs::OccupancyGrid local_map_;
    // geometry_msgs::PoseArray obs_poses_;

};

#endif // LOCAL_MAP_CREATOR_H