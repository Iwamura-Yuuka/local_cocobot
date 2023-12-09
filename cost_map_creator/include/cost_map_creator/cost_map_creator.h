
#ifndef COST_MAP_CREATOR_H
#define COST_MAP_CREATOR_H

#include <ros/ros.h>
#include <queue>
#include <math.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <pedsim_msgs/AgentState.h>
#include <pedsim_msgs/AgentStates.h>
#include <pedestrian_msgs/PersonState.h>
#include <pedestrian_msgs/PeopleStates.h>

class CostMapCreator
{
public:
    CostMapCreator();
    void process();

private:
    // コールバック関数
    void pedestrian_data_callback(const pedsim_msgs::AgentStatesConstPtr& agents);
    void robot_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);

    // 引数あり関数
    void get_ped_data(pedestrian_msgs::PeopleStates& current_people, ros::Time now);
    double calc_distance(const double robot_x, const double robot_y, const double person_x, const double person_y);
    double calc_direction(const double robot_x, const double robot_y, const double person_x, const double person_y);
    double normalize_angle(double theta);
    void predict_future_ped_states(const pedestrian_msgs::PeopleStates& current_people, pedestrian_msgs::PeopleStates& future_people, ros::Time now);
    bool is_in_map(const double dist, const double angle);
    int get_grid_index(const double dist, const double angle);
    int xy_to_grid_index(const double x, const double y);
    double calc_y(const double x, const double a, const double b, const double theta);
    void calc_cost(const pedestrian_msgs::PeopleStates& future_people);
    void visualize_people_pose(const pedestrian_msgs::PeopleStates& people, const ros::Publisher& pub_people_poses, ros::Time now);

    // 引数なし関数
    void init_map();
    void create_cost_map();

    // yamlファイルで設定可能な変数
    int hz_;
    bool visualize_current_people_poses_;
    bool visualize_future_people_poses_;
    std::string robot_frame_;
    std::string people_frame_;
    double map_size_;
    double map_reso_;
    double predict_dist_border_;
    double predict_time_resolution_;

    // その他の変数
    double tmp_robot_x_;
    double tmp_robot_y_;
    
    // msgの受け取り判定用
    bool flag_ped_states_ = false;
    bool flag_robot_odom_ = false;

    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber
    ros::Subscriber sub_ped_states_;
    ros::Subscriber sub_robot_odom_;

    // Publisher
    ros::Publisher pub_current_ped_poses_;
    ros::Publisher pub_current_people_states_;
    ros::Publisher pub_future_ped_poses_;
    ros::Publisher pub_future_people_states_;
    ros::Publisher pub_cost_map_;

    // 各種オブジェクト
    std::queue<pedsim_msgs::AgentStatesConstPtr> ped_states_;  // 歩行者情報
    nav_msgs::Odometry robot_odom_;                            // ロボットの位置情報
    nav_msgs::OccupancyGrid cost_map_;                         // コストマップ
    // pedestrian_msgs::PersonState current_state_;
    // pedsim_msgs::AgentStateConstPtr person_;
    // geometry_msgs::PoseArray obs_poses_;

};

#endif // LOCAL_MAP_CREATOR_H