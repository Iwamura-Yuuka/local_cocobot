
#ifndef COST_MAP_CREATOR_H
#define COST_MAP_CREATOR_H

#include <ros/ros.h>
#include <queue>
// #include <math.h>
// #include <geometry_msgs/PoseArray.h>
// #include <geometry_msgs/Pose.h>
// #include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2/utils.h>

// original_msgs
#include <pedestrian_msgs/PersonState.h>
#include <pedestrian_msgs/PeopleStates.h>

// ===== 構造体 =====
struct Coordinate
{
    double x;  // [m]
    double y;  // [m]
};

// ===== クラス =====
class CostMapCreator
{
public:
    CostMapCreator();
    void process();

private:
    // コールバック関数
    void current_people_states_callback(const pedestrian_msgs::PeopleStatesConstPtr& msg);
    void future_people_states_callback(const pedestrian_msgs::PeopleStatesConstPtr& msg);
    // void robot_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);

    // 引数あり関数
    void init_map(nav_msgs::OccupancyGrid& map);
    double calc_speed(const double linear_x, const double linear_y);
    double calc_ellipse_long_param(const pedestrian_msgs::PersonState& person);
    double calc_ellipse_short_param(const double x, const double y);
    double calc_direction(const double x, const double y);
    double normalize_angle(double theta);
    bool is_in_map(nav_msgs::OccupancyGrid& map, const double x, const double y);
    int xy_to_grid_index(nav_msgs::OccupancyGrid& map, const double x, const double y);
    int count_grid(std::vector<Coordinate>& side, const double start_x, const double start_y, const double length, const double theta);
    void assign_cost_for_person_cost_map(const double x, const double y, const double cost, int& min_index, int& max_index);
    double calc_distance(const double person_x, const double person_y, const double x, const double y);
    double calc_short_side_length(const double x, const double a, const double b);
    // void get_ped_data(pedestrian_msgs::PeopleStates& current_people, ros::Time now);
    // double calc_distance(const double robot_x, const double robot_y, const double person_x, const double person_y);
    // void predict_future_ped_states(const pedestrian_msgs::PeopleStates& current_people, pedestrian_msgs::PeopleStates& future_people, ros::Time now);
    // bool is_in_map(nav_msgs::OccupancyGrid& map, const double dist, const double angle);
    // int get_grid_index(const double dist, const double angle);
    // int xy_to_grid_index(const double x, const double y);
    // double calc_y(const double x, const double a, const double b, const double theta);
    // void calc_cost(const pedestrian_msgs::PeopleStates& future_people);
    void create_person_cost_map(const pedestrian_msgs::PersonState& current_person, const pedestrian_msgs::PersonState& future_person, int& min_index, int& max_index);
    void copy_cost(const int min_index, const int max_index);
    // void visualize_people_pose(const pedestrian_msgs::PeopleStates& people, const ros::Publisher& pub_people_poses, ros::Time now);

    // 引数なし関数
    // void init_map(nav_msgs::OccupancyGrid& map);
    void create_cost_map();
    // void create_person_cost_map();

    // yamlファイルで設定可能な変数
    int hz_;                                // ループ周波数 [Hz]
    // bool visualize_current_people_poses_;
    // bool visualize_future_people_poses_;
    std::string people_frame_;             // 歩行者の位置情報のframe_id
    std::string cost_map_frame_;           // コストマップのframe_id
    double map_size_;                     // マップの一辺の長さ [m]
    double map_reso_;                     // マップの解像度 [m/cell]
    double ellipse_front_long_max_;       // 走行コストの楕円の長軸（前方）の最大値 [m]
    double ellipse_back_long_max_;        // 走行コストの楕円の長軸（後方）の最大値 [m]
    double ellipse_short_max_;      // 走行コストの楕円の短軸の最大値 [m]
    double weight_distance_;       // ロボットからの距離に関する項の重み定数
    double weight_speed_;       // 歩行者の速さに関する項の重み定数
    double ped_speed_max_;         // 歩行者の歩く速さの最大値 [m/s]
    double count_reso_;                   // コストを割り当てる際に計算する座標の刻み幅 [m]
    int min_cost_;              // 割り当てるコストの最小値
    // double predict_dist_border_;
    // double predict_time_resolution_;

    // その他の変数
    // double tmp_robot_x_;
    // double tmp_robot_y_;
    
    // msgの受け取り判定用
    bool flag_current_people_states_ = false;
    bool flag_future_people_states_ = false;
    
    // 歩行者情報のマッチング確認用
    bool flag_ped_data_matching_ = false;

    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber
    ros::Subscriber sub_current_people_states_;
    ros::Subscriber sub_future_people_states_;
    ros::Subscriber sub_robot_odom_;

    // Publisher
    ros::Publisher pub_cost_map_;

    // 各種オブジェクト
    std::queue<pedestrian_msgs::PeopleStatesConstPtr> current_people_states_;  // 現在の歩行者情報
    std::queue<pedestrian_msgs::PeopleStatesConstPtr> future_people_states_;   // 予測した歩行者情報
    // pedestrian_msgs::PeopleStatesConstPtr current_people_states_;
    // pedestrian_msgs::PeopleStatesConstPtr future_people_states_;
    // nav_msgs::Odometry robot_odom_;                                            // ロボットの位置情報
    nav_msgs::OccupancyGrid cost_map_;                                         // コストマップ
    nav_msgs::OccupancyGrid person_map_;                                       // 歩行者1人分のコストを計算する用
    // pedestrian_msgs::PersonState current_state_;
    // pedsim_msgs::AgentStateConstPtr person_;
    // geometry_msgs::PoseArray obs_poses_;

};

#endif // LOCAL_MAP_CREATOR_H