#ifndef PEDESTRIAN_STATE_PREDICTOR_H
#define PEDESTRIAN_STATE_PREDICTOR_H

#include <ros/ros.h>
#include <queue>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

// original_msgs
#include <pedsim_msgs/AgentState.h>
#include <pedsim_msgs/AgentStates.h>
#include <pedestrian_msgs/PersonState.h>
#include <pedestrian_msgs/PeopleStates.h>

class PedestrianStatePredictor
{
public:
    PedestrianStatePredictor();
    void process();

private:
    // コールバック関数
    void pedestrian_data_callback(const pedsim_msgs::AgentStatesConstPtr& agents);
    void robot_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);

    // 引数あり関数
    void get_ped_data(pedestrian_msgs::PeopleStates& current_people, ros::Time now);                                                                                                                           // 歩行者データを取得
    double calc_distance(const double robot_x, const double robot_y, const double person_x, const double person_y);                                                                                            // ロボットと歩行者の間の距離を計算
    void transform_and_calc_speed(const pedestrian_msgs::PersonState& current_person, pedestrian_msgs::PersonState& selected_current_person, const double after_ones_x, const double after_ones_y);            // 歩行者情報をodomからbase_footprintに変更 & base_link座標系での速度を計算
    double calc_direction(const double robot_x, const double robot_y, const double person_x, const double person_y);                                                                                           // 方位を計算
    double normalize_angle(double theta);                                                                                                                                                                      // 適切な角度(-M_PI ~ M_PI)を返す
    void transform_ped_pose(const double before_x, const double before_y,  pedestrian_msgs::PersonState& future_person);                                                                                       // 歩行者情報をodomからbase_footprintに変更
    void predict_future_ped_states(const pedestrian_msgs::PeopleStates& current_people, pedestrian_msgs::PeopleStates& future_people, pedestrian_msgs::PeopleStates& selected_current_people, ros::Time now);  // 歩行者の将来位置を予測
    void visualize_people_pose(const pedestrian_msgs::PeopleStates& people, const ros::Publisher& pub_people_poses, ros::Time now);                                                                            // 歩行者の位置情報を可視化

    // 引数なし関数
    void update_ped_state();  // 歩行者情報を更新

    // yamlファイルで設定可能な変数
    int hz_;                                        // ループ周波数 [Hz]
    bool visualize_current_people_poses_;           // 歩行者の現在位置（odom）を可視化するかの設定用
    bool visualize_selected_current_people_poses_;  // ロボットに近い歩行者の現在位置（base_footprint）を可視化するかの設定用
    bool visualize_future_people_poses_;            // 予測した歩行者の将来位置を可視化するかの設定用
    std::string sim_frame_;                         // シミュレーターからの歩行者の位置情報のframe_id
    std::string robot_frame_;                       // ロボットの位置情報のframe_id
    std::string people_frame_;                      // 歩行者の位置情報のframe_id
    double predict_dist_border_;                    // 歩行者の将来位置を予測する距離 [m]
    double predict_time_resolution_;                // 1秒先を予測する距離 [m]

    // その他の変数
    double tmp_robot_x_;                            // 1ループ前のロボットのx座標格納用
    double tmp_robot_y_;                            // 1ループ前のロボットのy座標格納用
    
    // msgの受け取り判定用
    bool flag_ped_states_ = false;
    bool flag_robot_odom_ = false;

    // 座標変換の判定用
    bool flag_frame_change_ = false;

    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber
    ros::Subscriber sub_ped_states_;
    ros::Subscriber sub_robot_odom_;

    // Publisher
    ros::Publisher pub_current_ped_poses_;
    ros::Publisher pub_selected_current_ped_poses_;
    ros::Publisher pub_selected_current_people_states_;
    ros::Publisher pub_future_ped_poses_;
    ros::Publisher pub_future_people_states_;

    // tf
    tf2_ros::Buffer tf_buffer_;

    // 各種オブジェクト
    std::queue<pedsim_msgs::AgentStatesConstPtr> ped_states_;  // 歩行者情報
    nav_msgs::Odometry robot_odom_;                            // ロボットの位置情報
};


#endif // PEDESTRIAN_STATE_PREDICTOR_H