#ifndef STRAIGHT_LINE_CREATOR_H
#define STRAIGHT_LINE_CREATOR_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

class StraightLineCreator
{
public:
    StraightLineCreator();
    void process();

private:
    // 引数あり関数
    // void visualize_point(const double x, const double y, const ros::Publisher& pub_point, ros::Time now);

    // 引数なし関数
    void create_path();

    // yamlファイルで設定可能な変数
    int hz_;                  // ループ周波数 [Hz]
    std::string path_frame_;  // 生成するpathのframe_id
    double init_x_;          // スタート地点のx座標
    double init_y_;          // スタート地点のy座標
    // double goal_x_;           // ゴール地点のx座標
    // double goal_y_;           // ゴール地点のy座標
    double init_theta_;       // 生成する軌道の向き
    double cource_length_;       // 目標軌道の長さの最大値 [m]
    double resolution_;              // 軌道生成時の刻み幅 [m]

    // その他の変数

    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Publisher
    ros::Publisher pub_global_path_;
    // ros::Publisher pub_start_point_;
    // ros::Publisher pub_goal_point_;

    // 各種オブジェクト
    nav_msgs::Path global_path_;  // global path
    // geometry_msgs::PoseStamped start_point_;  // スタート地点
    // geometry_msgs::PoseStamped goal_point_;   // ゴール地点
};

#endif // STRAIGHT_LINE_CREATOR_H