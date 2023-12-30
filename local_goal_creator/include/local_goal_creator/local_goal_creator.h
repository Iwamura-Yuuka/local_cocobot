#ifndef LOCAL_GOAL_CREATOR_H
#define LOCAL_GOAL_CREATOR_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

class LocalGoalCreator
{
    public:
        LocalGoalCreator();
        void process();

    private:
        //コールバック関数
        void global_path_callback(const nav_msgs::Path::ConstPtr& msg);
        void robot_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);

        //引数なし関数
        double calc_dist_to_local_goal();   // local_goalまでの距離を計算
        void update_local_goal();           // local_goalの更新

        //yamlファイルで設定可能な変数
        int hz_;                            //ループ周波数[Hz]
        std::string goal_frame_;            // local_goalのframe_id
        double dist_to_update_local_goal_;  // local_goalを更新する基準となるロボットとlocal_goalの距離 [m]
        int global_path_index_;             // global_pathのインデックス
        int index_step_;                    // 1回で更新するインデックス数

        //msgの受け取り判定用
        bool flag_global_path_ = false;
        bool flag_robot_odom_ = false;

        //NodeHandle
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        //Subscriber
        ros::Subscriber sub_global_path_;
        ros::Subscriber sub_robot_odom_;

        //Publisher
        ros::Publisher pub_local_goal_;

        // 各種オブジェクト
        nav_msgs::Path global_path_;              // global_path
        nav_msgs::Odometry robot_odom_;           // ロボットの位置情報
        geometry_msgs::PointStamped local_goal_;  // local_goal
};

#endif  // LOCAL_GOAL_CREATOR_H