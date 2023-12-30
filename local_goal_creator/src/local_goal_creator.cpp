#include "local_goal_creator/local_goal_creator.h"

LocalGoalCreator::LocalGoalCreator():private_nh_("~")
{
    private_nh_.param("hz", hz_, {10});
    private_nh_.param("goal_frame", goal_frame_, {"odom"});
    private_nh_.param("dist_to_update_local_goal", dist_to_update_local_goal_, {5.0});
    private_nh_.param("global_path_index", global_path_index_, {100});
    private_nh_.param("index_step", index_step_, {5});

    //Subscriber
    sub_global_path_ = nh_.subscribe("/global_path", 1, &LocalGoalCreator::global_path_callback, this, ros::TransportHints().reliable().tcpNoDelay());
    sub_robot_odom_ = nh_.subscribe("/robot_odom", 1, &LocalGoalCreator::robot_odom_callback, this, ros::TransportHints().reliable().tcpNoDelay());

    //Publisher
    pub_local_goal_ = nh_.advertise<geometry_msgs::PointStamped>("local_goal", 1);
}

// global_pathのコールバック関数
void LocalGoalCreator::global_path_callback(const nav_msgs::Path::ConstPtr& msg)
{
    global_path_ = *msg;
    flag_global_path_ = true;
}

// ロボットのodomのコールバック関数
void LocalGoalCreator::robot_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    robot_odom_ = *msg;
    flag_robot_odom_ = true;
}

// local_goalまでの距離を計算
double LocalGoalCreator::calc_dist_to_local_goal()
{
    double dx = global_path_.poses[global_path_index_].pose.position.x - robot_odom_.pose.pose.position.x;
    double dy = global_path_.poses[global_path_index_].pose.position.y - robot_odom_.pose.pose.position.y;

    return hypot(dx, dy);
}

//local_goalの更新
void LocalGoalCreator::update_local_goal()
{
    double dist_to_local_goal = calc_dist_to_local_goal();

    while(dist_to_local_goal < dist_to_update_local_goal_)
    {
        global_path_index_ += index_step_;  //パスのインデックスを進める
        dist_to_local_goal = calc_dist_to_local_goal();

        //global_path_indexがglobal_path_.posesの配列の要素数を超えたら、local_goalとしてglobal_pathのゴールを設定
        if(global_path_index_ >= global_path_.poses.size())
        {
            global_path_index_ = global_path_.poses.size() - 1;
            break;
        }
    }

    //local_goalの更新
    local_goal_.header.stamp = ros::Time::now();
    local_goal_.header.frame_id = goal_frame_;

    local_goal_.point.x = global_path_.poses[global_path_index_].pose.position.x;
    local_goal_.point.y = global_path_.poses[global_path_index_].pose.position.y;

    pub_local_goal_.publish(local_goal_);
}

//メイン文で実行する関数
void LocalGoalCreator::process()
{
    ros::Rate loop_rate(hz_);

    while(ros::ok())
    {
        if((flag_global_path_ == true) && (flag_robot_odom_ == true))
        {
            update_local_goal();
        }

        // msgの受け取り判定用flagをfalseに戻す
        flag_global_path_ = false;
        flag_robot_odom_ = false;

        ros::spinOnce();
        loop_rate.sleep();
    }
}