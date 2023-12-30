#include "straight_line_creator/straight_line_creator.h"

StraightLineCreator::StraightLineCreator():private_nh_("~")
{
    // param
    private_nh_.param("hz", hz_, {10});
    private_nh_.param("path_frame", path_frame_, {"odom"});
    private_nh_.param("init_x", init_x_, {0.0});
    private_nh_.param("init_y", init_y_, {0.0});
    // private_nh_.param("goal_x", goal_x_, {5.0});
    // private_nh_.param("goal_x", goal_x_, {20.0});
    private_nh_.param("init_theta", init_theta_, {0.0});
    private_nh_.param("cource_length", cource_length_, {20.0});
    private_nh_.param("resolution", resolution_, {0.05});

    // publisher
    pub_global_path_ = nh_.advertise<nav_msgs::Path>("/global_path", 1);

    // debug
    // pub_start_point_ = nh_.advertise<geometry_msgs::PoseStamped>("/start_point", 1);
    // pub_goal_point_ = nh_.advertise<geometry_msgs::PoseStamped>("/goal_point", 1);
}

// 軌道を生成
void StraightLineCreator::create_path()
{
    nav_msgs::Path path;
    path.header.frame_id = path_frame_;

    for(double x=0.0; x<cource_length_; x+=resolution_)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = path_frame_;

        pose.pose.position.x = init_x_ + x * cos(init_theta_);
        pose.pose.position.y = init_y_ + x * sin(init_theta_);

        path.poses.push_back(pose);        
    }

    global_path_ = path;
}

//メイン文で実行する関数
void StraightLineCreator::process()
{
    ros::Rate loop_rate(hz_);
    create_path();
    
    while(ros::ok())
    {
        global_path_.header.stamp = ros::Time::now();
        pub_global_path_.publish(global_path_);
        loop_rate.sleep();
    }
}