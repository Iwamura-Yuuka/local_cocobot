#include "cost_map_creator/cost_map_creator.h"

CostMapCreator::CostMapCreator():private_nh_("~")
{
    // param
    private_nh_.param("hz", hz_, {30});
    private_nh_.param("visualize_current_people_poses", visualize_current_people_poses_, {false});
    private_nh_.param("robot_frame", robot_frame_, {"odom"});
    private_nh_.param("people_frame", people_frame_, {"odom"});


    // subscriber
    sub_ped_states_ = nh_.subscribe("/pedsim_simulator/simulated_agents", 1, &CostMapCreator::pedestrian_data_callback, this);
    sub_robot_odom_ = nh_.subscribe("/pedsim_simulator/robot_position", 1, &CostMapCreator::robot_odom_callback, this);

    // publisher

    // debug
    pub_current_ped_poses_ = nh_.advertise<geometry_msgs::PoseArray>("/current_ped_poses", 1);
    pub_current_people_states_ = nh_.advertise<pedestrian_msgs::PeopleStates>("/current_people_states", 1);
}

// 歩行者データのコールバック関数
void CostMapCreator::pedestrian_data_callback(const pedsim_msgs::AgentStatesConstPtr& agents)
{
    ped_states_.emplace(agents);
    // ROS_INFO_STREAM("id[1]'s x is " << agents->agent_states[1].pose.position.x);  // デバック用
    flag_ped_states_ = true;
}

// ロボットのodomのコールバック関数
void CostMapCreator::robot_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    robot_odom_ = *msg;
    // ROS_INFO_STREAM("robot x is " << robot_odom_.pose.pose.position.x);  // デバック用
    flag_robot_odom_ = true;
}

// 歩行者データを取得
void CostMapCreator::id_veiwer()
{
    pedestrian_msgs::PersonState current_person;
    pedestrian_msgs::PeopleStates current_people;
    current_people.header.stamp  = ros::Time::now();
    current_people.header.frame_id  = "odom";
    
    // ped_states_の配列のうち，1回のpublish分のデータ（配列の先頭の要素）のみ取得
    const auto people_states = ped_states_.front();

    // msg型を pedsim_msgs/AgentStates から pedestrian_msgs/PeopleStates に変更
    for(const auto& person : people_states->agent_states)
    {
        current_person.id = person.id;
        current_person.pose.position.x = person.pose.position.x;
        current_person.pose.position.y = person.pose.position.y;
        current_person.twist.linear.x = person.twist.linear.x;
        current_person.twist.linear.y = person.twist.linear.y;

        current_people.people_states.push_back(current_person);
    }

    pub_current_people_states_.publish(current_people);

    // 歩行者の現在位置の可視化
    if(visualize_current_people_poses_)
    {
        ros::Time now = ros::Time::now();
        visualize_people_pose(current_people, pub_current_ped_poses_, now);
    }

    // ped_states_の配列のうち取得済みのデータ（配列の先頭の要素）を削除
    // これをしないと，front() でデータを取得する際，同じデータしか取得できない
    ped_states_.pop();
}

// 歩行者の位置情報を可視化
void CostMapCreator::visualize_people_pose(const pedestrian_msgs::PeopleStates& people, const ros::Publisher& pub_people_poses, ros::Time now)
{
    geometry_msgs::Pose person_pose;        // 歩行者1人の位置情報
    geometry_msgs::PoseArray people_poses;  // 全歩行者の位置情報
    people_poses.header.stamp = now;
    people_poses.header.frame_id  = people_frame_;

    // msg型を pedestrian_msgs/PeopleStates から geometry_msgs/PoseArray に変更
    for(const auto& person : people.people_states)
    {
        person_pose.position.x = person.pose.position.x;
        person_pose.position.y = person.pose.position.y;

        people_poses.poses.push_back(person_pose);
    }

    pub_people_poses.publish(people_poses);
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