#include "cost_map_creator/cost_map_creator.h"

CostMapCreator::CostMapCreator():private_nh_("~")
{
    // param
    private_nh_.param("hz", hz_, {30});
    private_nh_.param("visualize_current_people_poses", visualize_current_people_poses_, {false});
    private_nh_.param("visualize_future_people_poses", visualize_future_people_poses_, {false});
    private_nh_.param("robot_frame", robot_frame_, {"odom"});
    private_nh_.param("people_frame", people_frame_, {"odom"});
    private_nh_.param("predict_dist_border", predict_dist_border_, {8.0});
    private_nh_.param("predict_time_resolution", predict_time_resolution_, {2.5});

    // subscriber
    sub_ped_states_ = nh_.subscribe("/pedsim_simulator/simulated_agents", 1, &CostMapCreator::pedestrian_data_callback, this);
    sub_robot_odom_ = nh_.subscribe("/pedsim_simulator/robot_position", 1, &CostMapCreator::robot_odom_callback, this);

    // publisher

    // debug
    pub_current_ped_poses_ = nh_.advertise<geometry_msgs::PoseArray>("/current_ped_poses", 1);
    pub_future_ped_poses_ = nh_.advertise<geometry_msgs::PoseArray>("/future_ped_poses", 1);
    pub_current_people_states_ = nh_.advertise<pedestrian_msgs::PeopleStates>("/current_people_states", 1);
    pub_future_people_states_ = nh_.advertise<pedestrian_msgs::PeopleStates>("/future_people_states", 1);
}

// 歩行者データのコールバック関数
void CostMapCreator::pedestrian_data_callback(const pedsim_msgs::AgentStatesConstPtr& agents)
{
    ped_states_.emplace(agents);
    flag_ped_states_ = true;
}

// ロボットのodomのコールバック関数
void CostMapCreator::robot_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    robot_odom_ = *msg;
    flag_robot_odom_ = true;
}

// 歩行者データを取得
void CostMapCreator::get_ped_data(pedestrian_msgs::PeopleStates& current_people, ros::Time now)
{
    pedestrian_msgs::PersonState current_person;

    current_people.header.stamp = now;
    current_people.header.frame_id = people_frame_;
    
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
}

// ロボットと歩行者の間の距離を計算
double CostMapCreator::calc_distance(const double robot_x, const double robot_y, const double person_x, const double person_y)
{
    const double dx = person_x - robot_x;
    const double dy = person_y - robot_y;

    return hypot(dx, dy);
}

// 歩行者の将来位置を予測
void CostMapCreator::predict_future_ped_states(const pedestrian_msgs::PeopleStates& current_people, pedestrian_msgs::PeopleStates& future_people, ros::Time now)
{
    pedestrian_msgs::PersonState future_person;

    future_people.header.stamp = now;
    future_people.header.frame_id = people_frame_;

    for(const auto& current_person : current_people.people_states)
    {
        // ロボットと歩行者の間の距離を計算
        const double dist = calc_distance(robot_odom_.pose.pose.position.x, robot_odom_.pose.pose.position.y, current_person.pose.position.x, current_person.pose.position.y);

        // ロボットからの距離が一定以下の歩行者に関して，将来の位置を予測
        // 可能であれば，ロボットの進行方向のみ予測する形に変える
        if(dist <= predict_dist_border_)
        {
            // 何秒先の将来位置を予測するか
            const double predict_time = dist / predict_time_resolution_;

            // 将来位置を計算
            const double future_x = current_person.pose.position.x + (current_person.twist.linear.x * predict_time);
            const double future_y = current_person.pose.position.y + (current_person.twist.linear.y * predict_time);

            // 歩行者の予測データを格納
            future_person.id = current_person.id;
            future_person.pose.position.x = future_x;
            future_person.pose.position.y = future_y;
            future_person.twist.linear.x = current_person.twist.linear.x;
            future_person.twist.linear.y = current_person.twist.linear.y;
            ROS_INFO_STREAM("id : " << future_person.id);  // デバック用
            ROS_INFO_STREAM("dist is " << dist);  // デバック用

            future_people.people_states.push_back(future_person);
        }
    }

    pub_future_people_states_.publish(future_people);
}

void CostMapCreator::create_cost_map()
{
    pedestrian_msgs::PeopleStates current_people;
    pedestrian_msgs::PeopleStates future_people;
    ros::Time now = ros::Time::now();

    // 歩行者データを取得
    get_ped_data(current_people, now);

    // 歩行者の現在位置の可視化
    if(visualize_current_people_poses_)
        visualize_people_pose(current_people, pub_current_ped_poses_, now);
    
    // 歩行者の将来位置を予測
    predict_future_ped_states(current_people, future_people, now);

    // 歩行者の将来位置（予測）の可視化
    if(visualize_future_people_poses_)
        visualize_people_pose(future_people, pub_future_ped_poses_, now);

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
            create_cost_map();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}