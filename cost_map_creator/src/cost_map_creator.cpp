#include "cost_map_creator/cost_map_creator.h"

CostMapCreator::CostMapCreator():private_nh_("~")
{
    // param
    private_nh_.param("hz", hz_, {10});
    // private_nh_.param("visualize_current_people_poses", visualize_current_people_poses_, {false});
    // private_nh_.param("visualize_future_people_poses", visualize_future_people_poses_, {false});
    private_nh_.param("people_frame", people_frame_, {"odom"});
    private_nh_.param("cost_map_frame", cost_map_frame_, {"base_link"});
    private_nh_.param("map_size", map_size_, {5.0});
    private_nh_.param("map_reso", map_reso_, {0.025});
    // private_nh_.param("predict_dist_border", predict_dist_border_, {8.0});
    // private_nh_.param("predict_time_resolution", predict_time_resolution_, {2.5});
    // private_nh_.param("tmp_robot_x", tmp_robot_x_, {0.0});
    // private_nh_.param("tmp_robot_y", tmp_robot_y_, {0.0});

    // subscriber
    sub_current_people_states_ = nh_.subscribe("/current_people_states", 1, &CostMapCreator::current_people_states_callback, this);
    sub_future_people_states_ = nh_.subscribe("/future_people_states", 1, &CostMapCreator::future_people_states_callback, this);
    sub_robot_odom_ = nh_.subscribe("/pedsim_simulator/robot_position", 1, &CostMapCreator::robot_odom_callback, this);

    // publisher
    pub_cost_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("/cost_map", 1);

    // debug

    // --- 基本設定 ---
    // header
    cost_map_.header.frame_id = cost_map_frame_;
    // // info
    cost_map_.info.resolution = map_reso_;
    cost_map_.info.width      = int(round(map_size_/map_reso_));
    cost_map_.info.height     = int(round(map_size_/map_reso_));
    cost_map_.info.origin.position.x = -map_size_/2.0;
    cost_map_.info.origin.position.y = -map_size_/2.0;
    // // data
    cost_map_.data.reserve(cost_map_.info.width * cost_map_.info.height);
}

// 現在の歩行者情報のコールバック関数
void CostMapCreator::current_people_states_callback(const pedestrian_msgs::PeopleStatesConstPtr& msg)
{
    geometry_msgs::TransformStamped transform;

    current_people_states_.emplace(msg);

    try
    {
        transform = tf_buffer_.lookupTransform(cost_map_frame_, people_frame_, ros::Time(0));
        flag_current_people_states_ = true;
    }
    catch(tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
        flag_current_people_states_ = false;
        return;
    }

    tf2::doTransform(*msg, current_people_states_, transform);
}

// 予測した歩行者情報のコールバック関数
void CostMapCreator::future_people_states_callback(const pedestrian_msgs::PeopleStatesConstPtr& msg)
{
    future_people_states_.emplace(msg);
    flag_future_people_states_ = true;
}

// ロボットのodomのコールバック関数
void CostMapCreator::robot_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    robot_odom_ = *msg;
    flag_robot_odom_ = true;
}

// 歩行者データを取得
// void CostMapCreator::get_ped_data(pedestrian_msgs::PeopleStates& current_people, ros::Time now)
// {
//     pedestrian_msgs::PersonState current_person;

//     current_people.header.stamp = now;
//     current_people.header.frame_id = people_frame_;
    
//     // ped_states_の配列のうち，1回のpublish分のデータ（配列の先頭の要素）のみ取得
//     const auto people_states = ped_states_.front();

//     // msg型を pedsim_msgs/AgentStates から pedestrian_msgs/PeopleStates に変更
//     for(const auto& person : people_states->agent_states)
//     {
//         current_person.id = person.id;
//         current_person.pose.position.x = person.pose.position.x;
//         current_person.pose.position.y = person.pose.position.y;
//         current_person.twist.linear.x = person.twist.linear.x;
//         current_person.twist.linear.y = person.twist.linear.y;

//         current_people.people_states.push_back(current_person);
//     }

//     pub_current_people_states_.publish(current_people);
// }

// ロボットと歩行者の間の距離を計算
// double CostMapCreator::calc_distance(const double robot_x, const double robot_y, const double person_x, const double person_y)
// {
//     const double dx = person_x - robot_x;
//     const double dy = person_y - robot_y;

//     return hypot(dx, dy);
// }

// 方位を計算
// double CostMapCreator::calc_direction(const double x1, const double y1, const double x2, const double y2)
// {
//     const double theta = atan2(y2 - y1, x2 - x1);

//     return normalize_angle(theta);
// }

// 適切な角度(-M_PI ~ M_PI)を返す
// double CostMapCreator::normalize_angle(double theta)
// {
//     if(theta > M_PI)
//         theta -= 2.0 * M_PI;
//     if(theta < -M_PI)
//         theta += 2.0 * M_PI;

//     return theta;
// }

// マップの初期化(すべて「未知」にする)
// void CostMapCreator::init_map()
// {
//     cost_map_.data.clear();

//     // マップサイズを計算
//     const int size = cost_map_.info.width * cost_map_.info.height;

//     for(int i=0; i<size; i++)
//     {
//         cost_map_.data.push_back(-1);  //「未知」にする
//     }
// }

// マップ内の場合、trueを返す
// bool CostMapCreator::is_in_map(const double dist, const double angle)
// {
//     const double x = dist * cos(angle);
//     const double y = dist * sin(angle);
//     const int index_x = int(round((x - cost_map_.info.origin.position.x) / cost_map_.info.resolution));
//     const int index_y = int(round((y - cost_map_.info.origin.position.y) / cost_map_.info.resolution));

//     if(index_x<cost_map_.info.width and index_y<cost_map_.info.height)
//         return true;
//     else
//         return false;
// }

// 距離と角度からグリッドのインデックスを返す
// int CostMapCreator::get_grid_index(const double dist, const double angle)
// {
//     const double x = dist * cos(angle);
//     const double y = dist * sin(angle);

//     return xy_to_grid_index(x, y);
// }

// 座標からグリッドのインデックスを返す
// int CostMapCreator::xy_to_grid_index(const double x, const double y)
// {
//     const int index_x = int(round((x - cost_map_.info.origin.position.x) / cost_map_.info.resolution));
//     const int index_y = int(round((y - cost_map_.info.origin.position.y) / cost_map_.info.resolution));

//     return index_x + (index_y * cost_map_.info.width);
// }

// x座標から対応するy座標を計算
// double CostMapCreator::calc_y(const double x, const double a, const double b, const double theta)
// {
//     // y = b / (sinθ+cosθ) * √(1-(x^2*(cosθ-sinθ)^2) / a^2)
//     double y = b / (sin(theta)+cos(theta)) * sqrt(1 - (pow(x,2.0) * pow(cos(theta)-sin(theta),2.0) / pow(a,2.0)));
    
//     return y;
// }

// 走行コストを計算
// void CostMapCreator::calc_cost(const pedestrian_msgs::PeopleStates& future_people)
// {
    




// }

// 1人のみ考慮したコストマップを作成
// void CostMapCreator::create_person_cost_map()
// {
//     nav_msgs::OccupancyGrid person_cost_map; 
//     // --- 基本設定 ---
//     // header
//     person_cost_map.header.frame_id = "odom";
//     // info
//     person_cost_map.info.resolution = map_reso_;
//     person_cost_map.info.width      = int(round(map_size_/map_reso_));
//     person_cost_map.info.height     = int(round(map_size_/map_reso_));
//     person_cost_map.info.origin.position.x = -map_size_/2.0;
//     person_cost_map.info.origin.position.y = -map_size_/2.0;
//     // data
//     person_cost_map.data.reserve(cost_map_.info.width * cost_map_.info.height);
// }

void CostMapCreator::create_cost_map()
{
    // pedestrian_msgs::PeopleStates current_people;
    // pedestrian_msgs::PeopleStates future_people;
    // ros::Time now = ros::Time::now();

    // // マップの初期化
    // init_map();

    // // 歩行者データを取得
    // get_ped_data(current_people, now);

    // // 歩行者の現在位置の可視化
    // if(visualize_current_people_poses_)
    //     visualize_people_pose(current_people, pub_current_ped_poses_, now);
    
    // // 歩行者の将来位置を予測
    // predict_future_ped_states(current_people, future_people, now);

    // // 歩行者の将来位置（予測）の可視化
    // if(visualize_future_people_poses_)
    //     visualize_people_pose(future_people, pub_future_ped_poses_, now);

    // // 走行コストを計算
    // calc_cost(future_people);

    // pub_cost_map_.publish(cost_map_);
    // // ped_states_の配列のうち取得済みのデータ（配列の先頭の要素）を削除
    // // これをしないと，front() でデータを取得する際，同じデータしか取得できない
    // ped_states_.pop();

    // // ロボットの位置を格納
    // tmp_robot_x_ = robot_odom_.pose.pose.position.x;
    // tmp_robot_y_ = robot_odom_.pose.pose.position.y;
}

// 歩行者の位置情報を可視化
// void CostMapCreator::visualize_people_pose(const pedestrian_msgs::PeopleStates& people, const ros::Publisher& pub_people_poses, ros::Time now)
// {
//     geometry_msgs::Pose person_pose;        // 歩行者1人の位置情報
//     geometry_msgs::PoseArray people_poses;  // 全歩行者の位置情報
//     people_poses.header.stamp = now;
//     people_poses.header.frame_id  = people_frame_;

//     // msg型を pedestrian_msgs/PeopleStates から geometry_msgs/PoseArray に変更
//     for(const auto& person : people.people_states)
//     {
//         person_pose.position.x = person.pose.position.x;
//         person_pose.position.y = person.pose.position.y;

//         people_poses.poses.push_back(person_pose);
//     }

//     pub_people_poses.publish(people_poses);
// }

//メイン文で実行する関数
void CostMapCreator::process()
{
    ros::Rate loop_rate(hz_);
    tf2_ros::TransformListener tf_listener(tf_buffer_);

    while(ros::ok())
    {
        if((flag_current_people_states_ == true) && (flag_future_people_states_ == true) && (flag_robot_odom_ == true))
        {
            ROS_INFO_STREAM("get ped_states & robot_state!");  // デバック用
            create_cost_map();
        }

        // msgの受け取り判定用flagをfalseに戻す
        flag_current_people_states_ = false;
        flag_future_people_states_ = false;
        flag_robot_odom_ = false;

        ros::spinOnce();
        loop_rate.sleep();
    }
}