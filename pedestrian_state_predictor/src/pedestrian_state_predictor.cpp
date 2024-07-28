#include "pedestrian_state_predictor/pedestrian_state_predictor.h"

PedestrianStatePredictor::PedestrianStatePredictor():private_nh_("~")
{
    // param
    private_nh_.param("hz", hz_, {10});
    private_nh_.param("visualize_current_people_poses", visualize_current_people_poses_, {false});
    private_nh_.param("visualize_selected_current_people_poses", visualize_selected_current_people_poses_, {false});
    private_nh_.param("visualize_future_people_poses", visualize_future_people_poses_, {false});
    private_nh_.param("flag_prediction", flag_prediction_, {true});
    private_nh_.param("sim_frame", sim_frame_, {"odom"});
    private_nh_.param("robot_frame", robot_frame_, {"odom"});
    private_nh_.param("people_frame", people_frame_, {"base_footprint"});
    private_nh_.param("consider_dist_border", consider_dist_border_, {8.0});
    private_nh_.param("predict_dist_border", predict_dist_border_, {3.0});
    private_nh_.param("dt", dt_, {0.1});
    private_nh_.param("tmp_robot_x", tmp_robot_x_, {0.0});
    private_nh_.param("tmp_robot_y", tmp_robot_y_, {0.0});

    // subscriber
    sub_ped_states_ = nh_.subscribe("/pedsim_simulator/simulated_agents", 1, &PedestrianStatePredictor::pedestrian_data_callback, this, ros::TransportHints().reliable().tcpNoDelay());
    sub_robot_odom_ = nh_.subscribe("/robot_odom", 1, &PedestrianStatePredictor::robot_odom_callback, this, ros::TransportHints().reliable().tcpNoDelay());

    // publisher
    pub_selected_current_people_states_ = nh_.advertise<pedestrian_msgs::PeopleStates>("/selected_current_people_states", 1);
    pub_future_people_states_ = nh_.advertise<pedestrian_msgs::PeopleStates>("/future_people_states", 1);

    // debug
    pub_current_ped_poses_ = nh_.advertise<geometry_msgs::PoseArray>("/current_ped_poses", 1);
    pub_selected_current_ped_poses_ = nh_.advertise<geometry_msgs::PoseArray>("/selected_current_ped_poses", 1);
    pub_future_ped_poses_ = nh_.advertise<geometry_msgs::PoseArray>("/future_ped_poses", 1);
}

// 歩行者データのコールバック関数
void PedestrianStatePredictor::pedestrian_data_callback(const pedsim_msgs::AgentStatesConstPtr& agents)
{
    ped_states_.emplace(agents);
    flag_ped_states_ = true;
}

// ロボットのodomのコールバック関数
void PedestrianStatePredictor::robot_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    robot_odom_ = *msg;
    flag_robot_odom_ = true;
}

// 歩行者データを取得
void PedestrianStatePredictor::get_ped_data(pedestrian_msgs::PeopleStates& current_people, ros::Time now)
{
    pedestrian_msgs::PersonState current_person;

    current_people.header.stamp = now;
    current_people.header.frame_id = sim_frame_;

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
}

// ロボットと歩行者の間の距離を計算
double PedestrianStatePredictor::calc_distance(const double robot_x, const double robot_y, const double person_x, const double person_y)
{
    const double dx = person_x - robot_x;
    const double dy = person_y - robot_y;

    return hypot(dx, dy);
}

// 速度を計算
double PedestrianStatePredictor::calc_speed(const double tmp_x, const double tmp_y, const double current_x, const double current_y)
{
    const double dx = current_x - tmp_x;
    const double dy = current_y - tmp_y;

    return hypot(dx, dy) * hz_;
}

// 方位を計算
double PedestrianStatePredictor::calc_direction(const double x1, const double y1, const double x2, const double y2)
{
    const double theta = atan2(y2 - y1, x2 - x1);

    return normalize_angle(theta);
}

// 適切な角度(-M_PI ~ M_PI)を返す
double PedestrianStatePredictor::normalize_angle(double theta)
{
    if(theta > M_PI)
        theta -= 2.0 * M_PI;
    if(theta < -M_PI)
        theta += 2.0 * M_PI;

    return theta;
}

// 歩行者情報をodomからbase_footprintに変更
void PedestrianStatePredictor::transform_ped_pose(const double before_x, const double before_y, pedestrian_msgs::PersonState& future_person)
{
    geometry_msgs::TransformStamped tf;

    // 歩行者情報をodomからbase_footprintに変更
    try
    {
        tf = tf_buffer_.lookupTransform(people_frame_, sim_frame_, ros::Time(0));
        flag_frame_change_ = true;

        geometry_msgs::Pose person_pose_before;    // 座標変換前の歩行者位置格納用
        geometry_msgs::Pose person_pose_after;     // 座標変換後の歩行者位置格納用

        person_pose_before.position.x = before_x;
        person_pose_before.position.y = before_y;

        // 歩行者の現在位置をdoTransformで座標変換
        tf2::doTransform(person_pose_before, person_pose_after, tf);

        future_person.pose.position = person_pose_after.position;
        future_person.pose.orientation.x = 0.0;
        future_person.pose.orientation.y = 0.0;
        future_person.pose.orientation.z = 0.0;
        future_person.pose.orientation.w = 0.0;
    }
    catch(tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
        flag_frame_change_ = false;
        return;
    }
}

// 歩行者の将来位置を予測
void PedestrianStatePredictor::predict_future_ped_states(const pedestrian_msgs::PeopleStates& current_people, pedestrian_msgs::PeopleStates& future_people, pedestrian_msgs::PeopleStates& selected_current_people, ros::Time now)
{
    pedestrian_msgs::PersonState future_person;
    pedestrian_msgs::PersonState selected_current_person;

    future_people.header.stamp = now;
    future_people.header.frame_id = people_frame_;
    selected_current_people.header.stamp = now;
    selected_current_people.header.frame_id = people_frame_;

    for(const auto& current_person : current_people.people_states)
    {
        // ロボットと歩行者の間の距離を計算
        double dist = calc_distance(robot_odom_.pose.pose.position.x, robot_odom_.pose.pose.position.y, current_person.pose.position.x, current_person.pose.position.y);

        // ロボットからの距離が一定以下の歩行者に関して，将来位置を予測
        if(dist <= consider_dist_border_)
        {
            // 将来位置を定義
            double future_x = current_person.pose.position.x;
            double future_y = current_person.pose.position.y;

            // 将来位置を計算
            if((flag_prediction_ == true) && (dist <= predict_dist_border_))
            {
                // ロボットの速度を計算（odom座標系）
                double robot_vel = calc_speed(tmp_robot_x_, tmp_robot_y_, robot_odom_.pose.pose.position.x, robot_odom_.pose.pose.position.y);           

                // 何秒先の将来位置を予測するか
                double predict_time = 0.0;  // 相対速度が0以下の場合は，現在位置を予測位置とする

                // ロボットと歩行者の最小距離
                double min_dist = consider_dist_border_;  // とりうる最大値で初期化

                // ロボットと歩行者が最接近する際の位置を計算
                while(dist <= min_dist)
                {
                    // 予測時刻を計算
                    predict_time += dt_;

                    // 歩行者の予測位置を計算（odom座標系）
                    const double predict_ped_x = current_person.pose.position.x + (current_person.twist.linear.x * predict_time);
                    const double predict_ped_y = current_person.pose.position.y + (current_person.twist.linear.y * predict_time);

                    // ロボットと歩行者の距離を計算
                    dist = calc_distance(robot_odom_.pose.pose.position.x, robot_odom_.pose.pose.position.y, predict_ped_x, predict_ped_y) - (robot_vel * predict_time);

                    // 最小距離と将来位置を更新
                    if((dist < min_dist) && (dist >= 0))
                    {
                        min_dist = dist;
                        future_x = predict_ped_x;
                        future_y = predict_ped_y;
                    }
                    else if(dist < 0)
                    {
                        break;
                    }
                }
            }

            // base_link座標系での速度計算のために1秒先の位置を予測
            const double after_ones_x = current_person.pose.position.x + (current_person.twist.linear.x * 1.0);
            const double after_ones_y = current_person.pose.position.y + (current_person.twist.linear.y * 1.0);
            
            // 歩行者の現在位置をbase_footprint座標系に変換
            selected_current_person.id = current_person.id;
            transform_ped_pose(current_person.pose.position.x, current_person.pose.position.y, selected_current_person);

            // 速度計算のための1秒先の位置もbase_footprint座標系に変換
            pedestrian_msgs::PersonState after_ones_person;
            transform_ped_pose(after_ones_x, after_ones_y, after_ones_person);

            // base_footprint座標系での速度を計算
            selected_current_person.twist.linear.x = after_ones_person.pose.position.x - selected_current_person.pose.position.x;
            selected_current_person.twist.linear.y = after_ones_person.pose.position.y - selected_current_person.pose.position.y;

            // base_footprint座標系での歩行者の現在データを格納
            selected_current_people.people_states.push_back(selected_current_person);
    
            // ロボットの進行方向の方位を計算
            const double robot_theta = calc_direction(tmp_robot_x_, tmp_robot_y_, robot_odom_.pose.pose.position.x, robot_odom_.pose.pose.position.y);

            // ロボットに対する歩行者の将来位置の方位を計算
            const double person_theta = calc_direction(robot_odom_.pose.pose.position.x, robot_odom_.pose.pose.position.y, future_x, future_y);

            // ロボットに対する歩行者の将来位置の方位を計算
            const double theta = person_theta - robot_theta;

            // ロボットの進行方向前方に移動する歩行者のみ考慮
            // if((theta <= M_PI/2) && (theta >= -M_PI/2))
            // {
            //     // 歩行者の予測データを格納
            //     future_person.id = current_person.id;
            //     transform_ped_pose(future_x, future_y, future_person);  //歩行者の将来位置をbase_footprint座標系に変換
            //     future_person.twist.linear.x = selected_current_person.twist.linear.x;
            //     future_person.twist.linear.y = selected_current_person.twist.linear.y;

            //     future_people.people_states.push_back(future_person);
            // }

            // 歩行者の予測データを格納
            future_person.id = current_person.id;
            transform_ped_pose(future_x, future_y, future_person);  //歩行者の将来位置をbase_footprint座標系に変換
            future_person.twist.linear.x = selected_current_person.twist.linear.x;
            future_person.twist.linear.y = selected_current_person.twist.linear.y;

            future_people.people_states.push_back(future_person);
        }
    }

    if(flag_frame_change_ == true)
    {
        pub_selected_current_people_states_ .publish(selected_current_people);
        pub_future_people_states_.publish(future_people);

    }
}

// 歩行者の位置情報を可視化
void PedestrianStatePredictor::visualize_people_pose(const pedestrian_msgs::PeopleStates& people, const ros::Publisher& pub_people_poses, ros::Time now)
{
    geometry_msgs::Pose person_pose;        // 歩行者1人の位置情報
    geometry_msgs::PoseArray people_poses;  // 全歩行者の位置情報
    people_poses.header.stamp = now;
    people_poses.header.frame_id  = people.header.frame_id;

    // msg型を pedestrian_msgs/PeopleStates から geometry_msgs/PoseArray に変更
    for(const auto& person : people.people_states)
    {
        person_pose.position.x = person.pose.position.x;
        person_pose.position.y = person.pose.position.y;
        person_pose.orientation = person.pose.orientation;
        people_poses.poses.push_back(person_pose);
    }

    pub_people_poses.publish(people_poses);
}

// 歩行者情報を更新
void PedestrianStatePredictor::update_ped_state()
{
    pedestrian_msgs::PeopleStates current_people;
    pedestrian_msgs::PeopleStates future_people;
    pedestrian_msgs::PeopleStates selected_current_people;
    ros::Time now = ros::Time::now();

    // 歩行者データを取得
    get_ped_data(current_people, now);

    // 歩行者の現在位置（odom）の可視化
    if(visualize_current_people_poses_)
        visualize_people_pose(current_people, pub_current_ped_poses_, now);

    // 歩行者の将来位置を予測
    predict_future_ped_states(current_people, future_people, selected_current_people, now);

    if(flag_frame_change_ == true)
    {
        // ロボットに近い歩行者の現在位置（base_footprint）の可視化
        if(visualize_selected_current_people_poses_)
            visualize_people_pose(selected_current_people, pub_selected_current_ped_poses_, now);

        // 歩行者の将来位置（予測）の可視化
        if(visualize_future_people_poses_)
            visualize_people_pose(future_people, pub_future_ped_poses_, now);
    }

    // ped_states_の配列のうち取得済みのデータ（配列の先頭の要素）を削除
    // これをしないと，front() でデータを取得する際，同じデータしか取得できない
    ped_states_.pop();

    // ロボットの位置を格納
    tmp_robot_x_ = robot_odom_.pose.pose.position.x;
    tmp_robot_y_ = robot_odom_.pose.pose.position.y;
}

//メイン文で実行する関数
void PedestrianStatePredictor::process()
{
    ros::Rate loop_rate(hz_);
    tf2_ros::TransformListener tf_listener(tf_buffer_);

    while(ros::ok())
    {
        if((flag_ped_states_ == true) && (flag_robot_odom_ == true))
        {
            update_ped_state();
        }

        // msgの受け取り判定用flagをfalseに戻す
        flag_ped_states_ = false;
        flag_robot_odom_ = false;

        ros::spinOnce();
        loop_rate.sleep();
    }
}