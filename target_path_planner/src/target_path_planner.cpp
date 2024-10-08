#include "target_path_planner/target_path_planner.h"

TargetPathPlanner::TargetPathPlanner():private_nh_("~")
{
    private_nh_.param("hz", hz_, {10});
    private_nh_.param("node_frame", node_frame_, {"base_footprint"});
    private_nh_.param("path_frame", path_frame_, {"odom"});
    private_nh_.param("goal_frame", goal_frame_, {"odom"});
    private_nh_.param("goal_tolerance", goal_tolerance_, {0.5});
    private_nh_.param("finish_dist", finish_dist_, {4.9});
    private_nh_.param("max_vel", max_vel_, {1.2});
    private_nh_.param("max_yawrate", max_yawrate_, {1.0});
    private_nh_.param("max_speed", max_speed_, {3.0});
    private_nh_.param("speed_reso", speed_reso_, {0.1});
    private_nh_.param("max_steer_angle", max_steer_angle_, {20});
    private_nh_.param("steer_angle_reso", steer_angle_reso_, {1.0});
    private_nh_.param("tread", tread_, {0.5});
    private_nh_.param("path_reso", path_reso_, {0.05});
    private_nh_.param("theta_reso", theta_reso_, {0.01});
    private_nh_.param("search_step", search_step_, {10});
    private_nh_.param("weight_heading", weight_heading_, {0.1});
    private_nh_.param("weight_cost_map", weight_cost_map_, {0.9});
    private_nh_.param("min_cost", min_cost_, {10.0});
    private_nh_.param("tmp_x", tmp_x_, {0.0});
    private_nh_.param("tmp_y", tmp_y_, {0.0});
    private_nh_.param("tmp_yaw", tmp_yaw_, {0.0});

    //Subscriber
    sub_cost_map_ = nh_.subscribe("/cost_map", 1, &TargetPathPlanner::cost_map_callback, this, ros::TransportHints().reliable().tcpNoDelay());
    sub_local_goal_ = nh_.subscribe("/local_goal", 1, &TargetPathPlanner::local_goal_callback, this, ros::TransportHints().reliable().tcpNoDelay());

    //Publisher
    pub_target_path_ = nh_.advertise<nav_msgs::Path>("/target_path", 1);
}

// cost_mapのコールバック関数
void TargetPathPlanner::cost_map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    cost_map_ = *msg;
    flag_cost_map_ = true;
}

// local_goalコールバック関数
void TargetPathPlanner::local_goal_callback(const geometry_msgs::PointStampedConstPtr& msg)
{
    geometry_msgs::TransformStamped transform;

    try
    {
        transform = tf_buffer_.lookupTransform(node_frame_, goal_frame_, ros::Time(0));
        flag_local_goal_ = true;
    }
    catch(tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
        flag_local_goal_ = false;
        return;
    }

    tf2::doTransform(*msg, local_goal_, transform);
}

// 機構的制約内で旋回可能な角度を計算
double TargetPathPlanner::calc_rad_with_steer()
{
    double max_omega = 0.0;
    const double max_steer_angle_rad = max_steer_angle_ / 180 * M_PI;

    // ステア角を探索
    for(double i=-max_steer_angle_; i<max_steer_angle_; i+=steer_angle_reso_)
    {
        // ステア角の最大値をdegからradに変換
        const double steer_angle = i / 180 * M_PI;

        for(double v_r=speed_reso_; v_r<=max_speed_; v_r+=speed_reso_)
        {
            for(double v_l=speed_reso_; v_l<=max_speed_; v_l+=speed_reso_)
            {
                const double v_x = v_l*cos(max_steer_angle_rad)/2 + v_r*cos(steer_angle)/2;
                const double v_y = v_l*sin(max_steer_angle_rad)/2 + v_r*sin(steer_angle)/2;

                // v_xとv_yが制約を満たしていたら，最大旋回速度を探索
                const double v = hypot(v_x, v_y);
                const double speed_border = 0.01;
                if((abs(v - max_vel_) <= speed_border) && (v_r > v_l))
                {
                    double r_l = sin(steer_angle) / sin(max_steer_angle_rad-steer_angle) * tread_;
                    double r_r = sin(max_steer_angle_rad) / sin(max_steer_angle_rad-steer_angle) * tread_;

                    if((r_l < 0.0) || (r_r < 0.0))
                        continue;
                    
                    if(abs(v_r/r_r - v_l/r_l) >= 0.1)
                        continue;

                    double omega = abs(v_r-v_l) / abs(r_r-r_l);

                    if(omega > max_omega)
                    {
                        max_omega = omega;
                        ROS_INFO_STREAM(omega << "  " << steer_angle << "  " << v_r << "  " << v_l << "  " << v << "  " << r_l << "  " << r_r);


                    }
                }
            }
        }
    }

    ROS_INFO_STREAM("max: " << max_omega);

    const double  max_rad = max_omega / max_vel_ * path_reso_;

    return max_rad;

}

double TargetPathPlanner::calc_rad_no_steer()
{
    // 旋回で変わる方位の最大値を計算
    const double  max_rad = max_yawrate_ / max_vel_ * path_reso_;

    return max_rad;
}

// pathの終端がlocal_goalから一定距離内に到達したか確認
// goalするまでfalseを返す
bool TargetPathPlanner::is_goal_check(const double x, const double y)
{
    // local_goalまでの距離を計算
    double dx = local_goal_.point.x - x;
    double dy = local_goal_.point.y - y;
    double dist_to_goal = hypot(dx, dy);

    if(dist_to_goal > goal_tolerance_)
        return false;
    else
        return true;
}

// pathの終端がロボットから一定距離離れたか確認
// 一定距離離れるまでfalseを返す
bool TargetPathPlanner::is_finish_check(const double x, const double y)
{
    // ロボットからの距離を計算
    double dist = hypot(x, y);

    if(dist < finish_dist_)
        return false;
    else
        return true;
}

// 評価関数を計算する
double TargetPathPlanner::calc_evaluation(const std::vector<State>& traj)
{
    double heading_value = weight_heading_ * calc_heading_eval(traj);
    double cost_map_value = weight_cost_map_ * calc_cost_map_eval(traj);

    double total_score = heading_value - cost_map_value;

    return total_score;
}

// heading（1項目）の評価関数を計算する
// 大きいほど良い
double TargetPathPlanner::calc_heading_eval(const std::vector<State>& traj)
{
    //最終時刻でのロボットの方向
    double theta = traj.back().yaw;

    //最終時刻での位置に対するゴール方向
    double goal_theta = atan2(local_goal_.point.y - traj.back().y, local_goal_.point.x - traj.back().x);

    //ゴールまでの方位差分
    double target_theta = 0.0;  //初期化
    if(goal_theta > theta)
        target_theta = goal_theta - theta;
    else if(goal_theta < theta)
        target_theta = theta -goal_theta;

    //headingの評価値を計算
    double heading_eval = (M_PI - abs(normalize_angle(target_theta))) / M_PI;  //正規化

    return heading_eval;
}

// cost_map（2項目）の評価関数を計算する
// 大きいほど良くない
double TargetPathPlanner::calc_cost_map_eval(const std::vector<State>& traj)
{
    double total_cost = 0;  // 通過した場所の走行コストの合計値

    for(auto& state : traj)
    {
        const int grid_index = xy_to_grid_index(state.x, state.y);
        double cost = cost_map_.data[grid_index];

        total_cost += cost;
    }

    int traj_size = traj.size();
    if(traj_size == 0)
        traj_size = 1;  // ゼロ割り防止
    
    double cost_map_eval = total_cost / (min_cost_ * traj_size);

    return cost_map_eval;
}

// 適切な角度(-M_PI ~ M_PI)を返す
double TargetPathPlanner::normalize_angle(double theta)
{
    if(theta > M_PI)
        theta -= 2.0 * M_PI;
    if(theta < -M_PI)
        theta += 2.0 * M_PI;

    return theta;
}

// 座標からグリッドのインデックスを返す
int TargetPathPlanner::xy_to_grid_index(const double x, const double y)
{
    const int index_x = int(floor((x - cost_map_.info.origin.position.x) / cost_map_.info.resolution));
    const int index_y = int(floor((y - cost_map_.info.origin.position.y) / cost_map_.info.resolution));

    return index_x + (index_y * cost_map_.info.width);
}

// 次のノードを探索
void TargetPathPlanner::search_node(const double max_rad, std::vector<State>& nodes)
{
    // 最適なノードの情報格納用
    double opt_x = 0.0;
    double opt_y = 0.0;
    double opt_yaw = 0.0;

    double max_score = -1000.0;  //評価値の最大値格納用

    for(double theta=-max_rad; theta<=max_rad;theta+=theta_reso_)
    {
        std::vector<State> traj;  // 軌跡格納用
        State state = {0.0, 0.0, 0.0}; // 軌跡生成用の仮想ロボット

        // yaw角を計算
        double yaw = tmp_yaw_ + theta;
        yaw = normalize_angle(yaw);

        // 軌跡を生成
        for(int step=1; step<=search_step_; step++)
        {
            state.x = tmp_x_ + (path_reso_ * step * cos(yaw));
            state.y = tmp_y_ + (path_reso_ * step * sin(yaw));
            state.yaw = yaw;

            traj.push_back(state);
        }

        // 軌跡に対する評価値を計算
        double score = calc_evaluation(traj);

        //評価値が一番大きいデータの探索
        if(score > max_score)
        {
            max_score = score;

            opt_x = traj[0].x;
            opt_y = traj[0].y;
            opt_yaw = traj[0].yaw;        
        }
    }

    // ノードの探索結果を格納
    State opt_node;
    opt_node.x = opt_x;
    opt_node.y = opt_y;
    opt_node.yaw = opt_yaw;
    nodes.push_back(opt_node);
}

// 目標軌道を生成
void TargetPathPlanner::create_path(const double max_rad)
{
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = path_frame_;

    std::vector<State> nodes;  // ノード情報格納用

    // 初期値を代入
    State node = {0.0, 0.0, 0.0};
    nodes.push_back(node);

    // 初期化
    tmp_x_ = 0.0;
    tmp_y_ = 0.0;
    tmp_yaw_ = 0.0;
    flag_goal_check_ = false;

    int step_counter = 0;

    // local_goalに近づくまでノードを探索
    while(flag_goal_check_ == false)
    {
        search_node(max_rad, nodes);
        step_counter++;

        // 終了判定
        if(is_goal_check(nodes.back().x, nodes.back().y) == true)
            flag_goal_check_ = true;
        else if(step_counter > 100)
            flag_goal_check_ = true;
        else if(is_finish_check(nodes.back().x, nodes.back().y) == true)
            flag_goal_check_ = true;
        
        // 今回の探索結果を格納
        tmp_x_ = nodes.back().x;
        tmp_y_ = nodes.back().y;
        tmp_yaw_ = nodes.back().yaw;
    }
    
    // 探索し終わったノード情報から目標軌道を生成
    transform_node_to_path(nodes, path);

    if(flag_frame_change_ == true)
        pub_target_path_.publish(path);
}

// ノード情報からパスを生成
// base_footprintからodomに変換
void TargetPathPlanner::transform_node_to_path(const std::vector<State>& nodes, nav_msgs::Path& path)
{
    geometry_msgs::TransformStamped tf;
    
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = path_frame_;

    for(auto& node : nodes)
    {
        geometry_msgs::Pose node_before;    // 座標変換前のノード格納用
        geometry_msgs::Pose node_after;     // 座標変換後のノード位置格納用

        node_before.position.x = node.x;
        node_before.position.y = node.y;

        // ノードをbase_footprintからodomに変更
        try
        {
            tf = tf_buffer_.lookupTransform(path_frame_, node_frame_, ros::Time(0));
            flag_frame_change_ = true;

            // ノードをdoTransformで座標変換
            tf2::doTransform(node_before, node_after, tf);

            pose.pose.position = node_after.position;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 0.0;

            path.poses.push_back(pose);
        }
        catch(tf2::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            flag_frame_change_ = false;
            return;
        }
    }
}

//メイン文で実行する関数
void TargetPathPlanner::process()
{
    ros::Rate loop_rate(hz_);
    tf2_ros::TransformListener tf_listener(tf_buffer_);

    double max_rad = 0.0;

    if(max_steer_angle_ == 0.0)
    {
        max_rad = calc_rad_no_steer();
    }
    else
    {
        max_rad = calc_rad_with_steer();
    }

    ROS_INFO_STREAM("max_rad : " << max_rad);

    while(ros::ok())
    {
        if((flag_cost_map_ == true) && (flag_local_goal_ == true))
        {
            create_path(max_rad);
        }

        // msgの受け取り判定用flagをfalseに戻す
        flag_cost_map_ = false;
        flag_local_goal_ = false;

        ros::spinOnce();
        loop_rate.sleep();
    }
}