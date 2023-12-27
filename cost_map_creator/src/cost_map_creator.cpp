#include "cost_map_creator/cost_map_creator.h"

CostMapCreator::CostMapCreator():private_nh_("~")
{
    // param
    private_nh_.param("hz", hz_, {10});
    // private_nh_.param("visualize_current_people_poses", visualize_current_people_poses_, {false});
    // private_nh_.param("visualize_future_people_poses", visualize_future_people_poses_, {false});
    private_nh_.param("people_frame", people_frame_, {"base_footprint"});
    private_nh_.param("cost_map_frame", cost_map_frame_, {"base_footprint"});
    private_nh_.param("map_size", map_size_, {5.0});
    private_nh_.param("map_reso", map_reso_, {0.025});
    private_nh_.param("ellipse_front_long_max", ellipse_front_long_max_, {1.0});
    private_nh_.param("ellipse_back_long_max", ellipse_back_long_max_, {0.5});
    private_nh_.param("ellipse_short_max", ellipse_short_max_, {0.5});
    private_nh_.param("weight_distance", weight_distance_, {0.5});
    private_nh_.param("weight_speed", weight_speed_, {0.5});
    private_nh_.param("ped_speed_max", ped_speed_max_, {1.5});
    private_nh_.param("min_cost", min_cost_, {10});

    // private_nh_.param("predict_dist_border", predict_dist_border_, {8.0});
    // private_nh_.param("predict_time_resolution", predict_time_resolution_, {2.5});
    // private_nh_.param("tmp_robot_x", tmp_robot_x_, {0.0});
    // private_nh_.param("tmp_robot_y", tmp_robot_y_, {0.0});

    // subscriber
    sub_current_people_states_ = nh_.subscribe("/selected_current_people_states", 1, &CostMapCreator::current_people_states_callback, this, ros::TransportHints().reliable().tcpNoDelay());
    sub_future_people_states_ = nh_.subscribe("/future_people_states", 1, &CostMapCreator::future_people_states_callback, this, ros::TransportHints().reliable().tcpNoDelay());

    // publisher
    pub_cost_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("/cost_map", 1);

    // debug

    // --- 基本設定（コストマップ） ---
    // header
    cost_map_.header.frame_id = cost_map_frame_;
    // info
    cost_map_.info.resolution = map_reso_;
    cost_map_.info.width      = int(round(map_size_/map_reso_));
    cost_map_.info.height     = int(round(map_size_/map_reso_));
    cost_map_.info.origin.position.x = -map_size_/2.0;
    cost_map_.info.origin.position.y = -map_size_/2.0;
    // data
    cost_map_.data.reserve(cost_map_.info.width * cost_map_.info.height);

    // --- 基本設定（personマップ） ---
    // header
    person_map_.header.frame_id = cost_map_frame_;
    // info
    person_map_.info.resolution = map_reso_;
    person_map_.info.width      = int(round(map_size_/map_reso_));
    person_map_.info.height     = int(round(map_size_/map_reso_));
    person_map_.info.origin.position.x = -map_size_/2.0;
    person_map_.info.origin.position.y = -map_size_/2.0;
    // // data
    person_map_.data.reserve(person_map_.info.width * person_map_.info.height);
}

// 現在の歩行者情報のコールバック関数
void CostMapCreator::current_people_states_callback(const pedestrian_msgs::PeopleStatesConstPtr& msg)
{
    current_people_states_.emplace(msg);
    flag_current_people_states_ = true;
}

// 予測した歩行者情報のコールバック関数
void CostMapCreator::future_people_states_callback(const pedestrian_msgs::PeopleStatesConstPtr& msg)
{
    future_people_states_.emplace(msg);
    flag_future_people_states_ = true;

}

// マップの初期化(すべて「未知」にする)
void CostMapCreator::init_map(nav_msgs::OccupancyGrid& map)
{
    map.data.clear();

    // マップサイズを計算
    const int size = map.info.width * map.info.height;

    for(int i=0; i<size; i++)
    {
        map.data.push_back(-1);  //「未知」にする
    }
}

// 速さを計算
double CostMapCreator::calc_speed(const double linear_x, const double linear_y)
{
    const double abs_linear_x = abs(linear_x);
    const double abs_linear_y = abs(linear_y);

    return hypot(abs_linear_x, abs_linear_y);
}

// 楕円の長軸の長さのパラメーターを計算
double CostMapCreator::calc_ellipse_long_param(const pedestrian_msgs::PersonState& person)
{
    // ロボットからの距離を計算
    const double dist = hypot(person.pose.position.x, person.pose.position.y);
    const double normalize_dist = dist / map_size_;  // 正規化

    // 歩行者の速さを計算
    const double speed = calc_speed(person.twist.linear.x, person.twist.linear.y);
    const double normalize_speed = speed / ped_speed_max_;  // 正規化

    return weight_distance_*normalize_dist + weight_speed_*normalize_speed;
}

// 楕円の短軸の長さのパラメーターを計算
double CostMapCreator::calc_ellipse_short_param(const double x, const double y)
{
    // ロボットからの距離を計算
    const double dist = hypot(x, y);
    const double normalize_dist = dist / map_size_;  // 正規化

    return normalize_dist;
}

// 方位を計算
double CostMapCreator::calc_direction(const double x, const double y)
{
    const double theta = atan2(y, x);

    return normalize_angle(theta);
}

// 適切な角度(-M_PI ~ M_PI)を返す
double CostMapCreator::normalize_angle(double theta)
{
    if(theta > M_PI)
        theta -= 2.0 * M_PI;
    if(theta < -M_PI)
        theta += 2.0 * M_PI;

    return theta;
}

// マップ内の場合、trueを返す
bool CostMapCreator::is_in_map(nav_msgs::OccupancyGrid& map, const double x, const double y)
{
    const int index_x = int(round((x - map.info.origin.position.x) / map.info.resolution));
    const int index_y = int(round((y - map.info.origin.position.y) / map.info.resolution));

    if((index_x < map.info.width) && (index_y < map.info.height))
        return true;
    else
        return false;
}

// 座標からグリッドのインデックスを返す
int CostMapCreator::xy_to_grid_index(nav_msgs::OccupancyGrid& map, const double x, const double y)
{
    const int index_x = int(round((x - map.info.origin.position.x) / map.info.resolution));
    const int index_y = int(round((y - map.info.origin.position.y) / map.info.resolution));

    return index_x + (index_y * cost_map_.info.width);
}

// 長軸方向のマスをカウント
int CostMapCreator::count_grid(std::vector<Coordinate>& side, const double start_x, const double start_y, const double length, const double theta)
{
    const double count_reso = 0.01;      // 座標を計算する距離の刻み幅
    Coordinate next = {start_x, start_y};  // 計算後の座標格納用

    next.x = start_x + (count_reso * cos(theta));
    next.y = start_y + (count_reso * sin(theta));

    // グリッドのインデックスが変わったかの判定用（刻み幅）
    double change_reso_x = map_reso_;
    double change_reso_y = map_reso_;

    // xyが増加していればtrue
    bool flag_change_reso_x = true;
    bool flag_change_reso_y = true;

    // xの増減の仕方を確認
    if(next.x <= start_x)
    {
        change_reso_x = -1 * map_reso_;
        flag_change_reso_x = false;
    }
    
    // yの増減の仕方を確認
     if(next.y <= start_y)
    {
        change_reso_y = -1 * map_reso_;
        flag_change_reso_y = false;
    }

    // グリッドのインデックスが変わったかの判定用（座標）
    double change_border_x = start_x + change_reso_x;
    double change_border_y = start_y + change_reso_y;

    int grid_counter = 0;

    // マスをカウント
    for(double l=count_reso; l <= length; l+=count_reso)
    {
        // 座標を計算
        next.x = start_x + (l * cos(theta));
        next.y = start_y + (l * sin(theta));

        // グリッドのインデックスが変わったかの判定
        if((flag_change_reso_x == true) && (flag_change_reso_y == true))
        {
            if((next.x >= change_border_x) && (next.y >= change_border_y))
            {
                // 座標を格納
                side.push_back(next);

                // インデックスが変わったかの判定用の座標を更新
                change_border_x += change_reso_x;
                change_border_y += change_reso_y;

                grid_counter++;
            }
            else if(next.x >= change_border_x)
            {
                // 座標を格納
                side.push_back(next);

                // インデックスが変わったかの判定用の座標を更新
                change_border_x += change_reso_x;

                grid_counter++;
            }
            else if(next.y >= change_border_y)
            {
                // 座標を格納
                side.push_back(next);

                // インデックスが変わったかの判定用の座標を更新
                change_border_y += change_reso_y;

                grid_counter++;
            }
        }
        else if((flag_change_reso_x == true) && (flag_change_reso_y == false))
        {
            if((next.x >= change_border_x) && (next.y <= change_border_y))
            {
                // 座標を格納
                side.push_back(next);

                // インデックスが変わったかの判定用の座標を更新
                change_border_x += change_reso_x;
                change_border_y += change_reso_y;

                grid_counter++;
            }
            else if(next.x >= change_border_x)
            {
                // 座標を格納
                side.push_back(next);

                // インデックスが変わったかの判定用の座標を更新
                change_border_x += change_reso_x;

                grid_counter++;
            }
            else if(next.y <= change_border_y)
            {
                // 座標を格納
                side.push_back(next);

                // インデックスが変わったかの判定用の座標を更新
                change_border_y += change_reso_y;

                grid_counter++;
            }
        }
        else if((flag_change_reso_x == false) && (flag_change_reso_y == true))
        {
            if((next.x <= change_border_x) && (next.y >= change_border_y))
            {
                // 座標を格納
                side.push_back(next);

                // インデックスが変わったかの判定用の座標を更新
                change_border_x += change_reso_x;
                change_border_y += change_reso_y;

                grid_counter++;
            }
            else if(next.x <= change_border_x)
            {
                // 座標を格納
                side.push_back(next);

                // インデックスが変わったかの判定用の座標を更新
                change_border_x += change_reso_x;

                grid_counter++;
            }
            else if(next.y >= change_border_y)
            {
                // 座標を格納
                side.push_back(next);

                // インデックスが変わったかの判定用の座標を更新
                change_border_y += change_reso_y;

                grid_counter++;
            }
        }
        else if((flag_change_reso_x == false) && (flag_change_reso_y == false))
        {
            if((next.x <= change_border_x) && (next.y <= change_border_y))
            {
                // 座標を格納
                side.push_back(next);

                // インデックスが変わったかの判定用の座標を更新
                change_border_x += change_reso_x;
                change_border_y += change_reso_y;

                grid_counter++;
            }
            else if(next.x <= change_border_x)
            {
                // 座標を格納
                side.push_back(next);

                // インデックスが変わったかの判定用の座標を更新
                change_border_x += change_reso_x;

                grid_counter++;
            }
            else if(next.y <= change_border_y)
            {
                // 座標を格納
                side.push_back(next);

                // インデックスが変わったかの判定用の座標を更新
                change_border_y += change_reso_y;

                grid_counter++;
            }
        }
    }

    return grid_counter;
}


// ロボットと歩行者の間の距離を計算
// double CostMapCreator::calc_distance(const double robot_x, const double robot_y, const double person_x, const double person_y)
// {
//     const double dx = person_x - robot_x;
//     const double dy = person_y - robot_y;

//     return hypot(dx, dy);
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

// 歩行者1人のみ考慮したコストマップを作成
void CostMapCreator::create_person_cost_map(const pedestrian_msgs::PersonState& current_person, const pedestrian_msgs::PersonState& future_person, int& min_index, int& max_index)
{
    // 走行コストの楕円の軸の長さを計算
    const double long_param = calc_ellipse_long_param(current_person);
    const double ellipse_front_long = ellipse_front_long_max_ * long_param;                                                                      // 楕円の長軸（前方）
    const double ellipse_back_long = ellipse_back_long_max_ * long_param;                                                                        // 楕円の長軸（後方）
    const double ellipse_short = ellipse_short_max_ * calc_ellipse_short_param(current_person.pose.position.x, current_person.pose.position.y);  // 楕円の短軸

    // 歩行者の進行方向を計算
    const double theta = calc_direction(current_person.twist.linear.x, current_person.twist.linear.y);

    // 予測した歩行者の将来位置のグリッドを占有に変える
    if(is_in_map(person_map_, future_person.pose.position.x, future_person.pose.position.y))
    {
        const int grid_index = xy_to_grid_index(person_map_, future_person.pose.position.x, future_person.pose.position.y);
        person_map_.data[grid_index] = 100;  // 占有にする
    }

    // 長軸方向のグリッドを探索
    std::vector<Coordinate> long_side;
    const int grid_size = count_grid(long_side, future_person.pose.position.x, future_person.pose.position.y, ellipse_front_long, theta);

    // 探索した長軸方向のグリッドにコストを割り当てる
    const double front_long_cost_reso = (100 - min_cost_) / grid_size;
    double front_long_cost = 100;

    for(const auto& front_point : long_side)
    {
        // コストを計算
        front_long_cost -= front_long_cost_reso;

        // 対応するグリッドがマップ内であれば，コストを割り当て
        if(is_in_map(person_map_, front_point.x, front_point.y))
        {
            const int grid_index = xy_to_grid_index(person_map_, front_point.x, front_point.y);

            // すでに割り当てられているコストより大きければ更新
            if(person_map_.data[grid_index] < front_long_cost)
                person_map_.data[grid_index] = front_long_cost;

            // コストを割り当てたindexの最小値と最大値を更新
            if(grid_index < min_index)
                min_index = grid_index;

            if(grid_index > max_index)
                max_index = grid_index;

        }

        // 垂直方向に関しても探索
    }
}

// person_map_のコストをコストマップにコピー
void CostMapCreator::copy_cost(const int min_index, const int max_index)
{
    // コストマップに現在割り当てられているコストより大きければ，コストを更新
    for(int i=min_index; i<=max_index; i++)
    {
        if(person_map_.data[i] > cost_map_.data[i])
            cost_map_.data[i] = person_map_.data[i];
    }
}

void CostMapCreator::create_cost_map()
{
    // コストマップの初期化
    init_map(cost_map_);

    pedestrian_msgs::PersonState current_person;

    const auto current_people = current_people_states_.front();
    const auto future_people = future_people_states_.front();
    // bool flag2 = true;
    
    // 予測した歩行者の将来位置に対して走行コストを計算
    for(const auto& future_person : future_people->people_states)
    {
        // ROS_INFO_STREAM("--- future ---");
        // ROS_INFO_STREAM("id : " << future_person.id);
        // ROS_INFO_STREAM("position_x : " << future_person.pose.position.x);
        // ROS_INFO_STREAM("linear_x" << future_person.twist.linear.x);

        // 対応する現在の歩行者情報を探索
        for(const auto& person : current_people->people_states)
        {
            if(person.id == future_person.id)
            {
                current_person = person;
                flag_ped_data_matching_ = true;

                const double speed = calc_speed(current_person.twist.linear.x, current_person.twist.linear.y);
                // ROS_INFO_STREAM("--- current ---");
                // ROS_INFO_STREAM("id : " << current_person.id);
                // ROS_INFO_STREAM("position_x : " << current_person.pose.position.x);
                // ROS_INFO_STREAM("linear_x : " << current_person.twist.linear.x);
                // ROS_INFO_STREAM("speed : " << speed);

                // パーソンマップの初期化
                init_map(person_map_);

                // コストを割り当てるindexの最小値と最大値を初期化
                int min_index = cost_map_.info.width * cost_map_.info.height;  // マップのindexの最大値
                int max_index = 0;                                             // マップのindexの最小値

                // 歩行者1人のみ考慮したコストマップを作成
                create_person_cost_map(current_person, future_person, min_index, max_index);

                // person_map_のコストをコストマップにコピー
                copy_cost(min_index, max_index);

                break;
            }
        }

        // 対応する歩行者情報を見つけられなかった場合はメッセージを表示
        if(flag_ped_data_matching_ == false)
            ROS_WARN_STREAM("!!! No matching ped_data !!!");

        // 歩行者情報のマッチング確認用flagをfalseに戻す
        flag_ped_data_matching_ = false;
    }
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

    pub_cost_map_.publish(cost_map_);

    // current_people_states_とfuture_people_states_の配列のうち取得済みのデータ（配列の先頭の要素）を削除
    // これをしないと，front() でデータを取得する際，同じデータしか取得できない
    current_people_states_.pop();
    future_people_states_.pop();
    // while(!current_people_states_.empty())
    // {
    //     current_people_states_.pop();
    //     ROS_INFO_STREAM("clean!");
    // }

    // while(!future_people_states_.empty())
    // {
    //     future_people_states_.pop();
    //     ROS_INFO_STREAM("kirei!");
    // }

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

    while(ros::ok())
    {
        if((flag_current_people_states_ == true) && (flag_future_people_states_ == true))
        {
            //ROS_INFO_STREAM("get ped_states!");  // デバック用
            create_cost_map();
        }

        // msgの受け取り判定用flagをfalseに戻す
        flag_current_people_states_ = false;
        flag_future_people_states_ = false;
        // flag_robot_odom_ = false;

        ros::spinOnce();
        loop_rate.sleep();
    }
}