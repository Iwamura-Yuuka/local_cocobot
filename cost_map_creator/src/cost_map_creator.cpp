#include "cost_map_creator/cost_map_creator.h"

CostMapCreator::CostMapCreator():private_nh_("~")
{
    // param
    private_nh_.param("flag_cost", flag_cost_, {false});
    private_nh_.param("hz", hz_, {10});
    private_nh_.param("people_frame", people_frame_, {"base_footprint"});
    private_nh_.param("cost_map_frame", cost_map_frame_, {"base_footprint"});
    private_nh_.param("map_size", map_size_, {5.0});
    private_nh_.param("map_reso", map_reso_, {0.05});
    private_nh_.param("ellipse_front_long_max", ellipse_front_long_max_, {1.0});
    private_nh_.param("ellipse_front_long_min", ellipse_front_long_min_, {0.5});
    private_nh_.param("ellipse_back_long_max", ellipse_back_long_max_, {0.5});
    private_nh_.param("ellipse_back_long_min", ellipse_back_long_min_, {0.3});
    private_nh_.param("ellipse_short_max", ellipse_short_max_, {0.5});
    private_nh_.param("ellipse_short_min", ellipse_short_min_, {0.3});
    private_nh_.param("margin", margin_, {0.25});
    private_nh_.param("weight_distance", weight_distance_, {0.5});
    private_nh_.param("weight_speed", weight_speed_, {0.5});
    private_nh_.param("ped_speed_max", ped_speed_max_, {1.5});
    private_nh_.param("count_reso", count_reso_, {0.02});
    private_nh_.param("min_cost", min_cost_, {10.0});

    // subscriber
    sub_current_people_states_ = nh_.subscribe("/selected_current_people_states", 1, &CostMapCreator::current_people_states_callback, this, ros::TransportHints().reliable().tcpNoDelay());
    sub_future_people_states_ = nh_.subscribe("/future_people_states", 1, &CostMapCreator::future_people_states_callback, this, ros::TransportHints().reliable().tcpNoDelay());

    // publisher
    pub_cost_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("/cost_map", 1);

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
    // data
    person_map_.data.reserve(person_map_.info.width * person_map_.info.height);
}

// 現在の歩行者情報のコールバック関数
void CostMapCreator::current_people_states_callback(const pedestrian_msgs::PeopleStatesConstPtr& msg)
{
    while(current_people_states_.size() > 0)
    {
        // current_people_states_の配列のうち取得済みのデータ（配列の先頭の要素）を削除
        // これをしないと，front() でデータを取得する際，同じデータしか取得できない
        current_people_states_.pop();
    }
    
    current_people_states_.emplace(msg);
    flag_current_people_states_ = true;
}

// 予測した歩行者情報のコールバック関数
void CostMapCreator::future_people_states_callback(const pedestrian_msgs::PeopleStatesConstPtr& msg)
{
    while(future_people_states_.size() > 0)
    {
        // future_people_states_の配列のうち取得済みのデータ（配列の先頭の要素）を削除
        // これをしないと，front() でデータを取得する際，同じデータしか取得できない
        future_people_states_.pop();
    }

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
    const int index_x = int(floor((x - map.info.origin.position.x) / map.info.resolution));
    const int index_y = int(floor((y - map.info.origin.position.y) / map.info.resolution));

    if((index_x < map.info.width) && (index_y < map.info.height))
        return true;
    else
        return false;
}

// 座標からグリッドのインデックスを返す
int CostMapCreator::xy_to_grid_index(nav_msgs::OccupancyGrid& map, const double x, const double y)
{
    const int index_x = int(floor((x - map.info.origin.position.x) / map.info.resolution));
    const int index_y = int(floor((y - map.info.origin.position.y) / map.info.resolution));

    return index_x + (index_y * map.info.width);
}

// マスを探索
void CostMapCreator::search_grid_size(std::vector<Coordinate>& side, const double start_x, const double start_y, const double length, const double theta)
{
    Coordinate next = {start_x, start_y};  // 計算後の座標格納用

    next.x = start_x + (count_reso_ * cos(theta));
    next.y = start_y + (count_reso_ * sin(theta));

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
    for(double l=count_reso_; l <= length; l+=count_reso_)
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
}

// 走行コストを割り当てるマスをカウント
// 障害物を膨張させた範囲は除く
double CostMapCreator::count_grid(const std::vector<Coordinate> side, std::vector<Coordinate>& cost_side, const double person_x, const double person_y)
{
    double counter = 0;  // マスをカウントする用

    Coordinate cost_point = {person_x, person_y};  // 走行コストを割り当てるの座標格納用

    // マスをカウント
    for(const auto& side_point : side)
    {
        const double dist = calc_distance(person_x, person_y, side_point.x, side_point.y);

        // 膨張させた範囲外ならカウント
        if(dist > margin_)
        {
            // 座標をカウント
            cost_point.x = side_point.x;
            cost_point.y = side_point.y;
            cost_side.push_back(cost_point);

            counter += 1.0;
        }
    }

    return counter;
}

// person_map_にコストを割り当てる
void CostMapCreator::assign_cost_for_person_cost_map(const double x, const double y, const double cost, int& min_index, int& max_index)
{
    const int grid_index = xy_to_grid_index(person_map_, x, y);

    // すでに割り当てられているコストより大きければ更新
    if(person_map_.data[grid_index] < cost)
        person_map_.data[grid_index] = cost;

    // コストを割り当てたindexの最小値と最大値を更新
    if(grid_index < min_index)
            min_index = grid_index;

    if(grid_index > max_index)
        max_index = grid_index;
}

// 歩行者位置までの距離を計算
double CostMapCreator::calc_distance(const double person_x, const double person_y, const double x, const double y)
{
    const double dx = x - person_x;
    const double dy = y - person_y;

    return hypot(dx, dy);
}

// 長軸方向の位置から対応する短軸方向の長さを計算
// a : 長軸の長さ　b : 短軸の長さ
double CostMapCreator::calc_short_side_length(const double x, const double a, const double b)
{
    const double y = b * sqrt(1 - ((x*x) / (a*a)));

    return y;
}

// 長軸方向のグリッドを探索
void CostMapCreator::search_long_side_grid(const double person_x, const double person_y, const double theta, const double ellipse_long_length, const double ellipse_short_length, int& min_index, int& max_index)
{
    std::vector<Coordinate> long_side;
    std::vector<Coordinate> cost_long_side;

    // 長軸方向のマス数を探索
    search_grid_size(long_side, person_x, person_y, ellipse_long_length, theta);

    // 走行コストを割り当てるマスをカウント
    const double long_grid_size = count_grid(long_side, cost_long_side, person_x, person_y);

    double long_grid_size2;  // ゼロ割を防ぐためのもの
    if(long_grid_size == 0)
        long_grid_size2 = 1.0;
    else
        long_grid_size2 = long_grid_size;

    // 探索した長軸方向（前）のグリッドにコストを割り当てる
    const double long_cost_reso = (100 - min_cost_) / long_grid_size2;
    double long_cost = 100;

    for(const auto& long_side_point : long_side)
    {
        const double d = calc_distance(person_x, person_y, long_side_point.x, long_side_point.y);

        // 膨張させた範囲外なら走行コストを計算
        if(d > margin_)
        {
            // コストを計算
            long_cost -= long_cost_reso;

            // 対応するグリッドがマップ内であれば，コストを割り当て
            if(is_in_map(person_map_, long_side_point.x, long_side_point.y))
                assign_cost_for_person_cost_map(long_side_point.x, long_side_point.y, long_cost, min_index, max_index);
        }

        // 垂直方向の長さを計算
        const double dist = calc_distance(person_x, person_y, long_side_point.x, long_side_point.y);
        const double short_side_length = calc_short_side_length(dist, ellipse_long_length, ellipse_short_length);

        // 垂直方向（上）に関して探索
        std::vector<Coordinate> short_side_plus;
        std::vector<Coordinate> cost_short_side_plus;

        // 垂直方向（上）のマス数を探索
        search_grid_size(short_side_plus, long_side_point.x, long_side_point.y, short_side_length, theta-(M_PI/2));

        // 走行コストを割り当てるマスをカウント
        const double short_plus_grid_size = count_grid(short_side_plus, cost_short_side_plus, person_x, person_y);

        double short_plus_grid_size2;  // ゼロ割を防ぐためのもの
        if(short_plus_grid_size == 0)
            short_plus_grid_size2 = 1.0;
        else
            short_plus_grid_size2 = short_plus_grid_size;

        // 探索した短軸方向（上）のグリッドにコストを割り当てる
        const double short_plus_cost_reso = (long_cost - min_cost_) / short_plus_grid_size2;
        double short_plus_cost = long_cost;

        for(const auto& short_plus_point : cost_short_side_plus)
        {
            // コストを計算
            short_plus_cost -= short_plus_cost_reso;

            // 対応するグリッドがマップ内であれば，コストを割り当て
            if(is_in_map(person_map_, short_plus_point.x, short_plus_point.y))
                assign_cost_for_person_cost_map(short_plus_point.x, short_plus_point.y, short_plus_cost, min_index, max_index);
        }

        // 垂直方向（下）に関して探索
        std::vector<Coordinate> short_side_minus;
        std::vector<Coordinate> cost_short_side_minus;

        // 垂直方向（下）のマス数を探索
        search_grid_size(short_side_minus, long_side_point.x, long_side_point.y, short_side_length, theta+(M_PI/2));

        // 走行コストを割り当てるマスをカウント
        const double short_minus_grid_size = count_grid(short_side_minus, cost_short_side_minus, person_x, person_y);

         double short_minus_grid_size2;  // ゼロ割を防ぐためのもの
        if(short_minus_grid_size == 0)
            short_minus_grid_size2 = 1.0;
        else
            short_minus_grid_size2 = short_minus_grid_size;

        // 探索した短軸方向（上）のグリッドにコストを割り当てる
        const double short_minus_cost_reso = (long_cost - min_cost_) / short_minus_grid_size2;
        double short_minus_cost = long_cost;

        for(const auto& short_minus_point : cost_short_side_minus)
        {
            // コストを計算
            short_minus_cost -= short_minus_cost_reso;

            // 対応するグリッドがマップ内であれば，コストを割り当て
            if(is_in_map(person_map_, short_minus_point.x, short_minus_point.y))
                assign_cost_for_person_cost_map(short_minus_point.x, short_minus_point.y, short_minus_cost, min_index, max_index);
        }
    }
}

// 短軸方向のグリッドを探索
void CostMapCreator::search_short_side_grid(const double person_x, const double person_y, const double theta, const double ellipse_short_length, int& min_index, int& max_index)
{
    std::vector<Coordinate> short_side;
    std::vector<Coordinate> cost_short_side;

    // 短軸方向のマス数を探索
    search_grid_size(short_side, person_x, person_y, ellipse_short_length, theta);

    // 走行コストを割り当てるマスをカウント
    const double short_grid_size = count_grid(short_side, cost_short_side, person_x, person_y);

    double short_grid_size2;  // ゼロ割を防ぐためのもの
    if(short_grid_size == 0)
        short_grid_size2 = 1;
    else
        short_grid_size2 = short_grid_size;

    // 探索した長軸方向（前）のグリッドにコストを割り当てる
    const double short_cost_reso = (100 - min_cost_) / short_grid_size2;
    double short_cost = 100;

    for(const auto& short_side_point : cost_short_side)
    {
        // コストを計算
        short_cost -= short_cost_reso;

        // 対応するグリッドがマップ内であれば，コストを割り当て
        if(is_in_map(person_map_, short_side_point.x, short_side_point.y))
            assign_cost_for_person_cost_map(short_side_point.x, short_side_point.y, short_cost, min_index, max_index);
    }
}

// 歩行者1人のみ考慮したコストマップを作成
void CostMapCreator::create_person_cost_map(const pedestrian_msgs::PersonState& current_person, const pedestrian_msgs::PersonState& future_person, int& min_index, int& max_index)
{
    // 走行コストの楕円の軸の長さを計算
    const double long_param = calc_ellipse_long_param(current_person);
    const double ellipse_front_long = ellipse_front_long_min_ + (ellipse_front_long_max_ - ellipse_front_long_min_) * long_param;                                                                      // 楕円の長軸（前方）
    const double ellipse_back_long = ellipse_back_long_min_ + (ellipse_back_long_max_- ellipse_back_long_min_) * long_param;                                                                        // 楕円の長軸（後方）
    const double ellipse_short = ellipse_short_min_ + (ellipse_short_max_ - ellipse_short_min_) * calc_ellipse_short_param(current_person.pose.position.x, current_person.pose.position.y);  // 楕円の短軸

    // 歩行者の進行方向を計算
    const double theta = calc_direction(current_person.twist.linear.x, current_person.twist.linear.y);

    // 予測した歩行者の将来位置のグリッドを占有に変える
    if(is_in_map(person_map_, future_person.pose.position.x, future_person.pose.position.y))
        assign_cost_for_person_cost_map(future_person.pose.position.x, future_person.pose.position.y, 100, min_index, max_index);

    // 歩行者の周りの衝突半径分を占有にする
    expand_obstacle(future_person.pose.position.x, future_person.pose.position.y, min_index, max_index);

    if(flag_cost_ == true)  // trueなら走行コストを設定
    {
        // 長軸方向(前)のグリッドを探索
        search_long_side_grid(future_person.pose.position.x, future_person.pose.position.y, theta, ellipse_front_long, ellipse_short, min_index, max_index);

        // 長軸方向(後)のグリッドを探索
        search_long_side_grid(future_person.pose.position.x, future_person.pose.position.y, theta+M_PI, ellipse_back_long, ellipse_short, min_index, max_index);

        // 短軸方向（上）のグリッドを探索
        search_short_side_grid(future_person.pose.position.x, future_person.pose.position.y, theta-(M_PI/2), ellipse_short, min_index, max_index);

        // 短軸方向（下）のグリッドを探索
        search_short_side_grid(future_person.pose.position.x, future_person.pose.position.y, theta+(M_PI/2), ellipse_short, min_index, max_index);
    }
}

// 歩行者の周りの衝突半径分を占有にする
void CostMapCreator::expand_obstacle(const double person_x, const double person_y, int& min_index, int& max_index)
{
    // 探索を開始する座標を計算
    const double start_x = person_x - margin_;  // x座標
    const double start_y = person_y - margin_;  // y座標

    const double end_x = person_x + margin_;    // x座標
    const double end_y = person_y + margin_;    // y座標

    // 膨張させる範囲（直径）
    const double diameter = margin_ * 2;

    for(double x=start_x; x<=end_x; x+=map_reso_)
    {
        for(double y=start_y; y<=end_y; y+=map_reso_)
        {
            const double dist = calc_distance(person_x, person_y, x, y);

            // 衝突半径内であれば占有にする
            if(dist <= margin_)
            {
                if(is_in_map(person_map_, x, y))
                    assign_cost_for_person_cost_map(x, y, 100, min_index, max_index);
            }
        }
    }
}

// person_map_の穴を埋める
void CostMapCreator::fix_person_cost_map(const int min_index, const int max_index)
{
    const int w = cost_map_.info.width;

    //左右もしくは上下のマスにコストが割り当てられているのに，現在のマスにコストが割り当てられていない場合は修正
    for(int i=min_index+1; i<max_index; i++)
    {
        if(person_map_.data[i] < 0)  // 「未知」（-1）の場合
        {
            bool flag_in_map = false;  // falseのときはセグフォ
        
            // セグフォしないかのチェック
            if((i-w >= 0) && (i+w < cost_map_.info.width * cost_map_.info.height))
                flag_in_map = true;  // セグフォしてない

            if(flag_in_map)
            {
                if((person_map_.data[i-1] > 0) && (person_map_.data[i+1] > 0))  // 上下方向
                {
                    // 上下のマスに割り当てられているコストの平均値を割り当てる
                    person_map_.data[i] = (person_map_.data[i-1] + person_map_.data[i+1]) / 2;
                }
                else if((person_map_.data[i-w] > 0) && (person_map_.data[i+w] > 0))  // 左右方向
                {
                    // 左右のマスに割り当てられているコストの平均値を割り当てる
                    person_map_.data[i] = (person_map_.data[i-w] + person_map_.data[i+w]) / 2;
                }
            }
            else
            {
                if((person_map_.data[i-1] > 0) && (person_map_.data[i+1] > 0))  // 上下方向
                {
                    // 上下のマスに割り当てられているコストの平均値を割り当てる
                    person_map_.data[i] = (person_map_.data[i-1] + person_map_.data[i+1]) / 2;
                }
            }
        }
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

// コストマップを作成
void CostMapCreator::create_cost_map()
{
    // コストマップの初期化
    init_map(cost_map_);

    pedestrian_msgs::PersonState current_person;

    const auto current_people = current_people_states_.front();
    const auto future_people = future_people_states_.front();
    
    // 予測した歩行者の将来位置に対して走行コストを計算
    for(const auto& future_person : future_people->people_states)
    {
        // 対応する現在の歩行者情報を探索
        for(const auto& person : current_people->people_states)
        {
            if(person.id == future_person.id)
            {
                current_person = person;
                flag_ped_data_matching_ = true;

                // パーソンマップの初期化
                init_map(person_map_);

                // コストを割り当てるindexの最小値と最大値を初期化
                int min_index = (cost_map_.info.width * cost_map_.info.height) - 1;  // マップのindexの最大値
                int max_index = 0;                                             // マップのindexの最小値

                // 歩行者1人のみ考慮したコストマップを作成
                create_person_cost_map(current_person, future_person, min_index, max_index);

                // person_map_の穴を埋める
                fix_person_cost_map(min_index, max_index);

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

    pub_cost_map_.publish(cost_map_);
}

//メイン文で実行する関数
void CostMapCreator::process()
{
    ros::Rate loop_rate(hz_);

    while(ros::ok())
    {
        if((flag_current_people_states_ == true) && (flag_future_people_states_ == true))
        {
            create_cost_map();
        }

        // msgの受け取り判定用flagをfalseに戻す
        flag_current_people_states_ = false;
        flag_future_people_states_ = false;

        ros::spinOnce();
        loop_rate.sleep();
    }
}