#ifndef COST_MAP_CREATOR_H
#define COST_MAP_CREATOR_H

#include <ros/ros.h>
#include <queue>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

// original_msgs
#include <pedestrian_msgs/PersonState.h>
#include <pedestrian_msgs/PeopleStates.h>

// ===== 構造体 =====
struct Coordinate
{
    double x;  // [m]
    double y;  // [m]
};

// ===== クラス =====
class CostMapCreator
{
public:
    CostMapCreator();
    void process();

private:
    // コールバック関数
    void current_people_states_callback(const pedestrian_msgs::PeopleStatesConstPtr& msg);
    void future_people_states_callback(const pedestrian_msgs::PeopleStatesConstPtr& msg);
    // void robot_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);

    // 引数あり関数
    void init_map(nav_msgs::OccupancyGrid& map);                                                                                                                                                        // マップの初期化(すべて「未知」にする)
    double calc_speed(const double linear_x, const double linear_y);                                                                                                                                    // 速さを計算
    double calc_ellipse_long_param(const pedestrian_msgs::PersonState& person);                                                                                                                         // 楕円の長軸の長さのパラメーターを計算
    double calc_ellipse_short_param(const double x, const double y);                                                                                                                                    // 楕円の短軸の長さのパラメーターを計算
    double calc_direction(const double x, const double y);                                                                                                                                              // 方位を計算
    double normalize_angle(double theta);                                                                                                                                                               // 適切な角度(-M_PI ~ M_PI)を返す
    bool is_in_map(nav_msgs::OccupancyGrid& map, const double x, const double y);                                                                                                                       // マップ内の場合、trueを返す
    int xy_to_grid_index(nav_msgs::OccupancyGrid& map, const double x, const double y);                                                                                                                 // 座標からグリッドのインデックスを返す
    void search_grid_size(std::vector<Coordinate>& side, const double start_x, const double start_y, const double length, const double theta);                                                          // マスを探索
    double count_grid(const std::vector<Coordinate> side, std::vector<Coordinate>& cost_side, const double person_x, const double person_y);                                                      // 走行コストを割り当てるマスをカウント
    void assign_cost_for_person_cost_map(const double x, const double y, const double cost, int& min_index, int& max_index);                                                                            // person_map_にコストを割り当てる
    double calc_distance(const double person_x, const double person_y, const double x, const double y);                                                                                                 // 歩行者位置までの距離を計算
    double calc_short_side_length(const double x, const double a, const double b);                                                                                                                      // 長軸方向の位置から対応する短軸方向の長さを計算
    void search_long_side_grid(const double person_x, const double person_y, const double theta, const double ellipse_long_length, const double ellipse_short_length, int& min_index, int& max_index);  // 長軸方向のグリッドを探索
    void search_short_side_grid(const double person_x, const double person_y, const double theta, const double ellipse_short_length, int& min_index, int& max_index);                                   // 短軸方向のグリッドを探索
    void create_person_cost_map(const pedestrian_msgs::PersonState& current_person, const pedestrian_msgs::PersonState& future_person, int& min_index, int& max_index);                                 // 歩行者1人のみ考慮したコストマップを作成
    void expand_obstacle(const double person_x, const double person_y, int& min_index, int& max_index);                                                                                                 // 歩行者の周りの衝突半径分を占有にする
    void fix_person_cost_map(const int min_index, const int max_index);                                                                                                                                 // person_map_の穴を埋める
    void copy_cost(const int min_index, const int max_index);                                                                                                                                           // person_map_のコストをコストマップにコピー

    // 引数なし関数
    void create_cost_map();  // コストマップを作成

    // yamlファイルで設定可能な変数
    bool flag_cost_;                 // 走行コストを設定するかの変更用
    int hz_;                         // ループ周波数 [Hz]
    std::string people_frame_;       // 歩行者の位置情報のframe_id
    std::string cost_map_frame_;     // コストマップのframe_id
    double map_size_;                // マップの一辺の長さ [m]
    double map_reso_;                // マップの解像度 [m/cell]
    double ellipse_front_long_max_;  // 走行コストの楕円の長軸（前方）の最大値 [m]
    double ellipse_front_long_min_;  // 走行コストの楕円の長軸（前方）の最小値 [m]
    double ellipse_back_long_max_;   // 走行コストの楕円の長軸（後方）の最大値 [m]
    double ellipse_back_long_min_;   // 走行コストの楕円の長軸（後方）の最小値 [m]
    double ellipse_short_max_;       // 走行コストの楕円の短軸の最大値 [m]
    double ellipse_short_min_;       // 走行コストの楕円の短軸の最小値 [m]
    double margin_;                  // 人間の肩幅の半分 [m]
    double weight_distance_;         // ロボットからの距離に関する項の重み定数
    double weight_speed_;            // 歩行者の速さに関する項の重み定数
    double ped_speed_max_;           // 歩行者の歩く速さの最大値 [m/s]
    double count_reso_;              // コストを割り当てる際に計算する座標の刻み幅 [m]
    double min_cost_;                   // 割り当てるコストの最小値

    // msgの受け取り判定用
    bool flag_current_people_states_ = false;
    bool flag_future_people_states_ = false;
    
    // 歩行者情報のマッチング確認用
    bool flag_ped_data_matching_ = false;

    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber
    ros::Subscriber sub_current_people_states_;
    ros::Subscriber sub_future_people_states_;
    ros::Subscriber sub_robot_odom_;

    // Publisher
    ros::Publisher pub_cost_map_;

    // 各種オブジェクト
    std::queue<pedestrian_msgs::PeopleStatesConstPtr> current_people_states_;  // 現在の歩行者情報
    std::queue<pedestrian_msgs::PeopleStatesConstPtr> future_people_states_;   // 予測した歩行者情報
    nav_msgs::OccupancyGrid cost_map_;                                         // コストマップ
    nav_msgs::OccupancyGrid person_map_;                                       // 歩行者1人分のコストを計算する用
};

#endif // COST_MAP_CREATOR_H