#ifndef LOCAL_PATH_PLANNER_H
#define LOCAL_PATH_PLANNER_H

#include <ros/ros.h>
#include <queue>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

// ===== 構造体 =====
struct State
{
    double x;    // [m]
    double y;    // [m]
    double yaw;  // [rad]
};

// ===== クラス =====
class LocalPathPlanner
{
public:
    LocalPathPlanner();
    void process();

private:
    // コールバック関数
    void cost_map_callback(const nav_msgs::OccupancyGridConstPtr& msg);
    void local_goal_callback(const geometry_msgs::PointStampedConstPtr& msg);

    // 引数あり関数
    bool is_goal_check(const double x, const double y);                                  // pathの終端がlocal_goalから一定距離内に到達したか確認
    double calc_evaluation(const std::vector<State>& traj);                              // 評価関数を計算する
    double calc_heading_eval(const std::vector<State>& traj);                            // heading（1項目）の評価関数を計算する
    double calc_cost_map_eval(const std::vector<State>& traj);                           // cost_map（2項目）の評価関数を計算する
    double normalize_angle(double theta);                                                // 適切な角度(-M_PI ~ M_PI)を返す
    int xy_to_grid_index(const double x, const double y);                                // 座標からグリッドのインデックスを返す
    void search_node(const double max_rad, std::vector<State>& nodes);                   // 次のノードを探索
    void create_path(const double max_rad);                                              // 目標軌道を生成
    void transform_node_to_path(const std::vector<State>& nodes, nav_msgs::Path& path);  // ノード情報からパスを生成


    // 引数なし関数
    double calc_rad();  // 機構的制約内で旋回可能な角度を計算

    // yamlファイルで設定可能な変数
    int hz_;                  // ループ周波数 [Hz]
    std::string path_frame_;  // 生成するpathのframe_id
    std::string goal_frame_;  // local_goalのframe_id
    double goal_tolerance_;   // local_goal_に対する許容誤差 [m]
    double max_vel_;          // 最高並進速度 [m/s]
    double max_yawrate_;      // 最高旋回速度 [rad/s]
    double max_steer_angle_;  // ステア角の最大値 [deg]
    double tread_;            // ccvのトレッド [m]
    double path_reso_;        // 生成するpathの刻み幅 [m]
    double theta_reso_;       // 候補となるpathを生成する際の方位の刻み幅 [rad]
    int search_step_;         // 何ステップ先まで評価値を計算するか
    double weight_heading_;   // 評価関数1項目　重みづけ定数
    double weight_cost_map_;  // 評価関数2項目　重みづけ定数
    double min_cost_;         // 割り当てるコストの最小値

    // その他の変数
    double tmp_x_;    // 1ステップ前のロボットのx座標格納用
    double tmp_y_;    // 1ステップ前のロボットのy座標格納用
    double tmp_yaw_;  // 1ステップ前のロボットのyaw角格納用

    // msgの受け取り判定用
    bool flag_cost_map_ = false;
    bool flag_local_goal_ = false;

    // local_goalから一定距離内に到達したかの確認用
    bool flag_goal_check_ = false;

    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber
    ros::Subscriber sub_cost_map_;
    ros::Subscriber sub_local_goal_;

    // Publisher
    ros::Publisher pub_local_path_;

    // tf
    tf2_ros::Buffer tf_buffer_;

    // 各種オブジェクト
    nav_msgs::OccupancyGrid cost_map_;        // コストマップ
    geometry_msgs::PointStamped local_goal_;  // local_goal
};

#endif // LOCAL_PATH_PLANNER_H