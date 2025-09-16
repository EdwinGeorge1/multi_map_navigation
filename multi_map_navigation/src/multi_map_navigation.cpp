#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include <sqlite3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <functional>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

class MultiMapNavigation : public rclcpp::Node
{
public:
    MultiMapNavigation() : Node("multi_map_navigation")
    {
        action_server_ = rclcpp_action::create_server<NavigateToPose>(
            this,
            "multi_map_navigation",
            std::bind(&MultiMapNavigation::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MultiMapNavigation::handle_cancel, this, std::placeholders::_1),
            std::bind(&MultiMapNavigation::handle_accepted, this, std::placeholders::_1));

        int rc = sqlite3_open("/home/ubuntu/ros2_ws/src/multi_map_navigation/sql/wormholes.db", &db_);
        if(rc){
            RCLCPP_ERROR(this->get_logger(), "Can't open database: %s", sqlite3_errmsg(db_));
        } else {
            RCLCPP_INFO(this->get_logger(), "Opened wormhole database successfully");
        }

        this->declare_parameter<std::string>("current_map", "room1");

        nav2_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        load_map_client_ = this->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");
    }

    ~MultiMapNavigation() {
        sqlite3_close(db_);
    }

private:
    rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
    rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr load_map_client_;
    sqlite3* db_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavigateToPose::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received goal for map: %s", goal->pose.header.frame_id.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
    {
        std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
    }

    void execute(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
    {
        auto goal = goal_handle->get_goal();
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "map";  // <-- set here
        target_pose.pose = goal->pose.pose;

        std::string target_map = goal->pose.header.frame_id;
        std::string current_map;
        this->get_parameter("current_map", current_map);

        if(current_map == target_map){
            RCLCPP_INFO(this->get_logger(), "Already in target map. Sending Nav2 goal...");
            send_nav2_goal(target_pose, [goal_handle]() {
                auto result = std::make_shared<NavigateToPose::Result>();
                goal_handle->succeed(result);
                RCLCPP_INFO(rclcpp::get_logger("multi_map_navigation"), "Goal completed.");
            });
        } else {
            RCLCPP_INFO(this->get_logger(), "Switching maps via wormhole...");

            std::string sql = "SELECT x, y, z, yaw FROM wormholes WHERE map_from='" + current_map + "' AND map_to='" + target_map + "';";
            sqlite3_stmt* stmt;
            if(sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, 0) == SQLITE_OK){
                if(sqlite3_step(stmt) == SQLITE_ROW){
                    double x = sqlite3_column_double(stmt, 0);
                    double y = sqlite3_column_double(stmt, 1);
                    double z = sqlite3_column_double(stmt, 2);
                    double yaw = sqlite3_column_double(stmt, 3);

                    geometry_msgs::msg::PoseStamped wormhole_pose;
                    wormhole_pose.header.frame_id = "map";  // <-- set here
                    wormhole_pose.pose.position.x = x;
                    wormhole_pose.pose.position.y = y;
                    wormhole_pose.pose.position.z = z;
                    tf2::Quaternion q;
                    q.setRPY(0, 0, yaw);
                    wormhole_pose.pose.orientation = tf2::toMsg(q);

                    RCLCPP_INFO(this->get_logger(), "Sending Nav2 goal to wormhole at (%.3f, %.3f)", x, y);

                    send_nav2_goal(wormhole_pose, [this, target_map, target_pose, goal_handle]() {
                        std::string map_yaml = "/home/ubuntu/ros2_ws/src/multi_map_navigation/maps/" + target_map + ".yaml";
                        switch_map(map_yaml, [this, target_pose, target_map, goal_handle]() {
                            this->set_parameter(rclcpp::Parameter("current_map", target_map));
                            send_nav2_goal(target_pose, [goal_handle]() {
                                auto result = std::make_shared<NavigateToPose::Result>();
                                goal_handle->succeed(result);
                                RCLCPP_INFO(rclcpp::get_logger("multi_map_navigation"), "Final goal reached.");
                            });
                        });
                    });

                } else {
                    RCLCPP_WARN(this->get_logger(), "No wormhole found from %s to %s", current_map.c_str(), target_map.c_str());
                    auto result = std::make_shared<NavigateToPose::Result>();
                    goal_handle->abort(result);
                }
                sqlite3_finalize(stmt);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to prepare SQL statement");
                auto result = std::make_shared<NavigateToPose::Result>();
                goal_handle->abort(result);
            }
        }
    }

    void send_nav2_goal(const geometry_msgs::msg::PoseStamped& pose, std::function<void()> done_cb)
    {
        if(!nav2_client_->wait_for_action_server(std::chrono::seconds(5))){
            RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = pose;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.feedback_callback = [](std::shared_ptr<rclcpp_action::ClientGoalHandle<NavigateToPose>>, std::shared_ptr<const NavigateToPose::Feedback>){};
        send_goal_options.result_callback = [done_cb](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result){
            if(result.code == rclcpp_action::ResultCode::SUCCEEDED){
                RCLCPP_INFO(rclcpp::get_logger("multi_map_navigation"), "Nav2 goal reached!");
            } else {
                RCLCPP_WARN(rclcpp::get_logger("multi_map_navigation"), "Nav2 goal failed or canceled");
            }
            if(done_cb) done_cb();
        };

        nav2_client_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(this->get_logger(), "Nav2 goal sent to (%.3f, %.3f)", pose.pose.position.x, pose.pose.position.y);
    }

    void switch_map(const std::string &map_yaml, std::function<void()> done_cb)
    {
        auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
        request->map_url = map_yaml;

        if(!load_map_client_->wait_for_service(std::chrono::seconds(5))){
            RCLCPP_ERROR(this->get_logger(), "Map server not available");
            return;
        }

        load_map_client_->async_send_request(request,
            [map_yaml, done_cb](rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedFuture future){
                if(future.valid()){
                    RCLCPP_INFO(rclcpp::get_logger("multi_map_navigation"), "Map switched to: %s", map_yaml.c_str());
                } else {
                    RCLCPP_ERROR(rclcpp::get_logger("multi_map_navigation"), "Failed to switch map");
                }
                if(done_cb) done_cb();
            }
        );
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiMapNavigation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

