#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtle_interfaces/msg/turtle_array.hpp"
#include "turtle_interfaces/srv/catch_turtle.hpp"

#include <iostream>
#include <vector>
#include <random>

 
class TurtleController : public rclcpp::Node
{
public:
    TurtleController() : Node("turtle_controller"), turtlesim_up_(true)
    {
        // Parameter
        this->declare_parameter("catch_closet_turtle_first", true);
        catch_closet_turtle_first_ = this->get_parameter("catch_closet_turtle_first").as_bool();

        // Node entity
        turtle1_pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10,
            std::bind(&TurtleController::turtlePoseSubsriptionCallback, this, std::placeholders::_1));
        turtles_info_subscriber_ = this->create_subscription<turtle_interfaces::msg::TurtleArray>(
            "alive_turtles", 10,
            std::bind(&TurtleController::turtleInfoSubscriberCallback, this, std::placeholders::_1));
        turtle_cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "turtle1/cmd_vel", 10);
        cmd_vel_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&TurtleController::catchTurtle, this));
        catched_turtle_client_ = this->create_client<turtle_interfaces::srv::CatchTurtle>("catch_turtle");
    }
 
private:

    void turtleInfoSubscriberCallback(const turtle_interfaces::msg::TurtleArray::SharedPtr msg)
    {
        /*
        *
        * @brief : This function is callback function for turtles_info_subscriber
        *          This function determines which turtle to catch.
        * @details
        * - If catch_closet_turtle_first is true, the closest turtle will be caught first.
        * - If catch_closet_turtle_first is false, target is the turtle of the first element in the TurtleArray.
        * 
        * @param
        * - msg : Shared pointer containing the turtle informations sent by turtle_spawner
        * 
        */

        if(msg->turtles.empty()){
            RCLCPP_INFO(this->get_logger(), "Turtle does not exist.");
        }
        else{

            if(catch_closet_turtle_first_){

                turtle_interfaces::msg::Turtle closest_turtle;
                double min_distance = 1000000; // trash value
                double distance = 0;

                // Measure the distance to all turtles in msg->turtles
                for (auto it = msg->turtles.begin(); it != msg->turtles.end(); ++it) {

                    distance = std::sqrt(std::pow(it->x - turtle1_pos_x_, 2) + std::pow(it->y - turtle1_pos_y_, 2));

                    if (distance < min_distance){
                        closest_turtle = *it;
                        min_distance = distance;                   
                    }
                }
                turtle_to_catch_ = closest_turtle;
            }
            else{
                turtle_to_catch_ = msg->turtles.at(0);
            }

            target_pos_x_ = turtle_to_catch_.x;
            target_pos_y_ = turtle_to_catch_.y;
            target_angle_z_ = turtle_to_catch_.theta;
        }

        RCLCPP_INFO(this->get_logger(), "Target_pose has been updated");
        RCLCPP_INFO(this->get_logger(), "%f, %f, %f, %s", target_pos_x_, target_pos_y_, target_angle_z_, turtle_to_catch_.name.c_str());
        catchTurtle();

    }

    void catchTurtle()
    {

        /*
        * @brief : This function is a control function of turtle1 to catch spawned turtle
        */

        if(!turtlesim_up_ || turtle_to_catch_.name == ""){
            return;
        }

        double dist_x = target_pos_x_ - turtle1_pos_x_;
        double dist_y = target_pos_y_ - turtle1_pos_y_;
        double distance = std::sqrt(dist_x * dist_x + dist_y * dist_y);

        auto msg = geometry_msgs::msg::Twist();

        if (distance > 0.25)
        {
            msg.linear.x = 2 * distance;
            double steering_angle = std::atan2(dist_y, dist_x);
            double angle_diff = steering_angle - turtle1_angle_z_;

            if (angle_diff > M_PI)
            {
                angle_diff -= 2 * M_PI;
            }
            else if (angle_diff < -M_PI)
            {
                angle_diff += 2 * M_PI;
            }
            msg.angular.z = 6 * angle_diff;
        }
        else
        {
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;

            kill_turtle_threads_.push_back(
                std::make_shared<std::thread>(std::bind(&TurtleController::sendCatchedTurtle, this, turtle_to_catch_.name))
            );
            turtle_to_catch_.name = ""; // * if you don't add this line, you will have a problem with multiple request calls to kill 

        }

        turtle_cmd_vel_publisher_->publish(msg);
    }

    void sendCatchedTurtle(std::string kill_turtle_name)
    {
        /*
        * @brief : Send a request to turtle_spawner to delete information about the turtle you captured.
        *
        * @param
        * - kill_turtle_name : turtle name to be deleted
        */

        while(!catched_turtle_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
        }

        // request 생성
        auto request = std::make_shared<turtle_interfaces::srv::CatchTurtle::Request>();
        request->name = kill_turtle_name;

        auto future = catched_turtle_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Turtle_controller sends goal asynchronously -> KILL %s", kill_turtle_name.c_str());
        try
        {
            auto response = future.get();
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Turtle_controller's kill request call failed...");
        }
    }

    void turtlePoseSubsriptionCallback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        /*
        * @brief : turtle1_pose_subscriber's callback function
        *          Reset turtle1's position and angle.
        * @param
        * - msg : Shared pointer containing the pose sent by turtlesim_node
        */

        turtle1_pos_x_ = msg->x;
        turtle1_pos_y_ = msg->y;
        turtle1_angle_z_ = msg->theta;
        turtlesim_up_ = true;
        
    }

    // Subscribes target turtle's pose.
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle1_pose_subscriber_;
    // Subscribe turtles information.
    rclcpp::Subscription<turtle_interfaces::msg::TurtleArray>::SharedPtr turtles_info_subscriber_;
    // Publishes turtle1's cmd_vel.
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle_cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr cmd_vel_timer_;
    // Request to remove the cathed turtle from the turtles_info variable.
    rclcpp::Client<turtle_interfaces::srv::CatchTurtle>::SharedPtr catched_turtle_client_;

    // Information of the turtles you need to catch
    turtle_interfaces::msg::Turtle turtle_to_catch_;

    //thread for TurtleController::sendCatchedTurtle function
    std::vector<std::shared_ptr<std::thread>> kill_turtle_threads_;

    // Variables that enable turtle1 to grab other turtles
    bool turtlesim_up_;
    bool catch_closet_turtle_first_; 

    // turtle1's variable
    double turtle1_pos_x_; // TODO : this can be replaced to turtlesim/msg/Pose
    double turtle1_pos_y_;
    double turtle1_angle_z_;

    // target turtle's variable
    double target_pos_x_;
    double target_pos_y_;
    double target_angle_z_;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}