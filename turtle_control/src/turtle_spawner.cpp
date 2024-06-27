#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtle_interfaces/msg/turtle_array.hpp"
#include "turtle_interfaces/srv/catch_turtle.hpp"

#include <random>
#include <unordered_map>
 
class TurtleSpawner : public rclcpp::Node
{
public:
    TurtleSpawner() : Node("turtle_spawner"), turtle_counter_(0)
    {
        // Parameters
        this->declare_parameter("spawn_frequency", 1.0); 
        this->declare_parameter("turtle_name_prefix", "");
        spawn_frequency_ = this->get_parameter("spawn_frequency").as_double();
        turtle_name_prefix_ = this->get_parameter("turtle_name_prefix").as_string();

        // Node Entity
        spawn_client_ = this->create_client<turtlesim::srv::Spawn>("spawn");
        kill_client_ = this->create_client<turtlesim::srv::Kill>("kill");
        cathed_turtle_server = this->create_service<turtle_interfaces::srv::CatchTurtle>(
            "catch_turtle",
            std::bind(&TurtleSpawner::cathedTurtleServerCallback, this, std::placeholders::_1, std::placeholders::_2));
        turtle_publisher_ = this->create_publisher<turtle_interfaces::msg::TurtleArray>("alive_turtles", 10);
        turtle_spawner_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(int(1000.0 / spawn_frequency_)),
            std::bind(&TurtleSpawner::spawnNewTurtle, this));
    }
 
private:

    void cathedTurtleServerCallback(const turtle_interfaces::srv::CatchTurtle::Request::SharedPtr request,
                                    const turtle_interfaces::srv::CatchTurtle::Response::SharedPtr response)
    {
        /*
        * @brief : catched_turtle_server's callback function
        *
        * @param
        * - request : Shared pointer of client's request
        * - response : Shared pointer of client's response
        */

        kill_turtle_threads_.push_back(
            std::make_shared<std::thread>(
                std::bind(&TurtleSpawner::killTurtle, this, request->name)
            )
        );

        response->response = "Request was sent successfully";
    }

    void killTurtle(std::string turtle_name)
    {
        /*
        * @brief : Request to remove catched turtle from turtlesim_node
        *
        * @param
        * - turtle_name : Name of the turtle to remove from turtlesim_node
        */
       
        // Deleting turtles from spawned_turtleInfo_
        for (auto it = turtleInfo_.begin(); it != turtleInfo_.end(); ++it)
        {
            if (it->name == turtle_name)
            {
                RCLCPP_INFO(this->get_logger(), "Erasing %s", it->name.c_str());
                turtleInfo_.erase(it);
                break;
            }
        }

        //After creating the kill request, send the request
        while(!kill_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
        }

        auto kill_request = std::make_shared<turtlesim::srv::Kill::Request>();
        kill_request->name = turtle_name;
        
        auto future = kill_client_->async_send_request(kill_request);
        RCLCPP_INFO(this->get_logger(), "Turtle_spanwer sends goal asynchronously -> KILL %s", turtle_name.c_str());
        try
        {
            auto response = future.get();
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Turtle_spawner's kill request call failed...");
        }
        
        // Send a new list of turtles.
        publishAliveTurtles();
        
    }

    void publishAliveTurtles()
    {
        /*
        * @brief : Publish spawned turtle's information to turtle_controller node
        */
        turtle_interfaces::msg::TurtleArray msg;
        msg.turtles.assign(turtleInfo_.begin(), turtleInfo_.end());
        turtle_publisher_->publish(msg);
    }

    double generateRandomNumber()
    {
        /*
        * @brief : Generate a random number between 0 and 1.
        *
        * @return
        * - random number between 0 and 1
        */

        std::random_device rd;  // 하드웨어 엔트로피를 사용하는 난수 생성기
        std::mt19937 gen(rd()); // Mersenne Twister 엔진 사용
        std::uniform_real_distribution<> rng(0.0, 1.0);

        return rng(gen);
    }

    void spawnNewTurtle()
    {   
        /*
        * @brief : This is a function that behaves periodically for spawning turtle
        *          Period is regulated by turtle_spawner_timer_.
        */

        turtle_counter_++;

        double x = generateRandomNumber() * 10;
        double y = generateRandomNumber() * 10;
        double theta = generateRandomNumber() * 2 * M_PI;

        turtle_interfaces::msg::Turtle turtle;
        turtle.x = x;
        turtle.y = y;
        turtle.theta = theta;
        turtle.name = std::string("spawned_turtle") + std::string(turtle_name_prefix_) + std::to_string(turtle_counter_);
        turtleInfo_.push_back(turtle);

        // thread를 사용해 spawn시키기
        spawn_turtle_threads_.push_back(
            std::make_shared<std::thread>(
                std::bind(&TurtleSpawner::spawnTurtleService, this, x, y, theta)));
    }

    void spawnTurtleService(double x, double y, double theta)
    {
        /*
        * @brief : Request turtlesim_node to spawn a turtle.
        * 
        * @param
        * - x : spawned turtle's x position
        * - y : spawned turtle's y position
        * - theta : spawned turtle's angle
        */

        // Waiting for Service Server
        while(!spawn_client_->wait_for_service(std::chrono::microseconds(500)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
        }

        // Service Request
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = x;
        request->y = y;
        request->theta = theta;
        request->name = std::string("spawned_turtle") + std::string(turtle_name_prefix_) + std::to_string(turtle_counter_);

        // send_service asyncronosly
        auto future = spawn_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Turtle_spawner sends goal asynchronously -> Spawn %s", request->name.c_str());
        try
        {
            auto response = future.get();
            publishAliveTurtles();
        }
        catch(const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Turtle_spawner's spawn service call failed...");
        }
    }

    // Request turtlesim_node to spawn a turtle
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::TimerBase::SharedPtr turtle_spawner_timer_;
    // Request turtlesim_node to kill a turtle
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;
    // Publishes turtles information to turtle_controller node
    rclcpp::Publisher<turtle_interfaces::msg::TurtleArray>::SharedPtr turtle_publisher_;
    // Set the target turtle
    rclcpp::Service<turtle_interfaces::srv::CatchTurtle>::SharedPtr cathed_turtle_server;

    // Parameters
    double spawn_frequency_;
    std::string turtle_name_prefix_;

    std::vector<std::shared_ptr<std::thread>> spawn_turtle_threads_;
    std::vector<std::shared_ptr<std::thread>> kill_turtle_threads_;
    int turtle_counter_; // number of spawned turtle

    // Spawned turtles information
    std::vector<turtle_interfaces::msg::Turtle> turtleInfo_;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}