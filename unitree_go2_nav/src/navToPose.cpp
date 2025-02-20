// High level control of GO2 using navigation goal from the Nav2 stack.

#include <exception>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

// #include "/home/sayantani/Documents/Winter/project/ws2/src/unitree_go2_nav/unitree_ros2/cyclonedds_ws/unitree/unitree_go/msg/sport_mode_state.hpp"

// #include "unitree_api/msg/request.hpp"

// /home/sayantani/Documents/Winter/project/ws2/src/unitree_go2_nav/unitree_ros2/example/src/include/common
// #include "/home/sayantani/Documents/Winter/project/ws2/src/unitree_go2_nav/unitree_ros2/example/src/include/common/ros2_sport_client.h"

using namespace std::chrono_literals;
// using std::placeholders::_1;



// State machine
enum class State
{
    IDLE,
    MANUAL_NAV_MODE,
    AUTO_NAV_MODE,
    START,
    MOVING,
    WAIT_FOR_MOVEMENT_COMPLETE,
    REACHED_GOAL,
};


class NavToPose : public rclcpp::Node
{
    public:

     NavToPose():Node("nav_to_pose")
     {       
        //Action client: navigation 
        // act_nav_to_pose_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
        
        // //Service call to trigger manual navigation: later need to do that msg type thing, see official documentation.
        // nav_start_trigger = create_service<std_srvs::srv::Empty>("nav_start_trigger", std::bind(&NavToPose::nav_start_trigger_callback, this, std::placeholders::_1));
        
        // Subsribers.
        cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>("cmd_vel_nav", 10, std::bind(&NavToPose::cmd_vel_callback, this, std::placeholders::_1));
        // the state_suber is set to subscribe "high_level_ctrl" topic
        // state_suber = this->create_subscription<unitree_go::msg::SportModeState>("sportmodestate", 10, std::bind(&NavToPose::state_callback, this, _1));


        // Publishers.
        // The req_puber is set to subscribe "/api/sport/request" topic with dt
        // req_puber = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        nav_msg_pub = this->create_publisher<geometry_msgs::msg::Twist>("nav_twist", 10);
        
        //Timer
        // std::chrono::milliseconds(int(dt * 1000))
        // t = -1; // Runing time count
        mTimer = this->create_wall_timer(100ms, std::bind(&NavToPose::timer_callback, this));      


     }       

    private:
        rclcpp::TimerBase::SharedPtr mTimer;
        State mRobotState = State::IDLE;
        // rclcpp::Service<std_srvs::srv::Empty>::SharedPtr nav_start_trigger;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr nav_msg_pub;
        geometry_msgs::msg::Twist cmd_vel;
        // rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_suber;
        // rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;
        // unitree_api::msg::Request req; // Unitree Go2 ROS2 request message
        // SportClient sport_req;
        // double px0 = 0;  // initial x position
        // double py0 = 0;  // initial y position
        // double yaw0 = 0; // initial yaw angle

        // void nav_start_trigger_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response)
        // {            
        //     mRobotState = State::MANUAL_NAV_MODE;
        //     RCLCPP_INFO(get_logger(), "Manual navigation mode started.");
        //     response->success = true;
        // }

        void cmd_vel_callback(const geometry_msgs::msg::Twist msg)
        {
            // geometry_msgs::msg::Twist cmd_vel;
            cmd_vel = msg;


            // RCLCPP_INFO(get_logger(), "Cmd vel x is %f", msg.linear.x);
            // RCLCPP_INFO(get_logger(), "Cmd vel y is %f", msg.linear.y);
            // RCLCPP_INFO(get_logger(), "Cmd vel angular z/ YAW is %f", msg.angular.z);
        }

        // void state_callback(unitree_go::msg::SportModeState::SharedPtr data)
        // {
        //     // Get current position of robot when t<0
        //     // This position is used as the initial coordinate system
        //     // Basically odom.

        //     if (t < 0)
        //     {
        //         // Get initial position
        //         px0 = data->position[0];
        //         py0 = data->position[1];
        //         yaw0 = data->imu_state.rpy[2];
        //         std::cout << "[Odom]";
        //         std::cout << px0 << ", " << py0 << ", " << yaw0 << std::endl;
        //     }
        // }

        void timer_callback()
        { 
            // RCLCPP_INFO(get_logger(), "Inside timer, robotsate is %f.", enum_cast<int>(mRobotState)); 
            if (mRobotState == State::IDLE) //MANUAL_NAV_MODE
            {
                // std::vector<PathPoint> path;
                // float vx = cmd_vel.linear.x;
                // float vy = cmd_vel.linear.y;
                // float vyaw = cmd_vel.angular.z;

                nav_msg_pub->publish(cmd_vel);



                // // Give a forward path.
                // sport_req.Move(req, vx, vy, vyaw);
                // // Publish request messages
                // req_puber->publish(req);
            }         
            
            else
            {
                RCLCPP_ERROR(get_logger(), "Unhandled state.");
                throw std::logic_error("Unhandled state.");
            }
            
        }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavToPose>());
  rclcpp::shutdown();
  return 0;
}










// TO_DO:
// I need a node that subscribes to :
// geometry_msgs/msg/Twist  topic: /cmd_vel_nav
// a trigger to start the navigation -> action client/service call probably.



// use that to command the high level control and move the robot.

// Nav might take care of that.
// subscribe odom/base link lookup, to get the current pose
// maybe write a control PID loop to move the robot to the desired pose.