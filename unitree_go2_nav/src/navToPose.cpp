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

using namespace std::chrono_literals;

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
        //Parameters
        // auto param = rcl_interfaces::msg::ParameterDescriptor{};
        // param.description = "The frame in which poses are sent.";
        // declare_parameter("pose_frame", "map", param);
        // goal_msg_.pose.header.frame_id = get_parameter("pose_frame").get_parameter_value().get<std::string>();
        
        //Action client: navigation 
        // act_nav_to_pose_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
        
        // //Service call to trigger manual navigation: later need to do that msg type thing, see official documentation.
        // nav_start_trigger = create_service<std_srvs::srv::Empty>("nav_start_trigger", std::bind(&NavToPose::nav_start_trigger_callback, this, std::placeholders::_1));
        
        //Subsribers.
        cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>("cmd_vel_nav", 10, std::bind(&NavToPose::cmd_vel_callback, this, std::placeholders::_1));

        // TO_DO:
        // I need a node that subscribes to :
        // geometry_msgs/msg/Twist  topic: /cmd_vel_nav
        // use that to command the high level control and move the robot.
        // a trigger to start the navigation -> action client/service call probably.
        // subscribe odom/base link lookup, to get the current pose
        // maybe write a control PID loop to move the robot to the desired pose.


        //Timer
        mTimer = this->create_wall_timer(100ms, std::bind(&NavToPose::timer_callback, this));      


     }       

    private:
        rclcpp::TimerBase::SharedPtr mTimer;
        State mRobotState = State::IDLE;
        // rclcpp::Service<std_srvs::srv::Empty>::SharedPtr nav_start_trigger;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;

        // void nav_start_trigger_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response)
        // {            
        //     mRobotState = State::MANUAL_NAV_MODE;
        //     RCLCPP_INFO(get_logger(), "Manual navigation mode started.");
        //     response->success = true;
        // }

        void cmd_vel_callback(const geometry_msgs::msg::Twist msg)
        {
            geometry_msgs::msg::Twist cmd_vel;
            //Store received cmd_vel
            cmd_vel = msg;
            RCLCPP_INFO(get_logger(), "Cmd vel x is %f", msg.linear.x);
            RCLCPP_INFO(get_logger(), "Cmd vel y is %f", msg.linear.y);
            RCLCPP_INFO(get_logger(), "Cmd vel angular x is %f", msg.angular.x);
            RCLCPP_INFO(get_logger(), "Cmd vel angular y is %f", msg.angular.y);
        }

        void timer_callback()
        {            
            switch(mRobotState)
            {
                case State::IDLE:
                {
                    break;
                }
                case State::MANUAL_NAV_MODE:
                {
                    break;
                }
                case State::AUTO_NAV_MODE:
                {
                    break;
                }
                case State::START:
                {
                    break;
                }
                case State::MOVING:
                {
                    break;
                }
                case State::WAIT_FOR_MOVEMENT_COMPLETE:
                {
                    break;
                }
                case State::REACHED_GOAL:
                {
                    break;
                }
                default:
                {
                    RCLCPP_ERROR(get_logger(), "Unhandled state.");
                    throw std::logic_error("Unhandled state.");
                    break;
                }
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