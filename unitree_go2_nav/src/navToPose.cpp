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
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" // Corrected for Foxy
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "unitree_go2_nav_interfaces/msg/nav_to_pose.hpp"
#include "geometry_msgs/msg/twist.hpp" // Added for Twist message definition

using namespace std::chrono_literals;
// using std::placeholders::_1; // This line is commented out, which is fine as std::placeholders::_1 is used explicitly.

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
        // Subscribers.
        // The template argument for create_subscription matches the message type.
        // The callback signature will determine if it takes a value or a shared_ptr.
        cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_smoothed", 
            10, 
            std::bind(&NavToPose::cmd_vel_callback, this, std::placeholders::_1)
        );

        // Publishers.
        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        qos_profile.keep_last(50);  // Increase history depth
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);  // Or TRANSIENT_LOCAL if needed

        nav_comp_msg_pub = this->create_publisher<unitree_go2_nav_interfaces::msg::NavToPose>("nav_twist", qos_profile);

        
        // Timer
        mTimer = this->create_wall_timer(100ms, std::bind(&NavToPose::timer_callback, this));      
     }       

    private:
        rclcpp::TimerBase::SharedPtr mTimer;
        State mRobotState = State::IDLE;
        // The subscription member variable type is correct (SharedPtr to the message)
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
        rclcpp::Publisher<unitree_go2_nav_interfaces::msg::NavToPose>::SharedPtr nav_comp_msg_pub;
        geometry_msgs::msg::Twist cmd_vel; // This member stores the actual message data (not a pointer)
        bool cmd_vel_received = false; 

        // Corrected callback signature: takes a shared_ptr to a const message
        void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            // Dereference the shared_ptr to assign the message data to the member variable
            cmd_vel = *msg; 
            cmd_vel_received = true;

            RCLCPP_INFO(get_logger(), "Cmd vel x is %f", msg->linear.x);       // Use -> to access members of the shared_ptr
            RCLCPP_INFO(get_logger(), "Cmd vel y is %f", msg->linear.y);       // Use -> to access members of the shared_ptr
            RCLCPP_INFO(get_logger(), "Cmd vel angular z/ YAW is %f", msg->angular.z); // Use -> to access members of the shared_ptr
        }


        void timer_callback()
        { 
            // RCLCPP_INFO(get_logger(), "Inside timer, robotsate is %f.", enum_cast<int>(mRobotState)); 
            if (!cmd_vel_received)
            {
                RCLCPP_WARN(get_logger(), "[NavToPose] cmd_vel not received yet, skipping publishing.");
                return;
            }
             
            if (mRobotState == State::IDLE) //MANUAL_NAV_MODE
            {
                unitree_go2_nav_interfaces::msg::NavToPose msg;
                // Here, cmd_vel is the geometry_msgs::msg::Twist object, so use .
                msg.x = cmd_vel.linear.x; 
                msg.y = cmd_vel.linear.y;
                msg.yaw = cmd_vel.angular.z;
                nav_comp_msg_pub->publish(msg);

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
