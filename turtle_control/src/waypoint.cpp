#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "turtlesim_msgs/srv/teleport_absolute.hpp"
#include "turtlesim_msgs/srv/teleport_relative.hpp"
#include "turtlesim_msgs/srv/set_pen.hpp"
#include "turtlesim_msgs/msg/pose.hpp"

using namespace std::chrono_literals;

// interfaces with turtlesim, teleports where ever some other node tells it to go!

using Empty = std_srvs::srv::Empty;
using TeleportAbsolute = turtlesim_msgs::srv::TeleportAbsolute;
using TeleportRelative = turtlesim_msgs::srv::TeleportRelative;
using SetPen = turtlesim_msgs::srv::SetPen;
using Pose = turtlesim_msgs::msg::Pose;
 
enum class State
{
    INIT,
    WAITING,
    READY,
};

enum class Actions
{
  X,              // placeholder
  F,              // move forward
  MINUS,          // turn right
  PLUS,           // turn left
  SAVE,           // save current state
  RESTORE,        // restore saved state
};

class Waypoint : public rclcpp::Node
{
public:
  Waypoint()
  : Node("waypoint")
  {
    // initialize state
    state_ = State::INIT;

    // initialize rule list
    action_list_ = {Actions::X};

    // create service client
    abs_tele_client_ = this->create_client<TeleportAbsolute>("/turtle1/teleport_absolute");
    rel_tele_client_ = this->create_client<TeleportRelative>("/turtle1/teleport_relative");
    pen_client_ = this->create_client<SetPen>("/turtle1/set_pen");

    // wait for tele service to appear
    while(!abs_tele_client_->wait_for_service(std::chrono::seconds(1))) 
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
        }
        RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
    }

    // wait for tele service to appear
    while(!rel_tele_client_->wait_for_service(std::chrono::seconds(1))) 
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
        }
        RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
    }

    // wait for pen service to appear
    while(!pen_client_->wait_for_service(std::chrono::seconds(1))) 
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
        }
        RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
    }

    // lift pen up
    auto request = std::make_shared<SetPen::Request>();
    request->off = true;

    auto future1 = pen_client_->async_send_request(request);

    rclcpp::spin_until_future_complete(
      this->get_node_base_interface(),
      future1
    );

    // teleport turtle to starting position
    auto request1 = std::make_shared<TeleportAbsolute::Request>();
    request1->x = 5.0;
    request1->y = 0.0;
    request1->theta = M_PI/2;

    // publish message
    auto future2 = abs_tele_client_->async_send_request(request1);

    rclcpp::spin_until_future_complete(
      this->get_node_base_interface(),
      future2
    );

    // initialize length and increment
    len_ = 0.1;
    angle_ = 60.0f * M_PI / 180.0f;

    count_ = 0;
    super_count_ = 0;

    // change pen
    request->width = 0.5;
    request->r = 255;
    request->g = 255;
    request->b = 255;
    request->off = false;
    auto future3 = pen_client_->async_send_request(request);

    rclcpp::spin_until_future_complete(
      this->get_node_base_interface(),
      future3
    );

    // init pen status
    pen_ = false;
    
    // define timer callback function
    auto timer_callback =
      [this]() -> void 
      {

  
        if (state_ == State::INIT)
        {

          auto add_on = execute_rules(action_list_.at(count_));

          for (auto n : add_on)
          {
            new_list_.push_back(n);
          }

          count_++;

          RCLCPP_INFO(this->get_logger(), "count %d super_count %d length %d add_on %d new_list %d", count_, super_count_, int(action_list_.size()), int(new_list_.size()), int(add_on.size()));


          if (count_ >= int(action_list_.size()))
          {
            count_ = 0;
            action_list_ = new_list_;
            super_count_++;

            // init new list
            new_list_.clear();


            if (super_count_ >= 6)
            {
              state_ = State::READY;
              count_ = 0;
              RCLCPP_INFO(this->get_logger(), "DONE");

            }
            
          }

        }
        if (state_ == State::READY)
        {
          // change state to waiting until turtle movement is done
          state_ = State::WAITING;

          // move turtle
          execute_action_item(action_list_.at(count_));

          // update count
          count_++;

          if (count_ >= int(action_list_.size()))
          {
            // change state to waiting indefinitely
            state_ = State::WAITING;
          }
        }
      };

    // create timer
    timer_ = this->create_wall_timer(0.1ms, timer_callback);

    auto turtle_pose_cb =
            [this](Pose::SharedPtr msg) -> void {
              turtle_pose_ = msg;
            };
    subscription_ = 
        this->create_subscription<Pose>("/turtle1/pose", 10, turtle_pose_cb);


    RCLCPP_INFO(
            this->get_logger(),
            "Waypoint node up.");
  }

  void execute_action_item(Actions action)
  {
    using RelServiceResponseFuture = rclcpp::Client<TeleportRelative>::SharedFutureWithRequest;
    using AbsServiceResponseFuture = rclcpp::Client<TeleportAbsolute>::SharedFutureWithRequest;
    using PenServiceResponseFuture = rclcpp::Client<SetPen>::SharedFutureWithRequest;

    auto rel_response_received_callback =
      [this](RelServiceResponseFuture future) {
        auto request_response_pair = future.get();
        RCLCPP_DEBUG(
          this->get_logger(),
          "Result recieved. %f", len_);
        state_ = State::READY;
        
    };

    auto pen_response_received_callback2 =
      [this](PenServiceResponseFuture future) {
        auto request_response_pair = future.get();

        state_ = State::READY;
      
    };

    auto abs_response_received_callback =
      [this, pen_response_received_callback2](AbsServiceResponseFuture future) {
        auto request_response_pair = future.get();
        // put pen down
        auto request1 = std::make_shared<SetPen::Request>();
        request1->off = false;
        request1->width = 0.5;
        request1->r = 255;
        request1->g = 255;
        request1->b = 255;
        auto future2 = pen_client_->async_send_request(
          request1, std::move(pen_response_received_callback2)); 
        pen_ = false;
      
        
    };

    auto pen_response_received_callback1 =
      [this, abs_response_received_callback](PenServiceResponseFuture future) {
        auto request_response_pair = future.get();


        // get the last saved state
        auto p = saved_turtle_pose_.back();
        saved_turtle_pose_.pop_back();

        // create message
        auto request = std::make_shared<TeleportAbsolute::Request>();
        request->x = p.x;
        request->y = p.y;
        request->theta = p.theta;

        // publish message
        auto result = abs_tele_client_->async_send_request(
        request, std::move(abs_response_received_callback));

      
    };
    
    // move forward
    if (action == Actions::F)
    {
      RCLCPP_INFO(this->get_logger(), "Forward");

      // create message
      auto request = std::make_shared<TeleportRelative::Request>();
      request->linear = len_;

      // publish message
      auto result = rel_tele_client_->async_send_request(
        request, std::move(rel_response_received_callback));
      
    } else

    // turn right by angle_
    if (action == Actions::MINUS)
    {
      RCLCPP_INFO(this->get_logger(), "Turn Right");

      // create message
      auto request = std::make_shared<TeleportRelative::Request>();
      request->angular = -angle_;

      // publish message
      auto result = rel_tele_client_->async_send_request(
        request, std::move(rel_response_received_callback));
      
    } else

    // turn left by angle_
    if (action == Actions::PLUS)
    {
      RCLCPP_INFO(this->get_logger(), "Turn Left");

      // create message
      auto request = std::make_shared<TeleportRelative::Request>();
      request->angular = angle_;

      // publish message
      auto result = rel_tele_client_->async_send_request(
        request, std::move(rel_response_received_callback));
      
    } else

    // save current state
    if (action == Actions::SAVE)
    {
      RCLCPP_INFO(this->get_logger(), "Save state %d", int(saved_turtle_pose_.size() + 1));

      // save current turtle_pose_
      saved_turtle_pose_.push_back(*turtle_pose_);

      // set state ready
      state_ = State::READY;
      
    } else

    // restore saved state
    if (action == Actions::RESTORE)
    {
      RCLCPP_INFO(this->get_logger(), "Restore state %d", int(saved_turtle_pose_.size()));

      // lift pen up
      auto request1 = std::make_shared<SetPen::Request>();
      request1->off = true;

      auto future1 = pen_client_->async_send_request(
        request1, std::move(pen_response_received_callback1)); 
    } 

    else {
      // if it is X do nothing
      state_ = State::READY;
    }

  }

  std::vector<Actions> execute_rules(Actions act)
  {
  
    // Rule 1
    if (act == Actions::X){
      // replace with the sequence
      std::vector<Actions> new_list = {Actions::F, Actions::PLUS, Actions::SAVE, 
        Actions::SAVE, Actions::X, Actions::RESTORE, Actions::MINUS,
        Actions::X, Actions::RESTORE, Actions::MINUS, Actions::F, Actions::SAVE,
        Actions::MINUS, Actions::F, Actions::X, Actions::RESTORE, Actions::PLUS, Actions::X};
      return new_list;

    } 
    // Rule 2
    else if (act == Actions::F){

      // replace with 2 Fs
      std::vector<Actions> new_list = {Actions::F, Actions::F};
      return new_list;

    }
    // otherwise return
    else {
      std::vector<Actions> new_list = {act};
      return new_list;
    }

  }

private:
  State state_;
  float len_;
  float angle_;
  bool pen_;
  bool completed_;
  int count_;
  int super_count_;
  std::vector<Actions> new_list_;
  std::vector<Actions> action_list_;
  Pose::SharedPtr turtle_pose_;
  std::vector<Pose> saved_turtle_pose_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<Empty>::SharedPtr server_;
  rclcpp::Subscription<Pose>::SharedPtr subscription_;
  rclcpp::Client<TeleportAbsolute>::SharedPtr abs_tele_client_;
  rclcpp::Client<TeleportRelative>::SharedPtr rel_tele_client_;
  rclcpp::Client<SetPen>::SharedPtr pen_client_;
};

int main(int argc, char * argv[])
{
  // rclcpp::init(argc, argv);
  // rclcpp::executors::MultiThreadedExecutor executor;
  // executor.add_node(std::make_shared<Waypoint>());
  // executor.spin();
  // rclcpp::shutdown();

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Waypoint>());
  rclcpp::shutdown();
  return 0;
}