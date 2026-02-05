#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP


#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <crp_msgs/msg/ego.hpp>
#include <crp_msgs/msg/scenario.hpp>

#include "foxglove_msgs/msg/scene_update.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace crp
{
namespace apl
{

struct Quaternion
{
    double x;
    double y;
    double z;
    double w;
};

class Visualizer : public rclcpp::Node
{
public:
    Visualizer();

private:
    void egoCallback(const crp_msgs::msg::Ego::SharedPtr msg);
    void scenarioCallback(const crp_msgs::msg::Scenario::SharedPtr msg);
    
    void run();

    float dT = 0.02f; // model runs in 20ms

    void visualizeEgo();
    void visualizeLanes(const crp_msgs::msg::Scenario::SharedPtr msg);

    rclcpp::Subscription<crp_msgs::msg::Scenario>::SharedPtr m_sub_scenario_;
    rclcpp::Subscription<crp_msgs::msg::Ego>::SharedPtr m_sub_ego_;

    rclcpp::Publisher<foxglove_msgs::msg::SceneUpdate>::SharedPtr m_pub_egoVisualization_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_pub_laneVisualization_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_pub_leftBoundVisualization_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_pub_rightBoundVisualization_;

    rclcpp::TimerBase::SharedPtr m_timer_;
};

} // namespace apl
} // namespace crp
#endif // VISUALIZER_HPP
