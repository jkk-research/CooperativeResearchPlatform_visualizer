#include "visualizer/visualizer.hpp"


crp::apl::Visualizer::Visualizer() : Node("visualizer")
{
    m_sub_ego_   = this->create_subscription<crp_msgs::msg::Ego>(
        "/ego", 10, std::bind(&Visualizer::egoCallback, this, std::placeholders::_1));
    m_sub_scenario_   = this->create_subscription<crp_msgs::msg::Scenario>(
        "/scenario", 10, std::bind(&Visualizer::scenarioCallback, this, std::placeholders::_1));

    m_pub_egoVisualization_      = this->create_publisher<foxglove_msgs::msg::SceneUpdate>("egoVisualization", 10);
    m_pub_laneVisualization_      = this->create_publisher<visualization_msgs::msg::Marker>("laneVisualization", 10);
    m_pub_leftBoundVisualization_ = this->create_publisher<visualization_msgs::msg::Marker>("leftBoundVisualization", 10);    
    m_pub_rightBoundVisualization_ = this->create_publisher<visualization_msgs::msg::Marker>("rightBoundVisualization", 10);

    m_timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&Visualizer::run, this));

    RCLCPP_INFO(this->get_logger(), "visualizer has been started");
}

void crp::apl::Visualizer::egoCallback(const crp_msgs::msg::Ego::SharedPtr msg)
{
    // visualization
    foxglove_msgs::msg::SceneUpdate visu_msg;

    // Create an entity (represents your ego vehicle)
    foxglove_msgs::msg::SceneEntity entity;
    entity.id = "ego_vehicle";
    entity.timestamp = this->now();   // entity timestamp
    entity.frame_id = "map";    // or "map", depending on your TF setup
    entity.frame_locked = false;

    // Create a cube primitive
    foxglove_msgs::msg::CubePrimitive cube;
    cube.pose.position.x = msg->pose.pose.position.x;
    cube.pose.position.y = msg->pose.pose.position.y;
    cube.pose.position.z = 0.75;      // half height to sit on ground

    cube.pose.orientation.x = 1.0;
    cube.pose.orientation.y = 0.0;
    cube.pose.orientation.z = 0.0;
    cube.pose.orientation.w = 0.0;
    cube.size.x = 4.5;   // length
    cube.size.y = 2.0;   // width
    cube.size.z = 1.5;   // height
    cube.color.r = 0.0f;
    cube.color.g = 0.5f;
    cube.color.b = 1.0f;
    cube.color.a = 0.5f;

    // Attach cube to entity
    entity.cubes.push_back(cube);

    // Attach entity to scene update
    visu_msg.entities.push_back(entity);

    m_pub_egoVisualization_->publish(visu_msg);

    return;
}

void crp::apl::Visualizer::scenarioCallback(const crp_msgs::msg::Scenario::SharedPtr msg)
{
    visualizeLanes(msg);

    return;    
}

void crp::apl::Visualizer::visualizeLanes(const crp_msgs::msg::Scenario::SharedPtr msg)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->now();   

    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // scale.x = line width for LINE_STRIP
    marker.scale.x = 0.1;

    // color (RGBA)
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    marker.pose.orientation.w = 1.0;  // identity

    marker.lifetime = rclcpp::Duration::from_seconds(0.0); // persistent

    visualization_msgs::msg::Marker leftBoundMarker;
    leftBoundMarker.header.frame_id = "base_link";
    leftBoundMarker.header.stamp = this->now();   

    leftBoundMarker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    leftBoundMarker.action = visualization_msgs::msg::Marker::ADD;

    // scale.x = line width for LINE_STRIP
    leftBoundMarker.scale.x = 0.1;

    // color (RGBA)
    leftBoundMarker.color.r = 1.0f;
    leftBoundMarker.color.g = 0.0f;
    leftBoundMarker.color.b = 0.0f;
    leftBoundMarker.color.a = 1.0f;

    leftBoundMarker.pose.orientation.w = 1.0;  // identity

    leftBoundMarker.lifetime = rclcpp::Duration::from_seconds(0.0); // persistent

    visualization_msgs::msg::Marker rightBoundMarker;
    rightBoundMarker.header.frame_id = "base_link";
    rightBoundMarker.header.stamp = this->now();   

    rightBoundMarker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    rightBoundMarker.action = visualization_msgs::msg::Marker::ADD;

    // scale.x = line width for LINE_STRIP
    rightBoundMarker.scale.x = 0.1;

    // color (RGBA)
    rightBoundMarker.color.r = 1.0f;
    rightBoundMarker.color.g = 0.0f;
    rightBoundMarker.color.b = 0.0f;
    rightBoundMarker.color.a = 1.0f;

    rightBoundMarker.pose.orientation.w = 1.0;  // identity

    rightBoundMarker.lifetime = rclcpp::Duration::from_seconds(0.0); // persistent

    crp_msgs::msg::PathWithTrafficRules pathWithTrafficRules;
    pathWithTrafficRules.header.stamp = this->now();
    pathWithTrafficRules.header.frame_id = "base_link";

    tier4_planning_msgs::msg::PathWithLaneId path;

    if (msg->paths.size() > 0){
        path = msg->paths.at(0).path;
        for (long unsigned int np=0; np<path.points.size(); np++)
        {
            tier4_planning_msgs::msg::PathPointWithLaneId pathPoint;
            geometry_msgs::msg::Point leftBoundPoint;
            pathPoint = path.points.at(np);
            geometry_msgs::msg::Point p;
            p.x = pathPoint.point.pose.position.x;
            p.y = pathPoint.point.pose.position.y;
            p.z = pathPoint.point.pose.position.z;

            leftBoundPoint.x = pathPoint.point.pose.position.x;
            leftBoundPoint.y = pathPoint.point.pose.position.y + 1.875;

            marker.points.push_back(p);
            leftBoundMarker.points.push_back(leftBoundPoint);
        }

    }
   
    m_pub_laneVisualization_->publish(marker);
    m_pub_leftBoundVisualization_->publish(leftBoundMarker);
    m_pub_rightBoundVisualization_->publish(rightBoundMarker);

    return;
}

void crp::apl::Visualizer::run()
{
    return;
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<crp::apl::Visualizer>());
    rclcpp::shutdown();
    return 0;
}