/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/turtle_master/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

bool robit_driving::start2019 = false;
int robit_driving::cds_data = 0, robit_driving::direction_angle = 0;
geometry_msgs::Twist robit_driving::motor_value;

namespace turtle_master
{
    using namespace std;

    robit_driving Driving;
    bool isRecved = false;
    bool button_clicked = false;

    QNode::QNode(int argc, char **argv) : init_argc(argc),
                                          init_argv(argv),
                                          turtlebot_master_node_th(argc, argv),
                                          node2_th(argc, argv),
                                          direction_angle(0.0),
                                          is_tunnel(false),
                                          arrive_goal(false),
                                          escape_imu(0.0),
                                          turn_imu(0.0),
                                          lidarDistance(0.0),
                                          lidarRightDistance(0.0),
                                          before_distance(0.0),
                                          push_num(0),
                                          tunnelState(0)
    {
    }

    QNode::~QNode()
    {
        wait();
    }

    bool QNode::init()
    {
        ros::init(init_argc, init_argv, "CORE");
        if (!ros::master::check())
        {
            return false;
        }
        start();
        return true;
    }

    void QNode::run()
    {
        turtlebot_master_node_th.start();
        node2_th.start();
        turtlebot_master_node_th.wait();
        node2_th.wait();
        std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
        Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
    }

    void turtlebot_master_node::run()
    {
        ros::init(init_argc, init_argv, "node_1");
        if (!ros::master::check())
        {
            return;
        }
        ros::start(); // explicitly needed since our nodehandle is going out of scope.
        ros::NodeHandle n;

        sub = n.subscribe("simple_message", 10, &turtlebot_master_node::subcallback, this);
        node_1_function();
    }

    void turtlebot_master_node::node_1_function()
    {
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    void node2::node_2_function()
    {
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            std_msgs::String msg;
            std::stringstream ss;

            auto tinenow = std::chrono::system_clock::now();
            std::time_t _time = std::chrono::system_clock::to_time_t(tinenow);

            ss << " I am the publish node. Current time: " << std::ctime(&_time);
            msg.data = ss.str();
            pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

} // namespace turtle_master
