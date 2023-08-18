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
#include "../include/qthread_ros/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qthread_ros
{

    /*****************************************************************************
    ** Implementation
    *****************************************************************************/

    QNode::QNode(int argc, char **argv) : init_argc(argc),
                                          init_argv(argv),
                                          node1_th(argc, argv),
                                          node2_th(argc, argv)
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
        node1_th.start();
        node2_th.start();
        node2_th.wait();
        node1_th.wait();
        std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
        Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
    }

    void node1::run()
    {
        ros::init(init_argc, init_argv, "node_1");
        if (!ros::master::check())
        {
            return;
        }
        ros::start(); // explicitly needed since our nodehandle is going out of scope.
        ros::NodeHandle n;

        sub = n.subscribe("simple_message", 10, &node1::subcallback, this);
        node_1_function();
    }

    void node1::node_1_function()
    {
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    void node1::subcallback(const std_msgs::String::ConstPtr msg)
    {
        ROS_INFO("I heard: [%s]", msg->data.c_str());
    }

    void node2::run()
    {
        ros::init(init_argc, init_argv, "node_2");
        if (!ros::master::check())
        {
            return;
        }
        ros::start(); // explicitly needed since our nodehandle is going out of scope.
        ros::NodeHandle n;

        pub = n.advertise<std_msgs::String>("simple_message", 10);
        node_2_function();
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

} // namespace qthread_ros
