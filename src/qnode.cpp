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
                                          init_argv(argv), nodeth(init_argc, init_argv)
    {
    }

    QNode::~QNode()
    {
        wait();
    }

    bool QNode::init()
    {
        start();
        return true;
    }

    void QNode::run()
    {
        nodeth.start();
        nodeth.wait();
        Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
    }

    void node1::run()
    {
        ros::init(n_argc, n_argv, "node_1");
        if (!ros::master::check())
        {
            return;
        }
        ros::start(); // explicitly needed since our nodehandle is going out of scope.
        ros::NodeHandle n;
        ros::Rate loop_rate(100);
        while (ros::ok())
        {
            ROS_INFO("NODE1 RUNNING!");
            ros::spinOnce();
            loop_rate.sleep();
        }
        std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    }

} // namespace qthread_ros
