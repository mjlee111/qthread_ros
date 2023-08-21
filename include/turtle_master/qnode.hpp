/**
 * @file /include/turtle_master/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef turtle_master_QNODE_HPP_
#define turtle_master_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <ros/network.h>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/UInt32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <sensor_msgs/LaserScan.h>

#include <math.h>
#include "iostream"
#include <std_srvs/Empty.h>
#include "robit_driving.hpp"
#include "robit_msgs/vision_msg.h"
#include "robit_msgs/master_msg.h"
#include "robit_msgs/simple_move_msg.h"

#include <std_msgs/Header.h>
#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#endif

#define RAD2DEG (180.0 / M_PI)

#define THESH_RIGHT_DISTANCE 0.15 // 0.15
#define THESH_FRONT_DISTANCE 0.22 // 0.15
#define TURN_GAIN 0.025
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace turtle_master
{

	/*****************************************************************************
	** Class
	*****************************************************************************/
	class turtlebot_master_node : public QThread
	{
		Q_OBJECT

	public:
		turtlebot_master_node(int argc, char **argv) : init_argc(argc), init_argv(argv) {}
		void node_1_function();

		robit_msgs::vision_msg vision_data;
		robit_msgs::simple_move_msg simple_move_data;

		/*tunnel*/
		bool is_tunnel, arrive_goal;
		double start_tunning, escape_imu, turn_imu;
		double lidarDistance, lidarRightDistance;
		double before_distance;
		int push_num;
		int kill_MB;
		int clear_map_cnt = 0;
		int tunnelState, tunnelEscapeState;
		enum
		{
			TUNNEL_INIT,
			TUNNEL_MID,
			TUNNEL_END
		};
		enum
		{
			TUNNEL_WALL,
			TUNNEL_GATE,
			TUNNEL_ESCAPE
		};

		void rvizInit();
		void tunnelInit();
		void tunnelProcess();
		void escapeTunnel();

	Q_SIGNALS:
		void Recv_Data();

	public Q_SLOTS:
		void turtle_run();

	private:
		int init_argc;
		char **init_argv;

		double direction_angle;
		std_msgs::Bool set_imu_is_on;

		// tunnel data
		geometry_msgs::PoseWithCovarianceStamped initial_pose;
		geometry_msgs::PoseStamped goal_pose;
		actionlib_msgs::GoalID move_base_cancel;

		ros::ServiceClient client;

		// publisher
		ros::Publisher Master_Data_pub;
		ros::Publisher Mot_Data_pub;
		ros::Publisher initial_pose_pub;
		ros::Publisher goal_pose_pub;
		ros::Publisher set_imu_pub;
		ros::Publisher move_base_cancel_pub;

		// subscriber
		ros::Subscriber Vision_Data_sub;
		ros::Subscriber Sw_Data_sub1;
		ros::Subscriber Sw_Data_sub2;
		ros::Subscriber Imu_Data_sub;
		ros::Subscriber Cds_Data_sub;
		ros::Subscriber laser_sub;
		ros::Subscriber move_sub;
		ros::Subscriber goal_arrival_status_sub;
		ros::Subscriber dwa_error_sub;

		void VisionDataCallback(const robit_msgs::vision_msg::ConstPtr &vision_msg);
		void Sw1DataCallback(const std_msgs::UInt32::ConstPtr &sw_data);
		void Sw2DataCallback(const std_msgs::UInt32::ConstPtr &sw_data);
		void imuMsgCallback(const std_msgs::UInt32 &msg);
		void CdsDataCallback(const std_msgs::UInt32::ConstPtr &cds_msg);
		void laserMsgCallback(const sensor_msgs::LaserScan &laser_arr);
		void goalStateCallback(const move_base_msgs::MoveBaseActionResult &goal_arrival_status);
		void SimpleDataCallback(const robit_msgs::simple_move_msg::ConstPtr &simple_msg);
		void ClearcostmapsCallback(const std_msgs::Bool &msg);

	protected:
		void run();
	};

	class node2 : public QThread
	{
		Q_OBJECT

	public:
		node2(int argc, char **argv) : init_argc(argc), init_argv(argv) {}
		void node_2_function();

		ros::Publisher pub;

	Q_SIGNALS:

	private:
		int init_argc;
		char **init_argv;

	protected:
		void run();
	};

	class QNode : public QThread
	{
		Q_OBJECT
	public:
		QNode(int argc, char **argv);
		virtual ~QNode();
		bool init();
		void run();

		turtlebot_master_node turtlebot_master_node_th;
		node2 node2_th;

	Q_SIGNALS:
		void
		rosShutdown();

	private:
		int init_argc;
		char **init_argv;
	};

} // namespace turtle_master

#endif /* turtle_master_QNODE_HPP_ */
