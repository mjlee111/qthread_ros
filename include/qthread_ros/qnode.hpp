/**
 * @file /include/qthread_ros/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef qthread_ros_QNODE_HPP_
#define qthread_ros_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include "std_msgs/String.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qthread_ros
{

	/*****************************************************************************
	** Class
	*****************************************************************************/
	class node1 : public QThread
	{
		Q_OBJECT

	public:
		node1(int argc, char **argv) : n_argc(argc), n_argv(argv) {}
		void node_1_function();

		ros::Subscriber sub;
		void subcallback(const std_msgs::String::ConstPtr msg);

	Q_SIGNALS:

	private:
		int n_argc;
		char **n_argv;

	protected:
		void run();
	};

	class node2 : public QThread
	{
		Q_OBJECT

	public:
		node2(int argc, char **argv) : n_argc(argc), n_argv(argv) {}
		void node_2_function();

		ros::Publisher pub;

	Q_SIGNALS:

	private:
		int n_argc;
		char **n_argv;

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

		node1 node1_th;
		node2 node2_th;

	Q_SIGNALS:
		void
		rosShutdown();

	private:
		int init_argc;
		char **init_argv;
	};

} // namespace qthread_ros

#endif /* qthread_ros_QNODE_HPP_ */
