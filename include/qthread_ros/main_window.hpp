/**
 * @file /include/qthread_ros/main_window.hpp
 *
 * @brief Qt based gui for qthread_ros.s
 **/
#ifndef qthread_ros_MAIN_WINDOW_H
#define qthread_ros_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QThread>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace qthread_ros
{
	class MainWindow : public QMainWindow
	{
		Q_OBJECT

	public:
		MainWindow(int argc, char **argv, QWidget *parent = 0);
		~MainWindow();

	public Q_SLOTS:

	private:
		Ui::MainWindowDesign ui;
		QNode qnode;
	};

} // namespace qthread_ros

#endif // qthread_ros_MAIN_WINDOW_H
