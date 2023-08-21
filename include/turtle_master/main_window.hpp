/**
 * @file /include/turtle_master/main_window.hpp
 *
 * @brief Qt based gui for turtle_master.
 *
 * @date November 2010
 **/
#ifndef turtle_master_MAIN_WINDOW_H
#define turtle_master_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <time.h>

#include <QMainWindow>
#include <QMessageBox>
#include <QMainWindow>
#include <QtGui>
#include <iostream>
#include <sys/socket.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <QString>

#include "qnode.hpp"
#include "ui_main_window.h"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace turtle_master
{
	class MainWindow : public QMainWindow
	{
		Q_OBJECT

	public:
		MainWindow(int argc, char **argv, QWidget *parent = 0);
		~MainWindow() {}
		void getIP();
		QString ipAddress;

	public Q_SLOTS:
		void updateData(void);
		void on_pushButton_clicked();

	private:
		Ui::MainWindowDesign ui;
		QNode qnode;
	};

} // namespace turtle_master

#endif // turtle_master_MAIN_WINDOW_H
