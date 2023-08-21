/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMessageBox>
#include <QtGui>
#include <iostream>

#include "../include/turtle_master/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/
robit_msgs::vision_msg robit_driving::vision_msg;
robit_msgs::master_msg robit_driving::master_msg;
robit_msgs::simple_move_msg robit_driving::simple_msg;

namespace turtle_master
{

    using namespace Qt;
    using namespace std;
    extern bool isRecved;
    extern bool button_clicked;

    /*****************************************************************************
    ** Implementation [MainWindow]
    *****************************************************************************/

    MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
        : QMainWindow(parent), qnode(argc, argv)
    {
        button_clicked = false;
        ui.setupUi(this);
        qnode.init();

        QIcon icon("://images/icon.png");
        setWindowIcon(icon);
        getIP();
        move(0, 0);
        QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
        QObject::connect(&qnode, SIGNAL(Recv_Data()), this, SLOT(updateData()));
    }

    void MainWindow::updateData()
    {
        ui.label_imu->setText(QString::number(robit_driving::direction_angle));
        ui.label_rel->setText(QString::number(robit_driving::vision_msg.rel_angle));
        ui.label_cds->setText(QString::number(robit_driving::cds_data));
        ui.label_linear_x->setText(
            QString::number(robit_driving::motor_value.linear.x));
        ui.label_angular_z->setText(
            QString::number(robit_driving::motor_value.angular.z));
        ui.label_tunnel_detect->setText(QString::number(qnode.tunnelState));
        ui.rel_ratio->setText(
            QString::number(robit_driving::vision_msg.rel_angle_ratio));
        if (robit_driving::vision_msg.l_line_info)
        {
            ui.label_left_detect->setText("Detect");
            ui.label_left_pixel->setText(
                QString::number(robit_driving::vision_msg.l_diff_pixel));
        }
        else
        {
            ui.label_left_detect->setText("NO POINT");
            ui.label_left_pixel->setText("NO POINT");
        }

        if (robit_driving::master_msg.traffic_done == 1)
        {
            ui.label_traffic_green->setText("Detect");
        }
        else
        {
            ui.label_traffic_green->setText("NO");
        }

        if (robit_driving::vision_msg.r_line_info)
        {
            ui.label_right_detect->setText("Detect");
            ui.label_right_pixel->setText(
                QString::number(robit_driving::vision_msg.r_diff_pixel));
        }
        else
        {
            ui.label_right_detect->setText("NO POINT");
            ui.label_right_pixel->setText("NO POINT");
        }

        if (robit_driving::vision_msg.gatebar_detect)
            ui.label_gatebar_detect->setText("Detect");
        else
            ui.label_gatebar_detect->setText("NOPE");

        if (robit_driving::vision_msg.parking_detect)
            ui.label_parking_sign->setText("Detect");
        else
            ui.label_parking_sign->setText("NOPE");

        if (robit_driving::vision_msg.cross_detect)
            ui.label_cross->setText("Detect");
        else
            ui.label_cross->setText("NOPE");

        isRecved = false;
    }

    void MainWindow::on_pushButton_clicked()
    {
        if (button_clicked)
        {
            button_clicked = false;
            ui.pushButton->setText("start");
            robit_driving::master_msg.traffic_done = false;
            robit_driving::start = false;
        }
        else
        {
            button_clicked = true;
            ui.pushButton->setText("stop");
            robit_driving::master_msg.traffic_done = true;
            robit_driving::start = true;
        }
    }

    void MainWindow::getIP()
    {
        struct ifaddrs *ifaddr = nullptr;
        if (getifaddrs(&ifaddr) == -1)
        {
            std::cerr << "getifaddrs failed" << std::endl;
            return;
        }

        for (struct ifaddrs *ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next)
        {
            if (ifa->ifa_addr == nullptr)
            {
                continue;
            }

            if (ifa->ifa_addr->sa_family == AF_INET)
            { // IPv4 address
                char ip[INET_ADDRSTRLEN];
                struct sockaddr_in *addr = reinterpret_cast<struct sockaddr_in *>(ifa->ifa_addr);
                if (strcmp(ifa->ifa_name, "wlp4s0") == 0) // Check for wlp4s0 interface
                {
                    if (inet_ntop(AF_INET, &(addr->sin_addr), ip, INET_ADDRSTRLEN) != nullptr)
                    {
                        ipAddress = QString::fromUtf8(ip);
                        break;
                    }
                }
            }
        }

        freeifaddrs(ifaddr);

        if (ipAddress.isEmpty())
        {
            std::cerr << "No IP address found for wlp4s0" << std::endl;
            return;
        }

        ui.ip_address->setText(ipAddress);
    }

    /*
    void MainWindow::on_reset_button_clicked()
    {

            button_clicked = false;
            ui.reset_button->setText("clicked");
            robit_driving::master_msg.traffic_done = false;
            robit_driving::start = false;

    }*/

} // namespace turtle_master
