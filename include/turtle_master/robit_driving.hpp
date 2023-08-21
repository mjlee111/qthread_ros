#ifndef ROBIT_DRIVING_H
#define ROBIT_DRIVING_H

#include <geometry_msgs/Twist.h>

#include "iostream"
#include "robit_msgs/master_msg.h"
#include "robit_msgs/simple_move_msg.h"
#include "robit_msgs/vision_msg.h"

// linear Max speed  : 0.6366
// angualr Max speed : 7.9575

// 일반적으로 mode3을 위주로 대회 준비함. mode3이 속도랑 안정성이 제일 적당함.
// 더 빠르게 주행하고 싶으면 mode3은 일단 그대로 두고, mode4로 새롭게 건들면서
// 적당한 게인값 찾으면 됨.

#define mode3 // mode1 : slow, mode2 : normal, mode3 : fast

#ifdef mode1 // 주행 시 많이 비틀거림. 보정 필요.
#define Linear_MAX 0.22
#define STRAIGHT_GAIN 0.014

#define LEFT_CURVE_GAIN 0.0065 // bigger:inside, smaller:outside
#define RIGHT_CURVE_GAIN 0.009
#endif

#ifdef mode2
#define Linear_MAX 0.35
#define STRAIGHT_GAIN 0.002

#define LEFT_CURVE_GAIN 0.008 // bigger:inside, smaller:outside
#define RIGHT_CURVE_GAIN 0.0065
#endif

#ifdef mode3
#define Linear_MAX 0.90 // 0.480 //0.60 //0.48 //0.46  //0.44   //0.4

#define LINE_TURN_GAIN 0.045
#define STRAIGHT_GAIN LINE_TURN_GAIN    // 0.0010  //0.0015
#define LEFT_CURVE_GAIN LINE_TURN_GAIN  // 0.0085 //0.0085
#define RIGHT_CURVE_GAIN LINE_TURN_GAIN // 0.0075 //0.0075

#define P_GAIN 1.0
#define D_GAIN 6.0

#endif

#ifdef mode4
#define Linear_MAX 0.45
#define STRAIGHT_GAIN 0.0010

#define LEFT_CURVE_GAIN 0.0085 // bigger:inside, smaller:outside
#define RIGHT_CURVE_GAIN 0.0075

#define LEFT_CURVE_GAIN_LIGHT 0.0065
#define RIGHT_CURVE_GAIN_LIGHT 0.0060
#endif

#define ANGULAR_LINEAR_RATE 2
#define ANGLE_PIXEL_RATE 0.2
#define MAX_REVERSE 2.0

///////////////////////////////////////////

// 가감속의 게인값

#ifdef mode2
#define STRAIGHT_LINEAR_INCRESE_GAIN 0.0015
#define STRAIGHT_LINEAR_DECRESE_GAIN 0.003
#endif
#ifdef mode3
#define STRAIGHT_LINEAR_INCRESE_GAIN 0.1 // 0.0035
#define STRAIGHT_LINEAR_DECRESE_GAIN 0.08
#endif
#ifdef mode4
#define STRAIGHT_LINEAR_INCRESE_GAIN 0.0030
#define STRAIGHT_LINEAR_DECRESE_GAIN 0.0035 // 37
#endif

// #define STRAIGHT_ANGULAR_INCRESE_GAIN 0.004
// #define STRAIGHT_ANGULAR_DECRESE_GAIN 0.004
// #define CURVE_LINEAR_INCRESE_GAIN 0.004
// #define CURVE_LINEAR_DECRESE_GAIN 0.004
// #define CURVE_ANGULAR_INCRESE_GAIN 0.004
// #define CURVE_ANGULAR_DECRESE_GAIN 0.007

class robit_driving
{
public:
    robit_driving()
        : button_click(false),
          barCnt(false),
          rviz_init(false),
          tunnel_starts(false),
          zigzag_starts(false),
          navi_on(false)
    {
        master_msg.traffic_done = false;
        master_msg.cross_done = false;
        master_msg.gatebar_done = false;
        master_msg.parking_done = false;
        master_msg.construct_done = false;
        master_msg.zigzag_done = false;
        master_msg.tunnel_done = false;
        master_msg.led_init = false;
    }

    void Go();
    void update_parameter(const robit_msgs::vision_msg::ConstPtr &vision_data,
                          bool button_clicked);
    void analyze_situation();
    void set_speed(double linear, double angular);

    bool imu_turn();
    bool turn_on;
    double target_imu, target_spd;

    void linetracing(double speed);
    void linetracing2(double speed);
    void cross_motion();
    void construct_motion();
    void parking_motion();
    void retry();

    static geometry_msgs::Twist motor_value;
    static robit_msgs::vision_msg vision_msg;
    static robit_msgs::master_msg master_msg;
    static robit_msgs::simple_move_msg simple_msg;

    bool rviz_init;
    bool tunnel_starts;

    static int cds_data, direction_angle;
    static bool start2019;
    static bool start;

    static double now_linear_x;
    static double now_angular_z;
    static double before_linear_x;
    static double before_angular_z;

    static int mission_done_count;

private:
    bool button_click;
    double linear_x; //, angular_z;
    int left_line_x, right_line_x;
    int barCnt;

    int situation;
    enum
    {
        none = 0,
        traffic,
        cross,
        parking,
        construct,
        gatebar,
        zigzag,
        slow_go
    };
    enum
    {
        m_cross,
        m_construct,
        m_parking,
        m_gatebar,
        m_zigzag,
        m_tunnel
    };

    // cross
    enum
    {
        wait,
        drive,
        cross_end,
        turn_r1,
        turn_r1_1,
        turn_r1_2,
        turn_r1_3,
        turn_r2,
        turn_r3,
        turn_l1,
        turn_l1_1,
        turn_l2,
        turn_l3,
        turn_l4
    };

    // construct
    enum
    {
        slow,
        left_c1,
        front_c1,
        front_c2,
        front_c3,
        construct_end
    };

    // parking
    enum
    {
        mark,
        line1,
        go,
        gogo,
        turn1,
        line2,
        detect,
        turn2l,
        turn2r,
        back,
        turn3l,
        turn3r,
        line3,
        line4,
        turn4,
        parking_end
    };

    // zigzag
    bool zigzag_starts;

    // tunnel
    bool navi_on;

    // linetracing
    double min_angle;
    double max_angle;
    double min_linear_v;
    double max_linear_v;
};

#endif // ROBIT_DRIVING_H
