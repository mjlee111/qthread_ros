#include <cmath>

#include "../include/turtle_master/robit_driving.hpp"

#define SAFETY_MODE 0
#define TURN_GAIN 0.025 // imu turn gain
using namespace std;

double robit_driving::now_linear_x = 0.0;
double robit_driving::now_angular_z = 0.0;
double robit_driving::before_linear_x = 0.0;
double robit_driving::before_angular_z = 0.0;

bool robit_driving::start = false;
int robit_driving::mission_done_count = 1;

int MIDDLE_OF_IMAGE = 162; // 162  //158

void robit_driving::Go()
{
    // Sw1이 안눌렸으면 다이나믹셀을 0,0 줘서 가만히 있게 함.
    if (robit_driving::start2019 == 0)
    {
        set_speed(0.0, 0.0);
    }

    // Sw1을 눌르면 그때부터 움직이도록 함.
    else
    {
        ////for gain test
        ////push sw1 && !push start
        // linetracing(Linear_MAX);
        // return;
        if (start) // 처음엔 false인데, 신호등 감지하면 start == true 됨
        {
            master_msg.traffic_done = true;
            analyze_situation();
            // 이 함수를 통해 어떤상황인지 알아내고, 그 상황에 맞게 아래 switch문이
            // 돌아가는 것임.

            switch (situation)
            {
            // 일반적인, 교차로미션, 공사장미션, 주차미션, 게이트바 미션, 지그재그
            // 코스가 아닌 경우 : 즉, 일반적인 linetracing하는 상황이랑, 터널
            // 상황이라고 보면 됨.
            case none:
                if (tunnel_starts)
                {
                    master_msg.tunnel_done = true;
                    mission_done_count++;
                }
                if (mission_done_count == vision_msg.mission_sequence[m_cross])
                {
                    if (vision_msg.before_cross == 1)
                    {
                        // set_speed(0.0, 0.0);
                        set_speed(0.3 * fabs(vision_msg.rel_angle_ratio),
                                  1.1 * fabs(vision_msg.rel_angle_ratio));
                    }
                    else if (vision_msg.before_cross == 2 &&
                             !vision_msg.cross_detect)
                    {
                        set_speed(0.00, 0.0); // set_speed(0.0, 0.0); //iy 0.01, 0.0
                    }
                    else
                    {
                        linetracing(Linear_MAX);
                    }
                }
                if (vision_msg.slow_zone)
                {
                    linetracing(0.16);
                }
                else if (navi_on == true && (vision_msg.rel_angle < 280 &&
                                             vision_msg.rel_angle > 260))
                {
                    linetracing(0.28); // 25
                }
                else
                    linetracing(Linear_MAX);
                break;

            // 교차로 미션 상황
            case cross:
                cross_motion();
                break;

            // 공사장 미션 상황
            case construct:
                construct_motion();
                break;

            // 주차장 미션 상황
            case parking:
                parking_motion();
                break;

            // 차단바 미션 상황
            case gatebar:
                barCnt++;
                set_speed(0.0, 0.0);
                break;

            // 지그재그 상황
            case zigzag:
                zigzag_starts = true;
                linetracing(0.2);
                break;
            case slow_go:
                linetracing(0.2);
                break;
            default:
                break;
            }
        }
    }
}

void robit_driving::update_parameter(
    const robit_msgs::vision_msg::ConstPtr &vision_data, bool button_clicked)
{
    vision_msg.r_diff_pixel = vision_data->r_diff_pixel;
    vision_msg.l_diff_pixel = vision_data->l_diff_pixel;
    vision_msg.r_line_info = vision_data->r_line_info;
    vision_msg.l_line_info = vision_data->l_line_info;
    vision_msg.r_angle = vision_data->r_angle;
    vision_msg.l_angle = vision_data->l_angle;
    vision_msg.slow_mode = vision_data->slow_mode;

    right_line_x = vision_msg.r_diff_pixel;
    left_line_x = vision_msg.l_diff_pixel;

    vision_msg.traffic = vision_data->traffic;
    vision_msg.cross_detect = vision_data->cross_detect;
    vision_msg.before_cross = vision_data->before_cross;

    vision_msg.cross_info = vision_data->cross_info;
    vision_msg.cross_pixel = vision_data->cross_pixel;
    vision_msg.parking_detect = vision_data->parking_detect;
    vision_msg.parking_info = vision_data->parking_info;
    vision_msg.construct_detect = vision_data->construct_detect;
    vision_msg.construct_info = vision_data->construct_info;
    vision_msg.slow_zone = vision_data->slow_zone;
    vision_msg.gatebar_detect = vision_data->gatebar_detect;
    vision_msg.zigzag_detect = vision_data->zigzag_detect;
    vision_msg.rel_angle = vision_data->rel_angle;
    vision_msg.rel_angle_ratio = vision_data->rel_angle_ratio;
    vision_msg.just_before_tunnel_num = vision_data->just_before_tunnel_num;
    button_click = button_clicked;

    for (int i = 0; i < 6; i++)
        vision_msg.mission_sequence[i] = vision_data->mission_sequence[i];

    vision_msg.retry = vision_data->retry;
    if (vision_msg.retry != 0)
        retry();

    if (vision_msg.traffic == 1)
        start = true;

    // 차단바 미션 진입하기 전에 일부러 천천히 달리는 조건문임. 너무 빨리 달리다가
    // 차단바 내려오면 반응 못하고 부딪힘.
    if (barCnt > 2 && !vision_msg.gatebar_detect && !master_msg.gatebar_done)
    {
        master_msg.gatebar_done = true;
        mission_done_count++;
        set_speed(0.15, 0.0);
    }
    if (master_msg.parking_done &&
        (vision_msg.rel_angle > 350 || vision_msg.rel_angle < 10) &&
        !master_msg.gatebar_done)
    {
        linetracing2(0.2);
    }

    if (zigzag_starts && !vision_msg.zigzag_detect && !master_msg.zigzag_done)
    {
        master_msg.zigzag_done = true;
        mission_done_count++;
    }

    if (mission_done_count > vision_msg.just_before_tunnel_num && !navi_on)
    {
        std::cout << "!!!!now rviz on!!!! : " << endl;
        std::cout << mission_done_count << endl;
        rviz_init = true;
        navi_on = true;
    }
}

// 터틀봇이 실제 대회에서 주행 중에 라인이탈해서 빠지는 경우 뒤에서부터 다시
// 시작할때 재시작 버튼 누름.
void robit_driving::retry()
{
    /// cross
    if (vision_msg.mission_sequence[m_cross] >= vision_msg.retry)
    {
        master_msg.cross_done = false;
    }

    /// construct
    if (vision_msg.mission_sequence[m_construct] >= vision_msg.retry)
    {
        master_msg.construct_done = false;
    }

    /// parking
    if (vision_msg.mission_sequence[m_parking] >= vision_msg.retry)
    {
        master_msg.parking_done = false;
    }

    /// gatebar
    if (vision_msg.mission_sequence[m_gatebar] >= vision_msg.retry)
    {
        master_msg.gatebar_done = false;
        barCnt = 0;
    }

    /// zigzag
    if (vision_msg.mission_sequence[m_zigzag] >= vision_msg.retry)
    {
        master_msg.zigzag_done = false;
        zigzag_starts = false;
    }

    /// tunnel
    if (vision_msg.mission_sequence[m_tunnel] >= vision_msg.retry)
    {
        master_msg.tunnel_done = false;
        tunnel_starts = false;
        rviz_init = false;
        navi_on = false;
    }
    mission_done_count = vision_msg.retry;
}

void robit_driving::analyze_situation()
{
    // If we change below, we can modularize this code.
    if (vision_msg.slow_mode)
    {
        situation = slow_go;
    }
    else if (vision_msg.cross_detect && !master_msg.cross_done)
    {
        situation = cross;
    }
    else if (vision_msg.construct_detect && !master_msg.construct_done)
    {
        situation = construct;
    }
    else if (vision_msg.parking_detect && !master_msg.parking_done)
    {
        situation = parking;
    }
    else if (vision_msg.gatebar_detect && !master_msg.gatebar_done)
    {
        situation = gatebar;
    }
    else if (vision_msg.zigzag_detect && !master_msg.zigzag_done)
    {
        situation = zigzag;
    }
    else
    {
        situation = none;
    }
}

// 주차장 미션 진행할 때 직각 턴을 하는데 이때, 빠르고 정확하게 턴하기 위한 함수.
bool robit_driving::imu_turn()
{
    double imu_gap, angular_spd;
    imu_gap = fabs(target_imu - vision_msg.rel_angle);
    if (fabs(imu_gap) > 180)
        imu_gap = 360 - fabs(imu_gap);
    angular_spd = imu_gap * TURN_GAIN * 2;
    if (target_spd < 0)
        angular_spd *= -1;
    if ((target_spd > 0 && angular_spd > target_spd) ||
        (target_spd < 0 && angular_spd < target_spd))
        angular_spd = target_spd;
    if (fabs(imu_gap) < 3)
    {
        set_speed(0.0, 0.0);
        return false;
    }
    else
    {
        set_speed(0.0, angular_spd);
        return true;
    }
}

// 최종적으로 이 함수를 통해서 OpenCR과 연결된 다이나믹셀이 움직이게 됨.
void robit_driving::set_speed(double linear, double angular)
{
    motor_value.linear.x = linear;
    motor_value.angular.z = angular;

    // before_linear_x = linear;
    // before_angular_z = angular;
}

/*void robit_driving::linetracing(double speed) {
  // f1, f2는 이전의 data를 얼마나 신뢰해서 가져올 것인지 정도를 조절하는
  // 변수임.
  // f1 = 0.2    f2 = 0.8 이면, 현재 usb_cam으로 보는 값은 80%신뢰, 과거의
  // data는 20%신뢰해서 판단하겠다는 것임.
  double f1 = 0.20;      // 0.35  //before_ angular
  double f2 = 1.0 - f1;  // now _ cam

  linear_x = speed;
  static double pre_pixel_gap = 0.0;
  static int pixel_offset = 25;

#ifdef mode1
  double min_angle = 0.2;
  double max_angle = 1.8;
  double min_linear_v = 0.10;
  double max_linear_v = 0.22;
#endif

#ifdef mode2
  double min_angle = 0.1;
  double max_angle = 3.6;
  double min_linear_v = 0.1;
  double max_linear_v = 0.32;
#endif

#ifdef mode3
  min_angle = 0.1;
  max_angle = 2.5;  // 3.6
  min_linear_v = speed * 0.3;
  max_linear_v = speed * 0.7;
#endif

#ifdef mode4
  min_angle = 0.1;
  max_angle = 3.5;
  min_linear_v = 0.1;
  max_linear_v = 0.4;
#endif

  static double angular_z = 0.0;
  double pixel_gap = 0.0;
  if (vision_msg.r_line_info && vision_msg.l_line_info) {
    // 양쪽라인을 모두 감지하는 경우 : 직진 상황
    //////////////////////////////////////////////////////////Target Angular

    double middlePoint((left_line_x + right_line_x) / 2.0);
    double middleAngle((vision_msg.r_angle + vision_msg.l_angle) / 2.0);
    pixel_gap = ((middleAngle - 90.0) * (1.0 - ANGLE_PIXEL_RATE) +
                 ((double)160.0 - middlePoint) * ANGLE_PIXEL_RATE) *
                STRAIGHT_GAIN;  // 290
    // pixel_offset=(320-right_line_x+left_line_x)/2.0;
  } else if (vision_msg.r_line_info && !vision_msg.l_line_info) {
    //흰색 라인만 감지하는 경우 : 왼쪽으로 꺾어야 하는 상황

    //터틀봇 경기장 중에 지그재그 코스가 있는데 여기 코스에서는 linetracing 게인
    //값을 따로 주기 위한 조건문.
    if (vision_msg.zigzag_detect) {
      min_angle = 0.1;
      max_angle = 6;
      min_linear_v = 0.01;
      max_linear_v = 0.45;

      pixel_gap = fabs((double)(320 - pixel_offset) - right_line_x) *
                  LEFT_CURVE_GAIN;  // 290
    } else {  //지그재그 코스가 아닌 일반적인 linetracing 경우.
      // mode1, 2, 3, 4는 linetracing 속도라고 보면 됨. mode가 증가 할수록
      // 속도가 더 빠름, 대신 이탈가능성 높아짐. : 단순 게인값 튜닝이라고
      // 생각하면 됨.

      pixel_gap =
          ((vision_msg.r_angle - 90.0) * (1.0 - ANGLE_PIXEL_RATE) +
           ((double)(320 - pixel_offset) - right_line_x) * ANGLE_PIXEL_RATE) *
          LEFT_CURVE_GAIN;  // 290

      if (pixel_gap < 0) {  //우측을 볼때 우회전을 한다는 것은 직진을 할경우
                            //선이 또 보인다는 것을 의미
        if (pixel_gap < -MAX_REVERSE) {
          pixel_gap = -MAX_REVERSE;
        }
        pixel_gap = -pixel_gap;
        pre_pixel_gap = pixel_gap;

        angular_z =
            f2 * (pixel_gap * P_GAIN + (pixel_gap - pre_pixel_gap) * D_GAIN) +
            f1 * angular_z;
        if (angular_z > max_angle) {
          angular_z = max_angle;
        } else if (angular_z < -max_angle) {
          angular_z = -max_angle;
        }
        set_speed(before_linear_x,
                  angular_z * before_linear_x * ANGULAR_LINEAR_RATE);
        return;
      }
    }

  } else if (!vision_msg.r_line_info && vision_msg.l_line_info) {
    // 노란색 라인만 감지하는 경우 : 오른쪽으로 꺾어야 하는 상황
    //바로 위 else if 조건문의 반대 상황이라고 생각하면 됨.

    if (vision_msg.zigzag_detect) {
      min_angle = 0.1;
      max_angle = 6;
      min_linear_v = 0.01;
      max_linear_v = 0.45;
      pixel_gap = -fabs((double)pixel_offset - left_line_x) * RIGHT_CURVE_GAIN;

    } else {
      pixel_gap = ((vision_msg.l_angle - 90.0) * (1.0 - ANGLE_PIXEL_RATE) +
                   ((double)pixel_offset - left_line_x) * ANGLE_PIXEL_RATE) *
                  RIGHT_CURVE_GAIN;
      if (pixel_gap > 0) {
        if (pixel_gap > MAX_REVERSE) {
          pixel_gap = MAX_REVERSE;
        }
        pixel_gap = -pixel_gap;
        pre_pixel_gap = -pixel_gap;

        angular_z =
            f2 * (pixel_gap * P_GAIN + (pixel_gap - pre_pixel_gap) * D_GAIN) +
            f1 * angular_z;
        if (angular_z > max_angle) {
          angular_z = max_angle;
        } else if (angular_z < -max_angle) {
          angular_z = -max_angle;
        }
        set_speed(before_linear_x,
                  angular_z * before_linear_x * ANGULAR_LINEAR_RATE);
        return;
      }
    }
  } else {
    pre_pixel_gap = 0;
  }

  angular_z = f2 * (pixel_gap * P_GAIN + (pixel_gap - pre_pixel_gap) * D_GAIN) +
              f1 * angular_z;

  if (angular_z > max_angle) {
    angular_z = max_angle;
  } else if (angular_z < -max_angle) {
    angular_z = -max_angle;
  }

  if (fabs(angular_z) > min_angle) {
    linear_x = (((max_linear_v - min_linear_v) / (min_angle - max_angle)) *
                (fabs(angular_z) - min_angle)) +
               max_linear_v;
  }

  //////////////////////////////////////////////////////////Acceleration &
  /// Deacceleration START : 가감속 코드임. : 정해진 값을 그대로 주면 터틀봇이
  ///덜컹덜컹 거림. 정해진 값을 target으로 두고 target될 때까지 가속하는 식

  if (before_linear_x > linear_x + STRAIGHT_LINEAR_DECRESE_GAIN) {
    before_linear_x -= STRAIGHT_LINEAR_DECRESE_GAIN;
  } else if (before_linear_x < linear_x - STRAIGHT_LINEAR_INCRESE_GAIN) {
    before_linear_x += STRAIGHT_LINEAR_INCRESE_GAIN;
  } else {
    before_linear_x = linear_x;
  }
  //////////////////////////////////////////////////////////Acceleration &
  /// Deacceleration END

  set_speed(before_linear_x, angular_z * before_linear_x * ANGULAR_LINEAR_RATE);
  // 최종적으로 결정된 값을 set_speed 함수에 넣어줌 :
  // set_speed 함수는 다이나믹셀로 값 전달해주는 친구임.
  pre_pixel_gap = pixel_gap;
}*/

void robit_driving::linetracing(double speed)
{
    double min_spd = 0.5;
    double max_spd = 0.8;
    static double before_spd = 0;
    static double pixel_offset = 30;
    double now_spd;
    double th_c;
    double diff_c;
    if (vision_msg.r_line_info && vision_msg.l_line_info)
    {
        diff_c = 160 - (left_line_x + right_line_x) / 2.0;
        th_c =
            (90 - (vision_msg.r_angle + vision_msg.l_angle) / 2.0) * M_PI / 180.0;
    }
    else if (!vision_msg.r_line_info && vision_msg.l_line_info)
    {
        diff_c = pixel_offset - left_line_x;
        th_c = (90 - vision_msg.l_angle) * M_PI / 180.0;
    }
    else if (vision_msg.r_line_info && !vision_msg.l_line_info)
    {
        diff_c = (320 - pixel_offset) - right_line_x;
        th_c = (90 - vision_msg.r_angle) * M_PI / 180.0;
    }
    else
    {
        return;
    }
    double tan_th_r(tan(th_c) * 20 / 38);
    double diff_r(38 * diff_c / 320);
    double path_R(-(diff_r * tan_th_r + 23.5) / tan(atan(tan_th_r) / 2) / 100.0);

    std::cout << path_R << std::endl;
    now_spd = 0.2;

    if (before_spd > now_spd + STRAIGHT_LINEAR_DECRESE_GAIN)
    {
        before_spd -= STRAIGHT_LINEAR_DECRESE_GAIN;
    }
    else if (before_spd < now_spd - STRAIGHT_LINEAR_INCRESE_GAIN)
    {
        before_spd += STRAIGHT_LINEAR_INCRESE_GAIN;
    }
    else
    {
        before_spd = now_spd;
    }

    double ang_spd = before_spd / path_R;
    set_speed(before_spd, ang_spd);
}
void robit_driving::linetracing2(double speed)
{
    // f1, f2는 이전의 data를 얼마나 신뢰해서 가져올 것인지 정도를 조절하는
    // 변수임.
    // f1 = 0.2    f2 = 0.8 이면, 현재 usb_cam으로 보는 값은 80%신뢰, 과거의
    // data는 20%신뢰해서 판단하겠다는 것임.
    double f1 = 0.25;     // 0.35  //before_ angular
    double f2 = 1.0 - f1; // now _ cam

    linear_x = speed;

    if (vision_msg.r_line_info &&
        vision_msg.l_line_info) // 왼쪽 : 흰색, 오른쪽 : 노란색   양쪽라인을 모두
                                // 감지하는 경우 : 직진 상황
    {
        //////////////////////////////////////////////////////////Target Angular

        double middlePoint = 0.0;
        static double angular_z = 0.0;
        double pixel_gap = 0.0;

        middlePoint = (left_line_x + right_line_x) / 2.0;
        pixel_gap = ((double)MIDDLE_OF_IMAGE - middlePoint) * STRAIGHT_GAIN;
        angular_z = f2 * pixel_gap + f1 * angular_z;

        //////////////////////////////////////////////////////////Acceleration &
        /// Deacceleration START

        if (before_linear_x > linear_x + 0.01)
            before_linear_x -= STRAIGHT_LINEAR_DECRESE_GAIN;
        else if (before_linear_x < linear_x - 0.01)
            before_linear_x += STRAIGHT_LINEAR_INCRESE_GAIN;
        else if ((before_linear_x > linear_x - 0.01) &&
                 (before_linear_x < linear_x + 0.01))
            before_linear_x = linear_x;

        //////////////////////////////////////////////////////////Acceleration &
        /// Deacceleration END

        set_speed(before_linear_x, angular_z);
        // std::cout<< " linear : "<<before_linear_x <<"   angular : "<< angular_z
        // << " gogogo"<< endl; std::cout << " go go move move" << endl;
    }

    else if (vision_msg.r_line_info &&
             !vision_msg
                  .l_line_info) // only white line detect : 왼쪽 : 흰색 라인만
                                // 감지하는 경우 : 오른쪽으로 꺾어야 하는 상황
    {
        static double angular_z = 0.0;
        double pixel_gap = 0.0;

        // 터틀봇 경기장 중에 지그재그 코스가 있는데 여기 코스에서는 linetracing 게인
        // 값을 따로 주기 위한 조건문.
        if (vision_msg.zigzag_detect)
        {
            min_angle = 0.1;
            max_angle = 6;
            min_linear_v = 0.01;
            max_linear_v = 0.3;

            pixel_gap = fabs((double)290 - right_line_x) * LEFT_CURVE_GAIN; // 290
        }

        // 지그재그 코스가 아닌 일반적인 linetracing 경우.
        else
        {
            // mode1, 2, 3, 4는 linetracing 속도라고 보면 됨. mode가 증가 할수록
            // 속도가 더 빠름, 대신 이탈가능성 높아짐. : 단순 게인값 튜닝이라고
            // 생각하면 됨.
#ifdef mode1
            double min_angle = 0.2;
            double max_angle = 1.8;
            double min_linear_v = 0.10;
            double max_linear_v = 0.22;
#endif

#ifdef mode2
            double min_angle = 0.1;
            double max_angle = 3.6;
            double min_linear_v = 0.1;
            double max_linear_v = 0.32;
#endif

#ifdef mode3
            min_angle = 0.1;
            max_angle = 3.0; // 3.6
            min_linear_v = 0.1;
            max_linear_v = 0.32;
#endif

#ifdef mode4
            min_angle = 0.1;
            max_angle = 3.5;
            min_linear_v = 0.1;
            max_linear_v = 0.4;
#endif

            pixel_gap = fabs((double)290 - right_line_x) * LEFT_CURVE_GAIN; // 290
        }

        // std::cout<< " linear : "<<linear_x <<"   angular : "<< angular_z << "
        // left
        // "<< endl;

#ifdef mode4

        if (vision_msg.r_diff_pixel < 270) // 290 30
        {
            pixel_gap = fabs((double)290 - right_line_x) * LEFT_CURVE_GAIN;
            std::cout << " linear : " << linear_x << "   angular : " << angular_z
                      << "   left     " << endl;
        }

        else
        {
            pixel_gap = fabs((double)300 - right_line_x) * LEFT_CURVE_GAIN_LIGHT;
            std::cout << " linear : " << linear_x << "   angular : " << angular_z
                      << "   left light" << endl;
        }

#endif
        angular_z = f2 * pixel_gap + f1 * angular_z;

        if (angular_z > min_angle)
            linear_x = (((max_linear_v - min_linear_v) / (min_angle - max_angle)) *
                        (angular_z - min_angle)) +
                       max_linear_v;

        //////////////////////////////////////////////////////////Acceleration &
        /// Deacceleration START : 가감속 코드임. : 정해진 값을 그대로 주면 터틀봇이
        /// 덜컹덜컹 거림. 정해진 값을 target으로 두고 target될 때까지 가속하는 식

        if (before_linear_x > linear_x + 0.01)
            before_linear_x = linear_x;
        else if (before_linear_x < linear_x - 0.01)
            before_linear_x += STRAIGHT_LINEAR_INCRESE_GAIN;
        else if ((before_linear_x > linear_x - 0.01) &&
                 (before_linear_x < linear_x + 0.01))
            before_linear_x = linear_x;

        //////////////////////////////////////////////////////////Acceleration &
        /// Deacceleration END

        //        if(vision_msg.zigzag_detect)

        //        {
        //            set_speed(0.3, 1.0);

        //        }
        //            else{
        set_speed(
            before_linear_x,
            angular_z); // 최종적으로 결정된 값을 set_speed 함수에 넣어줌 :
                        // set_speed 함수는 다이나믹셀로 값 전달해주는 친구임.

        //        }
        // std::cout<< " linear : "<<linear_x <<"   angular : "<< angular_z << endl;

        // std::cout << " go go left left " << endl;
    }

    else if (!vision_msg.r_line_info &&
             vision_msg.l_line_info) // only yellow line detect  : 오른쪽 :
                                     // 노란색 라인만 감지하는 경우 : 왼쪽으로
                                     // 꺾어야 하는 상황
    {
        // 바로 위 else if 조건문의 반대 상황이라고 생각하면 됨.

        static double angular_z = 0.0;
        double pixel_gap = 0.0;

        if (vision_msg.zigzag_detect)
        {
            min_angle = 0.1;
            max_angle = 6;
            min_linear_v = 0.01;
            max_linear_v = 0.3;
            pixel_gap = -fabs((double)30 - left_line_x) * RIGHT_CURVE_GAIN;
        }
        else
        {
#ifdef mode1
            double min_angle = 0.2;
            double max_angle = 1.8;
            double min_linear_v = 0.15;
            double max_linear_v = 0.22;
#endif

#ifdef mode2
            double min_angle = 0.1;
            double max_angle = 3.6;
            double min_linear_v = 0.1;
            double max_linear_v = 0.32;
#endif

#ifdef mode3
            min_angle = 0.1;
            max_angle = 3.0; // 3.8
            min_linear_v = 0.1;
            max_linear_v = 0.32;
#endif

#ifdef mode4
            min_angle = 0.1;
            max_angle = 3.5;
            min_linear_v = 0.1;
            max_linear_v = 0.4;
#endif
            pixel_gap = -fabs((double)30 - left_line_x) * RIGHT_CURVE_GAIN;
        }

        // std::cout<< " linear : "<<linear_x <<"   angular : "<< angular_z << "
        // right
        // "<< endl;

#ifdef mode4

        if (vision_msg.l_diff_pixel > 50)
        {
            pixel_gap = -fabs((double)30 - left_line_x) * RIGHT_CURVE_GAIN;
            std::cout << " linear : " << linear_x << "   angular : " << angular_z
                      << "   right      " << endl;
        }
        else
        {
            pixel_gap = -fabs((double)20 - left_line_x) * RIGHT_CURVE_GAIN_LIGHT;
            std::cout << " linear : " << linear_x << "   angular : " << angular_z
                      << "   right light " << endl;
        }
#endif
        angular_z = f2 * pixel_gap + f1 * angular_z;

        if (angular_z < -min_angle)
            linear_x = (((max_linear_v - min_linear_v) / (min_angle - max_angle)) *
                        ((-angular_z) - min_angle)) +
                       max_linear_v;

        //////////////////////////////////////////////////////////Acceleration &
        /// Deacceleration START

        if (before_linear_x > linear_x + 0.01)
            before_linear_x = linear_x;
        else if (before_linear_x < linear_x - 0.01)
            before_linear_x += STRAIGHT_LINEAR_INCRESE_GAIN;
        else if ((before_linear_x > linear_x - 0.01) &&
                 (before_linear_x < linear_x + 0.01))
            before_linear_x = linear_x;

        //////////////////////////////////////////////////////////Acceleration &
        /// Deacceleration END

        //         if(vision_msg.zigzag_detect)

        //         {
        //             set_speed(0.3, -1.0);

        //         }
        //             else{
        set_speed(before_linear_x, angular_z);

        //         }

        //   std::cout<< " linear : "<<linear_x <<"   angular : "<< angular_z <<
        //   endl;
        //              std::cout << " go go right right " << endl;
    }
}

void robit_driving::cross_motion()
{
    // khd
    switch (vision_msg.cross_info)
    {
    case wait:
        linetracing(0.3);
        break;

    case drive:
        linetracing(Linear_MAX);
        //  set_speed(0.0, 0.0);
        break;

    case turn_r1:
        set_speed(0, 1.0); // 0.2
        break;

    case turn_r1_1: // turn after identifying the sign
        set_speed(0.2, 0.3);
        break;

    case turn_r1_2: // turn after identifying the sign
        set_speed(0.2, -1.8);
        // //set_speed(0.28, -1.2);
        break;

    case turn_r1_3: // turn after identifying the sign
        set_speed(0.2, -2.1);
        // //set_speed(0.28, -1.2);
        break;
    case turn_r2:
        set_speed(0.3, 0.5);
        break;

    case turn_r3:             // exit
        set_speed(0.5, -2.1); // 2.6//2.3
        break;

    case turn_l1: // x
        set_speed(0.25, 2.0);
        break;
    case turn_l1_1: // x
        set_speed(0, 2.0);
        break;

    case turn_l2:
        set_speed(0.25, 1.4);
        break;

    case turn_l3:
        set_speed(0.30, -1.0); // set_speed(0.30, -1.0);//0.8
        break;

    case turn_l4:
        set_speed(0.2, 2.0); // 1.9
        break;

    case cross_end:
        linetracing(Linear_MAX);
        master_msg.cross_done = true;
        mission_done_count++;
        std::cout << "mission_done_count : " << mission_done_count << endl;
        break;
    }
}

void robit_driving::construct_motion()
{
    switch (vision_msg.construct_info)
    {
    case slow:
        linetracing(0.4); // Linear_MAX
        break;
    case left_c1:
        set_speed(0.35, (138 - vision_msg.rel_angle) / 27.0); // 0.05
        break;
    case front_c1:
        set_speed(0.40, -1.2); // 75
        break;
    case front_c2:
        set_speed(0.35, -1.0);
        break;
    case front_c3:
        set_speed(0.3, 1.25);
        break;

    case construct_end:
        linetracing(Linear_MAX);
        master_msg.construct_done = true;
        mission_done_count++;
        std::cout << "mission_done_count : " << mission_done_count << endl;
        break;
    }
}

void robit_driving::parking_motion()
{
    switch (vision_msg.parking_info)
    {
    case mark:
        linetracing(0.4); // Linear_MAX
        break;
    case line1:
        linetracing(0.4); // Linear_MAX
        break;
    case go:
        linetracing(0.4); // Linear_MAX
        break;
        ////////////////////////// left yellow line detect (o x o)

    case gogo:
        set_speed(0.25, 0.0); // 0.22
        break;

    case turn1:               // parking jinip
        set_speed(0.32, 1.1); // 0.3, 1.44// 0.3, 1.5
        break;

    case line2:
        linetracing2(0.3);
        break;

    case detect:
        linetracing2(0.1);
        break;

    case turn2l: // the obstacle exists at right
        turn_on = true;
        target_imu = 25.0; // 10.0; //5.0;
        target_spd = -2.50;
        break;

    case turn2r: // the obstacle exists at left
        turn_on = true;
        target_imu = 155.0; // 170.0; //180.0;
        target_spd = 2.5;   // 2.84
        break;

    case back:
        turn_on = false;
        set_speed(-0.29, 0.0);
        break;

    case turn3l:               // right obs
        set_speed(0.3, -1.10); // set_speed(0.25, -1.10); //-1.30
        break;

    case turn3r:              // left obs
        set_speed(0.3, 1.10); // set_speed(0.25, 1.00);  //1.60
        break;

    case line3:
        linetracing2(0.30);
        break;

    case line4:
        linetracing2(0.30);
        break;

    case turn4:
        set_speed(0.24, 1.1); // set_speed(0.20,1.23); //set_speed(0.25,1.23);
                              // //set_speed(0.18,1.0);//1.23
        break;

    case parking_end:
        linetracing(Linear_MAX);
        master_msg.parking_done = true;
        mission_done_count++;
        std::cout << "mission_done_count : " << mission_done_count << endl;
        break;
    }
}
