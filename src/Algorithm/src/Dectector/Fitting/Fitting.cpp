#include "Dectector/Fitting/Fitting.h"

namespace detector
{

    /*-----------Fitting-----------*/
    Fitting::Fitting(){}

    bool Fitting::run(base::RuneArmor armor_1, vector<cv::Point2f> &nextPosition, TrackState armor_state, base::Mode rune_mode)
    {
        // 数据处理
        if (!processDataState(armor_1, armor_state))
            return false;

        // 确定方向
        initDirection();
        if(!is_direction_inited)
            return false;

        // 对于大符模式，Fitting_data超过一定数量才能进行拟合
        if(rune_mode == base::Mode::RUNE && fitting_data.size() < N_min)
            return false;

        nextPosition.clear();
        vector<cv::Point2f> pts;
        armor_1.getPoints(pts);
        double delta = fit.run(fitting_data, N, rune_mode);            // 旋转角度
        for (int i = 0; i < 4; i++)
            nextPosition.push_back(calNextPosition(pts[i], armor_1.circle_center, delta));
        
        return true;         
    }

    void Fitting::clearData()
    {
        cout << "Clear Fitting Data!" << endl;
        fitting_data.clear();
        armor_buffer.clear();
        judge.resetJudge();
        is_direction_inited = false;
    }

    cv::Point2f Fitting::calNextPosition(cv::Point2f point, cv::Point2f org, float rotate_angle)
    {
        double radius = calDistance(point, org);
        cv::Point2f relative_point = point - org;                                         // 相对坐标
        double relative_angle = atan2(relative_point.y, relative_point.x);                // 与圆心所成角度
        double next_angle;

        if (is_clockwise) // 顺时针运动
        {
            next_angle = relative_angle + rotate_angle;
            if (next_angle > CV_PI)
                next_angle -= 2.0 * CV_PI;
        }
        else
        {
            next_angle = relative_angle - rotate_angle;
            if (next_angle < - CV_PI)
                next_angle += 2.0 * CV_PI;
        }

        return cv::Point2f(cos(next_angle) * radius, sin(next_angle) * radius) + org;
    }

    bool Fitting::processDataState(RuneArmor armor_1, TrackState armor_state)
    {
        switch (armor_state)
        {
        case base::TrackState::DETECTING:
            clearArmorBuffer();
            armor_buffer.push_back(armor_1);
            break;

        case base::TrackState::TRACKING:
            armor_buffer.push_back(armor_1);
            while (armor_buffer.size() > 1 + DN)
            {
                double delta_time = armor_1.timestamp - armor_buffer[0].timestamp;
                if (delta_time > armor_buffer_erase_threshold)
                    armor_buffer.erase(armor_buffer.begin());
                else if (delta_time > call_speed_threshold)
                {
                    double temp_speed = calAngleSpeed(armor_1, armor_buffer[armor_buffer.size() - 1 - DN]);
                    if(judge.Judge(temp_speed, is_direction_inited, is_clockwise))
                    {
                        SpeedTime temp_state = SpeedTime(temp_speed, (armor_1.timestamp + armor_buffer[armor_buffer.size() - 1 - DN].timestamp) / 2);
                        pushFittingData(temp_state);
                    }
                    break;
                }
                else
                    break;
            }

            break;

        case TrackState::LOST:
            armor_buffer.clear();
            fitting_data.clear();
            judge.resetJudge();
            return false;
            break;
            
        default:
            return false;
            break;
        }
        while (fitting_data.size() > N)
            fitting_data.erase(fitting_data.begin());
        
        return true;
    }

    void Fitting::pushFittingData(SpeedTime new_data)
    {
        if (fitting_data.empty())
        {
            fitting_data.push_back(new_data);
            return;
        }
        SpeedTime flag_data = fitting_data[fitting_data.size() - 1];
        
        double n = new_data.time - flag_data.time;

        // 时间差过长则清除数据;0.8为超参数,可调 
        if (n > clear_data_threshold)
        {
            clearData();
            return;
        }

        fitting_data.push_back(new_data);

        // 去除线性插值
        // if ((double)new_data.time - (double)flag_data.time - DT < 0)
        // {
        //     return;
        // }

        // double T = DT;

        // for (int i = 0; i < (int)n; i++)
        // {
        //     double temp_T = T * (i + 1);
        //     double delta = (double)new_data.time - (double)flag_data.time - temp_T;
        //     SpeedTime temp = SpeedTime(new_data.angle_speed * (temp_T / (temp_T + delta)) + flag_data.angle_speed * (delta / (temp_T + delta)), flag_data.time + (double)temp_T);  
        //     fitting_data.push_back(temp);  
        // }
    }

    void Fitting::initDirection()
    {
        if (fitting_data.size() < N_min)
            return;

        int clock = 0, clock_inv = 0;
        for (int i = 0; i < fitting_data.size(); i++)
        {
            if (fitting_data[i].angle_speed > 0)
                clock++;
            else
                clock_inv++;
        }
        is_direction_inited = true;
        is_clockwise = clock > clock_inv;

    }

    void Fitting::clearArmorBuffer()
    {
        armor_buffer.clear();
        judge.resetJudge();
    }

    double Fitting::calAngleSpeed(RuneArmor armor_1, RuneArmor armor_2)
    {
        double time_diff = (double)(armor_1.timestamp - armor_2.timestamp);

        if(time_diff < 0.005)
            time_diff += 0.005;

        double angle_diff = armor_1.angle - armor_2.angle;
        if (armor_1.angle < -CV_PI / 2.0 && armor_2.angle > CV_PI / 2.0)
            angle_diff = angle_diff + CV_PI * 2.0;
        else if (armor_1.angle > CV_PI / 2.0 && armor_2.angle < -CV_PI / 2.0)
            angle_diff = angle_diff - CV_PI * 2.0;
        
        return angle_diff / time_diff; // 转换单位
    }

}
