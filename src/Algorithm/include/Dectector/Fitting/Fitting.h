/**
 * (a * sin(w * (x_ + t)) + 2.090 - a)
 */

#ifndef FITTING_H
#define FITTING_H

# include "../Fitting/Fittool.h"

namespace detector{

using namespace std;
using namespace cv;
using namespace base;


class Fitting
{
public:
    vector<RuneArmor> armor_buffer;     // 存储一段时间内的未激活扇叶集合,用来计算速度
    vector<SpeedTime> fitting_data;     // 拟合数据集

private:

    Judgement judge;
    Fit fit;

    // double DT = 0.01;                // 采样时间间隔，单位：秒
    int N = 160;                         // 角速度采样数
    int N_min = 20;                     // 角速度最小采样数

    int DN = 1;                         // 逐差法测速度间距

    double start_time;                  // 拟合数据集中的第一个时间戳
    bool is_Inited = false;             // 大符拟合是否初始化
    bool is_direction_inited = false;   // 能量机关旋转方向初始化
    bool is_clockwise;                  // 顺时针

    double armor_buffer_erase_threshold = 0.2;    // 阈值，armor_buffer首末数据之差，若超过该阈值则清除armor_buffer首数据
    double call_speed_threshold = 0.005;          // 阈值，armor_buffer首末数据之差，若超过该阈值则计算角速度
    double clear_data_threshold = 0.8;            // 阈值，新来数据时间与上一时刻存入的时间之差，若超过该阈值则重置所有数据(进入clearData)

public:
    /**
     * @brief 初始化参数
    */
    Fitting();
    
    ~Fitting() = default;

    /**
     *  @brief  封装API
     */
    bool run(RuneArmor armor_1, vector<cv::Point2f> &nextPosition, TrackState armor_state, Mode rune_mode=Mode::RUNE);

protected:
    /**
     *  @brief  清空数据
     */
    void clearData();

    /**
     *  @brief  根据旋转角度和半径计算下一点(装甲板四个角点)的像素位置
     *  @param  point   动点
     *  @param  org     原点
     *  @param  angle   旋转角度
     */
    cv::Point2f calNextPosition(cv::Point2f point, cv::Point2f org, float angle);

    /**
     *  @brief  根据状态处理数据
     *  @param  armor_1 处理完的装甲板
     *  @param  timestamp   原图像时间戳
     */
    bool processDataState(RuneArmor armor_1, TrackState armor_state);

    /**
     *  @brief  数据作插值处理
     */
    void pushFittingData(SpeedTime new_data);

    /**
     *  @brief  判断能量机关旋转方向
     */
    void initDirection();

    /**
     *  @brief  清空armor_buffer
     */
    void clearArmorBuffer();

    /**
     *  @brief  计算瞬时角速度 (armor_1 - armor_2)
     *  @param  armor_1 新目标
     *  @param  armor_2 老目标
     *  @return 角速度，单位 弧度/秒
     */
    double calAngleSpeed(RuneArmor armor_1, RuneArmor armor_2);

    /**
     * @brief 计算两点距离
    */
    float calDistance(cv::Point2f pt1, cv::Point2f pt2)
    {
        cv::Point2f dis = pt1 - pt2;
        return sqrt(pow(dis.x,2)+pow(dis.y,2));
    };

};

}
#endif