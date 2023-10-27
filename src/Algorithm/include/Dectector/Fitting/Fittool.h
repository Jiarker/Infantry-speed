/**
 * spd = (a * sin(w * (x_ + t)) + 2.090 - a)
 * 转化:spd = P1*sin(wt) + P2*cos(wt) + P3
 * 
 */

#ifndef FITTOOL_H
#define FITTOOL_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include "../../Base/rune_armor.hpp"
#include "../../Base/const.hpp"
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <filesystem>

namespace detector{

using namespace std;

typedef struct SpeedTime
{
    double angle_speed; // y
    double time;      // x

    SpeedTime(double speed = 0.0, double t = 0)
    {
        angle_speed = speed;
        time = t;
    }
} SpeedTime;

class Judgement{
public:
    Judgement(){};
    ~Judgement() = default;

    /**
     * @brief 判断数据是否为坏值
     * @param judge_speed 待判断数据
     * @param is_derection_inited 能量机关方向是否初始化
     * @param is_clockwise 能量机关方向是否是顺时针 
    */
    bool Judge(double& judge_speed, bool is_direction_inited, bool is_clockwise);

    /**
     * @brief 判断器重置
    */
    void resetJudge();

private:
    vector<double> speedJudge;          // 判断器数据集
    double n;
    double mean;
    double variance;
    double standard_deviation;
    int judge_clear_num = 0;            // 坏值累加器,若大于3,则重置判断器

    /**
     * @brief 得到判断器内数据集个数
    */
    void getN();

    /**
     * @brief 计算数据集的平均值
    */
    void getMean();

    /**
     * @brief 计算方差
    */
    void getVariance();

    /**
     * @brief 处理坏值
    */
    void solveBadData();
};

class Fit
{
public:
    Fit();
    ~Fit()= default;

    double delay_time;      // 预测时间，程序耗时时间 + 云台相应时间 + 子弹飞行时间
    int save_txt;           // 保存txt文件设置
    int print_result;       // 打印设置

    /**
     * @brief 返回旋转角，若为大符模式，则需要拟合；若为小符模式，直接计算旋转角度
     * @param Fittingdata 传来的数据
     * @param N Fittingdata的最大值
     * @param rune_mode 打符模式
    */
    double run(vector<SpeedTime> Fittingdata, double N, base::Mode rune_mode = base::Mode::NORMAL_RUNE);

private:
    double normal_rune_speed = CV_PI / 3;

    // 拟合数据
    double w_min = 1.884;
    double w_max = 2.000; 
    double dw = 0.001;
    
    double w = 1.9;
    double P1 = 0.9;
    double P2 = 0.0;
    double P3 = 1.19;

    double start_time = -1.0;          // 角度变化的初值,不为拟合SpeedTime的初始时间;成功拟合的情况下才改变

    // txt文件保存数据
    string path = "./src/Algorithm/configure/Detector/Fitting/buff_state/";
    string filename;
    bool have_file_count = false;
    bool have_first_time =false;
    double first_time;
    int fileCount = 0;
    ofstream txt;

    /**
     * @brief 拟合主函数
     * @param Fittingdata 传来的数据
     * @param w 遍历得到
     * @param output 拟合输出结果，为三维向量
     * @param error 拟合损失
    */
    bool fitting(vector<SpeedTime> Fittingdata, double w, Eigen::Vector3d &output, double& error);

    /**
     * @brief 得到旋转角
     * @param t 预测时刻与初始时间之差，注意初始时间不一定是数据的首数据
    */
    double getRotateAngle(double t);

    /**
     * @brief 得到预测数据
     * @param t 预测时刻与初始时间之差，注意初始时间不一定是数据的首数据
    */
    SpeedTime predictSpeed(double t);

    /**
     * @brief 绘制数据
     * @param predict 预测数据
     * @param now 当前数据
    */
    void drawData(SpeedTime predict, SpeedTime now);

    /**
     * @brief 计算文件夹的文件数量
    */
    void countFilesInDirectory();    
};

}

#endif