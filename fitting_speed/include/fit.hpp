#ifndef _FIT_HPP_
#define _FIT_HPP_

#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "time.h"
#include <filesystem>
#include <ceres/ceres.h>
#include <Eigen/Dense>

using namespace std;
using namespace cv;

typedef struct SpeedTime
{
    double angle_speed; // y
    double time;      // x

    SpeedTime(double speed = 0.0, double t = 0.0)
    {
        angle_speed = speed;
        time = t;
    }
} SpeedTime;

class newFitting
{
public:
    newFitting(){};
    ~newFitting()= default;

    bool run(vector<SpeedTime>& Fittingdata, double& w, Eigen::Vector3d& result);
private:
    double w_min = 1.884;
    double w_max = 2.000; 
    double dw = 0.001;
    bool fit(vector<SpeedTime> Fittingdata, double w, Eigen::Vector3d &output, double& error);

};

class DataWrite
{
public:
    DataWrite(){};
    ~DataWrite() = default;

    void countFilesInDirectory();

    void writeParam(double w, Eigen::Vector3d param);

    void writeToFile(SpeedTime data, int label);

private:
    string path = "../txt/";
    string filename;
    int fileCount = 0;
    ofstream txt;

};

#endif