#include "../include/fit.hpp"

bool newFitting::fit(vector<SpeedTime> Fittingdata, double w, Eigen::Vector3d &output, double &error)
{
    if(Fittingdata.empty())
        return false;

    // X,y 赋值
    Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
    Eigen::Vector3d N = Eigen::Vector3d::Zero();
    for(int i = 0; i < Fittingdata.size(); i++)
    {
        double x1i = sin(w * (Fittingdata[i].time - Fittingdata[0].time));
        double x2i = cos(w * (Fittingdata[i].time - Fittingdata[0].time));
        Eigen::Vector3d Xi(x1i, x2i, 1);
        M += Xi * Xi.transpose();
        double yi = Fittingdata[i].angle_speed;
        N += Xi * yi;
    }
    // 判断M是否有逆矩阵;为0,矩阵不可逆
    if(M.determinant() == 0)
        return 0;
    output = M.inverse() * N;
    // cout<<"output:"<<output<<endl;

    // 计算误差
    double temp_error = 0.0;
    for(int i = 0; i < Fittingdata.size(); i++)
    {
        double x1i = sin(w * (Fittingdata[i].time - Fittingdata[0].time));
        double x2i = cos(w * (Fittingdata[i].time - Fittingdata[0].time));           
        
        double yi = Fittingdata[i].angle_speed;
        double temp = x1i*output(0) + x2i*output(1) + output(2) - yi;
        temp_error += pow(temp,2);
    }
    error = temp_error;
    return true;
}


bool newFitting::run(vector<SpeedTime>& Fittingdata, double& w, Eigen::Vector3d& result)
{
    bool change_data = false;   // 拟合是否成功标志位
    double w_temp = this->w_min; 
    double w_best = w_min;
    double min_error = DBL_MAX;
    while(w_temp <= this->w_max)
    {
        Eigen::Vector3d output;
        double error;
        if(fit(Fittingdata, w_temp, output, error))
        {
            // cout<<"error:"<<error<<endl;
            if(min_error > error)
            {
                change_data = true;
                min_error = error;
                result = output;;
                w_best = w_temp;
            }
        }
        w_temp += this->dw;
    }

    if(!change_data)
    {
        cout<<"fit error"<<endl;
        return false;
    }

    cout<<"error:"<<min_error<<endl;
    w = w_best;;
    return true;
}

void DataWrite::countFilesInDirectory()
 {
    this->fileCount = 0;
    for (const auto& entry : std::filesystem::directory_iterator(path)) 
    {
        if (entry.is_regular_file() || entry.is_directory()) {
            this->fileCount++;
        }
    }
     // 创建文件夹
    this->filename = path + to_string(fileCount) + ".txt";
}

void DataWrite::writeParam(double w, Eigen::Vector3d param) 
{
    txt.open(filename, ios::app);
    if (txt.is_open()) 
    {
        txt << w << " " << param(0) << " " << param(1) << " " << param(2) << endl;
        txt.close();
    }
    else
    {
        std::cerr << "Unable to open the file.";
    }
}

void DataWrite::writeToFile(SpeedTime data, int label) {
    txt.open(filename, ios::app);
    if (txt.is_open()) {
        txt << data.time << " " << data.angle_speed << " " << label << endl;
        txt.close();
    } else {
        std::cerr << "Unable to open the file.";
    }
}