#pragma once

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <pangolin/pangolin.h>
#include <unistd.h>
#include <vector>

using namespace std;
using namespace Eigen;

struct Pose
{
    uint64_t timestamp;
    Vector3d position;
    Quaterniond orientation;
    Vector3d linear_vel;
    Vector3d ang_vel;
};

struct Point
{
    Vector3d pos;
    Matrix3d orien;
    Vector3d ang_vel;
    Vector3d linear_vel;
};

struct ImuFrame
{
    uint64_t timestamp;
    Vector3d ang_vel;
    Vector3d acc_vel;
};

/*
void LoadEuRocIMUData(const std::string &strImuPath, std::vector<IMUData> &imu_datas)
{

    std::ifstream fImus;
    fImus.open(strImuPath.c_str());
    imu_datas.reserve(30000);
    // int testcnt = 10;

    while (!fImus.eof())
    {
        std::string s;
        getline(fImus, s);
        if (!s.empty())
        {
            char c = s.at(0);
            if (c < '0' || c > '9')
                continue;

            std::stringstream ss;
            ss << s;
            double tmpd;
            int cnt = 0;
            double data[10]; // timestamp, wx,wy,wz, ax,ay,az
            while (ss >> tmpd)
            {
                data[cnt] = tmpd;
                cnt++;
                if (cnt == 7)
                    break;
                if (ss.peek() == ',' || ss.peek() == ' ')
                    ss.ignore();
            }
            data[0] *= 1e-9;
            IMUData imudata(data[1], data[2], data[3], data[4], data[5], data[6], data[0]);
            imu_datas.push_back(imudata);
        }
    }
}*/

/**
 * @brief
 *
 * @param[in]  path
 * @param[out] imu_msg_buffer
 * @return true
 * @return false
 */
bool readIMUData(const string &path, vector<ImuFrame> &imu_msg_buffer)
{
    ifstream fin;
    fin.open(path, ios::in);
    if (!fin.is_open())
    {
        cout << "open file failed." << endl;
        return false;
    }

    // char data[7];
    // while (fin.getline(data, sizeof(data)))
    // {
    //     ImuFrame IMUData;
    //     IMUData.timestamp = data[0];
    //     IMUData.ang_vel = Vector3d(data[1], data[2], data[3]);
    //     IMUData.acc_vel = Vector3d(data[4], data[5], data[6]);

    //     imu_msg_buffer.push_back(IMUData);
    // }

    for (int i = 0; i < 36820; i++)
    {
        double data[7];
        for (int j = 0; j < 7; j++)
        {
            fin >> data[j];
        }

        ImuFrame IMUData;
        IMUData.timestamp = data[0];
        IMUData.ang_vel = Vector3d(data[1], data[2], data[3]);
        IMUData.acc_vel = Vector3d(data[4], data[5], data[6]);

        imu_msg_buffer.push_back(IMUData);
    }

    // cout << "read imu data size: " << imu_msg_buffer.size() << endl;
    // for (auto &imu : imu_msg_buffer)
    //     cout << fixed << setprecision(17) << imu.ang_vel.x() << " " << imu.ang_vel.y() << " " << imu.ang_vel.z()
    //          << endl;

    return true;
}

/**
 * @brief
 *
 * @param filename
 * @param vPose
 */
void saveTrajectoryTum(const string &filename, vector<Pose> &vPose)
{
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for (auto iter = vPose.begin(); iter != vPose.end(); iter++)
    {
        Quaterniond q = (*iter).orientation;
        Vector3d t = (*iter).position;
        f << setprecision(6) << (*iter).timestamp << setprecision(7) << " " << t(0) << " " << t(1) << " " << t(2) << " "
          << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << endl;
    }
    f.close();
}

void showTrack(vector<Isometry3d, aligned_allocator<Isometry3d>> &poses)
{
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
                                      pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));

    pangolin::View &d_cam = pangolin::CreateDisplay()
                                .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
                                .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        // glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        // glLineWidth(20);

        // for (size_t i = 0; i < poses.size(); i++)
        // {
        //     Vector3d Ow = poses[i].translation();
        //     Vector3d Xw = poses[i] * (/*0.1 **/ Vector3d(1, 0, 0));
        //     Vector3d Yw = poses[i] * (/*0.1 **/ Vector3d(0, 1, 0));
        //     Vector3d Zw = poses[i] * (/*0.1 **/ Vector3d(0, 0, 1));
        //     glBegin(GL_LINES);
        //     glColor3f(1.0, 0.0, 0.0);
        //     glVertex3d(Ow[0], Ow[1], Ow[2]);
        //     glVertex3d(Xw[0], Xw[1], Xw[2]);
        //     glColor3f(0.0, 1.0, 0.0);
        //     glVertex3d(Ow[0], Ow[1], Ow[2]);
        //     glVertex3d(Yw[0], Yw[1], Yw[2]);
        //     glColor3f(0.0, 0.0, 1.0);
        //     glVertex3d(Ow[0], Ow[1], Ow[2]);
        //     glVertex3d(Zw[0], Zw[1], Zw[2]);
        //     glEnd();
        // }

        // for (size_t i = 0; i < poses.size(); i++)
        // {
        //     glColor3f(0.0, 0.0, 0.0);
        //     glBegin(GL_LINES);
        //     auto p1 = poses[i], p2 = poses[i + 1];
        //     glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
        //     glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
        //     glEnd();
        // }

        for (auto &pose : poses)
        {
            glPointSize(1.0);
            glBegin(GL_POINTS);
            glColor3f(0.0, 1.0, 0.0);
            glVertex3d(pose(0, 2), pose(1, 2), pose(2, 2)); // t
            glEnd();
        }

        pangolin::FinishFrame();
        usleep(5000); // sleep 5 ms
    }
}

class IMUTracker
{
  public:
    IMUTracker()
    {
        Vector3d zero{0.0, 0.0, 0.0};
        point_.pos = zero;
        point_.orien = Matrix3d::Identity();
        point_.linear_vel = zero;
        point_.ang_vel = zero;
        firstFrame_ = true;

        if (!readIMUData("../imu_data.txt", imu_msg_buffer_))
        {
            cout << "read imu data failed." << endl;
        }

        Pose p0;
        p0.timestamp = imu_msg_buffer_[0].timestamp;
        p0.position = Vector3d(0., 0., 0.);
        p0.orientation = Quaterniond(1., 0., 0., 0.);
        p0.linear_vel = Vector3d(0., 0., 0.);
        p0.ang_vel = Vector3d(0., 0., 0.);
        vp_.push_back(p0);
    }

    ~IMUTracker()
    {
    }

    void track()
    {
        for (auto &msg : imu_msg_buffer_)
        {
            if (firstFrame_)
            {
                prev_time_ = msg.timestamp;
                deltaT_ = 0;
                setGravity(msg.acc_vel);
                firstFrame_ = false;
            }
            else
            {
                deltaT_ = (msg.timestamp - prev_time_) * 1e-9;
                prev_time_ = msg.timestamp;
                calOrien(msg.ang_vel);
                calPos(msg.acc_vel);
                updatePos(point_);
            }
        }
    }

    /**
     * @brief set the first imu frame's acc_vel as gravity
     *
     * @param msg
     */
    void setGravity(Vector3d &msg)
    {
        // gravity_[0] = msg[0];
        // gravity_[1] = msg[1];
        // gravity_[2] = msg[2];
        gravity_ = Vector3d(msg);
    }

    /**
     * @brief
     *
     * @param msg acceleration velocity
     */
    void calPos(Vector3d &msg)
    {
        Vector3d acc_i(msg);
        Vector3d acc_w = point_.orien * acc_i;
        point_.linear_vel += deltaT_ * (acc_w - gravity_);
        point_.pos += deltaT_ * point_.linear_vel;
    }

    /**
     * @brief
     *
     * @param msg angular velocty
     */
    void calOrien(Vector3d &msg)
    {
        point_.ang_vel = msg;
        Matrix3d B; // 角速度 * 时间 = 角度（表示为反对称矩阵）
        B << 0, -msg.z() * deltaT_, msg.y() * deltaT_, msg.z() * deltaT_, 0, -msg.x() * deltaT_, -msg.y() * deltaT_,
            msg.x() * deltaT_, 0;

        double sigma = sqrt(pow(msg.x(), 2) + pow(msg.y(), 2) + pow(msg.z(), 2)) * deltaT_;
        point_.orien *= (Matrix3d::Identity() + (sin(sigma) / sigma) * B - ((1 - cos(sigma)) / pow(sigma, 2)) * B * B);
    }

    /**
     * @brief current pose
     *
     * @param point
     */
    void updatePos(Point &point)
    {
        Pose p;

        p.timestamp = prev_time_;

        p.position = Vector3d(point.pos);
        p.linear_vel = Vector3d(point.linear_vel);
        p.ang_vel = Vector3d(point.ang_vel);

        p.orientation.x() = (point.orien(2, 1) - point.orien(1, 2)) / 4;
        p.orientation.y() = (point.orien(0, 2) - point.orien(2, 0)) / 4;
        p.orientation.z() = (point.orien(1, 0) - point.orien(0, 1)) / 4;
        p.orientation.w() = sqrt(1 + point.orien(0, 0) + point.orien(1, 1) + point.orien(2, 2)) / 2;

        vp_.push_back(p);
    }

  public:
    Point point_;
    bool firstFrame_;
    vector<ImuFrame> imu_msg_buffer_;

    Vector3d gravity_;
    double deltaT_;
    uint64_t prev_time_; // current imu frame's timestamp
    vector<Pose> vp_;
};
