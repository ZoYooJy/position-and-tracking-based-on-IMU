#include "IMUTracker.h"
#include <iostream>
#include <vector>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{
    IMUTracker imu_tracker;
    imu_tracker.track();
    // saveTrajectoryTum("../imu_traj.tum", imu_tracker.vp_);

    // vector<Pose> -> vector<Isometry3d>
    // Isometry3d: [R t]
    //             [0 1]
    vector<Isometry3d, aligned_allocator<Isometry3d>> poses;
    for (auto &p : imu_tracker.vp_)
    {
        Isometry3d T(Quaterniond(p.orientation.w(), p.orientation.x(), p.orientation.y(), p.orientation.z()));
        T.pretranslate(Vector3d(p.position));
        poses.push_back(T);
    }
    showTrack(poses);

    return 0;
}