#ifndef __TEST_SLAM2D_H__
#define __TEST_SLAM2D_H__

#include "test_point2point.h"
#include "test_point2pose.h"
#include "test_slam_g2o.h"
#include "test_slam3d_g2o.h"

#include <iostream>
#include <chrono>



class Test
{
public:
    int Run()
    {
        // TestPoint2PointSlam test_point2point;
        // test_point2point.Run();
        // std::cout<<"------------------------------------"<<std::endl;
        // TestPoint2PoseSlam test_point2pose;
        // test_point2pose.Run();
        // std::cout<<"------------------------------------"<<std::endl;
        // TestSlamG2o test_slam_g2o;
        // test_slam_g2o.Run();
        // std::cout<<"------------------------------------"<<std::endl;
        TestSlam3dG2o test_slam3d_g2o;
        test_slam3d_g2o.Run();

        return 0;
    }
};

#endif // __TEST_SLAM2D_H__