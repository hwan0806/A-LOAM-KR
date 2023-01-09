// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk


// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <cmath>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <queue>

#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include "lidarFactor.hpp"

#define DISTORTION 0
/***
 * * #define은 지시문으로써 DISTORTION이 나타날 때 마다 0으로 바꾸는 역할을 한다. -> 즉, 그냥 전역변수라는 소린가?
 * * 그렇다면, 여기서 의문점/ 그냥 변수처럼 사용하면 안될까?
 * * 프로그램의 가독성을 높여주고, 유지보수도 용이하게 해주는 역할이라고 함. 또한, 변수를 사용하는 것보다 처리속도 또한 빠르다고 함.
*/
int corner_correspondence = 0, plane_correspondence = 0;
/***
 * * const란?
 * *     변수를 상수화 하기 위해 사용됨.
 * *     여기서 상수란, 데이터의 초기화가 이루어지면, 그 값을 바꿀 수 없도록 하는 것임.
 * *  constexpr이란?
 * *     const는 변수의 초기화를 런타임까지 지연시킬 수 있는 반면, constexpr 변수는 반드시 컴파일 타임에 초기화 되어 있어야 함.
 * *  무슨 말인지 잘 모르겠음...
*/

constexpr double SCAN_PERIOD = 0.1;
constexpr double DISTANCE_SQ_THRESHOLD = 25;
constexpr double NEARBY_SCAN = 2.5;

int skipFrameNum = 5;
bool systemInited = false;

double timeCornerPointsSharp = 0;
double timeCornerPointsLessSharp = 0;
double timeSurfPointsFlat = 0;
double timeSurfPointsLessFlat = 0;
double timeLaserCloudFullRes = 0;

pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());
pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());

pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsFlat(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsLessFlat(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
// 각 변수를 pcl::PointCloud type으로 선언

int laserCloudCornerLastNum = 0;
int laserCloudSurfLastNum = 0;

// Transformation from current frame to world frame
Eigen::Quaterniond q_w_curr(1, 0, 0, 0);
Eigen::Vector3d t_w_curr(0, 0, 0);

// q_curr_last(x, y, z, w), t_curr_last
double para_q[4] = {0, 0, 0, 1};
double para_t[3] = {0, 0, 0};

Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);
Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);

// 큐(Queue)란?
// 선입선출(First in First out)의 구조를 가진 형식으로, 말 그대로 제일 처음 넣은 데이터가 먼저 빠져나오는 방식이다.
// queue<데이터타입> 이름으로 선언할 수 있다.
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLessSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfFlatBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLessFlatBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullPointsBuf;

/**
 * * Mutex란?
 * * 다양한 쓰레드에서 공유자원을 접근해 값을 변경, 읽는 작업을 수행할 때에, 실제로는 같은 순간이 아니라
 * * 조금의 시간이 다를 수도 있음. 따라서 우리가 원했던 값을 읽어오지 못하고, 다른 값을 읽어올 수 있음.
 * * 즉, 여러 쓰레드의 공유자원에 대한 동시접근을 막아야 하고, 이 역할을 하는 것이 Mutex이다.
 * * 자물쇠라고 가정한다면, 자물쇠는 열거나(lock). 잠그는(unlock) 동작을 수행할 수 있음.
 * * 따라서, mutex 클래스에는 lock, try, unlock이라는 locking에 대한 함수가 존재함.
 * * 어떤 쓰레드가 특정 영역에 들어가서 mutex를 lock하면 들어가지 못하고, 대기함. mutex가 unlock되어
 * * 나오면, 다음 쓰레드가 들어가서 사용할 수 있게 됨.
 * * ceres solver를 풀 때, 다른 pointcloud로 풀도록 막게
*/

/** 
 * TODO 다양한 쓰레드에서 공유자원을 접근해 값을 변경, 읽는 작업을 수행할 때에, 실제로는 같은 순간이 아니라
 * TODO 조금의 시간이 다를 수도 있음. 따라서 우리가 원했던 값을 읽어오지 못하고, 다른 값을 읽어올 수 있음.
 * TODO 즉, 여러 쓰레드의 공유자원에 대한 동시접근을 막아야 하고, 이 역할을 하는 것이 Mutex이다.
 * TODO 자물쇠라고 가정한다면, 자물쇠는 열거나(lock). 잠그는(unlock) 동작을 수행할 수 있음.
 * TODO 따라서, mutex 클래스에는 lock, try, unlock이라는 locking에 대한 함수가 존재함.
 * TODO 어떤 쓰레드가 특정 영역에 들어가서 mutex를 lock하면 들어가지 못하고, 대기함. mutex가 unlock되어
 * TODO 나오면, 다음 쓰레드가 들어가서 사용할 수 있게 됨.

*/
std::mutex mBuf;
// mutex를 mBuf라는 변수로 지정.

// undistort lidar point
void TransformToStart(PointType const *const pi, PointType *const po)
{
    //interpolation ratio
    double s;
    if (DISTORTION)
        s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD;
    else
        s = 1.0;
    //s = 1;
    Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
    Eigen::Vector3d t_point_last = s * t_last_curr;
    Eigen::Vector3d point(pi->x, pi->y, pi->z);
    Eigen::Vector3d un_point = q_point_last * point + t_point_last;

    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();
    po->intensity = pi->intensity;
}

// transform all lidar points to the start of the next frame

void TransformToEnd(PointType const *const pi, PointType *const po)
{
    // undistort point first
    pcl::PointXYZI un_point_tmp;
    TransformToStart(pi, &un_point_tmp);

    Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
    Eigen::Vector3d point_end = q_last_curr.inverse() * (un_point - t_last_curr);

    po->x = point_end.x();
    po->y = point_end.y();
    po->z = point_end.z();

    //Remove distortion time info
    po->intensity = int(pi->intensity);
}

void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsSharp2)
{
    mBuf.lock();
    cornerSharpBuf.push(cornerPointsSharp2);
    mBuf.unlock();
}

void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsLessSharp2)
{
    mBuf.lock();
    cornerLessSharpBuf.push(cornerPointsLessSharp2);
    mBuf.unlock();
}

void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsFlat2)
{
    mBuf.lock();
    surfFlatBuf.push(surfPointsFlat2);
    mBuf.unlock();
}

void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsLessFlat2)
{
    mBuf.lock();
    surfLessFlatBuf.push(surfPointsLessFlat2);
    mBuf.unlock();
}

//receive all point cloud
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)
{
    mBuf.lock();
    fullPointsBuf.push(laserCloudFullRes2);
    mBuf.unlock();
}


// main 함수 시작
int main(int argc, char **argv)
// argc는 몇 개의 입력을 받는지를 숫자로 알 수 있고, argv는 문자로 입력받는다.
// 즉, 명령어 뒤의 옵션 개수를 argc, 각 옵션의 이름은 argv이다.
{
    ros::init(argc, argv, "laserOdometry");
    ros::NodeHandle nh;
    // ros::NodeHandle이라는 클래스를 nh라는 변수로 받아들임.

    nh.param<int>("mapping_skip_frame", skipFrameNum, 2);

    printf("Mapping %d Hz \n", 10 / skipFrameNum);

    ros::Subscriber subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100, laserCloudSharpHandler);
    ros::Subscriber subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100, laserCloudLessSharpHandler);
    '''
    Edge Point들을 의미
    Sharp와 Less Sharp, 2개의 차이는 아직 설명 불가
    '''

    ros::Subscriber subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100, laserCloudFlatHandler);
    ros::Subscriber subSurfPointsLessFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100, laserCloudLessFlatHandler);
    '''
    Planar Point들을 의미
    Flat와 Less Flat, 2개의 차이는 아직 설명 불가
    '''

    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100, laserCloudFullResHandler);
    // velodyne의 모든 Point들을 의미

    ros::Publisher pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100);
    ros::Publisher pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100);
    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100);
    ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);
    ros::Publisher pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_odom_path", 100);

    nav_msgs::Path laserPath;

    int frameCount = 0;
    ros::Rate rate(100);
    /**
     * * ros::Rate는 반복하고자하는 주기를 설정하게 된다. rate에 지정한 변수의 주기를 지키기 위해 하단의 sleep을 이용한다.
    */
    while (ros::ok())
    /**
    * * ros::ok가 False가 되는 경우는 다음과 같다.
    * * 1. Ctrl+C의 입력을 받았을 경우
    * * 2. 동일한 이름의 다른 노드로 인해 충돌이 발생한 경우
    * * 3. 다른 부분에서 ros::shutdown()이 호출된 경우
    * * 4. 모든 ros::NodeHandles가 종료된 경우
    * * 참고 : ros::ok()가 한번 False를 받으면, 다시 사용할 수 없다.
    */
    {
        ros::spinOnce();
        /**
        * * 큐에 요청된 콜백함수를 처리한다.
        * * ros::spinOnce()는 현재까지 요청된 콜백 함수를 모두 호출하고 코드의 다음부분으로 넘어가지만,
        * * ros::spin은 노드가 shutdown되거나 ctrl+C로 정지되기 이전까지 
        */

        if (!cornerSharpBuf.empty() && !cornerLessSharpBuf.empty() &&
            !surfFlatBuf.empty() && !surfLessFlatBuf.empty() &&
            !fullPointsBuf.empty())
        /**
        * * 위에서 각 변수를 PointCloud2 type의 queue로 정의하였고, PointColud2가 empty가 아닐 경우를 의미한다.
        * * 즉, message가 들어온 경우를 의미한다.
        */
        {
            timeCornerPointsSharp = cornerSharpBuf.front()->header.stamp.toSec();
            timeCornerPointsLessSharp = cornerLessSharpBuf.front()->header.stamp.toSec();
            timeSurfPointsFlat = surfFlatBuf.front()->header.stamp.toSec();
            timeSurfPointsLessFlat = surfLessFlatBuf.front()->header.stamp.toSec();
            timeLaserCloudFullRes = fullPointsBuf.front()->header.stamp.toSec();
            /**
            * * PointCloud2 mesaage 안에 있는 header.stamp.toSec()를 이용해 각 point들이 들어온 시점을 알 수 있고,
            * * 이를 timeCornerSharp라는 변수로 정의한다.
            */

            if (timeCornerPointsSharp != timeLaserCloudFullRes ||
                timeCornerPointsLessSharp != timeLaserCloudFullRes ||
                timeSurfPointsFlat != timeLaserCloudFullRes ||
                timeSurfPointsLessFlat != timeLaserCloudFullRes)
            {
                printf("unsync messeage!");
                ROS_BREAK();
            }
            /**
            * * 모든 PointCloud message의 time과 각각의 Edge, Planar point들의 time이 다르면
            * * 비동기화된 것으로 보고, 프로그램을 멈춘다.
            */
            /**
             * *교수님께 여쭤보기
            */

            mBuf.lock();
            /**
             * * mutex를 lock 시켜 다른 쓰레드의 접근을 막음
            */ 
            cornerPointsSharp->clear();
            /**
            * * pcl::pointcloud의 clear 함수를 의미하고, 이는 들어있는 pointcloud set을 제거하고,
            * * pointcloud의 height와 width를 0으로 맞춰주는 작업이다. -> 즉, pointcloud의 initialization 과정이다.
            * * 위의 cornerSharpBuf와 혼동할 수도 있는데, 여기서 cornerSharpBuf는 Pointcloud2 type으로 정의된 queue이며,
            * * queue에서는 clear를 지원하지 않는다.
            */
            pcl::fromROSMsg(*cornerSharpBuf.front(), *cornerPointsSharp);
            /**
            * * ROS에서 통신을 주고받을 때에는, PointCloud2 형식으로 해야한다. 하지만, 이 data를 처리하기 위해서는
            * * pcl::PointCloud 형식으로 바꾸어야하고, 이를 진행하는 과정이다.
            * * 즉, corenerSharpBuf.front()의 형식을 cornerPointsSharp의 형식으로 형 변환을 시켜주는 과정이다.
            * ? 참고 : https://limhyungtae.github.io/2021-09-10-ROS-Point-Cloud-Library-(PCL)-2.-%ED%98%95%EB%B3%80%ED%99%98-toROSMsg,-fromROSMsg/
            */
            cornerSharpBuf.pop();
            /**
            * * 이후, 형 변환을 시키고 남은 queue를 비운다 -> 아마 추정하기로는 메모리 상의 효율?이 아닐까?
            */
            cornerPointsLessSharp->clear();
            pcl::fromROSMsg(*cornerLessSharpBuf.front(), *cornerPointsLessSharp);
            cornerLessSharpBuf.pop();
            /**
            * * 위의 내용과 같음
            */
            surfPointsFlat->clear();
            pcl::fromROSMsg(*surfFlatBuf.front(), *surfPointsFlat);
            surfFlatBuf.pop();
            /**
            * * 위의 내용과 같음
            */
            surfPointsLessFlat->clear();
            pcl::fromROSMsg(*surfLessFlatBuf.front(), *surfPointsLessFlat);
            surfLessFlatBuf.pop();
            /**
            * * 위의 내용과 같음
            */
            laserCloudFullRes->clear();
            pcl::fromROSMsg(*fullPointsBuf.front(), *laserCloudFullRes);
            fullPointsBuf.pop();
            /**
            * * 위의 내용과 같음
            */
            mBuf.unlock();
            /**
             * * mutex를 unlock 시켜 다음 쓰레드가 들어올 수 있도록 함.
             */ 
            TicToc t_whole;
            /**
             * * c++의 timer라고 생각하면 됨. 여기서 tic은 시작점을 측정하고, toc은 도착점을 측정함.
             */

            // initializing
            if (!systemInited)
            /**
             * * 처음에 systemInited는 False라고 설정되어있음.
             * * 즉, 처음 시작할 때 systemInited를 True로 바꾸고, 계속 진행함.
             */
            {
                systemInited = true;
                std::cout << "Initialization finished \n";
            }
            else
            {
                int cornerPointsSharpNum = cornerPointsSharp->points.size();
                int surfPointsFlatNum = surfPointsFlat->points.size();
                /**
                 * * Edge, Planar point들, 각각의 point의 개수를 의미
                */
                TicToc t_opt;
                /**
                * * 위의 TicToc 내용과 같음
                */

                for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter)
                /** 
                 * 최적화를 2번 진행함.
                 */
                {
                    corner_correspondence = 0;
                    plane_correspondence = 0;

                    //ceres::LossFunction *loss_function = NULL;
                    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                    /**
                     * * // 먼저 ceres에 사용되는 loss fuction는 HuberLoss를 사용한다.
                     * * Huber loss (후버 손실) 이란 MSE(Mean Square Error) 와 MAE(Mean Absolute Error)를 절충한 함수이다.
                     * * 어떠한 일정한 범위를 정해서 그 안에 있으면, 오차를 제곱하고, 그 밖에 있으면 오차의 절대값을 구하는 방식이며,
                     * * 일정한 범위 내에 있을 때, 오차에 0.5를 곱해주는데 이는 두 함수가 만날 때, smooth 정도를 고려하기 위해서이다.
                     * ? 참고 : http://doc.mindscale.kr/km/data_mining/dm02.html
                     * * 또한, 여기서 new 변수를 알기위해서는 동적할당을 알아야한다.
                     * * 먼저 동적할당에 대해 이야기 하기 전에, C++의 세가지 기본 타입의 메모리 할당에 대해 알아본다.
                     * * 1. 정적 메모리 할당
                     * *      정적 변수와 전역 변수에 대해 발생하며, 이러한 타입의 변수에 대한 메모리는 실행할 때, 한 번 할당되며, 프로그램이 끝날 때까지 지속된다.
                     * * 2. 지동 메모리 할당
                     * *      함수 매개 변수와 지역 변수에 대해 발생하며, 이러한 타입의 변수에 대한 메모리는 관련 블록을 입력할 때, 할당되고, 블록을 종료할 때 필요에 따라 여러 번 해제할 수 있다.
                     * * 위의 두가지 메모리 할당은 변수 및 배열의 크기는 컴파일 타임에 알아야 한다는 것과, 할당 및 해체가 자동으로 수행된다는 공통점이 있다.
                     * * 하지만, 사용자가 입력을 처리할 때, 이러한 제약 조건으로 인해 문제가 발생하는 상황이 생길 수 있다.
                     * * 예를 들면, 문자열을 사용하여 누군가의 이름은 정할 수 있지만, 이름의 길이를 알 수 없다.
                     * * 따라서 변수의 최대 크기를 추측하고, 가정하는 방법을 이용하지만, 크게 2가지 문제가 있다.
                     * * 1. 변수가 실제로 사용되지 않으면 낭비되는 메모리가 많다.
                     * * 2. 대부분의 일반 변수는 stack에 할당되고, 기본적으로 설정된 메모리 정도를 초과하면 overflow가 발생하고, 운영체제가 프로그램을 강제로 종료하게 된다.
                     * * 즉, 많은 프로그램을 다루는 프로그램에서는 문제가 발생한다.
                    */
                    /**
                     * ! 근데 왜 loss function을 동적 메모리에 할당하는 거지? 들어오는 변수가 계속 바뀌어서 그런건가?
                    */
                   /**
                    * ? 참고 : https://boycoding.tistory.com/204
                   */

                    ceres::LocalParameterization *q_parameterization =
                        new ceres::EigenQuaternionParameterization();
                    
                    ceres::Problem::Options problem_options;

                    ceres::Problem problem(problem_options);
                    
                    problem.AddParameterBlock(para_q, 4, q_parameterization);
                    problem.AddParameterBlock(para_t, 3);

                    pcl::PointXYZI pointSel;
                    std::vector<int> pointSearchInd;
                    std::vector<float> pointSearchSqDis;

                    TicToc t_data;
                    // find correspondence for corner features
                    /**
                     * * corner features들 간의 correspondence 찾기
                    */
                    for (int i = 0; i < cornerPointsSharpNum; ++i)
                    /**
                     * * cornerPoint의 개수만큼 iteration
                    */
                    {
                        TransformToStart(&(cornerPointsSharp->points[i]), &pointSel);
                        /**
                         * * 위에서 정의한 quaternion(para_q)과 translation(para_t)를 이용해 point들에 Transformation 적용
                        */
                        kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
                        /**
                         * * pointSel이 query point가 됨.
                         * * pointcloud 상에서 기준 point(query point)를 기준으로부터 가장 가까운 k개의 point의 index를 return 해줌.
                         * * 사용방법 : nearestKSearch(query, N, idxes, sqr_dists) -> 즉, query와 가까운 N개의 point들의 idxes와 sqr_dists를 return하는 것이다.
                         * ? 참고 : https://limhyungtae.github.io/2021-09-12-ROS-Point-Cloud-Library-(PCL)-9.-KdTree%EB%A5%BC-%ED%99%9C%EC%9A%A9%ED%95%9C-K-nearest-Neighbor-Search-(KNN)/
                        */
                        int closestPointInd = -1, minPointInd2 = -1;
                        /**
                         * * 각 변수를 -1로 정의함.
                         * ! 왜...?! // 아마 Ind가 0부터 시작하니까 겹치지 않기 위해서?
                        */ 
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                        /**
                         * * query point와의 거리가 thershold보다 가까울 경우를 의미함.
                         * * 위에서 이는 25로 정의되어있음.
                        */
                        {
                            closestPointInd = pointSearchInd[0];
                            /**
                             * * 가장 가까운 point의 index를 return받은 index로 정의
                            */
                            int closestPointScanID = int(laserCloudCornerLast->points[closestPointInd].intensity);
                            /**
                             * * 가장 가까운 point의 intensity값으로 scanid 정의
                            */
                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
                            /**
                             * * 위에서 정의된 threshold를 다른 이름으로 정의함.
                            */
                            // search in the direction of increasing scan line
                            for (int j = closestPointInd + 1; j < (int)laserCloudCornerLast->points.size(); ++j)
                            /**
                             * * 가장 가까운 point의 index부터, 전체 point들의 개수까지 iteration -> 즉, 가장 가까운 index들보다 큰 index만 iteration한다는건데, 정렬이 되어 있는 건가?
                            */
                            {
                                // if in the same scan line, continue
                                if (int(laserCloudCornerLast->points[j].intensity) <= closestPointScanID)
                                    continue;
                                /**
                                 * * 같은 channel의 point이면 loop의 끝으로 이동시킨다. -> 즉, 그럼 같은 channel의 point들은 closetpoint로 간주하지 않는다.
                                 * ! 왜...?!
                                */

                                // if not in nearby scans, end the loop
                                if (int(laserCloudCornerLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                    break;
                                /**
                                 * * NEARBY_SCAN이라는 변수를 이용해 Threshold를 정하고, point들이 정한 Thershold보다 멀면, loop를 종료한다.
                                */

                                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                        (laserCloudCornerLast->points[j].x - pointSel.x) +
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                        (laserCloudCornerLast->points[j].y - pointSel.y) +
                                                    (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                        (laserCloudCornerLast->points[j].z - pointSel.z);
                                /**
                                 * * pointSel이 query point이기 때문에 query point와의 거리를 의미함.
                                */
                                if (pointSqDis < minPointSqDis2)
                                /**
                                 * * 위에서 지정한 DISTANCE_SQ_THRESHOLD의 값보다 작으면 correspondence로 인정한다는 의미이다.
                                */
                                {
                                    // find nearer point
                                    minPointSqDis2 = pointSqDis;
                                    /**
                                     * * 위에서 계산한 거리 값을 disatnce로 정의
                                    */
                                    minPointInd2 = j;
                                    /**
                                     * * 가장 가까울 때, iteration하고 있는 number j를 가장 가까울 때의 index로 정의
                                    */
                                }
                            }

                            // search in the direction of decreasing scan line
                            for (int j = closestPointInd - 1; j >= 0; --j)
                            /**
                             * * 바로 위의 for문은 같은 scan line에서 위의 index를 보았다면, 여기서는 밑의 index를 확인한다.
                             * ! 왜 이런식으로 구성하였을까?!
                            */
                            {
                                // if in the same scan line, continue
                                if (int(laserCloudCornerLast->points[j].intensity) >= closestPointScanID)
                                    continue;
                                /**
                                 * * 위와 동일
                                */

                                // if not in nearby scans, end the loop
                                if (int(laserCloudCornerLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                    break;
                                /**
                                 * * 위와 동일
                                */
                                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                        (laserCloudCornerLast->points[j].x - pointSel.x) +
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                        (laserCloudCornerLast->points[j].y - pointSel.y) +
                                                    (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                        (laserCloudCornerLast->points[j].z - pointSel.z);
                                /**
                                 * * 위와 동일
                                */
                                if (pointSqDis < minPointSqDis2)
                                {
                                    // find nearer point
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                                /**
                                 * * 위와 동일
                                */
                            }
                        }
                        /**
                         * * 먼저 KNN-search를 이용해 가장 가까운 point를 찾는다.
                         * * 이후, 가장 가까운 point와 가장 가까운 index의 number minPointInd2와 그 때의 distance minPointSqDis2를 찾는다.
                         * * 즉, 논문에서 이야기하고 있는, l, m, n 찾아서 확인하는 부분인듯?
                        */
                        if (minPointInd2 >= 0) // both closestPointInd and minPointInd2 is valid
                        /**
                         * * 위에서 minPointInd2를 -1로 정의하였기 때문에, 위의 조건문을 거치지 않으면, 여기의 조건문 또한 거치지 않게됨.
                         * * 즉, 위의 조건문을 거쳤다면, 즉, 가장 가까운 point와 그 point에 대해서 가까운 point도 찾았다는 의미가 됨.
                        */
                        {
                            Eigen::Vector3d curr_point(cornerPointsSharp->points[i].x,
                                                       cornerPointsSharp->points[i].y,
                                                       cornerPointsSharp->points[i].z);
                            /**
                             * * curr_point에 현재 point의 x, y, z 값을 넣음.
                            */
                            
                            Eigen::Vector3d last_point_a(laserCloudCornerLast->points[closestPointInd].x,
                                                         laserCloudCornerLast->points[closestPointInd].y,
                                                         laserCloudCornerLast->points[closestPointInd].z);
                            /**
                             * * 현재 point와 가장 가까운 point를 last_point_a에 넣음.
                            */
                            
                            Eigen::Vector3d last_point_b(laserCloudCornerLast->points[minPointInd2].x,
                                                         laserCloudCornerLast->points[minPointInd2].y,
                                                         laserCloudCornerLast->points[minPointInd2].z);
                            /**
                             * * last_point_a에 가장 가까운 point를 last_point_b에 넣음.
                            */

                            double s;
                            /**
                             * * s를 정의함.
                            */
                            if (DISTORTION)
                                s = (cornerPointsSharp->points[i].intensity - int(cornerPointsSharp->points[i].intensity)) / SCAN_PERIOD;
                            else
                                s = 1.0;
                            ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
                            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                            corner_correspondence++;
                        }
                    }

                    // find correspondence for plane features
                    for (int i = 0; i < surfPointsFlatNum; ++i)
                    /**
                     * * planarPoint의 개수만큼 iteration
                    */  
                    {
                        TransformToStart(&(surfPointsFlat->points[i]), &pointSel);
                        kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
                        /**
                         * * pointSel이 query point가 됨.
                         * * pointcloud 상에서 기준 point(query point)를 기준으로부터 가장 가까운 k개의 point의 index를 return 해줌.
                         * * 사용방법 : nearestKSearch(query, N, idxes, sqr_dists) -> 즉, query와 가까운 N개의 point들의 idxes와 sqr_dists를 return하는 것이다.
                        */

                        int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
                        /**
                         * * 위와 동일
                        */
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                        /**
                         * * DISTANCE_SQ_THRESHOLD보다 Distance가 적을 경우를 의미함.
                        */
                        {
                            closestPointInd = pointSearchInd[0];
                            /**
                             * * 가장 가까운 point의 index를 return받은 index로 정의
                            */

                            // get closest point's scan ID
                            int closestPointScanID = int(laserCloudSurfLast->points[closestPointInd].intensity);
                            /**
                             * * 가장 가까운 point의 intensity값으로 scanid 정의
                            */

                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;
                            /**
                             * * 위에서 정의된 threshold를 다른 이름으로 정의함.
                            */

                            // search in the direction of increasing scan line
                            for (int j = closestPointInd + 1; j < (int)laserCloudSurfLast->points.size(); ++j)
                            /**
                             * * 가장 가까운 point의 index부터, 전체 point들의 개수까지 iteration 
                            */
                            {
                                // if not in nearby scans, end the loop
                                if (int(laserCloudSurfLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                    break;
                                /**
                                 * * NEARBY_SCAN이라는 변수를 이용해 Threshold를 정하고, point들이 정한 Thershold보다 멀면, loop를 종료한다. -> 즉, 너무 멀면 종료한다.
                                */

                                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                                        (laserCloudSurfLast->points[j].x - pointSel.x) +
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) *
                                                        (laserCloudSurfLast->points[j].y - pointSel.y) +
                                                    (laserCloudSurfLast->points[j].z - pointSel.z) *
                                                        (laserCloudSurfLast->points[j].z - pointSel.z);
                                /**
                                 * * pointSel이 query point이기 때문에 query point와의 거리를 의미함.
                                */

                                // if in the same or lower scan line
                                if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                                /**
                                 * * 현재 point의 scan line보다 같거나 낮은 곳에서
                                 * * 위에서 찾은 현재 point와 가장 가까운 거리의 point의 channel보다 작고, 위에서 Threshold로 정의한 값보다 거리가 가까우면, planar surface point로 인정한다.
                                */

                                // if in the higher scan line
                                else if (int(laserCloudSurfLast->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3)
                                {
                                    minPointSqDis3 = pointSqDis;
                                    minPointInd3 = j;
                                }
                                /**
                                 * * 현재 point의 scan line보다 높은 곳에서
                                 * * 위에서 찾은 현재 point와 가장 가까운 거리의 point의 channel보다 작고, 위에서 Threshold로 정의한 값보다 거리가 가까우면, planar surface point로 인정한다.
                                */

                            }

                            // search in the direction of decreasing scan line
                            for (int j = closestPointInd - 1; j >= 0; --j)
                            /**
                            * * 가장 가까운 point의 index부터, 전체 point들의 개수까지 iteration 
                            */   
                            
                            {
                                // if not in nearby scans, end the loop
                                if (int(laserCloudSurfLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                    break;
                                /**
                                 * * NEARBY_SCAN이라는 변수를 이용해 Threshold를 정하고, point들이 정한 Thershold보다 멀면, loop를 종료한다. -> 즉, 너무 멀면 종료한다.
                                */

                                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                                        (laserCloudSurfLast->points[j].x - pointSel.x) +
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) *
                                                        (laserCloudSurfLast->points[j].y - pointSel.y) +
                                                    (laserCloudSurfLast->points[j].z - pointSel.z) *
                                                        (laserCloudSurfLast->points[j].z - pointSel.z);
                                /**
                                 * * 위와 동일
                                */
                                // if in the same or higher scan line
                                if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                                /**
                                 * * 위와 동일
                                */
                                else if (int(laserCloudSurfLast->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3)
                                {
                                    // find nearer point
                                    minPointSqDis3 = pointSqDis;
                                    minPointInd3 = j;
                                }
                                /**
                                 * * 위와 동일
                                */
                            }

                            if (minPointInd2 >= 0 && minPointInd3 >= 0)
                            /**
                             * * 위의 조건을 만족했다는 것을 의미
                            */
                            {

                                Eigen::Vector3d curr_point(surfPointsFlat->points[i].x,
                                                            surfPointsFlat->points[i].y,
                                                            surfPointsFlat->points[i].z);
                                /**
                                 * * 현재 point를 curr_point로 정의
                                */

                                Eigen::Vector3d last_point_a(laserCloudSurfLast->points[closestPointInd].x,
                                                                laserCloudSurfLast->points[closestPointInd].y,
                                                                laserCloudSurfLast->points[closestPointInd].z);
                                /**
                                 * * 현재 point에서 가장 가까운 point를 point_a로 정의
                                */
                                Eigen::Vector3d last_point_b(laserCloudSurfLast->points[minPointInd2].x,
                                                                laserCloudSurfLast->points[minPointInd2].y,
                                                                laserCloudSurfLast->points[minPointInd2].z);
                                /**
                                 * * point_a의 sacn line 보다 위에 있는 scan line 중 가장 가까운 것
                                */

                                Eigen::Vector3d last_point_c(laserCloudSurfLast->points[minPointInd3].x,
                                                                laserCloudSurfLast->points[minPointInd3].y,
                                                                laserCloudSurfLast->points[minPointInd3].z);
                                /**
                                 * * point_a의 sacn line 보다 밑에 있는 scan line 중 가장 가까운 것
                                */

                                double s;
                                if (DISTORTION)
                                    s = (surfPointsFlat->points[i].intensity - int(surfPointsFlat->points[i].intensity)) / SCAN_PERIOD;
                                else
                                    s = 1.0;
                                ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                                problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                                plane_correspondence++;
                            }
                        }
                    }

                    //printf("coner_correspondance %d, plane_correspondence %d \n", corner_correspondence, plane_correspondence);
                    printf("data association time %f ms \n", t_data.toc());

                    if ((corner_correspondence + plane_correspondence) < 10)
                    {
                        printf("less correspondence! *************************************************\n");
                    }
                    /**
                     * * corner_point의 개수와 plane_point의 개수가 적으면 less correspondence 출력
                    */

                    TicToc t_solver;
                    ceres::Solver::Options options;
                    options.linear_solver_type = ceres::DENSE_QR;
                    options.max_num_iterations = 4;
                    options.minimizer_progress_to_stdout = false;
                    ceres::Solver::Summary summary;
                    ceres::Solve(options, &problem, &summary);
                    printf("solver time %f ms \n", t_solver.toc());
                }
                printf("optimization twice time %f \n", t_opt.toc());

                t_w_curr = t_w_curr + q_w_curr * t_last_curr;
                q_w_curr = q_w_curr * q_last_curr;
            }

            TicToc t_pub;

            // publish odometry
            nav_msgs::Odometry laserOdometry;
            
            laserOdometry.header.frame_id = "camera_init";
            laserOdometry.child_frame_id = "/laser_odom";
            laserOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
            laserOdometry.pose.pose.orientation.x = q_w_curr.x();
            laserOdometry.pose.pose.orientation.y = q_w_curr.y();
            laserOdometry.pose.pose.orientation.z = q_w_curr.z();
            laserOdometry.pose.pose.orientation.w = q_w_curr.w();
            laserOdometry.pose.pose.position.x = t_w_curr.x();
            laserOdometry.pose.pose.position.y = t_w_curr.y();
            laserOdometry.pose.pose.position.z = t_w_curr.z();
            pubLaserOdometry.publish(laserOdometry);

            geometry_msgs::PoseStamped laserPose;
            laserPose.header = laserOdometry.header;
            laserPose.pose = laserOdometry.pose.pose;
            laserPath.header.stamp = laserOdometry.header.stamp;
            laserPath.poses.push_back(laserPose);
            laserPath.header.frame_id = "camera_init";
            pubLaserPath.publish(laserPath);

            // transform corner features and plane features to the scan end point
            if (0)
            {
                int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
                for (int i = 0; i < cornerPointsLessSharpNum; i++)
                {
                    TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
                }

                int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
                for (int i = 0; i < surfPointsLessFlatNum; i++)
                {
                    TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
                }

                int laserCloudFullResNum = laserCloudFullRes->points.size();
                for (int i = 0; i < laserCloudFullResNum; i++)
                {
                    TransformToEnd(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
                }
            }

            pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
            cornerPointsLessSharp = laserCloudCornerLast;
            laserCloudCornerLast = laserCloudTemp;

            laserCloudTemp = surfPointsLessFlat;
            surfPointsLessFlat = laserCloudSurfLast;
            laserCloudSurfLast = laserCloudTemp;

            laserCloudCornerLastNum = laserCloudCornerLast->points.size();
            laserCloudSurfLastNum = laserCloudSurfLast->points.size();

            // std::cout << "the size of corner last is " << laserCloudCornerLastNum << ", and the size of surf last is " << laserCloudSurfLastNum << '\n';

            kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
            kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

            if (frameCount % skipFrameNum == 0)
            {
                frameCount = 0;

                sensor_msgs::PointCloud2 laserCloudCornerLast2;
                pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
                laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudCornerLast2.header.frame_id = "/camera";
                pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

                sensor_msgs::PointCloud2 laserCloudSurfLast2;
                pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
                laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudSurfLast2.header.frame_id = "/camera";
                pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

                sensor_msgs::PointCloud2 laserCloudFullRes3;
                pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
                laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudFullRes3.header.frame_id = "/camera";
                pubLaserCloudFullRes.publish(laserCloudFullRes3);
            }
            printf("publication time %f ms \n", t_pub.toc());
            printf("whole laserOdometry time %f ms \n \n", t_whole.toc());
            if(t_whole.toc() > 100)
                ROS_WARN("odometry process over 100ms");

            frameCount++;
        }
        rate.sleep();
    }
    return 0;
}