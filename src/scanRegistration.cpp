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

/// std 라이브러리
#include <cmath>    // c++의 std 라이브러리 : 각종 수학 함수(삼각함수 등등)
#include <vector>
#include <string>
/// custom 라이브러리 : rad-deg 변환, 시간 측정 등등
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
/// pcl 라이브러리
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
/// ros 라이브러리
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using std::atan2;
using std::cos;
using std::sin;

//const : unchangable value
const double scanPeriod = 0.1;
const int systemDelay = 0; 
int systemInitCount = 0;
bool systemInited = false;
int N_SCANS = 0;
float cloudCurvature[400000];
int cloudSortInd[400000];
int cloudNeighborPicked[400000];
int cloudLabel[400000];

// if cC[i] < cC[j], comp = true
bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }

// ros publisher 미리 전역으로 선언 => subscriber의 callback 함수를 통해 만들어진 데이터들이 이 전역 publisher 에 담길 예정
ros::Publisher pubLaserCloud;
ros::Publisher pubCornerPointsSharp;
ros::Publisher pubCornerPointsLessSharp;
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubSurfPointsLessFlat;
ros::Publisher pubRemovePoints;
std::vector<ros::Publisher> pubEachScan;

bool PUB_EACH_LINE = false;
double MINIMUM_RANGE = 0.1; 

// template 설정 : PointT가 템플릿 대상 자료형 됨.
template <typename PointT>


/**********************************

Purpose :  Remove the points that are closer than the threshold.
Input   :  cloud_in, thres
Output  :  cloud_out
Flow    :  1. make cloud_out in the form of cloud_in
           2. if the distance of cloud_in is larger than threshold, cloud_out=cloud_in
Note    :  What is cloud_in and cloud_out and their format.

**********************************/
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                              pcl::PointCloud<PointT> &cloud_out, float thres)
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // main문에서 동일한 변수를 할당한다면, cloud_in과 cloud_out이 다를 수가 있나?? 어떤 이유로 예외처리 하는지..?  /////////////////////////////////////////////////////////
    // if cloud_in and cloud_out is different, revise cloud_out to cloud_in.
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // size_t = unsigned int (32 or 64 bit)
    size_t j = 0;

    /// 거리 계산 : r^2 = x^2 + y^2 + z^2
    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // cloud_out의 자료형 pcl::PointCloud<pcl::PointXYZ>에 heigh, width, isdense라는 항목이 있는가?
    // reference : https://pointclouds.org/documentation/tutorials/basic_structures.html
    cloud_out.height = 1;
    /// static cast : https://blockdmask.tistory.com/236
    cloud_out.width = static_cast<uint32_t>(j); // 강제 형 변환
    cloud_out.is_dense = true;
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}


/**********************************

Purpose :  
Input   :  
Output  :  
Flow    :  
Note    :  

**********************************/
// main문에서 laserCloudHandler는 PointCloud2 메시지 자체를 subscribe하는데, 함수 정의부에선 매개변수가 PointCloud2ConstPtr로 정의됨??
// => 함수 호출 시, 원형 명시 X. laserCloudHandler는 원래 pointer 매개변수를 받는 함수임.
// ros subscriber에서 함수 포인터를 활용한 callback 구조 활용하는데, 이 때, 받은 토픽을 포인터 형식으로 함수에 넘겨줌.
//
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    // system 시작 여부에 대해 결정?? -> 이 과정 왜 진행하는지..?
    if (!systemInited)  // systemInited : false
    { 
        systemInitCount++;
        if (systemInitCount >= systemDelay)
        {
            systemInited = true;
        }
        else
            return;
    }

    TicToc t_whole;
    TicToc t_prepare;
    // N_SCANS 크기의 벡터 생성 and 0으로 초기화 : 하나의 pointcloud 데이터를 각 scanline으로 나누고, 이에 대한 start/end 인덱스 확인
    std::vector<int> scanStartInd(N_SCANS, 0);
    std::vector<int> scanEndInd(N_SCANS, 0);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //PointCloud 객체 => template 형식으로.. => 세부 자료형 확정 : 왜 이렇게 선언할까?? 그냥 pcl::PointXYZ로 선언하면 안되나??
    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // point cloud format change : sensor_msgs::PointCloud2 -> pcl::PointCloud
    // https://limhyungtae.github.io/2021-09-10-ROS-Point-Cloud-Library-(PCL)-2.-%ED%98%95%EB%B3%80%ED%99%98-toROSMsg,-fromROSMsg/
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);

    // removeNaNFromPointCloud에서 활용되어야 하는 vector 형 변수 : index 저장용
    std::vector<int> indices;

    // 1. Remove NaN : <pcl/filters/filter.h>
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
    removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE);


    int cloudSize = laserCloudIn.points.size();
    
    // 2. Coordinate conversion : ??? -> ???    // 얘 왜 하는지 앎???
    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
                          laserCloudIn.points[cloudSize - 1].x) + 2 * M_PI;  // 왜 2pi 더할까

    if (endOri - startOri > 3 * M_PI)
    {
        endOri -= 2 * M_PI;
    }
    else if (endOri - startOri < M_PI)
    {
        endOri += 2 * M_PI;
    }
    //printf("end Ori %f\n", endOri);

    bool halfPassed = false;
    int count = cloudSize;

    // common.h에 미리 선언된 template : pcl::PointXYZI 를 의미함.
    PointType point;

    // scan line 개수에 맞게 PointType 형식의 vector 설정 => 각 라인별로 저장할 예정!
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);

    // 3. Calculate scanId and assign intensity for every points
    // input  : laserCloudIn point
    // Output : laserCloudScans point has intensity
    for (int i = 0; i < cloudSize; i++)
    {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;

        // 라이다 원점 기준 각 포인트의 각 측정 => 스캔 라인 추정의 시작
        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID = 0;

        /// 각 라이다 스펙 별, 각도 당 scan line 추정 방법론
        // VLP_16 : fov_up : 15, fov_down : -15
        if (N_SCANS == 16)
        {
            scanID = int((angle + 15) / 2 + 0.5);   // +15를 통해 음수 영역을 양수로 변환 + 0.5와 int 자료형을 통해 반올림 효과
            // 이상치 탐색 : 해당 포인트 등록 안해버리고, 전체 cloud 수 갑소시킴.
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;   // 해당 줄에서 끝나버리고, for문으로 돌아가
            }
        }
        //// VLP_32 : fov_up : 15, fov_down : -25
        else if (N_SCANS == 32)
        {
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);   // 왜 각도가 이렇게 됨??
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {   count--;
                continue;
            }
        }
        //// VLP_64 : fov_up : 2, fov_down : -24.8
        else if (N_SCANS == 64)
        {
            // 얘도 각도 왜이러냐?
            if (angle >= -8.83)
                scanID = int((2 - angle) * 3.0 + 0.5);
            else
                scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            // use [0 50]  > 50 remove outlies 
            if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else
        {
            printf("wrong scan number\n");
            ROS_BREAK();
        }
        //printf("angle %f scanID %d \n", angle, scanID);

        // 얘 뭐함??
        float ori = -atan2(point.y, point.x);
        if (!halfPassed)
        { 
            if (ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > startOri + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }

            if (ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        else
        {
            ori += 2 * M_PI;
            if (ori < endOri - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > endOri + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }

        float relTime = (ori - startOri) / (endOri - startOri); // 회전에 대한 상대 시간 측정?? 이건 왜 구함
        point.intensity = scanID + scanPeriod * relTime;    // intensity 항에 ScanID와 회전 시간 정보 추가
        laserCloudScans[scanID].push_back(point); // 각 point를 scan line에 맞게 할당
    }
    
    cloudSize = count;
    printf("points size %d \n", cloudSize);

    /// pcl::PointCloud<PointType>::Ptr => boost::shared_ptr로 됨. => 소멸될 때 자동적으로 delete : vector의 memory leak과 연관
    /// 참고 자료 : https://limhyungtae.github.io/2021-09-09-ROS-Point-Cloud-Library-(PCL)-1.-Ptr,-ConstPtr%EC%9D%98-%EC%99%84%EB%B2%BD-%EC%9D%B4%ED%95%B4-(1)-shared_ptr/
    /// 아래 문법 형식 제대로 이해 안됨..
    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());

    // 4-1. vector로 구성된 scan line별 point들을 pcl::PointCloud<PointType>::Ptr로 이뤄진 laserCloud에 순차적으로 쌓아.
    // scan line 별 pointcloud가 laserCloud에 쌓이기 때문에, 각 라인별로 구분할 수 있는 단서를 scanStartInd와 scanEndInd로 나눠서 저장해놈.
    for (int i = 0; i < N_SCANS; i++)
    { 
        scanStartInd[i] = laserCloud->size() + 5;
        *laserCloud += laserCloudScans[i];
        scanEndInd[i] = laserCloud->size() - 6;
    }
    printf("prepare time %f \n", t_prepare.toc());

    // 4-2. Caculate smoothness
    // input  :  laserCloud
    // output :  cloudCurvature[i]
    /// 모든 포인트에 대해 곡률 계산.
    /// 의문점 : scanline이 변한는 부분이 생길텐데, 이 사이를 어떻게 분별함??
    for (int i = 5; i < cloudSize - 5; i++)
    { 
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

        // cloudcurvature는 전역 변수로 미리 선언됨(float)
        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloudSortInd[i] = i;
        // edge로 선택 여부를 등록시켜놈 : 0으로 초기화
        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0;
    }

    // 5. Extract features
    // note : cloudlabel == 2  :  sharp corner feature
    //        cloudlabel == 1  :  less sharp corner feature
    //        cloudlabel == 0  :  not assigned
    //        cloudlabel == -1 :  planar feature

    TicToc t_pts;

    pcl::PointCloud<PointType> cornerPointsSharp;
    pcl::PointCloud<PointType> cornerPointsLessSharp;
    pcl::PointCloud<PointType> surfPointsFlat;
    pcl::PointCloud<PointType> surfPointsLessFlat;

    float t_q_sort = 0;
    for (int i = 0; i < N_SCANS; i++)
    {
        // 한 라인의 point수가 6개보다 적으면 예외 처리 : 고려 자체를 안함.
        if( scanEndInd[i] - scanStartInd[i] < 6)
            continue;

        /// Scanline 별 subsection 나누고 edge/planar 포인트 뽑기
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
        for (int j = 0; j < 6; j++)
        {
            //  5-1. divide into 6 subregion? not 4? : LOAM 논문이랑 다른 점.
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6;               // sub section 별 시작 포인트
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;     // sub section 별 끝 포인트

            TicToc t_tmp;
            // sorting : front : high c, rear : low c
            // sorting : cloudSortInd는 배열, sp는 그냥 int형 변수인데 어떻게 둘을 더한걸 sort할 수 있음? : 메모리 주소로 이해하면 됨.
            std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, comp);     //sort 기준 :
            t_q_sort += t_tmp.toc();
            
            //  5-2. Extract edge features
            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--)
            {
                int ind = cloudSortInd[k]; 

                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > 0.1) // 아직 edge로 사용 안됨 & 곡률 0.1보다 크다면,
                {
                    // 곡률이 내림차순으로 설정됐으므로, 젤 처음 뽑힌 애가 젤 곡률 큰 놈일 것.
                    largestPickedNum++;
                    // 가장 곡률 큰 포인트 2개까지 뽑아 cornerPointsSharp 객체로 관리.
                    if (largestPickedNum <= 2)
                    {                        
                        cloudLabel[ind] = 2;    // 곡률 큰 놈들은 label 2번으로 따로 관리
                        cornerPointsSharp.push_back(laserCloud->points[ind]);
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    // 곡률 큰 포인트 20개까지 뽑아 cornerPointsLessSharp 객체로 관리.
                    else if (largestPickedNum <= 20)
                    {                        
                        cloudLabel[ind] = 1;    // 곡률 적당히 큰 놈들은 label 1번으로 관리
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    else
                    {
                        break;
                    }
                    //  to prevent picking the same pixel again.
                    //  and to pick the feature evenly, doesn't use +- 10 index pixel again.
                    // corner로 뽑힌 애는 Picked 1로 바꿔버림
                    cloudNeighborPicked[ind] = 1;
                    // 뽑힌 edge point 주변, 인접한 point들 간의 차이를 구해서, 별 차이가 없다면(=edge 외 나머지 점들은 상대적으로 flat하다면)
                    // 다시 edge로 뽑힐 수 없게 Picked 1로 관리
                    // 근데 0.05라는 threshold가 왜 나왔는지 잘 모르겠음.
                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            //  5-2. Extract planar features
            int smallestPickedNum = 0;
            for (int k = sp; k <= ep; k++)
            {
                int ind = cloudSortInd[k];

                // 아직 사용 안됨 && 곡률 낮은 포인트라면
                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < 0.1)
                {
                    cloudLabel[ind] = -1; 
                    surfPointsFlat.push_back(laserCloud->points[ind]);

                    smallestPickedNum++;
                    // planar 포인트는 4개까지
                    if (smallestPickedNum >= 4)
                    { 
                        break;
                    }

                    // 해당 포인트 사용됐다는 증거 남겨
                    cloudNeighborPicked[ind] = 1;
                    // 아래 과정에 사용되는 threshold가 edge 뽑을때랑 왜 똑같은지. 그 의미는 무엇인지.
                    for (int l = 1; l <= 5; l++)
                    { 
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            // 사용 안되거나, flat하게 지정된 포인트들은 모두 surfPointsLessFlatScan에 담아 관리
            for (int k = sp; k <= ep; k++)
            {
                if (cloudLabel[k] <= 0)
                {
                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                }
            }
        }

        // surfPointsLessFlatScan 다운사이징 : voxelgrid 이용
        pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
        pcl::VoxelGrid<PointType> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilter.filter(surfPointsLessFlatScanDS);

        surfPointsLessFlat += surfPointsLessFlatScanDS;
    }
    printf("sort q time %f \n", t_q_sort);
    printf("seperate points time %f \n", t_pts.toc());

    // full pointcloud 토픽
    sensor_msgs::PointCloud2 laserCloudOutMsg;
    pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
    laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
    laserCloudOutMsg.header.frame_id = "camera_init";
    pubLaserCloud.publish(laserCloudOutMsg);

    // edge points 토픽(2개씩 뽑힌 상위 corner point)
    sensor_msgs::PointCloud2 cornerPointsSharpMsg;
    pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
    cornerPointsSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsSharpMsg.header.frame_id = "camera_init";
    pubCornerPointsSharp.publish(cornerPointsSharpMsg);

    // 20개씩 뽑힌 less edge points 토픽
    sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
    pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);
    cornerPointsLessSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsLessSharpMsg.header.frame_id = "camera_init";
    pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

    //4개씩 뽑힌 flat points 토픽
    sensor_msgs::PointCloud2 surfPointsFlat2;
    pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);
    surfPointsFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsFlat2.header.frame_id = "camera_init";
    pubSurfPointsFlat.publish(surfPointsFlat2);

    // 나머지 points 담당하는 lessFlatpoints 토픽
    sensor_msgs::PointCloud2 surfPointsLessFlat2;
    pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2);
    surfPointsLessFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsLessFlat2.header.frame_id = "camera_init";
    pubSurfPointsLessFlat.publish(surfPointsLessFlat2);

    // 각 scan line별로 pointcloud publish
    if(PUB_EACH_LINE)
    {
        for(int i = 0; i< N_SCANS; i++)
        {
            sensor_msgs::PointCloud2 scanMsg;
            pcl::toROSMsg(laserCloudScans[i], scanMsg);
            scanMsg.header.stamp = laserCloudMsg->header.stamp;
            scanMsg.header.frame_id = "camera_init";
            pubEachScan[i].publish(scanMsg);
        }
    }

    // registration 시간 너무 오래 걸린다면 warning 띄워..
    printf("scan registration time %f ms *************\n", t_whole.toc());
    if(t_whole.toc() > 100)
        ROS_WARN("scan registration process over 100ms");
}

int main(int argc, char **argv)
{
    // ros namespace의 init : ros master에 현재 프로세스를 노드로 등록하기 위한 함수
    ros::init(argc, argv, "scanRegistration");
    // NodeHandle 클래스 : 노드의 특성 가짐 => sub, pub 할 수 있는 주체 + param 및 서비스 통
    ros::NodeHandle nh;

    nh.param<int>("scan_line", N_SCANS, 16);

    nh.param<double>("minimum_range", MINIMUM_RANGE, 0.1);

    printf("scan line number %d \n", N_SCANS);

    if(N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64)
    {
        printf("only support velodyne with 16, 32 or 64 scan line!");
        return 0;
    }

    // ros subscriber : template으로 정의되어, 원하는 토픽 subscribe 할 수 있음 + 함수 포인터 활용한 call-back 구조 => 받은 토픽을 함수로 전달(포인터로 전달하는 듯)
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, laserCloudHandler);


    // 아래 publish 되는 변수들은 코드 젤 윗부분에서 전역적으로 미리 선언됨.
    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100);

    pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);

    pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100);

    pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);

    pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100);

    pubRemovePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_remove_points", 100);


    // PUB_EACH_LINE = 0 : 아래 if문 안돌아감.
    if(PUB_EACH_LINE)
    {
        for(int i = 0; i < N_SCANS; i++)
        {
            ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/laser_scanid_" + std::to_string(i), 100);
            pubEachScan.push_back(tmp);
        }
    }
    ros::spin();

    return 0;
}
