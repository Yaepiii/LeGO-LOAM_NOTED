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
//
// This is an implementation of the algorithm described in the following papers:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). October 2018.

#include "utility.h"

class FeatureAssociation{

private:

	ros::NodeHandle nh;

    ros::Subscriber subLaserCloud;
    ros::Subscriber subLaserCloudInfo;
    ros::Subscriber subOutlierCloud;
    ros::Subscriber subImu;

    ros::Publisher pubCornerPointsSharp;
    ros::Publisher pubCornerPointsLessSharp;
    ros::Publisher pubSurfPointsFlat;
    ros::Publisher pubSurfPointsLessFlat;

    pcl::PointCloud<PointType>::Ptr segmentedCloud;     // 有效分割点
    pcl::PointCloud<PointType>::Ptr outlierCloud;       // 外点

    pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
    pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
    pcl::PointCloud<PointType>::Ptr surfPointsFlat;
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;

    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan;
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScanDS;

    pcl::VoxelGrid<PointType> downSizeFilter;

    double timeScanCur;                 // 当前sweep的时间戳
    double timeNewSegmentedCloud;       // segmented_cloud时间戳
    double timeNewSegmentedCloudInfo;   // segmented_cloud_info时间戳
    double timeNewOutlierCloud;         // outlier_cloud时间戳

    bool newSegmentedCloud;
    bool newSegmentedCloudInfo;
    bool newOutlierCloud;

    cloud_msgs::cloud_info segInfo;     // 自定义点云信息话题
    std_msgs::Header cloudHeader;

    int systemInitCount;
    bool systemInited;

    std::vector<smoothness_t> cloudSmoothness;      // 平滑度数组
    float cloudCurvature[N_SCAN*Horizon_SCAN];      // 记录平滑度
    int cloudNeighborPicked[N_SCAN*Horizon_SCAN];   // 是否被选择
    int cloudLabel[N_SCAN*Horizon_SCAN];            // 类别

    int imuPointerFront;            // 比laserScan时间戳大的IMU队列索引
    int imuPointerLast;             // IMU最新的队列索引
    int imuPointerLastIteration;    // 指向上一次的IMU队列索引

    float imuRollStart, imuPitchStart, imuYawStart;
    float cosImuRollStart, cosImuPitchStart, cosImuYawStart, sinImuRollStart, sinImuPitchStart, sinImuYawStart;
    float imuRollCur, imuPitchCur, imuYawCur;

    float imuVeloXStart, imuVeloYStart, imuVeloZStart;
    float imuShiftXStart, imuShiftYStart, imuShiftZStart;

    float imuVeloXCur, imuVeloYCur, imuVeloZCur;
    float imuShiftXCur, imuShiftYCur, imuShiftZCur;

    float imuShiftFromStartXCur, imuShiftFromStartYCur, imuShiftFromStartZCur;
    float imuVeloFromStartXCur, imuVeloFromStartYCur, imuVeloFromStartZCur;

    float imuAngularRotationXCur, imuAngularRotationYCur, imuAngularRotationZCur;       // 当前帧绕三个轴的转角
    float imuAngularRotationXLast, imuAngularRotationYLast, imuAngularRotationZLast;    // 上一帧绕三个轴的转角
    float imuAngularFromStartX, imuAngularFromStartY, imuAngularFromStartZ;             // 相对于上一帧的转角

    double imuTime[imuQueLength];
    float imuRoll[imuQueLength];
    float imuPitch[imuQueLength];
    float imuYaw[imuQueLength];

    float imuAccX[imuQueLength];
    float imuAccY[imuQueLength];
    float imuAccZ[imuQueLength];

    float imuVeloX[imuQueLength];
    float imuVeloY[imuQueLength];
    float imuVeloZ[imuQueLength];

    float imuShiftX[imuQueLength];
    float imuShiftY[imuQueLength];
    float imuShiftZ[imuQueLength];

    float imuAngularVeloX[imuQueLength];
    float imuAngularVeloY[imuQueLength];
    float imuAngularVeloZ[imuQueLength];

    float imuAngularRotationX[imuQueLength];
    float imuAngularRotationY[imuQueLength];
    float imuAngularRotationZ[imuQueLength];



    ros::Publisher pubLaserCloudCornerLast;
    ros::Publisher pubLaserCloudSurfLast;
    ros::Publisher pubLaserOdometry;
    ros::Publisher pubOutlierCloudLast;

    int skipFrameNum;
    bool systemInitedLM;

    int laserCloudCornerLastNum;
    int laserCloudSurfLastNum;

    int pointSelCornerInd[N_SCAN*Horizon_SCAN];
    float pointSearchCornerInd1[N_SCAN*Horizon_SCAN];
    float pointSearchCornerInd2[N_SCAN*Horizon_SCAN];

    int pointSelSurfInd[N_SCAN*Horizon_SCAN];
    float pointSearchSurfInd1[N_SCAN*Horizon_SCAN];
    float pointSearchSurfInd2[N_SCAN*Horizon_SCAN];
    float pointSearchSurfInd3[N_SCAN*Horizon_SCAN];

    float transformCur[6];      // T_end_start:p_end = T_end_start * p_start
    float transformSum[6];      // T_init_end

    float imuRollLast, imuPitchLast, imuYawLast;
    float imuShiftFromStartX, imuShiftFromStartY, imuShiftFromStartZ;
    float imuVeloFromStartX, imuVeloFromStartY, imuVeloFromStartZ;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerLast;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfLast;

    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    PointType pointOri, pointSel, tripod1, tripod2, tripod3, pointProj, coeff;

    nav_msgs::Odometry laserOdometry;

    tf::TransformBroadcaster tfBroadcaster;
    tf::StampedTransform laserOdometryTrans;

    bool isDegenerate;
    cv::Mat matP;

    int frameCount;

public:

    FeatureAssociation():
        nh("~")
        {
        // 订阅和发布各类话题
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/segmented_cloud", 1, &FeatureAssociation::laserCloudHandler, this);
        subLaserCloudInfo = nh.subscribe<cloud_msgs::cloud_info>("/segmented_cloud_info", 1, &FeatureAssociation::laserCloudInfoHandler, this);
        subOutlierCloud = nh.subscribe<sensor_msgs::PointCloud2>("/outlier_cloud", 1, &FeatureAssociation::outlierCloudHandler, this);
        subImu = nh.subscribe<sensor_msgs::Imu>(imuTopic, 50, &FeatureAssociation::imuHandler, this);

        pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 1);
        pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 1);
        pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 1);
        pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 1);

        pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2);
        pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2);
        pubOutlierCloudLast = nh.advertise<sensor_msgs::PointCloud2>("/outlier_cloud_last", 2);
        pubLaserOdometry = nh.advertise<nav_msgs::Odometry> ("/laser_odom_to_init", 5);
        
        initializationValue();
    }

    // 各类参数的初始化
    void initializationValue()
    {
        cloudSmoothness.resize(N_SCAN*Horizon_SCAN);

        // 下采样滤波器设置叶子间距，就是格子之间的最小距离
		// voxelGrid类通过在点云数据中创建三维体素栅格，然后用每个体素的重心来近似表达体素中的其它点。
		// 具体用法：https://blog.csdn.net/kongli524/article/details/88534926
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);// 设置栅格大小，单位：m

        segmentedCloud.reset(new pcl::PointCloud<PointType>());
        outlierCloud.reset(new pcl::PointCloud<PointType>());

        cornerPointsSharp.reset(new pcl::PointCloud<PointType>());
        cornerPointsLessSharp.reset(new pcl::PointCloud<PointType>());
        surfPointsFlat.reset(new pcl::PointCloud<PointType>());
        surfPointsLessFlat.reset(new pcl::PointCloud<PointType>());

        surfPointsLessFlatScan.reset(new pcl::PointCloud<PointType>());
        surfPointsLessFlatScanDS.reset(new pcl::PointCloud<PointType>());

        timeScanCur = 0;
        timeNewSegmentedCloud = 0;
        timeNewSegmentedCloudInfo = 0;
        timeNewOutlierCloud = 0;

        newSegmentedCloud = false;
        newSegmentedCloudInfo = false;
        newOutlierCloud = false;

        systemInitCount = 0;
        systemInited = false;

        imuPointerFront = 0;
        imuPointerLast = -1;
        imuPointerLastIteration = 0;

        imuRollStart = 0; imuPitchStart = 0; imuYawStart = 0;
        cosImuRollStart = 0; cosImuPitchStart = 0; cosImuYawStart = 0;
        sinImuRollStart = 0; sinImuPitchStart = 0; sinImuYawStart = 0;
        imuRollCur = 0; imuPitchCur = 0; imuYawCur = 0;

        imuVeloXStart = 0; imuVeloYStart = 0; imuVeloZStart = 0;
        imuShiftXStart = 0; imuShiftYStart = 0; imuShiftZStart = 0;

        imuVeloXCur = 0; imuVeloYCur = 0; imuVeloZCur = 0;
        imuShiftXCur = 0; imuShiftYCur = 0; imuShiftZCur = 0;

        imuShiftFromStartXCur = 0; imuShiftFromStartYCur = 0; imuShiftFromStartZCur = 0;
        imuVeloFromStartXCur = 0; imuVeloFromStartYCur = 0; imuVeloFromStartZCur = 0;

        imuAngularRotationXCur = 0; imuAngularRotationYCur = 0; imuAngularRotationZCur = 0;
        imuAngularRotationXLast = 0; imuAngularRotationYLast = 0; imuAngularRotationZLast = 0;
        imuAngularFromStartX = 0; imuAngularFromStartY = 0; imuAngularFromStartZ = 0;

        for (int i = 0; i < imuQueLength; ++i)
        {
            imuTime[i] = 0;
            imuRoll[i] = 0; imuPitch[i] = 0; imuYaw[i] = 0;
            imuAccX[i] = 0; imuAccY[i] = 0; imuAccZ[i] = 0;
            imuVeloX[i] = 0; imuVeloY[i] = 0; imuVeloZ[i] = 0;
            imuShiftX[i] = 0; imuShiftY[i] = 0; imuShiftZ[i] = 0;
            imuAngularVeloX[i] = 0; imuAngularVeloY[i] = 0; imuAngularVeloZ[i] = 0;
            imuAngularRotationX[i] = 0; imuAngularRotationY[i] = 0; imuAngularRotationZ[i] = 0;
        }


        skipFrameNum = 1;

        for (int i = 0; i < 6; ++i){
            transformCur[i] = 0;
            transformSum[i] = 0;
        }

        systemInitedLM = false;

        imuRollLast = 0; imuPitchLast = 0; imuYawLast = 0;
        imuShiftFromStartX = 0; imuShiftFromStartY = 0; imuShiftFromStartZ = 0;
        imuVeloFromStartX = 0; imuVeloFromStartY = 0; imuVeloFromStartZ = 0;

        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        kdtreeCornerLast.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfLast.reset(new pcl::KdTreeFLANN<PointType>());

        laserOdometry.header.frame_id = "/camera_init";
        laserOdometry.child_frame_id = "/laser_odom";

        laserOdometryTrans.frame_id_ = "/camera_init";
        laserOdometryTrans.child_frame_id_ = "/laser_odom";
        
        isDegenerate = false;
        matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));

        frameCount = skipFrameNum;
    }

    // 更新当前帧初始时刻i=0时刻的rpy角的正余弦值
    void updateImuRollPitchYawStartSinCos(){
        cosImuRollStart = cos(imuRollStart);
        cosImuPitchStart = cos(imuPitchStart);
        cosImuYawStart = cos(imuYawStart);
        sinImuRollStart = sin(imuRollStart);
        sinImuPitchStart = sin(imuPitchStart);
        sinImuYawStart = sin(imuYawStart);
    }

	//该函数没用到？然而在TransformToStartIMU()函数中用到了imuShiftFromStartXYZCur，这三个值需要在该函数中进行计算，是否是写程序时遗漏了？
	//关于这个问题参考如下链接：https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/issues/157
	//概括地说就是IMU发散太快，采用IMU的位移补偿反而效果更差，因此不用
    void ShiftToStartIMU(float pointTime)
    {
        // 下面三个量表示的是世界坐标系下，从start到cur的坐标的漂移
        imuShiftFromStartXCur = imuShiftXCur - imuShiftXStart - imuVeloXStart * pointTime;
        imuShiftFromStartYCur = imuShiftYCur - imuShiftYStart - imuVeloYStart * pointTime;
        imuShiftFromStartZCur = imuShiftZCur - imuShiftZStart - imuVeloZStart * pointTime;

        // 从世界坐标系变换到start坐标系
        float x1 = cosImuYawStart * imuShiftFromStartXCur - sinImuYawStart * imuShiftFromStartZCur;
        float y1 = imuShiftFromStartYCur;
        float z1 = sinImuYawStart * imuShiftFromStartXCur + cosImuYawStart * imuShiftFromStartZCur;

        float x2 = x1;
        float y2 = cosImuPitchStart * y1 + sinImuPitchStart * z1;
        float z2 = -sinImuPitchStart * y1 + cosImuPitchStart * z1;

        imuShiftFromStartXCur = cosImuRollStart * x2 + sinImuRollStart * y2;
        imuShiftFromStartYCur = -sinImuRollStart * x2 + cosImuRollStart * y2;
        imuShiftFromStartZCur = z2;
    }

    // 将相对速度变换到相机坐标系下的初始时刻
    void VeloToStartIMU()
    {
        // imuVeloXStart,imuVeloYStart,imuVeloZStart是点云索引i=0时刻的速度
        // 此处计算的是相对于初始时刻i=0时的相对速度
        // 这个相对速度在世界坐标系下(即0时刻的左上前系)
        imuVeloFromStartXCur = imuVeloXCur - imuVeloXStart;
        imuVeloFromStartYCur = imuVeloYCur - imuVeloYStart;
        imuVeloFromStartZCur = imuVeloZCur - imuVeloZStart;

        // ！！！下面从世界坐标系转换到start坐标系
		//RPY角等价于旋转顺序为Y,P,R的欧拉角
		//因此第一步，绕世界系(即0时刻左上前系)Y轴旋转-imuYawStart
		//| cosImuYawStart   0  -sinImuYawStart|
		//| 0                1   0             |
		//| sinImuYawStart   0   cosImuYawStart|
        float x1 = cosImuYawStart * imuVeloFromStartXCur - sinImuYawStart * imuVeloFromStartZCur;
        float y1 = imuVeloFromStartYCur;
        float z1 = sinImuYawStart * imuVeloFromStartXCur + cosImuYawStart * imuVeloFromStartZCur;

		//第二步，绕世界系X轴旋转-imuPitchStart
		//| 1   0                  0               |
		//| 0   cosImuPitchStart   sinImuPitchStart|
		//| 0  -sinImuPitchStart   cosImuPitchStart|
        float x2 = x1;
        float y2 = cosImuPitchStart * y1 + sinImuPitchStart * z1;
        float z2 = -sinImuPitchStart * y1 + cosImuPitchStart * z1;

		//第三步，绕世界系Z轴旋转-imuRollStart
		//| cosImuRollStart   sinImuRollStart   0|
		//|-sinImuRollStart   cosImuRollStart   0|
		//| 0                 0                 1|
        imuVeloFromStartXCur = cosImuRollStart * x2 + sinImuRollStart * y2;
        imuVeloFromStartYCur = -sinImuRollStart * x2 + cosImuRollStart * y2;
        imuVeloFromStartZCur = z2;
    }

    // 该函数的功能是把点云坐标变换到初始时刻
    void TransformToStartIMU(PointType *p)
    {
        // 因为在adjustDistortion函数中有对xyz的坐标进行交换的过程
        // 交换的过程是x=原来的y，y=原来的z，z=原来的x
        // 所以下面其实是绕Z轴(原先的x轴)旋转，对应的是roll角
        //
        //     |cosrz  -sinrz  0|
        //  Rz=|sinrz  cosrz   0|
        //     |0       0      1|
        // [x1,y1,z1]^T=Rz*[x,y,z]
        //
        // 因为在imuHandler中进行过坐标变换，
        // 所以下面的roll其实已经对应于新坐标系中(X-Y-Z)的yaw
        float x1 = cos(imuRollCur) * p->x - sin(imuRollCur) * p->y;
        float y1 = sin(imuRollCur) * p->x + cos(imuRollCur) * p->y;
        float z1 = p->z;

        // 绕X轴(原先的y轴)旋转pitch
        // 
        // [x2,y2,z2]^T=Rx*[x1,y1,z1]
        //    |1     0        0|
        // Rx=|0   cosrx -sinrx|
        //    |0   sinrx  cosrx|
        float x2 = x1;
        float y2 = cos(imuPitchCur) * y1 - sin(imuPitchCur) * z1;
        float z2 = sin(imuPitchCur) * y1 + cos(imuPitchCur) * z1;

        // 最后再绕Y轴(原先的Z轴)旋转yaw
        //    |cosry   0   sinry|
        // Ry=|0       1       0|
        //    |-sinry  0   cosry|
        float x3 = cos(imuYawCur) * x2 + sin(imuYawCur) * z2;
        float y3 = y2;
        float z3 = -sin(imuYawCur) * x2 + cos(imuYawCur) * z2;

        // 下面部分的代码功能是从imu坐标的原点变换到i=0时imu的初始时刻(从世界坐标系变换到start坐标系)
        // 变换方式和函数VeloToStartIMU()中的类似
        // 变换顺序：Cur-->世界坐标系-->Start，这两次变换中，
        // 前一次是正变换，角度为正，后一次是逆变换，角度应该为负
        float x4 = cosImuYawStart * x3 - sinImuYawStart * z3;
        float y4 = y3;
        float z4 = sinImuYawStart * x3 + cosImuYawStart * z3;

        float x5 = x4;
        float y5 = cosImuPitchStart * y4 + sinImuPitchStart * z4;
        float z5 = -sinImuPitchStart * y4 + cosImuPitchStart * z4;

        // 绕z轴(原先的x轴)变换角度到初始imu时刻，另外需要加上imu的位移漂移
        // 后面加上的 imuShiftFromStart.. 表示从start时刻到cur时刻的漂移，
        // (imuShiftFromStart.. 在start坐标系下)
        p->x = cosImuRollStart * x5 + sinImuRollStart * y5 + imuShiftFromStartXCur;
        p->y = -sinImuRollStart * x5 + cosImuRollStart * y5 + imuShiftFromStartYCur;
        p->z = z5 + imuShiftFromStartZCur;
    }

    // 先将当前时刻加速度变换到初始时刻坐标系(世界坐标系)下，然后根据加速度，计算IMU累计位移、速度和角度
    void AccumulateIMUShiftAndRotation()
    {
        float roll = imuRoll[imuPointerLast];
        float pitch = imuPitch[imuPointerLast];
        float yaw = imuYaw[imuPointerLast];
        float accX = imuAccX[imuPointerLast];
        float accY = imuAccY[imuPointerLast];
        float accZ = imuAccZ[imuPointerLast];

		//将(t_左上前)系下的运动加速度acc转换至(0_左上前)系，并进行积分
		//第一步，acc(t_左上前)-->acc(t_IMU)。
		//             |0   0   1|                   |0   0   1|   |accX|   |accZ|
		//acc(t_IMU) = |1   0   0| * acc(t_左上前) =  |1   0   0| * |accY| = |accX|;
		//             |0   1   0|                   |0   1   0|   |accZ|   |accY|
		//
		//第二步，acc(t_IMU)-->acc(0_IMU)。
		//
		//知识点：
		//t_IMU系相对0_IMU系的RPY角为(roll, pitch, yaw)，
		//即将0_IMU系绕0_IMU系的X轴正向旋转roll，得到1_IMU系；
		//然后将1_IMU系绕0_IMU系的Y轴正向旋转pitch，得到2_IMU系；
		//然后将2_IMU系绕0_IMU系的Z轴正向旋转yaw，最后得到t_IMU系。
		//在这样的定义下，任一向量V，其坐标转换关系为，
		//V(t_IMU) = R(X,roll) * R(Y,pitch) * R(Z,yaw) * V(0_IMU);
		//即R(0_IMU到t_IMU) = R(X,roll) * R(Y,pitch) * R(Z,yaw)
		//   |1   0         0      |   |c(pitch)   0  -s(pitch)|   | c(yaw)   s(yaw)   0|
		// = |0   c(roll)   s(roll)| * |0          1   0       | * |-s(yaw)   c(yaw)   0|;
		//   |0  -s(roll)   c(roll)|   |s(pitch)   0   c(pitch)|   | 0        0        1|
		//因此，V(0_IMU) = invR(Z,yaw) * invR(Y,pitch) * invR(X,roll) * V(t_IMU);
		//即V(0_IMU) = R(Z,-yaw) * R(Y,-pitch) * R(X,-roll) * V(t_IMU);
		//
		//因此，acc(0_IMU) = R(Z,-yaw) * R(Y,-pitch) * R(X,-roll) * acc(t_IMU)
		//   | c(yaw)  -s(yaw)   0|   | c(pitch)   0   s(pitch)|   | 1   0         0      |   |accZ|
		// = | s(yaw)   c(yaw)   0| * | 0          1   0       | * | 0   c(roll)  -s(roll)| * |accX|
		//   | 0        0        1|   |-s(pitch)   0   c(pitch)|   | 0   s(roll)   c(roll)|   |accY|
		//
		//   | c(yaw)  -s(yaw)   0|   | c(pitch)   0   s(pitch)|   | accZ                           |
		// = | s(yaw)   c(yaw)   0| * | 0          1   0       | * | c(roll) * accX - s(roll) * accY|;
		//   | 0        0        1|   |-s(pitch)   0   c(pitch)|   | s(roll) * accX + c(roll) * accY|
		//
		//根据下面程序设置的变量，
		//|z1|   | accZ                           |
		//|x1| = | c(roll) * accX - s(roll) * accY|;
		//|y1|   | s(roll) * accX + c(roll) * accY|
		//
		//则
		//             | c(yaw)  -s(yaw)   0|   | c(pitch)   0   s(pitch)|   |z1|
		//acc(0_IMU) = | s(yaw)   c(yaw)   0| * | 0          1   0       | * |x1|
		//             | 0        0        1|   |-s(pitch)   0   c(pitch)|   |y1|
		//
		//   | c(yaw)  -s(yaw)   0|   | c(pitch) * z1 + s(pitch) * y1|
		// = | s(yaw)   c(yaw)   0| * | x1                           |;
		//   | 0        0        1|   |-s(pitch) * z1 + c(pitch) * y1|
		//
		//根据下面程序设置的变量，
		//|z2|   | c(pitch) * z1 + s(pitch) * y1|
		//|x2| = | x1                           |;
		//|y2|   |-s(pitch) * z1 + c(pitch) * y1|
		//
		//则
		//             | c(yaw)  -s(yaw)   0|   |z2|   | c(yaw) * z2 - s(yaw) * x2|
		//acc(0_IMU) = | s(yaw)   c(yaw)   0| * |x2| = | s(yaw) * z2 + c(yaw) * x2|;
		//             | 0        0        1|   |y2|   | y2                       |
		//
		//第三步，acc(0_IMU)-->acc(0_左上前)。
		//左上前系到前左上系的转换矩阵为 |0   0   1|，
		//                               |1   0   0|
		//                               |0   1   0|
		//因此前左上系到左上前系的转换矩阵为其转置，即 |0   1   0|，
		//                                             |0   0   1|
		//                                             |1   0   0|
		//因此，
		//                |accX|   |0   1   0|                | s(yaw) * z2 + c(yaw) * x2|
		//acc(0_左上前) = |accY| = |0   0   1| * acc(0_IMU) = | y2                       |;
		//                |accZ|   |1   0   0|                | c(yaw) * z2 - s(yaw) * x2|

        // 先绕Z轴(原x轴)旋转,下方坐标系示意imuHandler()中加速度的坐标轴交换
        //  z->Y
        //  ^  
        //  |    ^ y->X
        //  |   /
        //  |  /
        //  | /
        //  -----> x->Z
        //
        //     |cosrz  -sinrz  0|
        //  Rz=|sinrz  cosrz   0|
        //     |0       0      1|
        // [x1,y1,z1]^T=Rz*[accX,accY,accZ]
        // 因为在imuHandler中进行过坐标变换，
        // 所以下面的roll其实已经对应于新坐标系中(X-Y-Z)的yaw
        float x1 = cos(roll) * accX - sin(roll) * accY;
        float y1 = sin(roll) * accX + cos(roll) * accY;
        float z1 = accZ;

        // 绕X轴(原y轴)旋转
        // [x2,y2,z2]^T=Rx*[x1,y1,z1]
        //    |1     0        0|
        // Rx=|0   cosrx -sinrx|
        //    |0   sinrx  cosrx|
        float x2 = x1;
        float y2 = cos(pitch) * y1 - sin(pitch) * z1;
        float z2 = sin(pitch) * y1 + cos(pitch) * z1;

        // 最后再绕Y轴(原z轴)旋转
        //    |cosry   0   sinry|
        // Ry=|0       1       0|
        //    |-sinry  0   cosry|
		//这三个加速度分量是在0时刻的左上前系下的
        accX = cos(yaw) * x2 + sin(yaw) * z2;
        accY = y2;
        accZ = -sin(yaw) * x2 + cos(yaw) * z2;

        // 进行位移，速度，角度量的累加(以初始时刻坐标系为基坐标系)
        int imuPointerBack = (imuPointerLast + imuQueLength - 1) % imuQueLength;//上一时刻的IMU数据索引
        double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];
        if (timeDiff < scanPeriod) {//保证IMU数据时间间隔小于LiDAR扫描间隔
			//位置和速度是在0时刻左上前坐标系下累加的
            imuShiftX[imuPointerLast] = imuShiftX[imuPointerBack] + imuVeloX[imuPointerBack] * timeDiff + accX * timeDiff * timeDiff / 2;
            imuShiftY[imuPointerLast] = imuShiftY[imuPointerBack] + imuVeloY[imuPointerBack] * timeDiff + accY * timeDiff * timeDiff / 2;
            imuShiftZ[imuPointerLast] = imuShiftZ[imuPointerBack] + imuVeloZ[imuPointerBack] * timeDiff + accZ * timeDiff * timeDiff / 2;

            imuVeloX[imuPointerLast] = imuVeloX[imuPointerBack] + accX * timeDiff;
            imuVeloY[imuPointerLast] = imuVeloY[imuPointerBack] + accY * timeDiff;
            imuVeloZ[imuPointerLast] = imuVeloZ[imuPointerBack] + accZ * timeDiff;

			//角度是在0时刻IMU系，即0时刻前左上坐标系下累加的
            imuAngularRotationX[imuPointerLast] = imuAngularRotationX[imuPointerBack] + imuAngularVeloX[imuPointerBack] * timeDiff;
            imuAngularRotationY[imuPointerLast] = imuAngularRotationY[imuPointerBack] + imuAngularVeloY[imuPointerBack] * timeDiff;
            imuAngularRotationZ[imuPointerLast] = imuAngularRotationZ[imuPointerBack] + imuAngularVeloZ[imuPointerBack] * timeDiff;
        }
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
    {
        double roll, pitch, yaw;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(imuIn->orientation, orientation);
		//IMU系采用前左上坐标系
		//ROS中的RPY角是绕固定（参考）坐标轴旋转的，假设开始两个坐标系重合，
		//先将{B}绕{A}的X轴旋转roll，然后绕{A}的Y轴旋转pitch，最后绕{A}的Z轴旋转yaw，就能旋转到当前姿态。
		//称其为X-Y-Z fixed angles或RPY角(roll, pitch, yaw)。
		//且旋转矩阵是右乘的，即
		//R(A系到B系)=R(AX,roll)*R(AY,pitch)*R(AZ,yaw);
		//RPY分别为绕X正向的横滚角roll，绕Y轴正向的pitch角，绕Z轴正向的yaw角
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // 加速度去除重力影响，同时坐标轴进行变换
		//假定初始时刻IMU系与水平。先把重力从初始时刻IMU系转换到当前IMU系，根据RPY角的右乘法则，有
		//(g(t_IMU)表示t时刻IMU系下的重力加速度，g(0_IMU)表示0时刻IMU系下的重力加速度)
		//           | 0|   | 0   |
		//g(0_IMU) = | 0| = | 0   |
		//           |-g|   |-9.81|
		//
		//           |1   0         0      |   |c(pitch)   0  -s(pitch)|   | c(yaw)   s(yaw)   0|
		//g(t_IMU) = |0   c(roll)   s(roll)| * |0          1   0       | * |-s(yaw)   c(yaw)   0| * g(0_IMU)
		//           |0  -s(roll)   c(roll)|   |s(pitch)   0   c(pitch)|   | 0        0        1|
		//
		//   |1   0         0      |   |c(pitch)   0  -s(pitch)|   | 0|
		// = |0   c(roll)   s(roll)| * |0          1   0       | * | 0|
		//   |0  -s(roll)   c(roll)|   |s(pitch)   0   c(pitch)|   |-g|
		//
		//   |1   0         0      |   |s(pitch) * g |
		// = |0   c(roll)   s(roll)| * |0            |
		//   |0  -s(roll)   c(roll)|   |-c(pitch) * g|
		//
		//   |s(pitch) * g           |
		// = |-s(roll) * c(pitch) * g|
		//   |-c(roll) * c(pitch) * g|
		//
		//根据比力方程，运动加速度 = 比力 + 引力加速度，有
		//acc(t_IMU) = imuIn->linear_acceleration + g(t_IMU)
		//
		//   |imuIn->linear_acceleration.x + s(pitch) * g          |
		// = |imuIn->linear_acceleration.y - s(roll) * c(pitch) * g|
		//   |imuIn->linear_acceleration.z - c(roll) * c(pitch) * g|
		//
		//IMU为前左上坐标系，
		//               (z+)
		//                ^
		//                |      ^(x+，车头前向)
		//                |    /
		//                |  /
		//                |/
		//(y+)<-----------O
		//
		//随后进行坐标系转换，将运动加速度acc转换至左上前坐标系，
		//               (y+)
		//                ^
		//                |      ^(z+，车头前向)
		//                |    /
		//                |  /
		//                |/
		//(x+)<-----------O
        float accX = imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81;
        float accY = imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81;
        float accZ = imuIn->linear_acceleration.x + sin(pitch) * 9.81;

        imuPointerLast = (imuPointerLast + 1) % imuQueLength;   // 队列索引加1

        imuTime[imuPointerLast] = imuIn->header.stamp.toSec();  // imuTime记录时间戳

		//这里的roll，pitch，yaw是分别先后绕初始时刻的0_IMU系旋转的角度，
		//即0_IMU系下的RPY角，世界坐标系下RPY角
        imuRoll[imuPointerLast] = roll;
        imuPitch[imuPointerLast] = pitch;
        imuYaw[imuPointerLast] = yaw;

		//左上前坐标系下的三轴加速度
        imuAccX[imuPointerLast] = accX;
        imuAccY[imuPointerLast] = accY;
        imuAccZ[imuPointerLast] = accZ;

		//即0_IMU系下的RPY角速度
        imuAngularVeloX[imuPointerLast] = imuIn->angular_velocity.x;
        imuAngularVeloY[imuPointerLast] = imuIn->angular_velocity.y;
        imuAngularVeloZ[imuPointerLast] = imuIn->angular_velocity.z;

        AccumulateIMUShiftAndRotation();// 先将当前时刻加速度变换到初始时刻坐标系下，然后根据加速度，计算IMU累计位移、速度和角度
    }

    void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        cloudHeader = laserCloudMsg->header;

        timeScanCur = cloudHeader.stamp.toSec();
        timeNewSegmentedCloud = timeScanCur;

        segmentedCloud->clear();
        pcl::fromROSMsg(*laserCloudMsg, *segmentedCloud);

        newSegmentedCloud = true;
    }

    void outlierCloudHandler(const sensor_msgs::PointCloud2ConstPtr& msgIn){

        timeNewOutlierCloud = msgIn->header.stamp.toSec();

        outlierCloud->clear();
        pcl::fromROSMsg(*msgIn, *outlierCloud);

        newOutlierCloud = true;
    }

    void laserCloudInfoHandler(const cloud_msgs::cloud_infoConstPtr& msgIn)
    {
        timeNewSegmentedCloudInfo = msgIn->header.stamp.toSec();
        segInfo = *msgIn;
        newSegmentedCloudInfo = true;
    }

    // 进行插值，并将点云变换到当前帧的IMU初始时刻
    void adjustDistortion()
    {
        bool halfPassed = false;
        int cloudSize = segmentedCloud->points.size();

        PointType point;

        for (int i = 0; i < cloudSize; i++) {
            // 雷达坐标系转换到相机坐标系
            point.x = segmentedCloud->points[i].y;
            point.y = segmentedCloud->points[i].z;
            point.z = segmentedCloud->points[i].x;

            // -atan2(p.x,p.z)==>-atan2(y,x)
            // ori表示的是偏航角yaw，因为前面有负号，ori=[-M_PI,M_PI)
            // 因为segInfo.orientationDiff表示的范围是(PI,3PI)，在2PI附近
            // 下面过程的主要作用是调整ori大小，满足start<ori<end
            float ori = -atan2(point.x, point.z);
            if (!halfPassed) {
                if (ori < segInfo.startOrientation - M_PI / 2)
					// start-ori>M_PI/2，说明ori小于start，不合理，
					// 正常情况在前半圈的话，ori-stat范围[0,M_PI]
                    ori += 2 * M_PI;
                else if (ori > segInfo.startOrientation + M_PI * 3 / 2)
					// ori-start>3/2*M_PI,说明ori太大，不合理
                    ori -= 2 * M_PI;

                if (ori - segInfo.startOrientation > M_PI)
                    halfPassed = true;
            } else {
                ori += 2 * M_PI;

                if (ori < segInfo.endOrientation - M_PI * 3 / 2)
					// end-ori>3/2*PI,ori太小
                    ori += 2 * M_PI;
                else if (ori > segInfo.endOrientation + M_PI / 2)
					// ori-end>M_PI/2,太大
                    ori -= 2 * M_PI;
            }

            // 用 point.intensity 来保存时间
            float relTime = (ori - segInfo.startOrientation) / segInfo.orientationDiff;
            point.intensity = int(segmentedCloud->points[i].intensity) + scanPeriod * relTime;  // 与LOAM中定义一样 scanID + scanPeriod * relTime

            if (imuPointerLast >= 0) {//imuPointerLast被初始化为-1，imuPointerLast >= 0表示有IMU数据
                float pointTime = relTime * scanPeriod;//一帧内相对起始扫描点的时间
                imuPointerFront = imuPointerLastIteration;  // imuPointerLastIteration = 0
                // while循环内进行时间轴对齐
				//对齐到当前帧扫描开始时间后的第一个IMU数据
                while (imuPointerFront != imuPointerLast) {
					//timeScanCur为当前帧点云的起始扫描时间
                    if (timeScanCur + pointTime < imuTime[imuPointerFront]) {
                        break;
                    }
                    imuPointerFront = (imuPointerFront + 1) % imuQueLength;
                }

                // 对RPY角、速度、位移进行插值
                if (timeScanCur + pointTime > imuTime[imuPointerFront]) {
                    // 该条件内imu数据比激光数据早，但是没有更后面的数据
                    // (打个比方,激光在9点时出现，imu现在只有8点的)
                    // 这种情况上面while循环是以imuPointerFront == imuPointerLast结束的
                    imuRollCur = imuRoll[imuPointerFront];
                    imuPitchCur = imuPitch[imuPointerFront];
                    imuYawCur = imuYaw[imuPointerFront];

                    imuVeloXCur = imuVeloX[imuPointerFront];
                    imuVeloYCur = imuVeloY[imuPointerFront];
                    imuVeloZCur = imuVeloZ[imuPointerFront];

                    imuShiftXCur = imuShiftX[imuPointerFront];
                    imuShiftYCur = imuShiftY[imuPointerFront];
                    imuShiftZCur = imuShiftZ[imuPointerFront];
                } else {
                    // 在imu数据充足的情况下可以进行插补
                    // 当前timeScanCur + pointTime < imuTime[imuPointerFront]，
                    // 而且imuPointerFront是最早一个时间大于timeScanCur + pointTime的imu数据指针
                    int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
                    float ratioFront = (timeScanCur + pointTime - imuTime[imuPointerBack]) 
                                                     / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
                    float ratioBack = (imuTime[imuPointerFront] - timeScanCur - pointTime) 
                                                    / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
					
                    // 通过上面计算的ratioFront以及ratioBack进行插补
                    // 因为imuRollCur和imuPitchCur通常都在0度左右，变化不会很大，因此不需要考虑超过2M_PI的情况
                    // imuYaw转的角度比较大，需要考虑超过2*M_PI的情况
                    imuRollCur = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
                    imuPitchCur = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
                    if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] > M_PI) {
                        imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] + 2 * M_PI) * ratioBack;
                    } else if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] < -M_PI) {
                        imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] - 2 * M_PI) * ratioBack;
                    } else {
                        imuYawCur = imuYaw[imuPointerFront] * ratioFront + imuYaw[imuPointerBack] * ratioBack;
                    }

					// imu速度插补
                    imuVeloXCur = imuVeloX[imuPointerFront] * ratioFront + imuVeloX[imuPointerBack] * ratioBack;
                    imuVeloYCur = imuVeloY[imuPointerFront] * ratioFront + imuVeloY[imuPointerBack] * ratioBack;
                    imuVeloZCur = imuVeloZ[imuPointerFront] * ratioFront + imuVeloZ[imuPointerBack] * ratioBack;

                    // imu位置插补
                    imuShiftXCur = imuShiftX[imuPointerFront] * ratioFront + imuShiftX[imuPointerBack] * ratioBack;
                    imuShiftYCur = imuShiftY[imuPointerFront] * ratioFront + imuShiftY[imuPointerBack] * ratioBack;
                    imuShiftZCur = imuShiftZ[imuPointerFront] * ratioFront + imuShiftZ[imuPointerBack] * ratioBack;
                }

                if (i == 0) {//如果是第一个点，就把该点的IMU数据设为初值
                    // 此处更新过的角度值主要用在updateImuRollPitchYawStartSinCos()中,
                    // 更新每个角的正余弦值
                    imuRollStart = imuRollCur;
                    imuPitchStart = imuPitchCur;
                    imuYawStart = imuYawCur;

                    imuVeloXStart = imuVeloXCur;
                    imuVeloYStart = imuVeloYCur;
                    imuVeloZStart = imuVeloZCur;

                    imuShiftXStart = imuShiftXCur;
                    imuShiftYStart = imuShiftYCur;
                    imuShiftZStart = imuShiftZCur;

                    if (timeScanCur + pointTime > imuTime[imuPointerFront]) {
                        // 该条件内imu数据比激光数据早，但是没有更后面的数据
                        imuAngularRotationXCur = imuAngularRotationX[imuPointerFront];
                        imuAngularRotationYCur = imuAngularRotationY[imuPointerFront];
                        imuAngularRotationZCur = imuAngularRotationZ[imuPointerFront];
                    }else{
                        // 在imu数据充足的情况下可以进行插补
                        int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
                        float ratioFront = (timeScanCur + pointTime - imuTime[imuPointerBack]) 
                                                         / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
                        float ratioBack = (imuTime[imuPointerFront] - timeScanCur - pointTime) 
                                                        / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
                        imuAngularRotationXCur = imuAngularRotationX[imuPointerFront] * ratioFront + imuAngularRotationX[imuPointerBack] * ratioBack;
                        imuAngularRotationYCur = imuAngularRotationY[imuPointerFront] * ratioFront + imuAngularRotationY[imuPointerBack] * ratioBack;
                        imuAngularRotationZCur = imuAngularRotationZ[imuPointerFront] * ratioFront + imuAngularRotationZ[imuPointerBack] * ratioBack;
                    }

                    // 距离上一次插补（即上一帧），旋转过的角度变化值
                    imuAngularFromStartX = imuAngularRotationXCur - imuAngularRotationXLast;
                    imuAngularFromStartY = imuAngularRotationYCur - imuAngularRotationYLast;
                    imuAngularFromStartZ = imuAngularRotationZCur - imuAngularRotationZLast;

                    imuAngularRotationXLast = imuAngularRotationXCur;
                    imuAngularRotationYLast = imuAngularRotationYCur;
                    imuAngularRotationZLast = imuAngularRotationZCur;

                    // 这里更新的是i=0时刻的rpy角，后面将速度坐标投影过来会用到i=0时刻的值
                    updateImuRollPitchYawStartSinCos();
                } else {
                    // 将相对速度变换到相机坐标系下的初始时刻
                    VeloToStartIMU();
					// 将点的坐标变换到相机坐标系下的初始时刻
                    TransformToStartIMU(&point);
                }
            }
            segmentedCloud->points[i] = point;
        }

        imuPointerLastIteration = imuPointerLast;
    }

    // 计算平滑度
    void calculateSmoothness()
    {
        int cloudSize = segmentedCloud->points.size();
        for (int i = 5; i < cloudSize - 5; i++) {

			//这里计算曲率用的都是分割后的点云数据，是否采用原始点云数据更为合理？
			//即用原始点云数据计算segmentedCloud中的点的曲率
			//答：聚类后的点云也是连续的，和原始点云没有区别
			//而且和loam中不同，这里用的是距离标量，而loam中用的是距离矢量
            float diffRange = segInfo.segmentedCloudRange[i-5] + segInfo.segmentedCloudRange[i-4]
                            + segInfo.segmentedCloudRange[i-3] + segInfo.segmentedCloudRange[i-2]
                            + segInfo.segmentedCloudRange[i-1] - segInfo.segmentedCloudRange[i] * 10
                            + segInfo.segmentedCloudRange[i+1] + segInfo.segmentedCloudRange[i+2]
                            + segInfo.segmentedCloudRange[i+3] + segInfo.segmentedCloudRange[i+4]
                            + segInfo.segmentedCloudRange[i+5];            

            cloudCurvature[i] = diffRange*diffRange;

            // 在markOccludedPoints()函数中对该参数进行重新修改
            cloudNeighborPicked[i] = 0;
			// 在extractFeatures()函数中会对标签进行修改，
			// 初始化为0，surfPointsFlat标记为-1，surfPointsLessFlatScan为不大于0的标签
			// cornerPointsSharp标记为2，cornerPointsLessSharp标记为1
            cloudLabel[i] = 0;

            cloudSmoothness[i].value = cloudCurvature[i];
            cloudSmoothness[i].ind = i;
        }
    }

    // 剔除一些互相遮挡，或者所在平面与激光束近似平行的点
    void markOccludedPoints()
    {
        int cloudSize = segmentedCloud->points.size();

        for (int i = 5; i < cloudSize - 6; ++i){

            float depth1 = segInfo.segmentedCloudRange[i];
            float depth2 = segInfo.segmentedCloudRange[i+1];
            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[i+1] - segInfo.segmentedCloudColInd[i]));//点的列索引之差（点是按行存储的，也就是说点已经是同一行的了）

            if (columnDiff < 10){

                // 选择距离较远的那些点（即遮挡情况），并将他们标记为1
                if (depth1 - depth2 > 0.3){
                    cloudNeighborPicked[i - 5] = 1;
                    cloudNeighborPicked[i - 4] = 1;
                    cloudNeighborPicked[i - 3] = 1;
                    cloudNeighborPicked[i - 2] = 1;
                    cloudNeighborPicked[i - 1] = 1;
                    cloudNeighborPicked[i] = 1;
                }else if (depth2 - depth1 > 0.3){
                    cloudNeighborPicked[i + 1] = 1;
                    cloudNeighborPicked[i + 2] = 1;
                    cloudNeighborPicked[i + 3] = 1;
                    cloudNeighborPicked[i + 4] = 1;
                    cloudNeighborPicked[i + 5] = 1;
                    cloudNeighborPicked[i + 6] = 1;
                }
            }

            float diff1 = std::abs(segInfo.segmentedCloudRange[i-1] - segInfo.segmentedCloudRange[i]);
            float diff2 = std::abs(segInfo.segmentedCloudRange[i+1] - segInfo.segmentedCloudRange[i]);

            // 选择距离变化较大的点，并将他们标记为1（即距离变化大的点通常位于“与某个激光束大致平行的局部平面--loam”，不可靠，因此需要舍弃，详细解释见loam论文）
            if (diff1 > 0.02 * segInfo.segmentedCloudRange[i] && diff2 > 0.02 * segInfo.segmentedCloudRange[i])
                cloudNeighborPicked[i] = 1;
        }
    }

    // 特征提取
    void extractFeatures()
    {
        cornerPointsSharp->clear();
        cornerPointsLessSharp->clear();
        surfPointsFlat->clear();
        surfPointsLessFlat->clear();

        for (int i = 0; i < N_SCAN; i++) {

            surfPointsLessFlatScan->clear();

            for (int j = 0; j < 6; j++) {

                // sp和ep的含义是什么???startPointer,endPointer?
				//sp = segInfo.startRingIndex[i] + j / 6 * (segInfo.endRingIndex[i] - segInfo.startRingIndex[i]);
				//ep = segInfo.startRingIndex[i] + (j + 1) / 6 * (segInfo.endRingIndex[i] - segInfo.startRingIndex[i]) - 1;
				//即将第i线聚类后的点云均分成6份，开始列索引为sp，结束列索引为ep
                int sp = (segInfo.startRingIndex[i] * (6 - j)    + segInfo.endRingIndex[i] * j) / 6;
                int ep = (segInfo.startRingIndex[i] * (5 - j)    + segInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

				//这种情况在该线聚类后点数过少时会发生，因为
				//segMsg.startRingIndex[i] = sizeOfSegCloud-1 + 5;
				//segMsg.endRingIndex[i] = sizeOfSegCloud - 1 - 5;
				//前后会分别空出5个点，点数过少就会出问题
                if (sp >= ep)
                    continue;

                // 按照cloudSmoothness.value从小到大排序
				//c++ sort函数的用法，对结构体排序，可以参考下面的博客
				//https://blog.csdn.net/Architect_chaser/article/details/88322605
                std::sort(cloudSmoothness.begin()+sp, cloudSmoothness.begin()+ep, by_value());

				//该段循环的功能：
				//从segmentedCloud的第i线的sp到ep之间提取最大nFe=2个“曲率”的点，存入cornerPointsSharp中；提取最大20个“曲率”的点，存入cornerPointsLessSharp中
                int largestPickedNum = 0;
                for (int k = ep; k >= sp; k--) {
                    // 每次ind的值就是等于k??? 有什么意义?
                    // 因为上面对cloudSmoothness进行了一次从小到大排序，所以ind不一定等于k了
                    
                    // 对应论文中提到的：edge point只在segmented clusters中选择
                    int ind = cloudSmoothness[k].ind;           // ind为在segmentedCloud中的索引
                    if (cloudNeighborPicked[ind] == 0 &&        // 不是阻塞点
                        cloudCurvature[ind] > edgeThreshold &&  // “曲率”大于阈值    edgeThreshold = 0.1
                        segInfo.segmentedCloudGroundFlag[ind] == false) {//非地面点
                    
                        largestPickedNum++;
                        if (largestPickedNum <= 2) {
                            // 论文中nFe=2,cloudSmoothness已经按照从小到大的顺序排列，
                            // 所以这边只要选择最后两个放进队列即可
                            // cornerPointsSharp标记为2
                            cloudLabel[ind] = 2;
                            cornerPointsSharp->push_back(segmentedCloud->points[ind]);
                            cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
                        } else if (largestPickedNum <= 20) {
							// 塞20个点到cornerPointsLessSharp中去
							// cornerPointsLessSharp标记为1
                            cloudLabel[ind] = 1;
                            cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
                        } else {
                            break;
                        }

						//确保提取的角点不会太接近，与已经提取的角点相近的点被标记，从而不会再被提取
                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++) {
                            // 从ind+l开始后面5个点，每个点index之间的差值，
                            // 确保columnDiff<=10,然后标记为我们需要的点
                            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {
							// 从ind+l开始前面五个点，计算差值然后标记
                            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                int smallestPickedNum = 0;
                for (int k = sp; k <= ep; k++) {
                    int ind = cloudSmoothness[k].ind;

                    // 对应论文中提到的：planar point只在ground point中选择
                    if (cloudNeighborPicked[ind] == 0 &&//不是阻塞点
                        cloudCurvature[ind] < surfThreshold &&//“曲率”小于阈值
                        segInfo.segmentedCloudGroundFlag[ind] == true) {//只从地面点中提取平面点

						//surfPointsFlat标记为-1
                        cloudLabel[ind] = -1;
                        surfPointsFlat->push_back(segmentedCloud->points[ind]);

                        // 论文中nFp=4，将4个最平的平面点放入队列中
                        smallestPickedNum++;
                        if (smallestPickedNum >= 4) {
                            break;
                        }

						////确保提取的平面点不会太接近，与已经提取的平面点相近的点被标记，从而不会再被提取
                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++) {
                            // 从前面往后判断是否是需要的邻接点，是的话就进行标记
                            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {
                            // 从后往前开始标记
                            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

				// 剩下的点都归入less planar point
				// 剩下的点：距离选定特征比较近的前后5个点、非地面点形成簇的平面点、遮挡和与激光束近似平行平面上的点
                for (int k = sp; k <= ep; k++) {
                    if (cloudLabel[k] <= 0) {
                        surfPointsLessFlatScan->push_back(segmentedCloud->points[k]);
                    }
                }
            }

            // surfPointsLessFlatScan中有过多的点云，如果点云太多，计算量太大
            // 进行下采样，可以大大减少计算量
            surfPointsLessFlatScanDS->clear();
            downSizeFilter.setInputCloud(surfPointsLessFlatScan);
            downSizeFilter.filter(*surfPointsLessFlatScanDS);

            *surfPointsLessFlat += *surfPointsLessFlatScanDS;//surfPointsLessFlatScan下采样后加入surfPointsLessFlat中
        }
    }

    // 发布话题
    void publishCloud()
    {
        sensor_msgs::PointCloud2 laserCloudOutMsg;

	    if (pubCornerPointsSharp.getNumSubscribers() != 0){
	        pcl::toROSMsg(*cornerPointsSharp, laserCloudOutMsg);
	        laserCloudOutMsg.header.stamp = cloudHeader.stamp;
	        laserCloudOutMsg.header.frame_id = "/camera";
	        pubCornerPointsSharp.publish(laserCloudOutMsg);
	    }

	    if (pubCornerPointsLessSharp.getNumSubscribers() != 0){
	        pcl::toROSMsg(*cornerPointsLessSharp, laserCloudOutMsg);
	        laserCloudOutMsg.header.stamp = cloudHeader.stamp;
	        laserCloudOutMsg.header.frame_id = "/camera";
	        pubCornerPointsLessSharp.publish(laserCloudOutMsg);
	    }

	    if (pubSurfPointsFlat.getNumSubscribers() != 0){
	        pcl::toROSMsg(*surfPointsFlat, laserCloudOutMsg);
	        laserCloudOutMsg.header.stamp = cloudHeader.stamp;
	        laserCloudOutMsg.header.frame_id = "/camera";
	        pubSurfPointsFlat.publish(laserCloudOutMsg);
	    }

	    if (pubSurfPointsLessFlat.getNumSubscribers() != 0){
	        pcl::toROSMsg(*surfPointsLessFlat, laserCloudOutMsg);
	        laserCloudOutMsg.header.stamp = cloudHeader.stamp;
	        laserCloudOutMsg.header.frame_id = "/camera";
	        pubSurfPointsLessFlat.publish(laserCloudOutMsg);
	    }
    }










































    // 根据初始猜测，将点云进行插值，变换到该帧初始时刻
    void TransformToStart(PointType const * const pi, PointType * const po)
    {
        float s = 10 * (pi->intensity - int(pi->intensity));// 小数部分代表(该点时间-起始扫描时间)相对一圈时间(0.1s)的占比

        float rx = s * transformCur[0];
        float ry = s * transformCur[1];
        float rz = s * transformCur[2];
        float tx = s * transformCur[3];
        float ty = s * transformCur[4];
        float tz = s * transformCur[5];

        // 绕z轴旋转-rz
        float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
        float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
        float z1 = (pi->z - tz);

        // 绕x轴旋转-rx
        float x2 = x1;
        float y2 = cos(rx) * y1 + sin(rx) * z1;
        float z2 = -sin(rx) * y1 + cos(rx) * z1;

        // 绕y轴旋转-ry
        po->x = cos(ry) * x2 - sin(ry) * z2;
        po->y = y2;
        po->z = sin(ry) * x2 + cos(ry) * z2;
        po->intensity = pi->intensity;
    }

    // 先转到start，再从start旋转到end
    void TransformToEnd(PointType const * const pi, PointType * const po)
    {
        // 关于s, 参看上面  TransformToStart() 的注释
        float s = 10 * (pi->intensity - int(pi->intensity));

        float rx = s * transformCur[0];
        float ry = s * transformCur[1];
        float rz = s * transformCur[2];
        float tx = s * transformCur[3];
        float ty = s * transformCur[4];
        float tz = s * transformCur[5];

		//运动补偿，这里用迭代出的位姿线性插值进行补偿，将点全部转换至上一帧
        float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
        float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
        float z1 = (pi->z - tz);

        float x2 = x1;
        float y2 = cos(rx) * y1 + sin(rx) * z1;
        float z2 = -sin(rx) * y1 + cos(rx) * z1;

        float x3 = cos(ry) * x2 - sin(ry) * z2;
        float y3 = y2;
        float z3 = sin(ry) * x2 + cos(ry) * z2;

        rx = transformCur[0];
        ry = transformCur[1];
        rz = transformCur[2];
        tx = transformCur[3];
        ty = transformCur[4];
        tz = transformCur[5];

		//将点转换至当前帧
        float x4 = cos(ry) * x3 + sin(ry) * z3;
        float y4 = y3;
        float z4 = -sin(ry) * x3 + cos(ry) * z3;

        float x5 = x4;
        float y5 = cos(rx) * y4 - sin(rx) * z4;
        float z5 = sin(rx) * y4 + cos(rx) * z4;

        float x6 = cos(rz) * x5 - sin(rz) * y5 + tx;
        float y6 = sin(rz) * x5 + cos(rz) * y5 + ty;
        float z6 = z5 + tz;

		//去除由于加减速造成的畸变，由于畸变是在start参考系下，因此需要变到start坐标系下
        float x7 = cosImuRollStart * (x6 - imuShiftFromStartX) 
                 - sinImuRollStart * (y6 - imuShiftFromStartY);
        float y7 = sinImuRollStart * (x6 - imuShiftFromStartX) 
                 + cosImuRollStart * (y6 - imuShiftFromStartY);
        float z7 = z6 - imuShiftFromStartZ;

        float x8 = x7;
        float y8 = cosImuPitchStart * y7 - sinImuPitchStart * z7;
        float z8 = sinImuPitchStart * y7 + cosImuPitchStart * z7;

        float x9 = cosImuYawStart * x8 + sinImuYawStart * z8;
        float y9 = y8;
        float z9 = -sinImuYawStart * x8 + cosImuYawStart * z8;

		//再变回到end坐标系下
        float x10 = cos(imuYawLast) * x9 - sin(imuYawLast) * z9;
        float y10 = y9;
        float z10 = sin(imuYawLast) * x9 + cos(imuYawLast) * z9;

        float x11 = x10;
        float y11 = cos(imuPitchLast) * y10 + sin(imuPitchLast) * z10;
        float z11 = -sin(imuPitchLast) * y10 + cos(imuPitchLast) * z10;

        po->x = cos(imuRollLast) * x11 + sin(imuRollLast) * y11;
        po->y = -sin(imuRollLast) * x11 + cos(imuRollLast) * y11;
        po->z = z11;
        po->intensity = int(pi->intensity);
    }

    // (rx, ry, rz, imuPitchStart, imuYawStart, imuRollStart, 
    //  imuPitchLast, imuYawLast, imuRollLast, rx, ry, rz)
    // imuPitchStart：当前帧第一个点对应的角
    // imuPitchLast：当前帧最后一个点对应的角
    void PluginIMURotation(float bcx, float bcy, float bcz, float blx, float bly, float blz, 
                           float alx, float aly, float alz, float &acx, float &acy, float &acz)
    {
        /* blx、bly、blz：IMU测得的该次sweep的起始角度组成的位姿   R_w_start
   	 * alx、bly、alz：IMU测得的该次sweep的终止角度组成的位姿   R_w_end
  	 * bcx、bcy、bcz：修正前的纯粹由特征匹配得到的当前sweep相对于初始系的位姿      R_init_end
   	 * acx、acy、acz：修正后的纯粹由特征匹配得到的当前sweep相对于初始系的位姿      R`_init_end
  	 * 变换关系：R`_init_end = R_w_end * R_w_start^{-1} * R_init_end
   	 *                     = R_ZXY * R_ZXY^{-1} * R_ZXY
   	 */
        float sbcx = sin(bcx);
        float cbcx = cos(bcx);
        float sbcy = sin(bcy);
        float cbcy = cos(bcy);
        float sbcz = sin(bcz);
        float cbcz = cos(bcz);

        float sblx = sin(blx);
        float cblx = cos(blx);
        float sbly = sin(bly);
        float cbly = cos(bly);
        float sblz = sin(blz);
        float cblz = cos(blz);

        float salx = sin(alx);
        float calx = cos(alx);
        float saly = sin(aly);
        float caly = cos(aly);
        float salz = sin(alz);
        float calz = cos(alz);

        float srx = -sbcx*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly) 
                  - cbcx*cbcz*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
                  - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
                  - cbcx*sbcz*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
                  - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz);
        acx = -asin(srx);

        float srycrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
                     - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
                     - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
                     - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz) 
                     + cbcx*sbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
        float crycrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
                     - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz) 
                     - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
                     - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
                     + cbcx*cbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
        acy = atan2(srycrx / cos(acx), crycrx / cos(acx));
        
        float srzcrx = sbcx*(cblx*cbly*(calz*saly - caly*salx*salz) 
                     - cblx*sbly*(caly*calz + salx*saly*salz) + calx*salz*sblx) 
                     - cbcx*cbcz*((caly*calz + salx*saly*salz)*(cbly*sblz - cblz*sblx*sbly) 
                     + (calz*saly - caly*salx*salz)*(sbly*sblz + cbly*cblz*sblx) 
                     - calx*cblx*cblz*salz) + cbcx*sbcz*((caly*calz + salx*saly*salz)*(cbly*cblz 
                     + sblx*sbly*sblz) + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) 
                     + calx*cblx*salz*sblz);
        float crzcrx = sbcx*(cblx*sbly*(caly*salz - calz*salx*saly) 
                     - cblx*cbly*(saly*salz + caly*calz*salx) + calx*calz*sblx) 
                     + cbcx*cbcz*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx) 
                     + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) 
                     + calx*calz*cblx*cblz) - cbcx*sbcz*((saly*salz + caly*calz*salx)*(cblz*sbly 
                     - cbly*sblx*sblz) + (caly*salz - calz*salx*saly)*(cbly*cblz + sblx*sbly*sblz) 
                     - calx*calz*cblx*sblz);
        acz = atan2(srzcrx / cos(acx), crzcrx / cos(acx));
    }

    void AccumulateRotation(float cx, float cy, float cz, float lx, float ly, float lz, 
                            float &ox, float &oy, float &oz)
    {

		//下面是别人的注释：
        // 参考：https://www.cnblogs.com/ReedLW/p/9005621.html
        // 0--->(cx,cy,cz)--->(lx,ly,lz)
        // 从0时刻到(cx,cy,cz),然后在(cx,cy,cz)的基础上又旋转(lx,ly,lz)
        // 求最后总的位姿结果是什么？
        // R*p_cur = R_c*R_l*p_cur  ==> R=R_c* R_l
        //
        //     |cly*clz+sly*slx*slz  clz*sly*slx-cly*slz  clx*sly|
        // R_l=|    clx*slz                 clx*clz          -slx|
        //     |cly*slx*slz-clz*sly  cly*clz*slx+sly*slz  cly*clx|
        // R_c=...
        // -srx=(ccx*scy,-scx,cly*clx)*(clx*slz,clx*clz,-slx)
        // ...
        // 然后根据R再来求(ox,oy,oz)

        float srx = cos(lx)*cos(cx)*sin(ly)*sin(cz) - cos(cx)*cos(cz)*sin(lx) - cos(lx)*cos(ly)*sin(cx);
        ox = -asin(srx);

        float srycrx = sin(lx)*(cos(cy)*sin(cz) - cos(cz)*sin(cx)*sin(cy)) + cos(lx)*sin(ly)*(cos(cy)*cos(cz) 
                     + sin(cx)*sin(cy)*sin(cz)) + cos(lx)*cos(ly)*cos(cx)*sin(cy);
        float crycrx = cos(lx)*cos(ly)*cos(cx)*cos(cy) - cos(lx)*sin(ly)*(cos(cz)*sin(cy) 
                     - cos(cy)*sin(cx)*sin(cz)) - sin(lx)*(sin(cy)*sin(cz) + cos(cy)*cos(cz)*sin(cx));
        oy = atan2(srycrx / cos(ox), crycrx / cos(ox));

        float srzcrx = sin(cx)*(cos(lz)*sin(ly) - cos(ly)*sin(lx)*sin(lz)) + cos(cx)*sin(cz)*(cos(ly)*cos(lz) 
                     + sin(lx)*sin(ly)*sin(lz)) + cos(lx)*cos(cx)*cos(cz)*sin(lz);
        float crzcrx = cos(lx)*cos(lz)*cos(cx)*cos(cz) - cos(cx)*sin(cz)*(cos(ly)*sin(lz) 
                     - cos(lz)*sin(lx)*sin(ly)) - sin(cx)*(sin(ly)*sin(lz) + cos(ly)*cos(lz)*sin(lx));
        oz = atan2(srzcrx / cos(ox), crzcrx / cos(ox));
    }

    double rad2deg(double radians)
    {
        return radians * 180.0 / M_PI;
    }

    double deg2rad(double degrees)
    {
        return degrees * M_PI / 180.0;
    }

    void findCorrespondingCornerFeatures(int iterCount){

        int cornerPointsSharpNum = cornerPointsSharp->points.size();

        for (int i = 0; i < cornerPointsSharpNum; i++) {

			//将当前帧点云(当前帧Start系)，通过IMU，转换至上一帧的Start系下，提供位姿先验
            TransformToStart(&cornerPointsSharp->points[i], &pointSel);

            if (iterCount % 5 == 0) {

				//K近邻查找，这里K=1，即查找1个与pointSel最接近的点，点的索引存放在pointSearchInd中，近邻点的中心距的平方存放在pointSearchSqDis中
                kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
                int closestPointInd = -1, minPointInd2 = -1;
                
                if (pointSearchSqDis[0] < nearestFeatureSearchSqDist) {
                    closestPointInd = pointSearchInd[0];
                    int closestPointScan = int(laserCloudCornerLast->points[closestPointInd].intensity);

                    float pointSqDis, minPointSqDis2 = nearestFeatureSearchSqDist;
                    for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++) {
                        if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan + 2.5) {
                            break;
                        }

                        pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) * 
                                     (laserCloudCornerLast->points[j].x - pointSel.x) + 
                                     (laserCloudCornerLast->points[j].y - pointSel.y) * 
                                     (laserCloudCornerLast->points[j].y - pointSel.y) + 
                                     (laserCloudCornerLast->points[j].z - pointSel.z) * 
                                     (laserCloudCornerLast->points[j].z - pointSel.z);

                        if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan) {
                            if (pointSqDis < minPointSqDis2) {
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                        }
                    }
                    for (int j = closestPointInd - 1; j >= 0; j--) {
                        if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan - 2.5) {
                            break;
                        }

                        pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) * 
                                     (laserCloudCornerLast->points[j].x - pointSel.x) + 
                                     (laserCloudCornerLast->points[j].y - pointSel.y) * 
                                     (laserCloudCornerLast->points[j].y - pointSel.y) + 
                                     (laserCloudCornerLast->points[j].z - pointSel.z) * 
                                     (laserCloudCornerLast->points[j].z - pointSel.z);

                        if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan) {
                            if (pointSqDis < minPointSqDis2) {
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                        }
                    }
                }

                pointSearchCornerInd1[i] = closestPointInd;
                pointSearchCornerInd2[i] = minPointInd2;
            }

            if (pointSearchCornerInd2[i] >= 0) {

                tripod1 = laserCloudCornerLast->points[pointSearchCornerInd1[i]];
                tripod2 = laserCloudCornerLast->points[pointSearchCornerInd2[i]];

                float x0 = pointSel.x;
                float y0 = pointSel.y;
                float z0 = pointSel.z;
                float x1 = tripod1.x;
                float y1 = tripod1.y;
                float z1 = tripod1.z;
                float x2 = tripod2.x;
                float y2 = tripod2.y;
                float z2 = tripod2.z;

                float m11 = ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1));
                float m22 = ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1));
                float m33 = ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1));

                float a012 = sqrt(m11 * m11  + m22 * m22 + m33 * m33);

                float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                float la =  ((y1 - y2)*m11 + (z1 - z2)*m22) / a012 / l12;

                float lb = -((x1 - x2)*m11 - (z1 - z2)*m33) / a012 / l12;

                float lc = -((x1 - x2)*m22 + (y1 - y2)*m33) / a012 / l12;

                float ld2 = a012 / l12;

                float s = 1;
                if (iterCount >= 5) {
                    s = 1 - 1.8 * fabs(ld2);
                }

                if (s > 0.1 && ld2 != 0) {
                    coeff.x = s * la; 
                    coeff.y = s * lb;
                    coeff.z = s * lc;
                    coeff.intensity = s * ld2;
                  
                    laserCloudOri->push_back(cornerPointsSharp->points[i]);
                    coeffSel->push_back(coeff);
                }
            }
        }
    }

    void findCorrespondingSurfFeatures(int iterCount){

		//当前帧平面点数量
        int surfPointsFlatNum = surfPointsFlat->points.size();

        for (int i = 0; i < surfPointsFlatNum; i++) {
            // 坐标变换到开始时刻，参数0是输入，参数1是输出
			//将当前帧点云(当前帧Start系)，通过IMU，转换至上一帧的Start系下，提供位姿先验
            TransformToStart(&surfPointsFlat->points[i], &pointSel);

			//每5次搜索并更新1次最近邻
            if (iterCount % 5 == 0) {

                // k点最近邻搜索，这里k=1
				//K近邻查找，这里K=1，即查找1个与pointSel最接近的点，点的索引存放在pointSearchInd中，近邻点的中心距的平方存放在pointSearchSqDis中
                kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
                int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;

                // sq:平方，距离的平方值
                // 如果nearestKSearch找到的1(k=1)个邻近点满足条件
                if (pointSearchSqDis[0] < nearestFeatureSearchSqDist) {//近邻点距离小于设定阈值(5m)
                    closestPointInd = pointSearchInd[0];//最近邻点索引
					
                    int closestPointScan = int(laserCloudSurfLast->points[closestPointInd].intensity);  // 扫描线数scanID

                    // 主要功能是找到2个scan之内的最近点，并将找到的最近点及其序号保存
                    // 之前扫描的保存到minPointSqDis2，之后的保存到minPointSqDis2
                    float pointSqDis, minPointSqDis2 = nearestFeatureSearchSqDist, minPointSqDis3 = nearestFeatureSearchSqDist;
					// 向上查找
                    for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++) {

                        if (int(laserCloudSurfLast->points[j].intensity) > closestPointScan + 2.5) {
                            break;
                        }

                        pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) * 
                                     (laserCloudSurfLast->points[j].x - pointSel.x) + 
                                     (laserCloudSurfLast->points[j].y - pointSel.y) * 
                                     (laserCloudSurfLast->points[j].y - pointSel.y) + 
                                     (laserCloudSurfLast->points[j].z - pointSel.z) * 
                                     (laserCloudSurfLast->points[j].z - pointSel.z);

                        if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScan) {
                            if (pointSqDis < minPointSqDis2) {
                              minPointSqDis2 = pointSqDis;
                              minPointInd2 = j;
                            }
                        } else {
                            if (pointSqDis < minPointSqDis3) {
                                minPointSqDis3 = pointSqDis;
                                minPointInd3 = j;
                            }
                        }
                    }

                    // 向下查找
                    for (int j = closestPointInd - 1; j >= 0; j--) {
                        if (int(laserCloudSurfLast->points[j].intensity) < closestPointScan - 2.5) {
                            break;
                        }

                        pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) * 
                                     (laserCloudSurfLast->points[j].x - pointSel.x) + 
                                     (laserCloudSurfLast->points[j].y - pointSel.y) * 
                                     (laserCloudSurfLast->points[j].y - pointSel.y) + 
                                     (laserCloudSurfLast->points[j].z - pointSel.z) * 
                                     (laserCloudSurfLast->points[j].z - pointSel.z);

                        if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScan) {
                            if (pointSqDis < minPointSqDis2) {
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                        } else {
                            if (pointSqDis < minPointSqDis3) {
                                minPointSqDis3 = pointSqDis;
                                minPointInd3 = j;
                            }
                        }
                    }
                }

                pointSearchSurfInd1[i] = closestPointInd;
                pointSearchSurfInd2[i] = minPointInd2;
                pointSearchSurfInd3[i] = minPointInd3;
            }

            // 前后都能找到对应的最近点在给定范围之内
            // 那么就开始计算距离
            // [pa,pb,pc]是tripod1，tripod2，tripod3这3个点构成的一个平面的方向量，
            // ps是模长，它是三角形面积的2倍
            if (pointSearchSurfInd2[i] >= 0 && pointSearchSurfInd3[i] >= 0) {

                tripod1 = laserCloudSurfLast->points[pointSearchSurfInd1[i]];
                tripod2 = laserCloudSurfLast->points[pointSearchSurfInd2[i]];
                tripod3 = laserCloudSurfLast->points[pointSearchSurfInd3[i]];

				//向量叉乘求面积
				//| i       j       k    |
				//| x2-x1   y2-y1   z2-z1|
				//| x3-x1   y3-y1   z3-z1|
				//得到面积向量[pa,pb,pc]
                float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z) 
                         - (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
                float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x) 
                         - (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
                float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y) 
                         - (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
                float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

				//ps为面积向量[pa,pb,pc]的模，等于三角形面积的2倍
                float ps = sqrt(pa * pa + pb * pb + pc * pc);

				//将[pa,pb,pc]单位化
                pa /= ps;
                pb /= ps;
                pc /= ps;
				//pd = ([pa,pb,pc].*[x1,y1,z1])/ps;
				//即pd为[x1,y1,z1]在面元法向的投影
                pd /= ps;

                // 距离没有取绝对值，因为距离的正负会和后面雅可比矩阵的正负抵消
                // 两个向量的点乘，分母除以ps中已经除掉了，
                // 加pd原因:pointSel与tripod1构成的线段需要相减
				//上面已经把[pa,pb,pc]单位化了
				//[pointSel.x,pointSel.y,pointSel.z]在面元法向的投影与[x1,y1,z1]在面元法向的投影之差，即为四面体的高，即点到平面的距离
                float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                float s = 1;
                if (iterCount >= 5) {
                    // 权重
					//迭代次数大于等于5次后，如果点到平面的距离越大，则权重越小，如果点到扫描中心距离越小，权重越小
                    s = 1 - 1.8 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
                            + pointSel.y * pointSel.y + pointSel.z * pointSel.z));
                }

                if (s > 0.1 && pd2 != 0) {
                    // [x,y,z]是整个平面的单位法量
                    // intensity是平面外一点到该平面的距离
                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;

                    // 未经变换的点放入laserCloudOri队列，距离，法向量值放入coeffSel
                    laserCloudOri->push_back(surfPointsFlat->points[i]);
                    coeffSel->push_back(coeff);
                }
            }
        }
    }

    bool calculateTransformationSurf(int iterCount){

        int pointSelNum = laserCloudOri->points.size();

		//cv::Mat构造方法：
		//cv::Mat(行数,列数,数据类型,初始化数值);
		//https://blog.csdn.net/u012058778/article/details/90764430
        cv::Mat matA(pointSelNum, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(3, pointSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(3, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(3, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(3, 1, CV_32F, cv::Scalar::all(0));

        float srx = sin(transformCur[0]);
        float crx = cos(transformCur[0]);
        float sry = sin(transformCur[1]);
        float cry = cos(transformCur[1]);
        float srz = sin(transformCur[2]);
        float crz = cos(transformCur[2]);
        float tx = transformCur[3];
        float ty = transformCur[4];
        float tz = transformCur[5];

        float a1 = crx*sry*srz; float a2 = crx*crz*sry; float a3 = srx*sry; float a4 = tx*a1 - ty*a2 - tz*a3;
        float a5 = srx*srz; float a6 = crz*srx; float a7 = ty*a6 - tz*crx - tx*a5;
        float a8 = crx*cry*srz; float a9 = crx*cry*crz; float a10 = cry*srx; float a11 = tz*a10 + ty*a9 - tx*a8;

        float b1 = -crz*sry - cry*srx*srz; float b2 = cry*crz*srx - sry*srz;
        float b5 = cry*crz - srx*sry*srz; float b6 = cry*srz + crz*srx*sry;

        float c1 = -b6; float c2 = b5; float c3 = tx*b6 - ty*b5; float c4 = -crx*crz; float c5 = crx*srz; float c6 = ty*c5 + tx*-c4;
        float c7 = b2; float c8 = -b1; float c9 = tx*-b2 - ty*-b1;

        // 构建雅可比矩阵，求解
        for (int i = 0; i < pointSelNum; i++) {

            pointOri = laserCloudOri->points[i];
            coeff = coeffSel->points[i];

            float arx = (-a1*pointOri.x + a2*pointOri.y + a3*pointOri.z + a4) * coeff.x
                      + (a5*pointOri.x - a6*pointOri.y + crx*pointOri.z + a7) * coeff.y
                      + (a8*pointOri.x - a9*pointOri.y - a10*pointOri.z + a11) * coeff.z;

            float arz = (c1*pointOri.x + c2*pointOri.y + c3) * coeff.x
                      + (c4*pointOri.x - c5*pointOri.y + c6) * coeff.y
                      + (c7*pointOri.x + c8*pointOri.y + c9) * coeff.z;

            float aty = -b6 * coeff.x + c4 * coeff.y + b2 * coeff.z;

            float d2 = coeff.intensity;

            matA.at<float>(i, 0) = arx;
            matA.at<float>(i, 1) = arz;
            matA.at<float>(i, 2) = aty;
			//这里乘0.05的系数是为什么？
            matB.at<float>(i, 0) = -0.05 * d2;
        }

		//转置
        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
		// 利用高斯牛顿法进行求解，
		// 高斯牛顿法的原型是J^(T)*J * delta(x) = -J*f(x)
		// J是雅克比矩阵，这里是A，f(x)是优化目标，这里是-B(符号在给B赋值时候就放进去了)
		// 通过QR分解的方式，求解matAtA*matX=matAtB，得到解matX
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {
            cv::Mat matE(1, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(3, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(3, 3, CV_32F, cv::Scalar::all(0));

			//求matAtA的特征值和特征向量
			//matE是一个一维特征值组成的列矢量，并且特征值是按照降序排序
			//matV是特征向量组成的矩阵，矩阵中的每一行就是一个特征向量
            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

			//是否退化标志
			//具体描述见Loam作者Zhang J的<<On Degeneracy of Optimization-based State Estimation Problems>>
			//大概方法是通过Jacobian的eigenvalue判断哪个分量的约束不足, 不更新那个方向上的迭代
            isDegenerate = false;
            float eignThre[3] = {10, 10, 10};
            for (int i = 2; i >= 0; i--) {
				//这里特征值已经按降序排列了，如果某个特征值小于阈值，则后面的肯定都小于阈值，因此直接break，不用再做判断了
				//如果matAtA的某一特征值小于10
                if (matE.at<float>(0, i) < eignThre[i]) {
					//将对应的特征向量设为0
                    for (int j = 0; j < 3; j++) {
                        matV2.at<float>(i, j) = 0;
                    }
					//标记为退化
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }

        if (isDegenerate) {
            cv::Mat matX2(3, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
			//这里的变换没看懂？
            matX = matP * matX2;
        }

        transformCur[0] += matX.at<float>(0, 0);//rx
        transformCur[2] += matX.at<float>(1, 0);//rz
        transformCur[4] += matX.at<float>(2, 0);//ty

        for(int i=0; i<6; i++){
			//判断是不是NAN值(not a number非法数字)
            if(isnan(transformCur[i]))
                transformCur[i]=0;
        }

		//判断是否满足迭代结束条件
        float deltaR = sqrt(
                            pow(rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(rad2deg(matX.at<float>(1, 0)), 2));
        float deltaT = sqrt(
                            pow(matX.at<float>(2, 0) * 100, 2));

        if (deltaR < 0.1 && deltaT < 0.1) {
            return false;
        }
        return true;
    }

    bool calculateTransformationCorner(int iterCount){

        int pointSelNum = laserCloudOri->points.size();

        cv::Mat matA(pointSelNum, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(3, pointSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(3, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(3, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(3, 1, CV_32F, cv::Scalar::all(0));

        // 以下为开始计算A,A=[J的偏导],J的偏导的计算公式是什么?
        float srx = sin(transformCur[0]);
        float crx = cos(transformCur[0]);
        float sry = sin(transformCur[1]);
        float cry = cos(transformCur[1]);
        float srz = sin(transformCur[2]);
        float crz = cos(transformCur[2]);
        float tx = transformCur[3];
        float ty = transformCur[4];
        float tz = transformCur[5];

        float b1 = -crz*sry - cry*srx*srz; float b2 = cry*crz*srx - sry*srz; float b3 = crx*cry; float b4 = tx*-b1 + ty*-b2 + tz*b3;
        float b5 = cry*crz - srx*sry*srz; float b6 = cry*srz + crz*srx*sry; float b7 = crx*sry; float b8 = tz*b7 - ty*b6 - tx*b5;

        float c5 = crx*srz;

        for (int i = 0; i < pointSelNum; i++) {

            pointOri = laserCloudOri->points[i];
            coeff = coeffSel->points[i];

            float ary = (b1*pointOri.x + b2*pointOri.y - b3*pointOri.z + b4) * coeff.x
                      + (b5*pointOri.x + b6*pointOri.y - b7*pointOri.z + b8) * coeff.z;

            float atx = -b5 * coeff.x + c5 * coeff.y + b1 * coeff.z;

            float atz = b7 * coeff.x - srx * coeff.y - b3 * coeff.z;

            float d2 = coeff.intensity;


            // A=[J的偏导]; B=[权重系数*(点到直线的距离)] 求解公式: AX=B
            // 为了让左边满秩，同乘At-> At*A*X = At*B
            matA.at<float>(i, 0) = ary;
            matA.at<float>(i, 1) = atx;
            matA.at<float>(i, 2) = atz;
            matB.at<float>(i, 0) = -0.05 * d2;
        }

        // transpose函数求得matA的转置matAt
        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        // 通过QR分解的方法，求解方程AtA*X=AtB，得到X
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {
            cv::Mat matE(1, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(3, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(3, 3, CV_32F, cv::Scalar::all(0));

            // 计算At*A的特征值和特征向量
            // 特征值存放在matE，特征向量matV
            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            // 退化的具体表现是指什么？
            isDegenerate = false;
            float eignThre[3] = {10, 10, 10};
            for (int i = 2; i >= 0; i--) {
                if (matE.at<float>(0, i) < eignThre[i]) {
                    for (int j = 0; j < 3; j++) {
                        matV2.at<float>(i, j) = 0;
                    }
                    // 存在比10小的特征值则出现退化
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }

        if (isDegenerate) {
            cv::Mat matX2(3, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        transformCur[1] += matX.at<float>(0, 0);
        transformCur[3] += matX.at<float>(1, 0);
        transformCur[5] += matX.at<float>(2, 0);

        for(int i=0; i<6; i++){
            if(isnan(transformCur[i]))
                transformCur[i]=0;
        }

        float deltaR = sqrt(
                            pow(rad2deg(matX.at<float>(0, 0)), 2));
        float deltaT = sqrt(
                            pow(matX.at<float>(1, 0) * 100, 2) +
                            pow(matX.at<float>(2, 0) * 100, 2));

        if (deltaR < 0.1 && deltaT < 0.1) {
            return false;
        }
        return true;
    }

    bool calculateTransformation(int iterCount){

        int pointSelNum = laserCloudOri->points.size();

        cv::Mat matA(pointSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, pointSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

        float srx = sin(transformCur[0]);
        float crx = cos(transformCur[0]);
        float sry = sin(transformCur[1]);
        float cry = cos(transformCur[1]);
        float srz = sin(transformCur[2]);
        float crz = cos(transformCur[2]);
        float tx = transformCur[3];
        float ty = transformCur[4];
        float tz = transformCur[5];

        float a1 = crx*sry*srz; float a2 = crx*crz*sry; float a3 = srx*sry; float a4 = tx*a1 - ty*a2 - tz*a3;
        float a5 = srx*srz; float a6 = crz*srx; float a7 = ty*a6 - tz*crx - tx*a5;
        float a8 = crx*cry*srz; float a9 = crx*cry*crz; float a10 = cry*srx; float a11 = tz*a10 + ty*a9 - tx*a8;

        float b1 = -crz*sry - cry*srx*srz; float b2 = cry*crz*srx - sry*srz; float b3 = crx*cry; float b4 = tx*-b1 + ty*-b2 + tz*b3;
        float b5 = cry*crz - srx*sry*srz; float b6 = cry*srz + crz*srx*sry; float b7 = crx*sry; float b8 = tz*b7 - ty*b6 - tx*b5;

        float c1 = -b6; float c2 = b5; float c3 = tx*b6 - ty*b5; float c4 = -crx*crz; float c5 = crx*srz; float c6 = ty*c5 + tx*-c4;
        float c7 = b2; float c8 = -b1; float c9 = tx*-b2 - ty*-b1;

        for (int i = 0; i < pointSelNum; i++) {

            pointOri = laserCloudOri->points[i];
            coeff = coeffSel->points[i];

            float arx = (-a1*pointOri.x + a2*pointOri.y + a3*pointOri.z + a4) * coeff.x
                      + (a5*pointOri.x - a6*pointOri.y + crx*pointOri.z + a7) * coeff.y
                      + (a8*pointOri.x - a9*pointOri.y - a10*pointOri.z + a11) * coeff.z;

            float ary = (b1*pointOri.x + b2*pointOri.y - b3*pointOri.z + b4) * coeff.x
                      + (b5*pointOri.x + b6*pointOri.y - b7*pointOri.z + b8) * coeff.z;

            float arz = (c1*pointOri.x + c2*pointOri.y + c3) * coeff.x
                      + (c4*pointOri.x - c5*pointOri.y + c6) * coeff.y
                      + (c7*pointOri.x + c8*pointOri.y + c9) * coeff.z;

            float atx = -b5 * coeff.x + c5 * coeff.y + b1 * coeff.z;

            float aty = -b6 * coeff.x + c4 * coeff.y + b2 * coeff.z;

            float atz = b7 * coeff.x - srx * coeff.y - b3 * coeff.z;

            float d2 = coeff.intensity;

            matA.at<float>(i, 0) = arx;
            matA.at<float>(i, 1) = ary;
            matA.at<float>(i, 2) = arz;
            matA.at<float>(i, 3) = atx;
            matA.at<float>(i, 4) = aty;
            matA.at<float>(i, 5) = atz;
            matB.at<float>(i, 0) = -0.05 * d2;
        }

        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {
            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {10, 10, 10, 10, 10, 10};
            for (int i = 5; i >= 0; i--) {
                if (matE.at<float>(0, i) < eignThre[i]) {
                    for (int j = 0; j < 6; j++) {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }

        if (isDegenerate) {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        transformCur[0] += matX.at<float>(0, 0);
        transformCur[1] += matX.at<float>(1, 0);
        transformCur[2] += matX.at<float>(2, 0);
        transformCur[3] += matX.at<float>(3, 0);
        transformCur[4] += matX.at<float>(4, 0);
        transformCur[5] += matX.at<float>(5, 0);

        for(int i=0; i<6; i++){
            if(isnan(transformCur[i]))
                transformCur[i]=0;
        }

        // 计算旋转的模长
        float deltaR = sqrt(
                            pow(rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(rad2deg(matX.at<float>(1, 0)), 2) +
                            pow(rad2deg(matX.at<float>(2, 0)), 2));
        // 计算平移的模长
        float deltaT = sqrt(
                            pow(matX.at<float>(3, 0) * 100, 2) +
                            pow(matX.at<float>(4, 0) * 100, 2) +
                            pow(matX.at<float>(5, 0) * 100, 2));

        if (deltaR < 0.1 && deltaT < 0.1) {
            return false;
        }
        return true;
    }

	//初始化，即如果之前没有点云，则调用此函数，将本时刻提取的特征点存入laserCloudCornerLast和laserCloudSurfLast中给下一时刻用
    void checkSystemInitialization(){
        // 交换cornerPointsLessSharp和laserCloudCornerLast
		// 将上一时刻的次角点存入cornerPointsLessSharp中，将本时刻提取的次角点存入laserCloudCornerLast中待用
        pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
        cornerPointsLessSharp = laserCloudCornerLast;
        laserCloudCornerLast = laserCloudTemp;

        // 交换surfPointsLessFlat和laserCloudSurfLast
		//将上一时刻的次平面点存入surfPointsLessFlat中，将本时刻提取的次平面点存入laserCloudSurfLast中待用
        laserCloudTemp = surfPointsLessFlat;
        surfPointsLessFlat = laserCloudSurfLast;
        laserCloudSurfLast = laserCloudTemp;

		//构建kd树，用于最近邻搜索
        kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
        kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

        laserCloudCornerLastNum = laserCloudCornerLast->points.size();
        laserCloudSurfLastNum = laserCloudSurfLast->points.size();

        sensor_msgs::PointCloud2 laserCloudCornerLast2;
        pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
        laserCloudCornerLast2.header.stamp = cloudHeader.stamp;
        laserCloudCornerLast2.header.frame_id = "/camera";
        pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

        sensor_msgs::PointCloud2 laserCloudSurfLast2;
        pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
        laserCloudSurfLast2.header.stamp = cloudHeader.stamp;
        laserCloudSurfLast2.header.frame_id = "/camera";
        pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

        transformSum[0] += imuPitchStart;
        transformSum[2] += imuRollStart;

        systemInitedLM = true;
    }

    // 更新初始猜测
    void updateInitialGuess(){

        imuPitchLast = imuPitchCur;
        imuYawLast = imuYawCur;
        imuRollLast = imuRollCur;

        imuShiftFromStartX = imuShiftFromStartXCur;
        imuShiftFromStartY = imuShiftFromStartYCur;
        imuShiftFromStartZ = imuShiftFromStartZCur;

        imuVeloFromStartX = imuVeloFromStartXCur;
        imuVeloFromStartY = imuVeloFromStartYCur;
        imuVeloFromStartZ = imuVeloFromStartZCur;

        // imuAngularFromStartY是两帧之间相对角度差
        // imuVeloFromStartX是两帧之间相对速度差
        
        // 关于下面负号的说明：
        // transformCur是在Cur坐标系下的 p_start=R*p_cur+t
        // R和t是在Cur坐标系下的
        // 而imuAngularFromStart是在start坐标系下的，所以需要加负号
        
        // R_start_end
        if (imuAngularFromStartX != 0 || imuAngularFromStartY != 0 || imuAngularFromStartZ != 0){
            transformCur[0] = - imuAngularFromStartY;
            transformCur[1] = - imuAngularFromStartZ;
            transformCur[2] = - imuAngularFromStartX;
        }

        // imuVeloFromStartX * scanPeriod = t_start_curr
        // transformCur = t_curr_end
        // 速度乘以时间，当前变换中的位移
        if (imuVeloFromStartX != 0 || imuVeloFromStartY != 0 || imuVeloFromStartZ != 0){
            transformCur[3] -= imuVeloFromStartX * scanPeriod;
            transformCur[4] -= imuVeloFromStartY * scanPeriod;
            transformCur[5] -= imuVeloFromStartZ * scanPeriod;
        }
    }

    void updateTransformation(){

		//如果上一时刻的次角点或次平面点数量太少，则认为不可靠，不做位姿解算，直接舍弃
        if (laserCloudCornerLastNum < 10 || laserCloudSurfLastNum < 100)
            return;

        // 两步LM，第一步：优化tz、roll、pitch
        for (int iterCount1 = 0; iterCount1 < 25; iterCount1++) {
            laserCloudOri->clear();
            coeffSel->clear();

            // 找到对应的特征平面
            // 系数保存在coeffSel队列中，匹配成功的点保存在laserCloudOri中
            findCorrespondingSurfFeatures(iterCount1);

			//如果有效的(点-平面)过少，则数据无效，不做匹配
            if (laserCloudOri->points.size() < 10)
                continue;
            // 通过面特征的匹配，计算变换矩阵
            if (calculateTransformationSurf(iterCount1) == false)
                break;
        }

        // 两步LM，第二步：优化tx、ty、yaw
        for (int iterCount2 = 0; iterCount2 < 25; iterCount2++) {

            laserCloudOri->clear();
            coeffSel->clear();

            // 找到对应的特征边/角点
            // 寻找边特征的方法和寻找平面特征的很类似，过程可以参照寻找平面特征的注释
            findCorrespondingCornerFeatures(iterCount2);

            if (laserCloudOri->points.size() < 10)
                continue;
            // 通过角/边特征的匹配，计算变换矩阵
            if (calculateTransformationCorner(iterCount2) == false)
                break;
        }
    }

	// 旋转角的累计变化量
    void integrateTransformation(){
        float rx, ry, rz, tx, ty, tz; 

        // AccumulateRotation作用
        // 将计算的两帧之间的位姿“累加”起来，获得相对于第一帧的旋转矩阵
        // transformSum + (-transformCur) =(rx,ry,rz)
        AccumulateRotation(transformSum[0], transformSum[1], transformSum[2], 
                           -transformCur[0], -transformCur[1], -transformCur[2], rx, ry, rz);

        // 进行平移分量的更新
        float x1 = cos(rz) * (transformCur[3] - imuShiftFromStartX)     // imuShiftFromStartX这个应该是去畸变的，但是lego-loam没用到
                 - sin(rz) * (transformCur[4] - imuShiftFromStartY);
        float y1 = sin(rz) * (transformCur[3] - imuShiftFromStartX) 
                 + cos(rz) * (transformCur[4] - imuShiftFromStartY);
        float z1 = transformCur[5] - imuShiftFromStartZ;

        float x2 = x1;
        float y2 = cos(rx) * y1 - sin(rx) * z1;
        float z2 = sin(rx) * y1 + cos(rx) * z1;

        tx = transformSum[3] - (cos(ry) * x2 + sin(ry) * z2);
        ty = transformSum[4] - y2;
        tz = transformSum[5] - (-sin(ry) * x2 + cos(ry) * z2);

        // 与accumulateRotatio联合起来更新transformSum的rotation部分的工作
        // 可视为transformToEnd的下部分的逆过程
		//这一部分的变换没有看懂？猜测可能是上一帧的LessFlat和LessSharp点通过IMU预测转换至了上一帧的Last系，
		//这里要抵消IMU预测的部分，使rx, ry, rz表示为当前帧点云Start系相对于上一帧点云Start系的位姿
		//那么这样计算的tx，ty，tz不就有误差了吗？不应该先处理rx, ry, rz，然后再用处理好的rx, ry, rz去计算
		//tx，ty，tz吗？（这里先计算了tx，ty，tz，然后再计算的rx, ry, rz）
        PluginIMURotation(rx, ry, rz, imuPitchStart, imuYawStart, imuRollStart, 
                          imuPitchLast, imuYawLast, imuRollLast, rx, ry, rz);

        transformSum[0] = rx;
        transformSum[1] = ry;
        transformSum[2] = rz;
        transformSum[3] = tx;
        transformSum[4] = ty;
        transformSum[5] = tz;
    }

    void publishOdometry(){
        geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformSum[2], -transformSum[0], -transformSum[1]);

        // rx,ry,rz转化为四元数发布
		//对比mapOptmization.cpp和transformFusion.cpp中的laserOdometryHandler中的主体接收，
		//geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
		//tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);
		//transformSum[0] = -pitch;
		//transformSum[1] = -yaw;
		//transformSum[2] = roll;
		//可以发现只是在发布前交换了变量顺序，发布后又交换回来了，因此坐标系没变。
		//但不知道这么做的意义是什么？
        laserOdometry.header.stamp = cloudHeader.stamp;
        laserOdometry.pose.pose.orientation.x = -geoQuat.y;
        laserOdometry.pose.pose.orientation.y = -geoQuat.z;
        laserOdometry.pose.pose.orientation.z = geoQuat.x;
        laserOdometry.pose.pose.orientation.w = geoQuat.w;
        laserOdometry.pose.pose.position.x = transformSum[3];
        laserOdometry.pose.pose.position.y = transformSum[4];
        laserOdometry.pose.pose.position.z = transformSum[5];
        pubLaserOdometry.publish(laserOdometry);

        // laserOdometryTrans 是用于tf广播
        laserOdometryTrans.stamp_ = cloudHeader.stamp;
        laserOdometryTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
        laserOdometryTrans.setOrigin(tf::Vector3(transformSum[3], transformSum[4], transformSum[5]));
        tfBroadcaster.sendTransform(laserOdometryTrans);
    }

	//改变OutlierCloud的坐标系，用以发布
    void adjustOutlierCloud(){
        PointType point;
        int cloudSize = outlierCloud->points.size();
        for (int i = 0; i < cloudSize; ++i)
        {
            point.x = outlierCloud->points[i].y;
            point.y = outlierCloud->points[i].z;
            point.z = outlierCloud->points[i].x;
            point.intensity = outlierCloud->points[i].intensity;
            outlierCloud->points[i] = point;
        }
    }

    void publishCloudsLast(){

        updateImuRollPitchYawStartSinCos();

        int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
        for (int i = 0; i < cornerPointsLessSharpNum; i++) {
            // TransformToEnd的作用是将当前时刻的less特征点（Start系）（通过IMU预测）转移至当前时刻的sweep的结束位置处（Last系）的雷达坐标系下
            TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
        }


        int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
        for (int i = 0; i < surfPointsLessFlatNum; i++) {
            TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
        }

        pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
        cornerPointsLessSharp = laserCloudCornerLast;
        laserCloudCornerLast = laserCloudTemp;

        laserCloudTemp = surfPointsLessFlat;
        surfPointsLessFlat = laserCloudSurfLast;
        laserCloudSurfLast = laserCloudTemp;

        laserCloudCornerLastNum = laserCloudCornerLast->points.size();
        laserCloudSurfLastNum = laserCloudSurfLast->points.size();

		//构建kd树
        if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100) {
            kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
            kdtreeSurfLast->setInputCloud(laserCloudSurfLast);
        }

        frameCount++;

		//skipFrameNum被赋值为1，所以这里等价于frameCount >= 2，所以相当于每2帧发布一帧特征点用于构图
		//skipFrameNum是控制特征点发布频率的
        if (frameCount >= skipFrameNum + 1) {

            frameCount = 0;

            // 调整坐标系，x=y,y=z,z=x
            adjustOutlierCloud();
            sensor_msgs::PointCloud2 outlierCloudLast2;
            pcl::toROSMsg(*outlierCloud, outlierCloudLast2);
            outlierCloudLast2.header.stamp = cloudHeader.stamp;
            outlierCloudLast2.header.frame_id = "/camera";
            pubOutlierCloudLast.publish(outlierCloudLast2);

			//这里发布的是通过IMU预测转移到当前帧Last系下的LessSharp和LessFlat点
            sensor_msgs::PointCloud2 laserCloudCornerLast2;
            pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
            laserCloudCornerLast2.header.stamp = cloudHeader.stamp;
            laserCloudCornerLast2.header.frame_id = "/camera";
            pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

            sensor_msgs::PointCloud2 laserCloudSurfLast2;
            pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
            laserCloudSurfLast2.header.stamp = cloudHeader.stamp;
            laserCloudSurfLast2.header.frame_id = "/camera";
            pubLaserCloudSurfLast.publish(laserCloudSurfLast2);
        }
    }

    void runFeatureAssociation()
    {
        // 如果有新数据进来则执行，否则不执行任何操作
        if (newSegmentedCloud && newSegmentedCloudInfo && newOutlierCloud &&
            std::abs(timeNewSegmentedCloudInfo - timeNewSegmentedCloud) < 0.05 &&
            std::abs(timeNewOutlierCloud - timeNewSegmentedCloud) < 0.05){

            newSegmentedCloud = false;
            newSegmentedCloudInfo = false;
            newOutlierCloud = false;
        }else{
            return;
        }

        // 进行插值，并将点云变换到当前帧的IMU初始时刻
        adjustDistortion();

        // 计算平滑度
        calculateSmoothness();

        // 剔除一些互相遮挡，或者所在平面与激光束近似平行的点
        markOccludedPoints();

        // 特征提取
        extractFeatures();

        // 发布cornerPointsSharp等4种类型的点云话题
        publishCloud();

		//如果没有初始化，则初始化存入第一帧点云
        if (!systemInitedLM) {
            checkSystemInitialization();
            return;
        }

        // 更新初始猜测
        updateInitialGuess();

        // 更新变换
        updateTransformation();

        // 积分总变换
        integrateTransformation();

        publishOdometry();

        publishCloudsLast();   
    }
};




int main(int argc, char** argv)
{
    ros::init(argc, argv, "lego_loam");

    ROS_INFO("\033[1;32m---->\033[0m Feature Association Started.");

    FeatureAssociation FA;

    ros::Rate rate(200);
    while (ros::ok())
    {
        ros::spinOnce();

        FA.runFeatureAssociation();

        rate.sleep();
    }
    
    ros::spin();
    return 0;
}
