#include <fstream>
#include <math.h>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <optional>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"

#include "scancontext/Scancontext.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "scancontext/terminator.h"

using namespace gtsam;

using std::cout;
using std::endl;

double keyframeMeterGap;
double keyframeDegGap, keyframeRadGap;
double translationAccumulated = 1000000.0; // large value means must add the first given frame.
double rotaionAccumulated = 1000000.0; // large value means must add the first given frame.

bool isNowKeyFrame = false; 

Pose6D odom_pose_prev {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init 
Pose6D odom_pose_curr {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init pose is zero 

// @{ protected by mBuf 
std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf; // world_T_body(imu)
std::queue<sensor_msgs::PointCloud2ConstPtr> fullResBuf; // laser scan in body frame.
std::queue<sensor_msgs::NavSatFix::ConstPtr> gpsBuf;
std::queue<std::pair<int, int> > scLoopICPBuf;
std::mutex mBuf;
// @} protected by mBuf


ros::Time timeLaserOdometry(0, 0);
ros::Time timeLaser(0, 0);

Terminator terminator(300, 10.0);
std::atomic<bool> shutdown;

pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudMapAfterPGO(new pcl::PointCloud<PointType>());

// @{ protected by mKF
std::mutex mKF;
std::vector<pcl::PointCloud<PointType>::Ptr> keyframeLaserClouds; 
std::vector<Pose6D> keyframePoses;
std::vector<Pose6D> keyframePosesUpdated;
std::vector<ros::Time> keyframeTimes;
// @} protected by mKF


// for loop closure detection
std::atomic<bool> hasNewScanForLC(false);

// @{
std::mutex loopPairMutex;
std::unordered_map<int, int> loopIndexContainer;  // current keyframe index, previous keyframe index
// @}

pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeHistoryKeyPoses(new pcl::KdTreeFLANN<pcl::PointXYZ>());
ros::Publisher pubLoopConstraintEdge;

std::atomic<bool> gtSAMgraphMade(false);
// @{
std::mutex mtxPosegraph;
gtsam::NonlinearFactorGraph gtSAMgraph;
gtsam::Values initialEstimate;
std::vector<ros::Time> newStateTimes;
// @}
std::vector<ros::Time> isamStateTimes; // used by the single thread process_isam
gtsam::ISAM2 *isam; // used by the single thread process_isam
gtsam::Values isamCurrentEstimate; // used by the single thread process_isam

noiseModel::Diagonal::shared_ptr priorNoise;
noiseModel::Diagonal::shared_ptr odomNoise;
noiseModel::Base::shared_ptr robustLoopNoise;
noiseModel::Base::shared_ptr robustGPSNoise;

pcl::VoxelGrid<PointType> downSizeFilterScancontext;
// @{
SCManager scManager;
std::mutex scMutex;
// @}
double scDistThres, scMaximumRadius;

pcl::VoxelGrid<PointType> downSizeFilterICP;

pcl::PointCloud<PointType>::Ptr laserCloudMapPGO(new pcl::PointCloud<PointType>());
pcl::VoxelGrid<PointType> downSizeFilterMapPGO;
bool laserCloudMapPGORedraw = false;

bool useGPS = true;
sensor_msgs::NavSatFix::ConstPtr currGPS;
bool hasGPSforThisKF = false; // used only by process_pg thread
bool gpsOffsetInitialized = false; 
double gpsAltitudeInitOffset = 0.0;

// @{
std::mutex mtxRecentPose;
double recentOptimizedX = 0.0;
double recentOptimizedY = 0.0;
// @}
std::atomic<int> recentIdxUpdated(0);

ros::Publisher pubMapAftPGO, pubOdomAftPGO, pubPathAftPGO;
ros::Publisher pubLoopScanLocal, pubLoopSubmapLocal;
ros::Publisher pubOdomRepubVerifier;

std::string save_directory;
std::string pgTUMFormat, pgScansDirectory;
std::string odomTUMFormat;
std::string pgKITTIformat;
std::string odomKITTIformat;

std::ofstream scanMatchStream;
std::fstream pgTimeSaveStream;

// for front_end
ros::Publisher pubKeyFramesId;

// for loop closure
bool radiusSearch = true;
double historyKeyframeSearchRadius;
double historyKeyframeSearchTimeDiff;
int historyKeyframeSearchNum;
double loopClosureFrequency;
int graphUpdateTimes;
double graphUpdateFrequency;
double loopNoiseScore;
double vizmapFrequency;
double vizPathFrequency;
double speedFactor;
double filter_size;
ros::Publisher pubLoopScanLocalRegisted;
double loopFitnessScoreThreshold;  // user parameter but fixed low value is safe, e.g., 0.3

std::string padZeros(int val, int num_digits = 6) {
  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
  return out.str();
}

gtsam::Pose3 Pose6DtoGTSAMPose3(const Pose6D& p)
{
    return gtsam::Pose3( gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z) );
} // Pose6DtoGTSAMPose3

void saveOdometryVerticesKITTIformat(std::string _filename)
{
    // ref from gtsam's original code "dataset.cpp"
    std::fstream stream(_filename.c_str(), std::fstream::out);
    mKF.lock();
    for(const auto& _pose6d: keyframePoses) {
        gtsam::Pose3 pose = Pose6DtoGTSAMPose3(_pose6d);
        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << "\n";
    }
    mKF.unlock();
    stream.close();
}

void saveOdometryVerticesTUMFormat(std::string _filename)
{
    mKF.lock();
    if (keyframePoses.size() != keyframeTimes.size()) {
        ROS_ERROR_STREAM("Inconsistent keyframe #poses " << keyframePoses.size() 
                << " and #times " << keyframeTimes.size() << ".");
    }
    size_t minsize = std::min(keyframePoses.size(), keyframeTimes.size());
    std::fstream stream(_filename.c_str(), std::fstream::out);
    for(size_t i = 0; i < minsize; ++i) {
        const auto& _pose6d = keyframePoses[i];
        gtsam::Pose3 pose = Pose6DtoGTSAMPose3(_pose6d);
        Point3 t = pose.translation();
        Eigen::Quaterniond q = pose.rotation().toQuaternion();
        stream << keyframeTimes[i] << " " << std::fixed << std::setprecision(8) << t.x() << " " << t.y() << " " << t.z()
            << " " << std::setprecision(9) << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
    }
    mKF.unlock();
    stream.close();
}

void saveOptimizedVerticesKITTIformat(const gtsam::Values &_estimates, std::string _filename)
{
    using namespace gtsam;

    // ref from gtsam's original code "dataset.cpp"
    std::fstream stream(_filename.c_str(), std::fstream::out);
    mKF.lock();
    for(const auto& key_value: _estimates) {
        auto p = dynamic_cast<const GenericValue<Pose3>*>(&key_value.value);
        if (!p) continue;

        const Pose3& pose = p->value();

        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << "\n";
    }
    mKF.unlock();
    stream.close();
}

void saveOptimizedVerticesTUMFormat(const gtsam::Values &_estimates, std::string _filename)
{
    using namespace gtsam;
    if (_estimates.size() != isamStateTimes.size()) {
        ROS_ERROR_STREAM("Inconsistent keyframe estimated #poses " << _estimates.size() 
                << " and #times " << isamStateTimes.size() << ".");
    }
    std::fstream stream(_filename.c_str(), std::fstream::out);
    size_t minsize = std::min(_estimates.size(), isamStateTimes.size());
    size_t i = 0;
    for(const auto& key_value: _estimates) {
        auto p = dynamic_cast<const GenericValue<Pose3>*>(&key_value.value);
        if (!p) continue;
        if (i >= minsize) break;
        const Pose3& pose = p->value();

        Point3 t = pose.translation();
        Eigen::Quaterniond q = pose.rotation().toQuaternion();
        stream << isamStateTimes[i] << " " << std::fixed << std::setprecision(8) << t.x() << " " << t.y() << " " << t.z()
            << " " << std::setprecision(9) << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
        ++i;
    }
    stream.close();
}

void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &_laserOdometry)
{
	mBuf.lock();
	odometryBuf.push(_laserOdometry);
	mBuf.unlock();
	terminator.newPacket();
} // laserOdometryHandler

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &_laserCloudFullRes)
{
	mBuf.lock();
	fullResBuf.push(_laserCloudFullRes);
	mBuf.unlock();
} // laserCloudFullResHandler

void gpsHandler(const sensor_msgs::NavSatFix::ConstPtr &_gps)
{
    if(useGPS) {
        mBuf.lock();
        gpsBuf.push(_gps);
        mBuf.unlock();
    }
} // gpsHandler

void initNoises( void )
{
    gtsam::Vector priorNoiseVector6(6);
    priorNoiseVector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
    priorNoise = noiseModel::Diagonal::Variances(priorNoiseVector6);

    gtsam::Vector odomNoiseVector6(6);
    // odomNoiseVector6 << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
    odomNoiseVector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
    odomNoise = noiseModel::Diagonal::Variances(odomNoiseVector6);

    // double loopNoiseScore = 0.5; // constant is ok...
    gtsam::Vector robustNoiseVector6(6); // gtsam::Pose3 factor has 6 elements (6D)
    robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore;
    robustLoopNoise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6) );

    double bigNoiseTolerentToXY = 1000000000.0; // 1e9
    double gpsAltitudeNoiseScore = 250.0; // if height is misaligned after loop clsosing, use this value bigger
    gtsam::Vector robustNoiseVector3(3); // gps factor has 3 elements (xyz)
    robustNoiseVector3 << bigNoiseTolerentToXY, bigNoiseTolerentToXY, gpsAltitudeNoiseScore; // means only caring altitude here. (because LOAM-like-methods tends to be asymptotically flyging)
    robustGPSNoise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector3) );

} // initNoises

/**
 * get Pose6D from nav_msgs Odometry message. It is exactly the inverse of pcl::getTransformation.
 * return Pose6D p1
 * Eigen::Affine3f T = pcl::getTransformation(p1.x, p1.y, p1.z, p1.roll, p1.pitch, p1.yaw);
 * Eigen::Quaterniond(T.linear()) == _odom->pose.pose.orientation
 * T.translation() == _odom->pose.pose.position
 */
Pose6D getOdom(nav_msgs::Odometry::ConstPtr _odom)
{
    auto tx = _odom->pose.pose.position.x;
    auto ty = _odom->pose.pose.position.y;
    auto tz = _odom->pose.pose.position.z;

    double roll, pitch, yaw;
    geometry_msgs::Quaternion quat = _odom->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(quat.x, quat.y, quat.z, quat.w)).getRPY(roll, pitch, yaw);

    return Pose6D{tx, ty, tz, roll, pitch, yaw}; 
} // getOdom

Pose6D diffTransformation(const Pose6D& _p1, const Pose6D& _p2)
{
    Eigen::Affine3f SE3_p1 = pcl::getTransformation(_p1.x, _p1.y, _p1.z, _p1.roll, _p1.pitch, _p1.yaw);
    Eigen::Affine3f SE3_p2 = pcl::getTransformation(_p2.x, _p2.y, _p2.z, _p2.roll, _p2.pitch, _p2.yaw);
    Eigen::Matrix4f SE3_delta0 = SE3_p1.matrix().inverse() * SE3_p2.matrix();
    Eigen::Affine3f SE3_delta; SE3_delta.matrix() = SE3_delta0;
    float dx, dy, dz, droll, dpitch, dyaw;
    pcl::getTranslationAndEulerAngles (SE3_delta, dx, dy, dz, droll, dpitch, dyaw);
    // std::cout << "delta : " << dx << ", " << dy << ", " << dz << ", " << droll << ", " << dpitch << ", " << dyaw << std::endl;

    return Pose6D{double(abs(dx)), double(abs(dy)), double(abs(dz)), double(abs(droll)), double(abs(dpitch)), double(abs(dyaw))};
} // SE3Diff

/**
 * a, b are 3x3 rotation matrix, unitary and orthogonal.
 * https://courses.cs.duke.edu/fall13/compsci527/notes/rodrigues.pdf
*/
float diffRotation(const Eigen::Matrix3f &a, const Eigen::Matrix3f &b)
{
    Eigen::Matrix3f c = a.transpose() * b;
    float trace = c(0,0) + c(1,1) + c(2,2);
    float angle = acos((trace - 1.0) / 2.0);
    return angle;
}

pcl::PointCloud<PointType>::Ptr local2global(const pcl::PointCloud<PointType>::Ptr &cloudIn, const Pose6D& tf)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);
    
    int numberOfCores = 16;
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }

    return cloudOut;
}

void pubPath( void )
{
    // pub odom and path 
    nav_msgs::Odometry odomAftPGO;
    nav_msgs::Path pathAftPGO;
    pathAftPGO.header.frame_id = "camera_init";
    mKF.lock(); 
    // for (int node_idx=0; node_idx < int(keyframePosesUpdated.size()) - 1; node_idx++) // -1 is just delayed visualization (because sometimes mutexed while adding(push_back) a new one)
    for (int node_idx=0; node_idx < recentIdxUpdated; node_idx++) // -1 is just delayed visualization (because sometimes mutexed while adding(push_back) a new one)
    {
        const Pose6D& pose_est = keyframePosesUpdated.at(node_idx); // upodated poses
        // const gtsam::Pose3& pose_est = isamCurrentEstimate.at<gtsam::Pose3>(node_idx);

        nav_msgs::Odometry odomAftPGOthis;
        odomAftPGOthis.header.frame_id = "camera_init";
        odomAftPGOthis.child_frame_id = "/aft_pgo";
        odomAftPGOthis.header.stamp = keyframeTimes.at(node_idx);
        odomAftPGOthis.pose.pose.position.x = pose_est.x;
        odomAftPGOthis.pose.pose.position.y = pose_est.y;
        odomAftPGOthis.pose.pose.position.z = pose_est.z;
        odomAftPGOthis.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(pose_est.roll, pose_est.pitch, pose_est.yaw);
        odomAftPGO = odomAftPGOthis;

        geometry_msgs::PoseStamped poseStampAftPGO;
        poseStampAftPGO.header = odomAftPGOthis.header;
        poseStampAftPGO.pose = odomAftPGOthis.pose.pose;

        pathAftPGO.header.stamp = odomAftPGOthis.header.stamp;
        pathAftPGO.header.frame_id = "camera_init";
        pathAftPGO.poses.push_back(poseStampAftPGO);
    }
    mKF.unlock(); 
    pubOdomAftPGO.publish(odomAftPGO); // last pose 
    pubPathAftPGO.publish(pathAftPGO); // poses 

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odomAftPGO.pose.pose.position.x, odomAftPGO.pose.pose.position.y, odomAftPGO.pose.pose.position.z));
    q.setW(odomAftPGO.pose.pose.orientation.w);
    q.setX(odomAftPGO.pose.pose.orientation.x);
    q.setY(odomAftPGO.pose.pose.orientation.y);
    q.setZ(odomAftPGO.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odomAftPGO.header.stamp, "camera_init", "/aft_pgo"));
} // pubPath

void updatePoses(void)
{
    mKF.lock();
    int numPosesUpdated =  int(keyframePosesUpdated.size());
    for (int node_idx=0; node_idx < int(isamCurrentEstimate.size()); node_idx++)
    {
        Pose6D& p =keyframePosesUpdated[node_idx];
        p.x = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().x();
        p.y = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().y();
        p.z = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().z();
        p.roll = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().roll();
        p.pitch = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().pitch();
        p.yaw = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().yaw();
    }
    mKF.unlock();

    mtxRecentPose.lock();
    const gtsam::Pose3& lastOptimizedPose = isamCurrentEstimate.at<gtsam::Pose3>(int(isamCurrentEstimate.size())-1);
    recentOptimizedX = lastOptimizedPose.translation().x();
    recentOptimizedY = lastOptimizedPose.translation().y();

    recentIdxUpdated = numPosesUpdated - 1;

    mtxRecentPose.unlock();
} // updatePoses

void runISAM2opt(void)
{
    mtxPosegraph.lock();
    // called when a variable added 
    isamStateTimes.insert(isamStateTimes.end(), newStateTimes.begin(), newStateTimes.end());
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();

    laserCloudMapPGORedraw = newStateTimes.size() > 0;
    gtSAMgraph.resize(0);
    initialEstimate.clear();
    newStateTimes.clear();
    mtxPosegraph.unlock();

    isamCurrentEstimate = isam->calculateEstimate();
    updatePoses();
}

pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, Eigen::Affine3f transCur)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    PointType *pointFrom;

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    int numberOfCores = 8; // TODO move to yaml 
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        pointFrom = &cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom->x + transCur(0,1) * pointFrom->y + transCur(0,2) * pointFrom->z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom->x + transCur(1,1) * pointFrom->y + transCur(1,2) * pointFrom->z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom->x + transCur(2,1) * pointFrom->y + transCur(2,2) * pointFrom->z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom->intensity;
    }
    return cloudOut;
}

pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, gtsam::Pose3 transformIn)
{
    Eigen::Affine3f transCur = pcl::getTransformation(
                                transformIn.translation().x(), transformIn.translation().y(), transformIn.translation().z(), 
                                transformIn.rotation().roll(), transformIn.rotation().pitch(), transformIn.rotation().yaw() );
    return transformPointCloud(cloudIn, transCur);
}

/*
 * @brief: Find close keyframes to the keyframe with index key, and aggregate them into nearKeyframes
 * @param: nearKeyframes: the aggregated keyframes in the body frame of the keyframe with index root_idx
 */
void loopFindNearKeyframesCloud( pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& submap_size, const int& root_idx)
{
    // extract and stacking near keyframes (in global coord)
    nearKeyframes->clear();
    mKF.lock(); 
    for (int i = -submap_size; i <= submap_size; ++i) {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= int(keyframeLaserClouds.size()) )
            continue;
        *nearKeyframes += * local2global(keyframeLaserClouds[keyNear], keyframePosesUpdated[keyNear]);
    }
    const Pose6D W_T_B = keyframePosesUpdated[root_idx];
    mKF.unlock();

    if (nearKeyframes->empty())
        return;

    Eigen::Affine3f affine_W_T_B = pcl::getTransformation(W_T_B.x, W_T_B.y, W_T_B.z, W_T_B.roll, W_T_B.pitch, W_T_B.yaw);
    gtsam::Pose3 gtsam_W_T_B(gtsam::Rot3(affine_W_T_B.rotation().cast<double>()), affine_W_T_B.translation().cast<double>());
    gtsam::Pose3 gtsam_B_T_W = gtsam_W_T_B.inverse();
    nearKeyframes = transformPointCloud(nearKeyframes, gtsam_B_T_W);
    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
}

gtsam::Pose3 Pose6dTogtsamPose3(Pose6D pose)
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(pose.roll), double(pose.pitch), double(pose.yaw)),
                                gtsam::Point3(double(pose.x),    double(pose.y),     double(pose.z)));
}

Eigen::Affine3f toEigenAffine3f(const Pose6D &pose) {
    return pcl::getTransformation(pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw);
}

/**
 * return the relative pose B(loop_kf)_T_B(curr_kf)
 */
std::optional<gtsam::Pose3> doICPVirtualRelative( int _loop_kf_idx, int _curr_kf_idx )
{
    // parse pointclouds
    // int historyKeyframeSearchNum = 25; // enough. ex. [-25, 25] covers submap length of 50x1 = 50m if every kf gap is 1m
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr targetKeyframeCloud(new pcl::PointCloud<PointType>());
    // get current keyframe cloud in the current keyframe's body frame
    loopFindNearKeyframesCloud(cureKeyframeCloud, _curr_kf_idx, historyKeyframeSearchNum, _curr_kf_idx); 
    // *cureKeyframeCloud = *keyframeLaserClouds[_curr_kf_idx];
    // get loop keyframe clouds in the loop keyframe's body frame
    loopFindNearKeyframesCloud(targetKeyframeCloud, _loop_kf_idx, historyKeyframeSearchNum, _loop_kf_idx); 

    // loop verification
    // sensor_msgs::PointCloud2 cureKeyframeCloudMsg;
    // pcl::PointCloud<PointType>::Ptr cureKeyframeCloudWorld(new pcl::PointCloud<PointType>());
    // cureKeyframeCloudWorld = local2global(cureKeyframeCloud, keyframePosesUpdated[_curr_kf_idx]);
    // pcl::toROSMsg(*cureKeyframeCloudWorld, cureKeyframeCloudMsg);
    // cureKeyframeCloudMsg.header.frame_id = "camera_init";
    // pubLoopScanLocal.publish(cureKeyframeCloudMsg);

    // sensor_msgs::PointCloud2 targetKeyframeCloudMsg;
    // pcl::PointCloud<PointType>::Ptr targetKeyframeCloudWorld(new pcl::PointCloud<PointType>());
    // targetKeyframeCloudWorld = local2global(targetKeyframeCloud, keyframePosesUpdated[_loop_kf_idx]);
    // pcl::toROSMsg(*targetKeyframeCloudWorld, targetKeyframeCloudMsg);
    // targetKeyframeCloudMsg.header.frame_id = "camera_init";
    // pubLoopSubmapLocal.publish(targetKeyframeCloudMsg);

    // ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(2 * historyKeyframeSearchNum * keyframeMeterGap); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter 
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-1);

    // Align pointclouds to compute the transformation B(loop_kf)_T_B(curr_kf)
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(targetKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    mKF.lock();
    Pose6D loopKfPose = keyframePosesUpdated[_loop_kf_idx];
    Eigen::Affine3f guess = toEigenAffine3f(loopKfPose).inverse() * toEigenAffine3f(keyframePosesUpdated[_curr_kf_idx]);
    mKF.unlock();
    icp.align(*unused_result, guess.matrix());

    float rotang = diffRotation(guess.rotation(), icp.getFinalTransformation().block<3,3>(0,0));

    if (icp.hasConverged() == false || icp.getFitnessScore() > loopFitnessScoreThreshold) {
        ROS_INFO_STREAM("[SC loop] ICP fitness test failed (" << icp.getFitnessScore() << " > " << loopFitnessScoreThreshold << "). Reject this SC loop.");
        return std::nullopt;
    } else {
        ROS_INFO_STREAM("[SC loop] ICP fitness test passed (" << icp.getFitnessScore() << " < " << loopFitnessScoreThreshold 
            << ", rot angle change " << rotang * 180 / M_PI << "). Add this SC loop.");
        loopPairMutex.lock();
        loopIndexContainer[_curr_kf_idx] = _loop_kf_idx;
        loopPairMutex.unlock();
        
        sensor_msgs::PointCloud2 cureKeyframeCloudRegMsg;
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloudWorldReg(new pcl::PointCloud<PointType>());
        cureKeyframeCloudWorldReg = local2global(unused_result, loopKfPose);
        pcl::toROSMsg(*cureKeyframeCloudWorldReg, cureKeyframeCloudRegMsg);
        cureKeyframeCloudRegMsg.header.frame_id = "camera_init";
        pubLoopScanLocalRegisted.publish(cureKeyframeCloudRegMsg);
    }

    // Get pose transformation
    // float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame(icp.getFinalTransformation());
    Eigen::Matrix3f rot = correctionLidarFrame.rotation();
    Eigen::Vector3f trans = correctionLidarFrame.translation();

    return gtsam::Pose3(gtsam::Rot3(rot.cast<double>()), trans.cast<double>());
}

void process_pg()
{
    while(1)
    {
		while ( !odometryBuf.empty() && !fullResBuf.empty() )
        {
            //
            // pop and check keyframe is or not  
            // 
			mBuf.lock();       
            while (!odometryBuf.empty() && !fullResBuf.empty()) {
                if (odometryBuf.front()->header.stamp + ros::Duration(1e-4) < fullResBuf.front()->header.stamp)
                    odometryBuf.pop();
                else if (odometryBuf.front()->header.stamp > fullResBuf.front()->header.stamp + ros::Duration(1e-4))
                    fullResBuf.pop();
                else
                    break;
            }
            if (odometryBuf.empty() || fullResBuf.empty())
            {
                mBuf.unlock();
                break;
            }

            // Time equal check
            timeLaserOdometry = odometryBuf.front()->header.stamp;
            timeLaser = fullResBuf.front()->header.stamp;
            
            if (timeLaser != timeLaserOdometry) {
                double d = (timeLaser - timeLaserOdometry).toSec();
                ROS_INFO_STREAM("Large time gap between deskewed laser scan and odometry pose " << d << " sec.");
            }
            
            laserCloudFullRes->clear();
            pcl::PointCloud<PointType>::Ptr thisKeyFrame(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(*fullResBuf.front(), *thisKeyFrame);
            fullResBuf.pop();
            Pose6D pose_curr = getOdom(odometryBuf.front());
            odometryBuf.pop();

            // find nearest gps 
            double eps = 0.1; // find a gps topioc arrived within eps second 
            while (!gpsBuf.empty()) {
                auto thisGPS = gpsBuf.front();
                auto thisGPSTime = thisGPS->header.stamp;
                if (fabs((thisGPSTime - timeLaserOdometry).toSec()) < eps ) {
                    currGPS = thisGPS;
                    hasGPSforThisKF = true; 
                    break;
                } else {
                    hasGPSforThisKF = false;
                }
                gpsBuf.pop();
            }
            mBuf.unlock(); 

            //
            // Early reject by counting local delta movement (for equi-spereated kf drop)
            // 
            odom_pose_prev = odom_pose_curr;
            odom_pose_curr = pose_curr;
            Pose6D dtf = diffTransformation(odom_pose_prev, odom_pose_curr); // dtf means delta_transform

            double delta_translation = sqrt(dtf.x*dtf.x + dtf.y*dtf.y + dtf.z*dtf.z); // note: absolute value. 
            translationAccumulated += delta_translation;
            rotaionAccumulated += (dtf.roll + dtf.pitch + dtf.yaw); // sum just naive approach.  

            if( translationAccumulated > keyframeMeterGap || rotaionAccumulated > keyframeRadGap ) {
                isNowKeyFrame = true;
                translationAccumulated = 0.0; // reset 
                rotaionAccumulated = 0.0; // reset 
            } else {
                isNowKeyFrame = false;
            }

            if( ! isNowKeyFrame ) 
                continue; 

            if( !gpsOffsetInitialized ) {
                if(hasGPSforThisKF) { // if the very first frame 
                    gpsAltitudeInitOffset = currGPS->altitude;
                    gpsOffsetInitialized = true;
                } 
            }

            //
            // Save data and Add consecutive node 
            //
            pcl::PointCloud<PointType>::Ptr thisKeyFrameDS(new pcl::PointCloud<PointType>());
            downSizeFilterScancontext.setInputCloud(thisKeyFrame);
            downSizeFilterScancontext.filter(*thisKeyFrameDS);

            scMutex.lock();
            scManager.makeAndSaveScancontextAndKeys(*thisKeyFrameDS);
            scMutex.unlock();

            mKF.lock();
            keyframeLaserClouds.push_back(thisKeyFrameDS);
            keyframePoses.push_back(pose_curr);
            keyframePosesUpdated.push_back(pose_curr); // init
            keyframeTimes.push_back(timeLaserOdometry);

            hasNewScanForLC = true;

            const int init_node_idx = 0;
            gtsam::Pose3 poseOrigin = Pose6DtoGTSAMPose3(keyframePoses.at(init_node_idx));

            const int prev_node_idx = keyframePoses.size() - 2; 
            const int curr_node_idx = keyframePoses.size() - 1; // becuase cpp starts with 0 (actually this index could be any number, but for simple implementation, we follow sequential indexing)
            gtsam::Pose3 poseFrom;
            if (prev_node_idx >= 0)
                poseFrom = Pose6DtoGTSAMPose3(keyframePoses.at(prev_node_idx));
            gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(keyframePoses.at(curr_node_idx));
            mKF.unlock();

            if( ! gtSAMgraphMade /* prior node */) {
                mtxPosegraph.lock();
                {
                    // prior factor 
                    gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(init_node_idx, poseOrigin, priorNoise));
                    initialEstimate.insert(init_node_idx, poseOrigin);
                    newStateTimes.push_back(timeLaser);
                    // runISAM2opt();          
                }
                mtxPosegraph.unlock();

                gtSAMgraphMade = true; 

                cout << "posegraph prior node " << init_node_idx << " added" << endl;
            } else /* consecutive node (and odom factor) after the prior added */ { // == keyframePoses.size() > 1
                mtxRecentPose.lock();
                double copyRecentOptimizedX = recentOptimizedX;
                double copyRecentOptimizedY = recentOptimizedY;
                mtxRecentPose.unlock();
                mtxPosegraph.lock();
                {
                    // odom factor
                    // poseFrom.between(poseTo) means poseFrom.inverse() * poseTo
                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, poseFrom.between(poseTo), odomNoise));

                    // gps factor 
                    if(hasGPSforThisKF) {
                        double curr_altitude_offseted = currGPS->altitude - gpsAltitudeInitOffset;
                        gtsam::Point3 gpsConstraint(copyRecentOptimizedX, copyRecentOptimizedY, curr_altitude_offseted); // in this example, only adjusting altitude (for x and y, very big noises are set) 
                        gtSAMgraph.add(gtsam::GPSFactor(curr_node_idx, gpsConstraint, robustGPSNoise));
                        cout << "GPS factor added at node " << curr_node_idx << endl;
                    }
                    initialEstimate.insert(curr_node_idx, poseTo);
                    newStateTimes.push_back(timeLaser);            
                    // runISAM2opt();
                }
                mtxPosegraph.unlock();

                if(curr_node_idx % 100 == 0)
                    cout << "posegraph odom node " << curr_node_idx << " added." << endl;
            }
            // if want to print the current graph, use gtSAMgraph.print("\nFactor Graph:\n");
            // save utility 
            std::string curr_node_idx_str = padZeros(curr_node_idx);
            pcl::io::savePCDFileBinary(pgScansDirectory + curr_node_idx_str + ".pcd", *thisKeyFrame); // scan 
            pgTimeSaveStream << timeLaser << std::endl; // path 
        }

        // ps. 
        // scan context detector is running in another thread (in constant Hz, e.g., 1 Hz)
        // pub path and point cloud in another thread

        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
        shutdown = terminator.quit();
        if (shutdown) {
          std::cout << "shutdown ros!\n";
          pgTimeSaveStream.close();
          scanMatchStream.close();
          break;
        }
    }
} // process_pg

void performSCLoopClosure(void)
{
    mKF.lock();
    int numPoses = int(keyframePoses.size());
    mKF.unlock();
    if( numPoses < scManager.NUM_EXCLUDE_RECENT) // do not try too early 
        return;
    scMutex.lock();
    auto detectResult = scManager.detectLoopClosureID(); // first: nn index, second: yaw diff
    scMutex.unlock();
    int SCclosestHistoryFrameID = detectResult.first;
    if( SCclosestHistoryFrameID != -1 ) { 
        const int prev_node_idx = SCclosestHistoryFrameID;
        const int curr_node_idx = numPoses - 1; // because cpp starts 0 and ends n-1
        ROS_INFO_STREAM("Loop candidate between " << prev_node_idx << " and " << curr_node_idx << ".");

        mBuf.lock();
        scLoopICPBuf.push(std::pair<int, int>(prev_node_idx, curr_node_idx));
        // addding actual 6D constraints in the other thread, icp_calculation.
        mBuf.unlock();
    }
} // performSCLoopClosure

pcl::PointCloud<pcl::PointXYZ>::Ptr vector2pc(const std::vector<Pose6D> vectorPose6d){
    pcl::PointCloud<pcl::PointXYZ>::Ptr res( new pcl::PointCloud<pcl::PointXYZ> ) ;
    for( auto p : vectorPose6d){
        res->points.emplace_back(p.x, p.y, p.z);
    }
    return res;
}

bool detectLoopClosureDistance(int *loopKeyCur, int *loopKeyPre)
{
    mKF.lock();
    pcl::PointCloud<pcl::PointXYZ>::Ptr copy_cloudKeyPoses3D = vector2pc(keyframePoses);
    mKF.unlock();
    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;
    kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
    kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);

    mKF.lock();
    for(size_t i = 0; i < pointSearchIndLoop.size(); ++i)
    {
        int id = pointSearchIndLoop[i];
        if (std::fabs((keyframeTimes[id] - keyframeTimes[*loopKeyCur]).toSec()) > historyKeyframeSearchTimeDiff )
        {
            *loopKeyPre = id;
            break;
        }
    }
    mKF.unlock();

    if (*loopKeyPre == -1 || *loopKeyCur == *loopKeyPre)
        return false;

    return true;
}

void performRSLoopClosure(void)
{
    if (!hasNewScanForLC)
        return;
    else
        hasNewScanForLC = false;
    mKF.lock();
    int numPoses = int(keyframePoses.size());
    mKF.unlock();
    if (numPoses < scManager.NUM_EXCLUDE_RECENT) // do not try too early 
        return;

    int loopKeyCur = numPoses - 1;
    int loopKeyPre = -1;
    if ( detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) ){
        cout << "Loop candidate between " << loopKeyPre << " and " << loopKeyCur << "" << endl;
        mBuf.lock();
        scLoopICPBuf.push(std::pair<int, int>(loopKeyPre, loopKeyCur));
        // addding actual 6D constraints in the other thread, icp_calculation.
        mBuf.unlock();
    }
}

void visualizeLoopClosure()
{
    loopPairMutex.lock();
    std::unordered_map<int, int> copyLoopIndexContainer = loopIndexContainer;
    loopPairMutex.unlock();
    if (copyLoopIndexContainer.empty())
        return;
    
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker markerNode;
    markerNode.header.frame_id = "camera_init";
    markerNode.action = visualization_msgs::Marker::ADD;
    markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
    markerNode.ns = "loop_nodes";
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.3; markerNode.scale.y = 0.3; markerNode.scale.z = 0.3; 
    markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;
    markerNode.color.a = 1;

    visualization_msgs::Marker markerEdge;
    markerEdge.header.frame_id = "camera_init";
    markerEdge.action = visualization_msgs::Marker::ADD;
    markerEdge.type = visualization_msgs::Marker::LINE_LIST;
    markerEdge.ns = "loop_edges";
    markerEdge.id = 1;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.1;
    markerEdge.color.r = 0.9; markerEdge.color.g = 0.9; markerEdge.color.b = 0;
    markerEdge.color.a = 1;

    mKF.lock();
    markerNode.header.stamp = keyframeTimes.back();
    markerEdge.header.stamp = keyframeTimes.back();
    for (auto it = copyLoopIndexContainer.begin(); it != copyLoopIndexContainer.end(); ++it)
    {
        int key_cur = it->first;
        int key_pre = it->second;
        geometry_msgs::Point p;
        p.x = keyframePosesUpdated[key_cur].x;
        p.y = keyframePosesUpdated[key_cur].y;
        p.z = keyframePosesUpdated[key_cur].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
        p.x = keyframePosesUpdated[key_pre].x;
        p.y = keyframePosesUpdated[key_pre].y;
        p.z = keyframePosesUpdated[key_pre].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
    }
    mKF.unlock();

    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdge);
    pubLoopConstraintEdge.publish(markerArray);
}

void process_lcd(void)
{
    float loopClosureFrequency = 1.0; // can change 
    ros::Rate rate(loopClosureFrequency);
    while (ros::ok())
    {
        rate.sleep();
        if (radiusSearch)
            performRSLoopClosure();
        else
            performSCLoopClosure();
        visualizeLoopClosure();
        if (shutdown)
            break;
    }
} // process_lcd

void process_icp(void)
{
    while(ros::ok())
    {
		while ( !scLoopICPBuf.empty() )
        {
            mBuf.lock(); 
            if( scLoopICPBuf.size() > 30 ) {
                ROS_WARN("Too many loop clousre candidates to be ICPed is waiting ... Do process_lcd less frequently (adjust loopClosureFrequency)");
            }

            std::pair<int, int> loop_idx_pair = scLoopICPBuf.front();
            scLoopICPBuf.pop();
            mBuf.unlock(); 

            const int prev_node_idx = loop_idx_pair.first;
            const int curr_node_idx = loop_idx_pair.second;
            auto relative_pose_optional = doICPVirtualRelative(prev_node_idx, curr_node_idx);
            if(relative_pose_optional) {
                gtsam::Pose3 relative_pose = relative_pose_optional.value();
                mtxPosegraph.lock();
                gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, relative_pose, robustLoopNoise));
                // runISAM2opt();
                mtxPosegraph.unlock();
                mKF.lock();
                scanMatchStream << keyframeTimes[prev_node_idx] << " " << keyframeTimes[curr_node_idx] << "\n";
                mKF.unlock();
            }
        }

        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);

        if (shutdown)
            break;
    }
} // process_icp

void process_viz_path(void)
{
    ros::Rate rate(vizPathFrequency);
    while (ros::ok()) {
        rate.sleep();
        if(laserCloudMapPGORedraw) {
            pubPath();
        }
        if (shutdown)
            break;
    }
}

void process_isam(void)
{
    float hz = 1; 
    ros::Rate rate(hz);
    while (ros::ok()) {
        rate.sleep();
        if( gtSAMgraphMade ) {
            cout << "running isam2 optimization ..." << endl;
            runISAM2opt();

            if (shutdown) {
                // saveOptimizedVerticesKITTIformat(isamCurrentEstimate, pgKITTIformat); // pose
                // saveOdometryVerticesKITTIformat(odomKITTIformat); // pose
                ROS_INFO_STREAM("Saving optimized poses to file " << pgTUMFormat);
                saveOptimizedVerticesTUMFormat(isamCurrentEstimate, pgTUMFormat);
                ROS_INFO_STREAM("Saving odometry poses to file " << odomTUMFormat);
                saveOdometryVerticesTUMFormat(odomTUMFormat);
                break;
            }
        }
    }
}

void pubMap(void)
{
    int SKIP_FRAMES = 2; // sparse map visulalization to save computations 
    int counter = 0;

    laserCloudMapPGO->clear();

    mKF.lock(); 
    // for (int node_idx=0; node_idx < int(keyframePosesUpdated.size()); node_idx++) {
    for (int node_idx=0; node_idx < recentIdxUpdated; node_idx++) {
        if(counter % SKIP_FRAMES == 0) {
            *laserCloudMapPGO += *local2global(keyframeLaserClouds[node_idx], keyframePosesUpdated[node_idx]);
        }
        counter++;
    }
    mKF.unlock(); 

    downSizeFilterMapPGO.setInputCloud(laserCloudMapPGO);
    downSizeFilterMapPGO.filter(*laserCloudMapPGO);

    sensor_msgs::PointCloud2 laserCloudMapPGOMsg;
    pcl::toROSMsg(*laserCloudMapPGO, laserCloudMapPGOMsg);
    laserCloudMapPGOMsg.header.frame_id = "camera_init";
    pubMapAftPGO.publish(laserCloudMapPGOMsg);
}

void process_viz_map(void)
{
    ros::Rate rate(vizmapFrequency);
    while (ros::ok()) {
        rate.sleep();
        if(laserCloudMapPGORedraw) {
            pubMap();
        }
        if (shutdown)
            break;
    }
} // pointcloud_viz


int main(int argc, char **argv)
{
	ros::init(argc, argv, "laserPGO");
	ros::NodeHandle nh;
	bool terminateAtEnd = false;
	nh.param<bool>("terminate_at_end", terminateAtEnd, false);
	if (!terminateAtEnd) {
		 terminator.setWaitPacketsForNextPacket(-1);
	}
	nh.param<std::string>("save_directory", save_directory, "/"); // pose assignment every k m move 
    pgTUMFormat = save_directory + "keyscan_optimized_poses.txt";
    odomTUMFormat = save_directory + "keyscan_odom_poses.txt";
    pgTimeSaveStream = std::fstream(save_directory + "keyscan_times.txt", std::fstream::out); 
    pgTimeSaveStream.precision(std::numeric_limits<double>::max_digits10);
    scanMatchStream = std::ofstream(save_directory + "keyscan_matches.txt", std::fstream::out);
    pgScansDirectory = save_directory + "Scans/";
    auto unused = system((std::string("exec rm -r ") + pgScansDirectory).c_str());
    unused = system((std::string("mkdir -p ") + pgScansDirectory).c_str());

	nh.param<double>("keyframe_meter_gap", keyframeMeterGap, 2.0); // pose assignment every k m move 
	nh.param<double>("keyframe_deg_gap", keyframeDegGap, 10.0); // pose assignment every k deg rot 
    keyframeRadGap = deg2rad(keyframeDegGap);

	nh.param<double>("sc_dist_thres", scDistThres, 0.2);  
	nh.param<double>("sc_max_radius", scMaximumRadius, 80.0); // 80 is recommended for outdoor, and lower (ex, 20, 40) values are recommended for indoor 

    // for loop closure detection
    nh.param<bool>("radius_search", radiusSearch, true);
    if (radiusSearch)
        ROS_INFO_STREAM("Using radius search for loop closure.");
    else
        ROS_INFO_STREAM("Using scan context for loop closure.");
	nh.param<double>("historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);  
	nh.param<double>("historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);  
	nh.param<int>("historyKeyframeSearchNum", historyKeyframeSearchNum, 25);  
	nh.param<double>("loopNoiseScore", loopNoiseScore, 0.5);  
	nh.param<int>("graphUpdateTimes", graphUpdateTimes, 2);  
	nh.param<double>("loopFitnessScoreThreshold", loopFitnessScoreThreshold, 0.3);  

	nh.param<double>("speedFactor", speedFactor, 1);
    {
        nh.param<double>("sc_filter_size", filter_size, 0.4);
        nh.param<double>("loopClosureFrequency", loopClosureFrequency, 2);  
        loopClosureFrequency *= speedFactor;
        nh.param<double>("graphUpdateFrequency", graphUpdateFrequency, 1.0);  
        graphUpdateFrequency *= speedFactor;
        nh.param<double>("vizmapFrequency", vizmapFrequency, 0.1);  
        vizmapFrequency *= speedFactor;
        nh.param<double>("vizPathFrequency", vizPathFrequency, 10);  
        vizPathFrequency *= speedFactor;
        
    }

    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);
    initNoises();

    scManager.setSCdistThres(scDistThres);
    scManager.setMaximumRadius(scMaximumRadius);

    downSizeFilterScancontext.setLeafSize(filter_size, filter_size, filter_size);
    downSizeFilterICP.setLeafSize(filter_size, filter_size, filter_size);

    double mapVizFilterSize;
	nh.param<double>("mapviz_filter_size", mapVizFilterSize, 0.4); // pose assignment every k frames 
    downSizeFilterMapPGO.setLeafSize(mapVizFilterSize, mapVizFilterSize, mapVizFilterSize);

	ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_registered_local", 100, laserCloudFullResHandler);
	ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 100, laserOdometryHandler);
	ros::Subscriber subGPS = nh.subscribe<sensor_msgs::NavSatFix>("/gps/fix", 100, gpsHandler);

	pubOdomAftPGO = nh.advertise<nav_msgs::Odometry>("/aft_pgo_odom", 100);
	pubOdomRepubVerifier = nh.advertise<nav_msgs::Odometry>("/repub_odom", 100);

    // for front-end
    pubKeyFramesId = nh.advertise<std_msgs::Header>("/key_frames_ids", 10);

    // for loop closure
    pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>("/loop_closure_constraints", 1);
	pubLoopScanLocalRegisted = nh.advertise<sensor_msgs::PointCloud2>("/loop_scan_local_registed", 100);

	pubPathAftPGO = nh.advertise<nav_msgs::Path>("/aft_pgo_path", 100);
	pubMapAftPGO = nh.advertise<sensor_msgs::PointCloud2>("/aft_pgo_map", 100);

	pubLoopScanLocal = nh.advertise<sensor_msgs::PointCloud2>("/loop_scan_local", 100);
	pubLoopSubmapLocal = nh.advertise<sensor_msgs::PointCloud2>("/loop_submap_local", 100);

	std::thread posegraph_slam {process_pg}; // pose graph construction
	std::thread lc_detection {process_lcd}; // loop closure detection 
	std::thread icp_calculation {process_icp}; // loop constraint calculation via icp 
	std::thread isam_update {process_isam}; // if you want to call less isam2 run (for saving redundant computations and no real-time visulization is required), uncommment this and comment all the above runisam2opt when node is added. 

	std::thread viz_map {process_viz_map}; // visualization - map (low frequency because it is heavy)
	std::thread viz_path {process_viz_path}; // visualization - path (high frequency)

    ros::Rate r(10);
    while (!shutdown) {
 	    ros::spinOnce();
        r.sleep();
    }

    posegraph_slam.join();
    lc_detection.join();
    icp_calculation.join();
    isam_update.join();
    viz_map.join();
    viz_path.join();

	return 0;
}
