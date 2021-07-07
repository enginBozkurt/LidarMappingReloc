#include "utility.h"
#include "lio_sam/cloud_info.h"

struct smoothness_t{ 
    float value;
    size_t ind;
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

class FeatureExtraction : public ParamServer
{

public:

    ros::Subscriber subLaserCloudInfo;

    ros::Publisher pubLaserCloudInfo;
    ros::Publisher pubCornerPoints;
    ros::Publisher pubSurfacePoints;

    pcl::PointCloud<PointType>::Ptr surfaceCloud2;
    pcl::VoxelGrid<PointType> downSizeFilter2;
    ros::Publisher pubSurfacePoints2;

    pcl::PointCloud<PointType>::Ptr extractedCloud;
    pcl::PointCloud<PointType>::Ptr cornerCloud;
    pcl::PointCloud<PointType>::Ptr surfaceCloud;

    pcl::VoxelGrid<PointType> downSizeFilter;

    lio_sam::cloud_info cloudInfo;
    std_msgs::Header cloudHeader;

    std::vector<smoothness_t> cloudSmoothness;
    float *cloudCurvature;
    int *cloudNeighborPicked;
    int *cloudLabel;

    FeatureExtraction()
    {
        subLaserCloudInfo = nh.subscribe<lio_sam::cloud_info>("lio_sam/deskew/cloud_info", 1, &FeatureExtraction::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());

        pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info> ("lio_sam/feature/cloud_info", 1);
        pubCornerPoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_corner", 1);
        pubSurfacePoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_surface", 1);
        pubSurfacePoints2 = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_surface2", 1);
        initializationValue();
    }

    void initializationValue()
    {
        cloudSmoothness.resize(N_SCAN*Horizon_SCAN);

        downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);

        extractedCloud.reset(new pcl::PointCloud<PointType>());
        cornerCloud.reset(new pcl::PointCloud<PointType>());
        surfaceCloud.reset(new pcl::PointCloud<PointType>());
        surfaceCloud2.reset(new pcl::PointCloud<PointType>());

        cloudCurvature = new float[N_SCAN*Horizon_SCAN];
        cloudNeighborPicked = new int[N_SCAN*Horizon_SCAN];
        cloudLabel = new int[N_SCAN*Horizon_SCAN];
        // For BLC
        // cloudInfo.keypointX.assign(keypointNum, 0); // why still 0 size after assign operation??????????????????????????????????????????????????
        // cloudInfo.keypointY.assign(keypointNum, 0);
        cloudInfo.enoughNum = false;
    }

    void laserCloudInfoHandler(const lio_sam::cloud_infoConstPtr& msgIn)
    {
        // static int cnt=1;
        // cout<<"111"<<endl; // can keep up
        cloudInfo = *msgIn; // new cloud info
        cloudHeader = msgIn->header; // new cloud header

        pcl::fromROSMsg(msgIn->cloud_deskewed, *extractedCloud); // new cloud for extraction
        TicToc featureExtraction; 
        calculateSmoothness();
        // cout<<"222"<<endl; // can keep up
        markOccludedPoints();
        // cout<<"333"<<endl; // can keep up
        extractFeatures();
        
        // cout<<"Feature extraction takes: "<< featureExtraction.toc() <<" ms"<<endl;
        publishFeatureCloud();
        // cout<<"555"<<endl; // can keep up
    }

    void calculateSmoothness()
    {
        
        int cloudSize = extractedCloud->points.size();
        // cout<<"Before extraction "<<cloudSize<<endl;
        for (int i = 5; i < cloudSize - 5; i++)
        {
            // 原始LOAM的diffRange先求diffX，diffY，diffZ
            float diffRange = cloudInfo.pointRange[i-5] + cloudInfo.pointRange[i-4]
                            + cloudInfo.pointRange[i-3] + cloudInfo.pointRange[i-2]
                            + cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i] * 10
                            + cloudInfo.pointRange[i+1] + cloudInfo.pointRange[i+2]
                            + cloudInfo.pointRange[i+3] + cloudInfo.pointRange[i+4]
                            + cloudInfo.pointRange[i+5];            

            cloudCurvature[i] = diffRange*diffRange;//diffX * diffX + diffY * diffY + diffZ * diffZ;

            cloudNeighborPicked[i] = 0;
            cloudLabel[i] = 0;
            // cloudSmoothness for sorting
            cloudSmoothness[i].value = cloudCurvature[i];
            cloudSmoothness[i].ind = i;
        }
    }

    void markOccludedPoints()
    {
        int cloudSize = extractedCloud->points.size();
        // mark occluded points and parallel beam points
        for (int i = 5; i < cloudSize - 6; ++i)
        {
            // occluded points
            float depth1 = cloudInfo.pointRange[i];
            float depth2 = cloudInfo.pointRange[i+1];
            int columnDiff = std::abs(int(cloudInfo.pointColInd[i+1] - cloudInfo.pointColInd[i]));

            if (columnDiff < 10){
                // 10 pixel diff in range image
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
            // parallel beam
            float diff1 = std::abs(float(cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i]));
            float diff2 = std::abs(float(cloudInfo.pointRange[i+1] - cloudInfo.pointRange[i]));

            if (diff1 > 0.02 * cloudInfo.pointRange[i] && diff2 > 0.02 * cloudInfo.pointRange[i])
                cloudNeighborPicked[i] = 1;
        }
    }

    void extractFeatures()
    {
                
        cornerCloud->clear();
        surfaceCloud->clear();

        pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());
        // int midCornerNum = 0;
        // vector<int> allCornerPointX;
        // vector<int> allCornerPointY;
        // vector<pair<float,int>> allCornerPointSmoothness; // Choose the larger_curvature points

        // int midSurfaceNum = 0;
        // vector<int> allSurfacePointX;
        // vector<int> allSurfacePointY;
        // vector<pair<float,int>> allSurfacePointSmoothness; //Choose the smaller_curvature surface points

        for (int i = 0; i < N_SCAN; i++)
        {
            surfaceCloudScan->clear();

            for (int j = 0; j < 6; j++)
            {

                int sp = (cloudInfo.startRingIndex[i] * (6 - j) + cloudInfo.endRingIndex[i] * j) / 6;
                int ep = (cloudInfo.startRingIndex[i] * (5 - j) + cloudInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

                if (sp >= ep)
                    continue;

                std::sort(cloudSmoothness.begin()+sp, cloudSmoothness.begin()+ep, by_value());

                int largestPickedNum = 0;
                for (int k = ep; k >= sp; k--)
                {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold)
                    {
                        largestPickedNum++;
                        if (largestPickedNum <= 20){
                            cloudLabel[ind] = 1;
                            cornerCloud->push_back(extractedCloud->points[ind]);
                            // if (i < N_SCAN-keypointSize && i > keypointSize -1)
                            // {
                            //     allCornerPointY.push_back(i);
                            //     float horizonAngle = atan2(extractedCloud->points[ind].y,extractedCloud->points[ind].x) * 180 / M_PI;
                            //     static float ang_res_x = 360.0/float(Horizon_SCAN);
                            //     allCornerPointX.push_back(-round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2);
                            //     allCornerPointSmoothness.push_back(make_pair(cloudCurvature[ind],midCornerNum));
                            //     midCornerNum++;
                            // }
                        } 
                        else {
                            break;
                        }

                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++)
                        {
                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++)
                {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold)
                    {

                        cloudLabel[ind] = -1;
                        cloudNeighborPicked[ind] = 1;

                        for (int l = 1; l <= 5; l++) {

                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {

                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++)
                {
                    if (cloudLabel[k] <= 0){
                        surfaceCloudScan->push_back(extractedCloud->points[k]);

                        // if (i < N_SCAN-keypointSize && i > keypointSize -1)
                        // {
                        //     allSurfacePointY.push_back(i);
                        //     float horizonAngle = atan2(extractedCloud->points[k].y,extractedCloud->points[k].x) * 180 / M_PI;
                        //     static float ang_res_x = 360.0/float(Horizon_SCAN);
                        //     allSurfacePointX.push_back(-round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2);
                        //     allSurfacePointSmoothness.push_back(make_pair(cloudCurvature[k],midSurfaceNum));
                        //     midSurfaceNum++;
                        // }
                    }
                }
            }

            surfaceCloudScanDS->clear();
            downSizeFilter.setInputCloud(surfaceCloudScan);
            downSizeFilter.filter(*surfaceCloudScanDS);

            *surfaceCloud += *surfaceCloudScanDS;
        }

        // // cout<<"Available corner points per frame: "<<midCornerNum<<"surface points per frame: "<<midSurfaceNum<<endl;
        // if (midCornerNum < keypointCornerNum || midSurfaceNum < keypointSurfaceNum) cloudInfo.enoughNum = false;
        // else cloudInfo.enoughNum = true;

        // std::sort(allCornerPointSmoothness.begin(),allCornerPointSmoothness.end());
        // int index = 0;
        // // cout<<allCornerPointX.size()<<endl;
        // // cout<<cloudInfo.keypointX.size()<<endl;
        // for(int p = midCornerNum-1; p >= 0;p--){ // choose points with high curvature
        //     if (index >= keypointCornerNum) break;
        //     // cout<<allCornerPointSmoothness[p].second<<" "<<allCornerPointSmoothness[p].first<<endl;
        //     cloudInfo.keypointX.push_back(allCornerPointX[allCornerPointSmoothness[p].second]);
        //     cloudInfo.keypointY.push_back(allCornerPointY[allCornerPointSmoothness[p].second]);
        //     index++;
        // }


        // // //Try get more even-distributed clouds
        // // 2.0: 100; 1.0: 200;
        // float filterSize = 1.0;
        // downSizeFilter2.setLeafSize(filterSize, filterSize, filterSize);
        // downSizeFilter2.setInputCloud(surfaceCloud);
        // downSizeFilter2.filter(*surfaceCloud2);
        // cout<<"After downsampling, cloud size: "<<surfaceCloud->size()<<endl;
        // publishCloud(&pubSurfacePoints2, surfaceCloud2, cloudHeader.stamp, lidarFrame);

        // std::sort(allSurfacePointSmoothness.begin(),allSurfacePointSmoothness.end());
        // index = 0;
        // // cout<<allCornerPointX.size()<<endl;
        // // cout<<cloudInfo.keypointX.size()<<endl;
        // for(int p = 0; p < midSurfaceNum-1;p++){ // choose points with high curvature
        //     if (index >= keypointSurfaceNum) break;
        //     // cout<<allCornerPointSmoothness[p].second<<" "<<allCornerPointSmoothness[p].first<<endl;
        //     cloudInfo.keypointX.push_back(allSurfacePointX[allSurfacePointSmoothness[p].second]);
        //     cloudInfo.keypointY.push_back(allSurfacePointY[allSurfacePointSmoothness[p].second]);
        //     index++;
        // }

        // cout<<cloudInfo.keypointX.size()<<endl;
        // cout<<index<<endl;
    }

    void freeCloudInfoMemory()
    {
        cloudInfo.startRingIndex.clear();
        cloudInfo.endRingIndex.clear();
        cloudInfo.pointColInd.clear();
        cloudInfo.pointRange.clear();
    }

    void publishFeatureCloud()
    {
        // free cloud info memory
        freeCloudInfoMemory();
        // save newly extracted features
        // cout<<"corner point number is :" << cornerCloud->points.size()<<endl;
        cloudInfo.cloud_corner  = publishCloud(&pubCornerPoints,  cornerCloud,  cloudHeader.stamp, lidarFrame);
        cloudInfo.cloud_surface = publishCloud(&pubSurfacePoints, surfaceCloud, cloudHeader.stamp, lidarFrame);


        // publish to mapOptimization
        pubLaserCloudInfo.publish(cloudInfo);
        // static int cnt=1;
        // cout<<cnt++<<" feature clouds published!"<<endl; // can keep up
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam");

    FeatureExtraction FE;

    ROS_INFO("\033[1;32m----> Feature Extraction Started.\033[0m");
   
    ros::spin();

    return 0;
}