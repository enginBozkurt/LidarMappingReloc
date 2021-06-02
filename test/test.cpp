void Map::Save ( const string& filename )
 {
     cerr<<"Map Saving to "<<filename <<endl;
     ofstream f;
     f.open(filename.c_str(), ios_base::out|ios::binary);
     cerr << "The number of MapPoints is :"<<mspMapPoints.size()<<endl;
 
     //地图点的数目
     unsigned long int nMapPoints = mspMapPoints.size();
     f.write((char*)&nMapPoints, sizeof(nMapPoints) );
     //依次保存MapPoints
     for ( auto mp: mspMapPoints )
         SaveMapPoint( f, mp );
     //获取每一个MapPoints的索引值，即从0开始计数，初始化了mmpnMapPointsIdx       GetMapPointsIdx(); 
     cerr <<"The number of KeyFrames:"<<mspKeyFrames.size()<<endl;
     //关键帧的数目
     unsigned long int nKeyFrames = mspKeyFrames.size();
     f.write((char*)&nKeyFrames, sizeof(nKeyFrames));
 
     //依次保存关键帧KeyFrames
     for ( auto kf: mspKeyFrames )
         SaveKeyFrame( f, kf );
 
     for (auto kf:mspKeyFrames )
     {
         //获得当前关键帧的父节点，并保存父节点的ID
         KeyFrame* parent = kf->GetParent();
         unsigned long int parent_id = ULONG_MAX;
         if ( parent )
             parent_id = parent->mnId;
         f.write((char*)&parent_id, sizeof(parent_id));
         //获得当前关键帧的关联关键帧的大小，并依次保存每一个关联关键帧的ID和weight；
         unsigned long int nb_con = kf->GetConnectedKeyFrames().size();
         f.write((char*)&nb_con, sizeof(nb_con));
         for ( auto ckf: kf->GetConnectedKeyFrames())
         {
             int weight = kf->GetWeight(ckf);
             f.write((char*)&ckf->mnId, sizeof(ckf->mnId));
             f.write((char*)&weight, sizeof(weight));
         }
     }
 
     f.close();
     cerr<<"Map Saving Finished!"<<endl;
 }

 void Map::Save ( const string& filename )
 {
     cerr<<"Map Saving to "<<filename <<endl;
     ofstream f;
     f.open(filename.c_str(), ios_base::out|ios::binary);
     cerr << "The number of MapPoints is :"<<mspMapPoints.size()<<endl;
 
     //地图点的数目
     unsigned long int nMapPoints = mspMapPoints.size();
     f.write((char*)&nMapPoints, sizeof(nMapPoints) );
     //依次保存MapPoints
     for ( auto mp: mspMapPoints )
         SaveMapPoint( f, mp );
     //获取每一个MapPoints的索引值，即从0开始计数，初始化了mmpnMapPointsIdx       GetMapPointsIdx(); 
     cerr <<"The number of KeyFrames:"<<mspKeyFrames.size()<<endl;
     //关键帧的数目
     unsigned long int nKeyFrames = mspKeyFrames.size();
     f.write((char*)&nKeyFrames, sizeof(nKeyFrames));
 
     //依次保存关键帧KeyFrames
     for ( auto kf: mspKeyFrames )
         SaveKeyFrame( f, kf );
 
     for (auto kf:mspKeyFrames )
     {
         //获得当前关键帧的父节点，并保存父节点的ID
         KeyFrame* parent = kf->GetParent();
         unsigned long int parent_id = ULONG_MAX;
         if ( parent )
             parent_id = parent->mnId;
         f.write((char*)&parent_id, sizeof(parent_id));
         //获得当前关键帧的关联关键帧的大小，并依次保存每一个关联关键帧的ID和weight；
         unsigned long int nb_con = kf->GetConnectedKeyFrames().size();
         f.write((char*)&nb_con, sizeof(nb_con));
         for ( auto ckf: kf->GetConnectedKeyFrames())
         {
             int weight = kf->GetWeight(ckf);
             f.write((char*)&ckf->mnId, sizeof(ckf->mnId));
             f.write((char*)&weight, sizeof(weight));
         }
     }
 
     f.close();
     cerr<<"Map Saving Finished!"<<endl;
 }

 void Map::SaveKeyFrame( ofstream &f, KeyFrame* kf )
 {
//保存当前关键帧的ID和时间戳
     f.write((char*)&kf->mnId, sizeof(kf->mnId));
     f.write((char*)&kf->mTimeStamp, sizeof(kf->mTimeStamp));
     //保存当前关键帧的位姿矩阵
     cv::Mat Tcw = kf->GetPose();
     //通过四元数保存旋转矩阵
     std::vector<float> Quat = Converter::toQuaternion(Tcw);
     for ( int i = 0; i < 4; i ++ )
         f.write((char*)&Quat[i],sizeof(float));
     //保存平移矩阵
     for ( int i = 0; i < 3; i ++ )
         f.write((char*)&Tcw.at<float>(i,3),sizeof(float));
 
 
     //直接保存旋转矩阵
 //  for ( int i = 0; i < Tcw.rows; i ++ )
 //  {
 //      for ( int j = 0; j < Tcw.cols; j ++ )
 //      {
 //              f.write((char*)&Tcw.at<float>(i,j), sizeof(float));
 //              //cerr<<"Tcw.at<float>("<<i<<","<<j<<"):"<<Tcw.at<float>(i,j)<<endl;
 //      }
 //    }
 
     //保存当前关键帧包含的ORB特征数目
     //cerr<<"kf->N:"<<kf->N<<endl;
     f.write((char*)&kf->N, sizeof(kf->N));
     //保存每一个ORB特征点
     for( int i = 0; i < kf->N; i ++ )
     {
         cv::KeyPoint kp = kf->mvKeys[i];
         f.write((char*)&kp.pt.x, sizeof(kp.pt.x));
         f.write((char*)&kp.pt.y, sizeof(kp.pt.y));
         f.write((char*)&kp.size, sizeof(kp.size));
         f.write((char*)&kp.angle,sizeof(kp.angle));
         f.write((char*)&kp.response, sizeof(kp.response));
         f.write((char*)&kp.octave, sizeof(kp.octave));
 
         //保存当前特征点的描述符
         for (int j = 0; j < kf->mDescriptors.cols; j ++ )
                 f.write((char*)&kf->mDescriptors.at<unsigned char>(i,j), sizeof(char));
 
         //保存当前ORB特征对应的MapPoints的索引值
         unsigned long int mnIdx;
         MapPoint* mp = kf->GetMapPoint(i);
         if (mp == NULL  )
                 mnIdx = ULONG_MAX;
         else
                 mnIdx = mmpnMapPointsIdx[mp];
 
         f.write((char*)&mnIdx, sizeof(mnIdx));
     }
 }