#include<iostream>
#include<fstream>
#include<string>
#include<Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<boost/dynamic_bitset.hpp>
#include<boost/utility.hpp>

#include"DBoW3/DBoW3.h"
using namespace std;
using namespace cv;
#define mSize 500
      /**
   * Returns the Hamming distance between two descriptors
   * @param a first descriptor vector
   * @param b second descriptor vector
   * @return hamming distance
   */
    int  distance(const  cv::Mat &a,
    const  cv::Mat &b)
    {
    // Bit set count operation from
    // http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel

    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
    }
int main(){
    // Eigen::Matrix
    Eigen::Affine3f a=Eigen::Affine3f::Identity();; 
    cout<<a.matrix()<<endl;

    // // test vector assign
    // vector<int> vec;
    // vec.assign(10,0);
    // cout<<vec.size();

    // test descriptor and kp loading


    // // test bitset saving
    // std::vector<boost::dynamic_bitset<>> descriptors;
    // for(int i=0; i<10;i++){
    //     boost::dynamic_bitset<> bs(10);
    //     boost::dynamic_bitset<> bs(10);
    //     boost::dynamic_bitset<> bs(0x08,BOOST_BINARY(10101));
    //     cout<<bs<<endl;
    //     descriptors.push_back(bs);
    // }
    // // boost::dynamic_bitset<> bs;
    // // cout<<bs; // empty if no assignment
    // // descriptors.push_back(bs);
    // string filename = "1.txt";
    // ofstream out(filename,ios::out | ios::binary);
    // // out.open(filename,ios::out | ios::binary);
    // for(int i=0;i<descriptors.size();i++){
    //     string buffer;
    //     to_string(descriptors[i],buffer);
    //     out.write(buffer.c_str(),buffer.size());
    // }
    // out.close();

    // // test vector assign on c array
    // vector<int> a;
    // a.assign(10,1);
    // cout<<a.size()<<endl;

    // // test pair
    // vector<pair<int,int>> vec;
    // vec.push_back(make_pair(4,1));
    // vec.push_back(make_pair(3,2));
    // vec.push_back(make_pair(2,3));
    // sort(vec.begin(),vec.end());
    // for( auto i: vec) cout<<i.second<<endl;

    // // test binary writing for float numbers
    // string filename = "1.txt";
    // ofstream out(filename,ios::out);
    // float a = 124.23423;
    // out.write((char*)&a,sizeof(a));
    // out.close();
    // ifstream in(filename);
    // float b;
    // in.read((char*)&b,sizeof(float));
    // in.close();
    // cout<<b<<endl;

    // // test cv::Mat
    // int size = 200;
    // cv::Mat a(1,32,CV_8UC1,cv::Scalar::all(1));
    // a.ptr<uchar>(0)[1] = 0;
    // cv::Mat b(1,32,CV_8UC1,cv::Scalar::all(0));
    // int dist = distance(a,b);
    // cout<<dist<<endl;



    // Mat src = imread("../robComics.jpg",0);
    // cout<<src.rows<<" " << src.cols<<endl;
    // Mat src2(100,100,CV_8UC1,cv::Scalar::all(0));
    // cv::Mat h = src(cv::Rect(0,0,100,100));
    // src2.copyTo(h);
    // // cout<<"Type: "<<src.type()<<" channels: "<<src.channels()<<" step: "<<src.step<< " row: "<< src.rows << " col: "<< src.cols <<endl;
    // // unsigned int b = src.data[10*src.step + 11*src.channels()+1];
    // // cout<<b<<endl;
    // uchar* ptr = src.data;
    // for (int i=0;i<500;i++)
    //     for (int j=0;j< 32;j++){
    //         uchar a = 125;
    //         src.ptr<uchar>(i)[j] = 1;
    //     }
    // Mat src2 = src.clone();
    // vector<Mat> vec;
    // vec.push_back(src);
    // vec.push_back(src2);
    // cout<<vec[0].type()<<endl;
    // DBoW3::Vocabulary vocab;
    // vocab.create(vec);
    // cout<<vocab<<endl;

    // imshow("pic",src);
    // waitKey(0);

    // // eigen2cv
    // Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> eigenM;
    // eigenM = Eigen::MatrixXd::Random(mSize,mSize);
    // for( int i=0;i<500;i++)
    //     for(int j=0;j<500;j++){
    //         eigenM(i,j)=(i*500.0+j);
    // }
    // // eigenM.normalize();
    // eigenM /= eigenM.maxCoeff();
    // // cout<<eigenM<<endl;
    // // eigenM.fill(255);
    // Mat src2;
    // eigen2cv(eigenM,src2);
    // imshow("pic2",src2);
    // waitKey(0);
    // double a = 1454234124423.234324564;
    // ofstream f("a.txt");
    // f.setf(ios::fixed, ios::floatfield);  // 设定为 fixed 模式，以小数点表示浮点数
    // f.precision(5);
    // // f<<setprecision(18)<<a<<setprecision(12)<<endl;
    // f<<a;
    // f.close();

    // string b  = to_string(a);
    // cout<<b<<endl;
    // vector<int> a={1,2,3};
    // cout<<int(a.size())-1<<endl;
    // // int b=100;
    // // cout<<(std::to_string(b)+"+200=300").c_str()<<endl;
    // string filePath = "/home/binpeng/Documents/LIO-SAM/maps/office_rev/pose.txt";
    // ifstream fin(filePath);
    // vector<vector<float>> poseVec;
    // // while (!fin.eof()){ //why not this?
    // // while (fin.peek()!=EOF){
    //     while (true){
    //     vector<float> pose(6,0);
    //     fin>>pose[0]>>pose[1]>>pose[2]>>pose[3]>>pose[4]>>pose[5];
    //     // cout<<fin.peek()<<endl;
    //     // cout<<to_string(pose[0]).c_str()<<endl;
    //     // if(!fin){ // I don't understand how this works
    //     if(fin.peek()==EOF){ // it seems '\n' is not end of file yet, which is not skipped by ">>"
    //         break;
    //     }
    //     else{
    //         poseVec.push_back(pose);
    //     }
    // }
    // fin.close();
    // cout<<poseVec.size()<<endl;
    // for(int i=0;i<6;i++){
    //     cout<<poseVec.back()[i]<<endl;
    // }

    return 0;
}