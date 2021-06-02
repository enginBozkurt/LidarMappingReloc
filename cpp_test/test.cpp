#include<iostream>
#include<fstream>
#include<string>
#include<Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<boost/dynamic_bitset.hpp>
#include<boost/utility.hpp>
using namespace std;
using namespace cv;
#define mSize 500
int main(){
    // test descriptor and kp loading
    ifstream des("/home/binpeng/Documents/LIO-SAM/maps/office/BLC/descriptor128dim_60kp.txt",ios::in | ios::binary);
    // vector<cv::Mat> desOfAllKeyframes;
    int i=0;
    while(des.good()){
        char buffer[128];
        des.read(buffer,128);
        cout<<buffer<<endl;
        i++;
    }
    cout<<i<<endl;
    des.close();
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

    // // test cv::Mat

    // Mat src = imread("robComics.jpg",1);
    // // cout<<src<<endl;
    // cout<<"Type: "<<src.type()<<" channels: "<<src.channels()<<" step: "<<src.step<< " row: "<< src.rows << " col: "<< src.cols <<endl;
    // unsigned int b = src.data[10*src.step + 11*src.channels()+1];
    // cout<<b<<endl;
    // // imshow("pic",src);
    // // waitKey(0);

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