#include "detect.h"
#include <fstream>

bool decodeCoord(string _info,Vec3f& point);
string decodeCoordView(string _info);

int main(int argc,char** argv)
{
    string path_dir;
    if(argc<2)
    {
        cout<<"usage: ./QRPositions dasetDir"<<endl;
        return -1;
    }
    else
    {
        path_dir=argv[1];
        int len=path_dir.size();
        if(path_dir[len-1]!='/')
        path_dir+="/";
    }
    //读取每桢时间和名字
    vector<double> timeStamps;
    vector<string> paths;
    string correspondanceFile=path_dir+"timer.txt";
    cout<<"correspondanceFile = "<<correspondanceFile<<endl;
    std::ifstream corres(correspondanceFile);
    double time_temp;
    string path_temp;
    getline(corres,path_temp);
    while(corres>>path_temp>>time_temp){
        timeStamps.push_back(time_temp);
        paths.push_back(path_dir+path_temp);
    }
    cout<<"we have got total "<<timeStamps.size()<<" frames"<<endl;

    std::ofstream out1("trajectory.txt");

    //uvc 500万像素相机参数
    Mat distCoeffs(4,1,CV_32FC1);
    Mat cameraMatrix=Mat::zeros(3,3,CV_32FC1);
    double intrinsic[4]={1346.5939984735066, 1345.7135132758451, 1210.1954116491484, 909.4963486774401};
    //pinhole
    double fx=1346.5939984735066;
    double fy=1345.7135132758451;
    double cx=1210.1954116491484;
    double cy=909.4963486774401;
    double f=0.5*(fx+fy);
    cameraMatrix.at<float>(0,0)=(float)fx;//fx
    cameraMatrix.at<float>(0,2)=(float)cx;//cx
    cameraMatrix.at<float>(1,1)=(float)fy;//fy
    cameraMatrix.at<float>(1,2)=(float)cy;//cy 
    cameraMatrix.at<float>(2,2)=1.0;

    int num_images=300;
    if(argc>=3)
        num_images=atoi(argv[2]);
    vector<int> indexs;//存放每一桢的序号
    vector<double*> poses;//每一桢的姿态变量,9维，存放每桢的6维姿态和3维相机参数，前三维是旋转向量，中间三维是平移，后三维是相机参数
    vector<double*> codesInfo;//存放二维码的空间位置
    vector<vector<pair<int,Vec2d>>> FrameQRindexs;//存放每桢观察到的QRCode序号,和观测到的像素坐标

    //逐桢解码，获取优化初值
    int total_codes=0;
    for(int index=0;index<paths.size();index++)
    {
        string path_img=paths[index];
        Mat image_raw,image_binary,image_color;

        //后续计算需要的二维码信息参数
        vector<string> infos;//存放二维码的内容
        vector<vector<Vec2f>> positions;//对应地存放每个二维码的四个顶点坐标
        //显示用的参数
        vector<QuaVert> quas_raw;   //二维码四个顶点的位置
        vector<QuaVert> quas_expand;//将四个顶点向外扩张后的四边形
        cout<<"reading img from path : "<<path_img<<endl;
        //以灰度方式读入原始图
        image_raw=imread(path_img,IMREAD_GRAYSCALE);
        if(image_raw.empty())
        {
            cout<<"reding img from dir "<<path_img<<"failed!"<<endl;
            return -1;
        }
        detect_decode(image_raw,infos,positions,quas_raw,quas_expand);
        if(infos.size()!=positions.size()||positions.size()!=quas_raw.size())//对结果进行检测
        {
            cout<<"unmached msg!"<<endl;
            continue;
        }
        if(infos.size()==0)
        {
            cout<<"no msg have been detected!"<<endl;
            continue;
        }
        total_codes+=infos.size();
        cout<<infos.size()<<" QR code(s) have been detected!"<<endl;

        //框出检测到的二维码
        /**/ 
        Mat img_view_decoding;
        cvtColor(image_raw,img_view_decoding,COLOR_GRAY2BGR);
        for(int i=0;i<quas_raw.size();i++)
        {
            line(img_view_decoding,quas_raw[i].A,quas_raw[i].B,Scalar(0,0,255),4,LINE_AA);
            line(img_view_decoding,quas_raw[i].A,quas_raw[i].C,Scalar(0,0,255),4,LINE_AA);
            line(img_view_decoding,quas_raw[i].B,quas_raw[i].D,Scalar(0,0,255),4,LINE_AA);
            line(img_view_decoding,quas_raw[i].D,quas_raw[i].C,Scalar(0,0,255),4,LINE_AA);
            string str_show=decodeCoordView(infos[i]);
            putText(img_view_decoding,str_show,quas_raw[i].A,FONT_HERSHEY_SIMPLEX,1,Scalar(255,23,0),4,8);
        }
        namedWindow("result",WINDOW_NORMAL);
        imshow("result",img_view_decoding);
        cv::waitKey(10);
        
        //为P3P准备参数
        vector<Vec3f> objectPoints;
        vector<Vec2f> imagePoints;
        Vec3f tvecs;
        Vec3f rotation_vector;
        for(int i=0;i<positions.size();i++)
        {
            Vec3f objPoint;
            Vec2f imgPoint;
            decodeCoord(infos[i],objPoint);
            imgPoint[0]=positions[i][0][0];
            imgPoint[1]=positions[i][0][1];
            objectPoints.push_back(objPoint);
            imagePoints.push_back(imgPoint);
        }
        //求解pnp
        cout<<"objectPoints.size() = "<<objectPoints.size()<<endl;
        if(objectPoints.size()<5)
        {
            cout<<"No enough QR codes!"<<endl;
            continue;
        }
        int solutions=solvePnP(objectPoints,imagePoints,cameraMatrix,distCoeffs,rotation_vector,tvecs,0,SOLVEPNP_IPPE);
        Mat rotationMatrix,cor;
        cv::Rodrigues(rotation_vector,rotationMatrix);
        cor=-rotationMatrix.inv()*tvecs;
        if(solutions!=1)
        continue;
        out1<<std::setprecision(16)<<timeStamps[index]<<cor.at<float>(0)<<"\t"<<cor.at<float>(1)<<"\t"<<cor.at<float>(2)<<endl;
        //continue;   
    }

    cout<<"前"<<paths.size()<<"桢共识别出"<<total_codes<<"个二维码"<<endl;

#if 0
    //debug
    for(int i=0;i<codesInfo.size();i++)
    {
        cout<<"第"<<i<<"个二维码坐标 :("<<codesInfo[i][0]<<","<<codesInfo[i][1]<<","<<codesInfo[i][2]<<")"<<endl;
    }
    cout<<"FrameQRindexs.size() = "<<FrameQRindexs.size()<<endl;
    ofstream position_before("position_before.txt");
    ofstream position_after("position_after.txt");
    for(int i=0;i<poses.size();i++)
    {
        cout<<"poses: "<<fixed  <<poses[i][0]<<" "<<poses[i][1]<<" "<<poses[i][2]<<" "
                                <<poses[i][3]<<" "<<poses[i][4]<<" "<<poses[i][5]<<" "
                                <<poses[i][6]<<" "<<poses[i][7]<<" "<<poses[i][8]<<endl;
        position_before<<fixed  <<poses[i][3]<<" "<<poses[i][4]<<" "<<poses[i][5]<<"\n";
    }
#endif
    return 0;
}
//根据二维码编码方式进行解码
bool decodeCoord(string _info,Vec3f& point)
{
    if(_info.length()!=10)
    {
        cout<<"length incorrect;decoding failed!"<<endl;
        return false;
    }
    int flag_x=1,flag_y=1;
    if(_info[0]=='0')
    flag_x=-1;
    if(_info[5]=='0')
    flag_y=-1;
    point[0]=(double)flag_x*atoi(_info.substr(1,4).c_str())/1000.0;
    point[1]=(double)flag_y*atoi(_info.substr(6,4).c_str())/1000.0;
    point[2]=0.0;
    return true;
}
string decodeCoordView(string _info)
{
    if(_info.length()!=10)
    {
        cout<<"length incorrect;decoding failed!"<<endl;
        return "";
    }
    Vec3f point;
    int flag_x=1,flag_y=1;
    if(_info[0]=='0')
    flag_x=-1;
    if(_info[5]=='0')
    flag_y=-1;
    point[0]=(double)flag_x*atoi(_info.substr(1,4).c_str())/1000.0;
    point[1]=(double)flag_y*atoi(_info.substr(6,4).c_str())/1000.0;
    point[2]=0.0;
    string res="(";
    if(point[0]<0)
    res+="-"+to_string(abs(point[0]));
    else
    res+=to_string(abs(point[0]));
    res=res.substr(0,res.size()-4);
    res+=",";
    if(point[1]<0)
    res+="-"+to_string(abs(point[1]));
    else
    res+=to_string(abs(point[1]));
    res=res.substr(0,res.size()-4);
    res+=")";
    return res;
}