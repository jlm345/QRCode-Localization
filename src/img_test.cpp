#include "detect.h"

bool decodeCoord(string _info,Vec3f& point);
string decodeCoordView(string _info);

int main(int argc,char** argv)
{
    string path_img;
    if(argc==1)
    {
        path_img="1.jpg";
    }
    else
    {
        path_img=argv[1];
    }
    
    //uvc 相机参数
    //uvc 相机参数
    Mat distCoeffs(4,1,CV_32FC1);
    Mat cameraMatrix=Mat::zeros(3,3,CV_32FC1);
    //pinhole
    //fx
    cameraMatrix.at<float>(0,0)=1102.27949242;
    //cx
    cameraMatrix.at<float>(0,2)= 960.68274218;
    //fy
    cameraMatrix.at<float>(1,1)=1101.68693064;
    //cy
    cameraMatrix.at<float>(1,2)=560.07428354;
    cameraMatrix.at<float>(2,2)=1.0;
    //畸变系数,radtan模型
    distCoeffs.at<float>(0,0)=0.06224009809504005;
    distCoeffs.at<float>(0,1)=-0.08257649652392368;
    distCoeffs.at<float>(0,2)= -0.002623192556148226;
    distCoeffs.at<float>(0,3)=0.004323173038151403;
    cout<<"cameraMatrix = \n"<<cameraMatrix<<endl;

    Mat image_raw;
    //二维码信息参数
    vector<string> infos;//存放二维码的内容
    vector<vector<Vec2f>> positions;//对应地存放每个二维码的四个顶点坐标
    vector<QuaVert> quas_raw;   //二维码四个顶点的位置
    vector<QuaVert> quas_expand;//将四个顶点向外扩张后的四边形
    cout<<"reading img from path : "<<path_img<<endl;
    //以灰度方式读入原始图
    image_raw=imread(path_img,IMREAD_GRAYSCALE);
#if 0
    Mat img_1,img_4,img_hh;
    //img_hh=image_raw>119;
    adaptiveBinaryImg(image_raw,img_1,1);
    adaptiveBinaryImg(image_raw,img_4,4);
    cv::imwrite("img_1.jpg",img_1);
    cv::imwrite("img_4.jpg",img_4);
    cv::namedWindow("img_1",WINDOW_NORMAL);
    cv::namedWindow("img_4",WINDOW_NORMAL);
    cv::namedWindow("img_hh",WINDOW_NORMAL);
    cv::imshow("img_1",img_1);
    cv::imshow("img_4",img_4);
    cv::waitKey();
#endif

    //直方图均衡
    //equalizeHist(image_raw,image_raw);
    //cv::Ptr<cv::CLAHE> clahe=cv::createCLAHE(9.0,cv::Size(8,8));
	//clahe->apply(image_raw,image_raw);
    cv::namedWindow("image_raw",WINDOW_NORMAL);
    cv::imshow("image_raw",image_raw);
    cv::waitKey();
    if(image_raw.empty())
    {
        cout<<"reding img failed!"<<endl;
        return -1;
    }
    //开始解码
    detect_decode(image_raw,infos,positions,quas_raw,quas_expand,true);
    if(infos.size()!=positions.size()||positions.size()!=quas_raw.size())//对结果进行检测
    {
        cout<<"unmached msg!"<<endl;
        return -1;
    }
    if(infos.size()==0)
    {
        cout<<"no msg have been detected!"<<endl;
        return -1;
    }
    cout<<infos.size()<<" QR code(s) have been detected!"<<endl;

    //框出检测到的二维码
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
    cv::imwrite("result.jpg",img_view_decoding);
    cv::waitKey();

    //为P3P准备参数
    vector<Vec3f> objectPoints;
    vector<Vec2f> imagePoints;
    Mat rvec;
    Mat tvec;
    for(int i=0;i<positions.size();i++)
    {
        Vec3f objPoint;
        Vec2f imgPoint;
        decodeCoord(infos[i],objPoint);
        imgPoint[0]=2*960.68274218-positions[i][0][0];
        imgPoint[1]=2*560.07428354-positions[i][0][1];
        objectPoints.push_back(objPoint);
        imagePoints.push_back(imgPoint);
        cout<<"objPoint = "<<objPoint.t()<<endl;
        cout<<"imgPoint = "<<imgPoint.t()<<endl;
    }
    //求解pnp
    cout<<"objectPoints.size() = "<<objectPoints.size()<<endl;
    if(objectPoints.size()<4)
    {
        cout<<"No enough QR codes!"<<endl;
        return -1;
    }
    int solutions=solvePnP(objectPoints,imagePoints,cameraMatrix,cv::Mat(),rvec,tvec,0,SOLVEPNP_IPPE);
    cout<<"We have got "<<solutions<<" solutions"<<endl;
    if(solutions!=1)
    return 1;
    Mat rotationMatrix;
    cv::Rodrigues(rvec,rotationMatrix);
    cout<<"rvecs = "<<rvec.t()<<endl;
    cout<<"rotationMatrix :\n"<<rotationMatrix<<endl;
    cout<<"tvecs = "<<-rotationMatrix.inv()*tvec<<endl;
    cv::waitKey();
    return 1;
    
    /* 
    cout<<tvecs[i].t()<<endl;
    for(int i=0;i<tvecs.size();i++)
    {
        cout<<tvecs[i].t()<<endl;
    }
    ////显示检测之后的结果
    //二维码信息以及坐标
    for(int i=0;i<infos.size();i++)
    {
        cout<<"i = "<<i<<", info = "<<infos[i]
              <<" A : ("<<quas_raw[i].A.x<<","<<quas_raw[i].A.y<<")\t";
        cout<<"\t position[0] :"<<positions[i][0]<<endl;
    }
    return 1;*/
}
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