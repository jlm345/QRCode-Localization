#include "detect.h"
#include <fstream>

#include "ceres/ceres.h"
#include "ceres/rotation.h"

bool decodeCoord(string _info,Vec3f& point);
bool isFound(const vector<double*>& _codesInfo,const double* _code,int& index);
string decodeCoordView(string _info);

// Templated pinhole camera model for used with Ceres.  The pose is
// parameterized using 6 parameters: 3 for rotation, 3 for translation
struct SnavelyReprojectionError {
    SnavelyReprojectionError(double observed_x, double observed_y,double *K)
      : observed_x(observed_x), observed_y(observed_y),intrinsic(K){}

    template <typename T>
    bool operator()(const T* const pose,
                    const T* const point,
                    T* residuals) const {
    // camera[0,1,2] are the angle-axis rotation.
    T p[3];
    ceres::AngleAxisRotatePoint(pose, point, p);

    // camera[3,4,5] are the translation.
    //rotate to camera frame
    p[0] += pose[3];
    p[1] += pose[4];
    p[2] += pose[5];

    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    T xp =p[0] / p[2];
    T yp =p[1] / p[2];

    // Compute final projected point position.
    T predicted_x = intrinsic[2]+intrinsic[0] * xp;
    T predicted_y = intrinsic[3]+intrinsic[1] * yp;

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - observed_x;
    residuals[1] = predicted_y - observed_y;

    return true;
  }
  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y,
                                     double *K) {
    return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 6, 3>(
                new SnavelyReprojectionError(observed_x, observed_y,K)));
  }

  double observed_x;
  double observed_y;
  double *intrinsic;//fx，fy，cx,cy
};

// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 6 parameters: 3 for rotation, 3 for translation
struct SnavelyReprojectionErrorFixed {
    SnavelyReprojectionErrorFixed(double observed_x, double observed_y,const double* const p,const double* const K)
      : observed_x(observed_x), observed_y(observed_y),point(p),intrinsic(K) {}

    template <typename T>
    bool operator()(const T* const camera,
                    T* residuals) const {
    // camera[0,1,2] are the angle-axis rotation.
    T p[3];
    const T* const pp=(T*)point;
    ceres::AngleAxisRotatePoint(camera, pp, p);

    // camera[3,4,5] are the translation.
    //rotate to camera frame
    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];

    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    T xp =p[0] / p[2];
    T yp =p[1] / p[2];

    // Compute final projected point position.
    T predicted_x = intrinsic[2]+intrinsic[0] * xp;
    T predicted_y = intrinsic[3]+intrinsic[1] * yp;

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - observed_x;
    residuals[1] = predicted_y - observed_y;

    return true;
  }
  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y,
                                     const double* const p,
                                     const double* const K) {
    return (new ceres::AutoDiffCostFunction<SnavelyReprojectionErrorFixed, 2, 6>(
                new SnavelyReprojectionErrorFixed(observed_x, observed_y,p,K)));
  }

  double observed_x;
  double observed_y;
  const double* const point;
  const double* const intrinsic;//fx，fy，cx,cy
};

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

    std::ofstream out1("position_beforeBA.txt");
    std::ofstream out2("position_afterBA.txt");

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

#if 0
    //uvc 200万像素相机参数
    Mat distCoeffs(4,1,CV_32FC1);
    Mat cameraMatrix=Mat::zeros(3,3,CV_32FC1);
    double intrinsic[4]={1102.27949242,1101.68693064,960.68274218,560.07428354};
    //pinhole
    double fx=1102.27949242;
    double fy=1101.68693064;
    double cx=960.68274218;
    double cy=560.07428354;
    double f=0.5*(fx+fy);
    cameraMatrix.at<float>(0,0)=(float)fx;//fx
    cameraMatrix.at<float>(0,2)=(float)cx;//cx
    cameraMatrix.at<float>(1,1)=(float)fy;//fy
    cameraMatrix.at<float>(1,2)=(float)cy;//cy 
    cameraMatrix.at<float>(2,2)=1.0;
#endif
#if 0
    //畸变系数,radtan模型 200万像素相机
    distCoeffs.at<float>(0,0)=0.06224009809504005;
    distCoeffs.at<float>(0,1)=-0.08257649652392368;
    distCoeffs.at<float>(0,2)= -0.002623192556148226;
    distCoeffs.at<float>(0,3)=0.004323173038151403;
#endif

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

#if 0   //debug

        cout<<"We have got "<<solutions<<" solutions"<<endl;
        cout<<"rotation_vector = "<<rotation_vector.t()<<endl;
        cout<<"coordinate = "<<cor.t()<<endl;

        //打印预测的二维码的信息和预测信息
        cout<<"rotationMatrix = "<<rotationMatrix<<endl;
        cout<<"tvecs = "<<tvecs.t()<<endl;
        cout<<"objectPoints: "<<endl;
        for(auto x:objectPoints)
        cout<<x[0]<<" "<<x[1]<<" "<<x[2]<<endl;
        cout<<"imagePoints: "<<endl;
        for(auto x:imagePoints)
        cout<<x[0]<<"\t"<<x[1]<<endl;
        cout<<"predicted msg :"<<endl;
        for(auto x:objectPoints){
            Mat translation(3,1,CV_32F);
            translation.at<float>(0)=tvecs[0];
            translation.at<float>(1)=tvecs[1];
            translation.at<float>(2)=tvecs[2];
            Mat obj(3,1,CV_32F);
            obj.at<float>(0)=x[0];
            obj.at<float>(1)=x[1];
            obj.at<float>(2)=x[2];
            //cout<<"translation.type() = "<<translation.type()<<endl;
            //cout<<"obj.type() = "<<obj.type()<<endl;
            //cout<<"rotationMatrix.type() = "<<rotationMatrix.type()<<endl;
            Mat predicted(3,1,CV_32F);
            predicted=cameraMatrix*(rotationMatrix*obj+translation);
            predicted.at<float>(0)/=predicted.at<float>(2);
            predicted.at<float>(1)/=predicted.at<float>(2);
            cout<<"predicted u,v  = "<<predicted.at<float>(0)<<","<<predicted.at<float>(1)<<endl;
        }
#endif

        //添加每一桢的观测值，添加新的二维码信息
        indexs.push_back(index);
        double* pose=new double[9];
        pose[0]=(double)rotation_vector[0];
        pose[1]=(double)rotation_vector[1];
        pose[2]=(double)rotation_vector[0];
        pose[3]=(double)tvecs[0];
        pose[4]=(double)tvecs[1];
        pose[5]=(double)tvecs[2];
        pose[6]=(double)f;
        pose[7]=(double)cx;
        pose[8]=(double)cy;
        poses.push_back(pose);
        vector<pair<int,Vec2d>> perception;
        cout<<"objectPoints.size() = "<<objectPoints.size()<<endl;
        cout<<"poses: "<<fixed  <<pose[0]<<" "<<pose[1]<<" "<<pose[2]<<" "
                                <<pose[3]<<" "<<pose[4]<<" "<<pose[5]<<" "
                                <<pose[6]<<" "<<pose[7]<<" "<<pose[8]<<endl;
        for(int i=0;i<objectPoints.size();i++)
        {
            //添加新的二维码
            double posQR[3]={objectPoints[i][0],objectPoints[i][1],objectPoints[i][2]};
            int QRIndex=-1;
            bool QRCodeFound=isFound(codesInfo,posQR,QRIndex);
            if(!QRCodeFound)
            {
                QRIndex=codesInfo.size();
                double* pose_new=new double[3];
                pose_new[0]=posQR[0];
                pose_new[1]=posQR[1];
                pose_new[2]=posQR[2];
                codesInfo.push_back(pose_new);
            }
            perception.push_back(make_pair(QRIndex,imagePoints[i]));
        }
        FrameQRindexs.push_back(perception);
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

#if 1

    // Create residuals for each observation in the bundle adjustment problem. The
    // parameters for cameras and points are added automatically.
    ceres::Problem problem;
    for(int i=0;i<indexs.size();i++)
    {
        for(int j=0;j<FrameQRindexs[i].size();j++)
        {
            //fix 前三个二维码的世界坐标
            if(FrameQRindexs[i][j].first<3)
            {
                continue;
                ceres::CostFunction* cost_function =
                SnavelyReprojectionErrorFixed::Create(FrameQRindexs[i][j].second[0],
                                            FrameQRindexs[i][j].second[1],codesInfo[FrameQRindexs[i][j].first],intrinsic);
                problem.AddResidualBlock(cost_function,
                                new ceres::CauchyLoss(0.5),
                                poses[i]
                                );
            }
            else
            {
                ceres::CostFunction* cost_function =
                SnavelyReprojectionError::Create(FrameQRindexs[i][j].second[0],
                                            FrameQRindexs[i][j].second[1],intrinsic);
                problem.AddResidualBlock(cost_function,
                                new ceres::CauchyLoss(0.5) ,
                                poses[i],
                                codesInfo[FrameQRindexs[i][j].first]);
            }
        }
    }
    // Make Ceres automatically detect the bundle structure. Note that the
    // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
    // for standard bundle adjustment problems.
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    //options.function_tolerance = 1e-3;
    options.max_num_iterations=2000;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";

    //debug
    cout<<"经过优化后"<<endl;
    for(int i=0;i<codesInfo.size();i++)
    {
        cout<<"第"<<i<<"个二维码坐标 :("<<codesInfo[i][0]<<","<<codesInfo[i][1]<<","<<codesInfo[i][2]<<")"<<endl;
    }
    cout<<"FrameQRindexs.size() = "<<FrameQRindexs.size()<<endl;
    for(int i=0;i<poses.size();i++)
    {
        cv::Mat rvec(3,1,CV_64F),tvec(3,1,CV_64F);
        cv::Mat rotation;
        rvec.at<double>(0)=poses[i][0];
        rvec.at<double>(1)=poses[i][1];
        rvec.at<double>(2)=poses[i][2];
        tvec.at<double>(0)=poses[i][0];
        tvec.at<double>(1)=poses[i][1];
        tvec.at<double>(2)=poses[i][2];
        cv:Rodrigues(rvec,rotation);
        cv::Mat translation=-rotation.inv()*tvec;
        out2<<translation.at<float>(0)<<"\t"<<translation.at<float>(1)<<"\t"<<translation.at<float>(2)<<endl;
    }
#endif
    return 0;
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
bool isFound(const vector<double*>& _codesInfo,const double* _code,int& index)
{
    bool Found=false;
    for(int i=0;i<_codesInfo.size();i++)
    {
        double* p=_codesInfo[i];
        double distance_2=pow(_code[0]-p[0],2)+pow(_code[1]-p[1],2)+pow(_code[2]-p[2],2);
        if(distance_2<1e-8)
        {
            Found=true;
            index=i;
            break;
        }
    }
    if(Found)
    return true;
    else
    return false;
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