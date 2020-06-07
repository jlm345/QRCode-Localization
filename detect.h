#ifndef DETECT_H
#define DETECT_H

#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
using namespace std;

struct LINE{
    float A,B,C;
    //通过两点来构造一条直线
    LINE(Point p1,Point p2)
    {
        float x1=p1.x;
        float y1=p1.y;
        float x2=p2.x;
        float y2=p2.y;
        //先判断特殊情况
        if(fabs(x1-x2)<1e-3)
        {
            A=1.f;
            B=0.f;
            C=-x1;
            return;
        }
        if(fabs(y1-y2)<1e-3)
        {
            A=0.f;
            B=1.f;
            C=-y1;
            return;
        }
        //计算解析式,才发现，这个公式是通用的...
        A=y2-y1;
        B=x1-x2;
        C=-(A*x1+B*y1);
    }
    //通过赋值的方法建立直线
    LINE(float a,float b,float c):A(a),B(b),C(c)
    {
    }

    //判断点是否在法相量那一侧
    bool IsNormDirection(Point point)
    {
        float x=(float)point.x;
        float y=(float)point.y;
        return A*x+B*y+C>=0;
    }
    //反转直线的法相量
    void inverse()
    {
        A=-A;
        B=-B;
        C=-C;
    }
    //点p到该直线的距离公式
    float distance(Point p)
    {
        float x=p.x;
        float y=p.y;
        float num=fabs(A*x+B*y+C);
        //float den=sqrt(pow(x,2)+pow(y,2));
        float den=sqrt(pow(A,2)+pow(B,2));
        return (num/den);
    }
    //使直线沿着法相量的反方向后退距离d
    void back(float d)
    {
        float delta_C=d*sqrt(pow(A,2)+pow(B,2));
        //cout<<"delta_c = "<<delta_C<<endl;
        C+=delta_C;
    }
    float getXvalue(float y)
    {
        if(fabs(A)<1e-7)
            return -9999;
        return -(C+B*y)/A;
    }
    float getYvalue(float x)
    {
        if(fabs(B)<1e-7)
            return -9999;
        return -(C+A*x)/B;
    }
};

//按照ABDC的顺序组成一个四边形
struct QuaVert{
    Point A,B,D,C;
    bool existence=true;
};

Rect findRect(vector<Point>& contour, int img_width,int img_height);//寻找包围contour的旋转角度为0的最小矩形,且这个矩形不能超过图像边界
bool IsQrColorRate(cv::Mat& image);//横向和纵向黑白比例判断
bool IsQrPoint(vector<Point>& contour, Mat& img);
bool IsQrColorRateX(cv::Mat& image);//横向黑白比例判断
bool IsQrColorRateY(cv::Mat& image);//纵向黑白比例判断
bool IsQrRate(float rate);
double Distance_2(Point& A,Point& B);//求取两点距离的平方
float Distance(Point& A,Point& B);//两点距离
Point parallelogram(Point&A,Point&B,Point&C);//通过求到的四边形三个顶点得到另一个,A是二维码左上角，BC与A相邻
LINE vertical_line(Point& p1,Point& p2);//返回通过点p1且法相量由p1指向p2的直线方程
Point intersectionLines(LINE l1,LINE l2);//求两条直线的交点
void getQuaVert(cv::Mat& img, vector<vector<Point>>& boxes, QuaVert &qua_raw, QuaVert &qua_expand);//获取三个定位标对应的四边形的四个顶点
void decodeImg(cv::Mat& img, vector<vector<Point>>& boxes, vector<string>& infos, vector<vector<Vec2f>>& positions
               ,vector<QuaVert>& quas_raw,vector<QuaVert>& quas_expand,bool ROI_SHOW=false);
Point centerPosition(vector<Point>& points);//获取一组点的中心点
bool decodeQua(Mat& img,QuaVert qua,string& info,vector<Vec2f>& points,bool ROI_SHOW=false);//从四个顶点定义的四边形中检测二维码
bool cmp(const pair<int,float> a, const pair<int,float> b);
bool cmp_float(float a, float b);
void preProcessing(const cv::Mat& img_raw,cv::Mat& img_binary);//图像预处理
//para:
//infos:每个二维码对应的信息
//positions:由Opencv提取到的二维码四个角点位置
//quas_raw：由几何法得到的二维码四个角点位置
//quas_expand：向外扩张的四个角点位置
void detect_decode(cv::Mat& img,vector<string>& infos,vector<vector<Vec2f>>& positions
                   ,vector<QuaVert>& quas_raw,vector<QuaVert>& quas_expand,bool DetailedInfo=false);
//检查四边形中是否包含除了组成四边形的三个定位标之外的定位标
bool IsEffectiveQua(QuaVert& qua,vector<Point>& centers,vector<int>& idxs);
//返回三个点组成的三角形的最大角度
float maxAngle(vector<Point> points);
void adaptiveBinaryImg(cv::Mat& src,cv::Mat& dst,int size=8);//将图片分成16块，分别进行二值化
void averageBinary(cv::Mat& src);//通过图像均值来二值化图像
bool isValid(cv::Mat& img,vector<Vec2f>& points);//判断这四个点是否都在图像内部
#endif