#include "detect.h"
bool cmp(const pair<int,float> a, const pair<int,float> b) {
    return a.second<b.second;//自定义的比较函数,按float值由小到大排列
}
bool cmp_float(float a, float b) {
    return a>b;//自定义的比较函数,由大到小排列
}
bool IsQrPoint(vector<Point>& contour, Mat& img)
{
    //To do: 这种截取不是最小的覆盖矩形
    int img_width=img.cols;
    int img_height=img.rows;
    Rect minRect=findRect(contour,img_width,img_height);
    //RotatedRect rotatedRect = minAreaRect(contour);
    if (minRect.height < 10 || minRect.width < 10)
        return false;

    //将二维码从整个图上抠出来
    cv::Mat cropImg=img(minRect);
    
    //cout<<"正在提取的回形轮廓的一个坐标为 "<<"("<<contour[0].x<<","<<contour[0].y<<")\n";
    //cv::namedWindow("截取回形图标的正放的长方形",WINDOW_NORMAL);
    //cv::imshow("截取回形图标的正放的长方形",cropImg);
    /*cv::waitKey();
    cv::destroyWindow("截取回形图标的正放的长方形");
    */
    //横向黑白比例1:1:3:1:1
    bool result = IsQrColorRate(cropImg);
    return result;
}

//横向和纵向黑白比例判断
bool IsQrColorRate(cv::Mat& image)
{
    bool flag_x=IsQrColorRateX(image);
    if (!flag_x)
        return false;
    bool flag_y=IsQrColorRateY(image);
    if(!flag_y)
        return false;
    return true;
}
//横向黑白比例判断
bool IsQrColorRateX(cv::Mat& image)
{
    int nr = image.rows / 2;
    int nc = image.cols * image.channels();

    vector<int> vValueCount;
    vector<uchar> vColor;
    int count = 0;
    uchar lastColor = 0;

    uchar* data = image.ptr<uchar>(nr);
    for (int i = 0; i < nc; i++)
    {
        vColor.push_back(data[i]);
        uchar color = data[i];
        if (color > 0)
            color = 255;

        if (i == 0)
        {
            lastColor = color;
            count++;
        }
        else
        {
            if (lastColor != color)
            {
                vValueCount.push_back(count);
                count = 0;
            }
            count++;
            lastColor = color;
        }
    }

    if (count != 0)
        vValueCount.push_back(count);

    if (vValueCount.size() < 5||vValueCount.size()>11)
    {
        //cout<<"X方向的判断：颜色块不足五个，不是一个定位标!"<<endl;
        return false;
    }
    //横向黑白比例1:1:3:1:1
    int index = -1;//index用来表示最中间的那一块黑色
    int maxCount = -1;//直线经过最长距离的那一部分对应的应该是maxCount
    for (int i = 2; i < vValueCount.size(); i++)
    {
        if (i == 2)
        {
            index = i;
            maxCount = vValueCount[i];
        }
        else
        {
            if (vValueCount[i] > maxCount)
            {
                index = i;
                maxCount = vValueCount[i];
            }
        }
    }

    //debug
    //cout<<"index = "<<index<<"\nmaxCount = "<<maxCount<<endl;
    //cout<<endl<<"vValueCount : "<<endl;
    //for(int i=0;i<vValueCount.size();i++)
    //{
    //    cout<<"vValueCount["<<i<<"] = "<<vValueCount[i]<<endl;
    //}
    //cout<<endl;

    //左边 右边 都有两个值，才行
    if (index < 2)
    {
        //cout<<"X方向的判断：黑色块左边没有两个颜色块"<<endl;
        return false;
    }
    if ((vValueCount.size() - index) < 3)
    {
        //cout<<"X方向的判断：黑色块右边没有两个颜色块"<<endl;
        return false;
    }    

    //黑白比例1:1:3:1:1
    float rate = ((float)maxCount) / 3.00;

    //cout << "flag:" << flag << " ";

    float rate2 = vValueCount[index - 2] / rate;
    //cout << rate2 << " ";
    if (!IsQrRate(rate2))
    {
        //cout<<"X方向的判断：最左边的那一块与中间块三分之一长度的比值，不满足要求,rate2 = "<<rate2<<endl;
        return false;
    }

    rate2 = vValueCount[index - 1] / rate;
    //cout << rate2 << " ";
    if (!IsQrRate(rate2))
    {
        //cout<<"X方向的判断：左边靠近中间的一块与中间块三分之一长度的比值，不满足要求,rate2 = "<<rate2<<endl;
        return false;
    }

    rate2 = vValueCount[index + 1] / rate;
    //cout << rate2 << " ";
    if (!IsQrRate(rate2))
    {
        //cout<<"X方向的判断：右边靠近中间的一块与中间块三分之一长度的比值，不满足要求,rate2 = "<<rate2<<endl;
        return false;
    }

    rate2 = vValueCount[index + 2] / rate;
    //cout << rate2 << " ";
    if (!IsQrRate(rate2))
    {
        //cout<<"X方向的判断：最右边的那一块与中间块三分之一长度的比值，不满足要求,rate2 = "<<rate2<<endl;
        return false;
    }

    return true;
}
//纵向黑白比例判断
bool IsQrColorRateY(cv::Mat& image)
{
    int row = image.rows;
    int nc = image.cols * image.channels()/2;

    vector<int> vValueCount;
    vector<uchar> vColor;
    int count = 0;
    uchar lastColor = 0;

    for (int i = 0; i < row; i++)
    {
        vColor.push_back(image.at<char>(i,nc));
        uchar color = image.at<char>(i,nc);
        if (color > 0)
            color = 255;

        if (i == 0)
        {
            lastColor = color;
            count++;
        }
        else
        {
            if (lastColor != color)
            {
                vValueCount.push_back(count);
                count = 0;
            }
            count++;
            lastColor = color;
        }
    }

    if (count != 0)
        vValueCount.push_back(count);

    if (vValueCount.size() < 5)
    {
        //cout<<"颜色块不足五个，不是一个定位标!"<<endl;
        return false;
    }
    //横向黑白比例1:1:3:1:1
    int index = -1;//index用来表示最中间的那一块黑色
    int maxCount = -1;//直线经过最长距离的那一部分对应的应该是maxCount
    for (int i = 2; i < vValueCount.size(); i++)
    {
        if (i == 2)
        {
            index = i;
            maxCount = vValueCount[i];
        }
        else
        {
            if (vValueCount[i] > maxCount)
            {
                index = i;
                maxCount = vValueCount[i];
            }
        }
    }

    //左边 右边 都有两个值，才行
    if (index < 2)
    {
        //cout<<"Y方向的判断：黑色块左边没有两个颜色块"<<endl;
        return false;
    }
    if ((vValueCount.size() - index) < 3)
    {
        //cout<<"Y方向的判断：黑色块右边没有两个颜色块"<<endl;
        return false;
    }    

    //黑白比例1:1:3:1:1
    float rate = ((float)maxCount) / 3.00;

    //cout << "flag:" << flag << " ";

    float rate2 = vValueCount[index - 2] / rate;
    //cout << rate2 << " ";
    if (!IsQrRate(rate2))
    {
        //cout<<"Y方向的判断：最左边的那一块与中间块三分之一长度的比值，不满足要求,rate2 = "<<rate2<<endl;
        return false;
    }

    rate2 = vValueCount[index - 1] / rate;
    //cout << rate2 << " ";
    if (!IsQrRate(rate2))
    {
        //cout<<"Y方向的判断：左边靠近中间的一块与中间块三分之一长度的比值，不满足要求,rate2 = "<<rate2<<endl;
        return false;
    }

    rate2 = vValueCount[index + 1] / rate;
    //cout << rate2 << " ";
    if (!IsQrRate(rate2))
    {
        //cout<<"Y方向的判断：右边靠近中间的一块与中间块三分之一长度的比值，不满足要求,rate2 = "<<rate2<<endl;
        return false;
    }

    rate2 = vValueCount[index + 2] / rate;
    //cout << rate2 << " ";
    if (!IsQrRate(rate2))
    {
        //cout<<"Y方向的判断：最右边的那一块与中间块三分之一长度的比值，不满足要求,rate2 = "<<rate2<<endl;
        return false;
    }

    return true;
}

//寻找包围contour的旋转角度为0的最小矩形
Rect findRect(vector<Point>& contour, int img_width,int img_height)
{
    int min_x=9999;
    int min_y=9999;
    int max_x=0;
    int max_y=0;
    for(int i=0;i<contour.size();i++)
    {
        min_x=min_x<contour[i].x?min_x:contour[i].x;
        min_y=min_y<contour[i].y?min_y:contour[i].y;
        max_x=max_x>contour[i].x?max_x:contour[i].x;
        max_y=max_y>contour[i].y?max_y:contour[i].y;
    }
    min_x=std::max(min_x,0);
    min_y=std::max(min_y,0);
    int width=std::min(max_x-min_x,img_width-min_x);
    int height=std::min(max_y-min_y,img_height-min_y);
    Rect rect(min_x,min_y,width,height);
    return rect;
}

bool IsQrRate(float rate)
{
     //大概比例 不能太严格
    return rate > 0.3 && rate < 1.9;
}

//希望输入进来的是个灰度图像
void getQuaVert(cv::Mat& img,vector<vector<Point>>& boxes,QuaVert& qua_raw,QuaVert& qua_expand)
{
    //To Do 对输入图片的类型进行判断，需要一个灰度图
    if(img.type() != CV_8UC1)
    {
        qua_raw.existence=false;
        qua_expand.existence=false;
        cout<<"please input a gray image!"<<endl;
        return;
    }
    //需要的是三个定位标
    if(boxes.size()!=3)
    {
        cout<<"boxes should equal 3"<<endl;
        qua_raw.existence=false;
        qua_expand.existence=false;
        return;
    }

    //对每个定位标，如果拐点数量不够12个，就进行扩充，最多两次
    for(int i=0;i<boxes.size();i++)
    {
        if(boxes[i].size()<12)
        {
            vector<Point> contour_temp;
            for(int j=0;j<boxes[i].size();j++)
            {
                if(j<boxes[i].size()-1)
                {
                    contour_temp.push_back(boxes[i][j]);
                    Point temp;
                    temp.x=0.5*(float)(boxes[i][j].x+boxes[i][j+1].x);
                    temp.y=0.5*(float)(boxes[i][j].y+boxes[i][j+1].y);
                    contour_temp.push_back(temp);
                }
                else
                    contour_temp.push_back(boxes[i][j]);
            }
            boxes[i].clear();
            boxes[i].insert(boxes[i].begin(),contour_temp.begin(),contour_temp.end());
        }
        if(boxes[i].size()<12)
        {
            vector<Point> contour_temp;
            for(int j=0;j<boxes[i].size();j++)
            {
                if(j<boxes[i].size()-1)
                {
                    contour_temp.push_back(boxes[i][j]);
                    Point temp;
                    temp.x=0.5*(float)(boxes[i][j].x+boxes[i][j+1].x);
                    temp.y=0.5*(float)(boxes[i][j].y+boxes[i][j+1].y);
                    contour_temp.push_back(temp);
                }
                else
                    contour_temp.push_back(boxes[i][j]);
            }
            boxes[i].insert(boxes[i].begin(),contour_temp.begin(),contour_temp.end());
        }
    }

    //对三个定位标轮廓分别求中心点
    vector<Point> averagePositions;
    for(int idx_box=0;idx_box<boxes.size();idx_box++)
    {
        Point averagePosition;
        double x=0.0;
        double y=0.0;
        double N=boxes[idx_box].size();
        for(int idx_point=0;idx_point<boxes[idx_box].size();idx_point++)
        {
            x+=(double)boxes[idx_box][idx_point].x/N;
            y+=(double)boxes[idx_box][idx_point].y/N;
        }
        //cout<<"x = "<<x<<" y = "<<y<<endl;
        averagePosition.x=x;
        averagePosition.y=y;
        averagePositions.push_back(averagePosition);
    }

    /*
    //打印中心坐标
    cout<<"centers: ";
    for(int i=0;i<averagePositions.size();i++)
    {
        cout<<"("<<averagePositions[i].x<<","<<averagePositions[i].y<<") ";
    }
    cout<<endl;
    */

    //定位左上角的那个定位标
    int up_left_idx=0;
    int other_idx_1,other_idx_2;
    double distance_01=0.0;
    double distance_02=0.0;
    double distance_12=0.0;
    distance_01=Distance_2(averagePositions[0],averagePositions[1]);
    distance_02=Distance_2(averagePositions[0],averagePositions[2]);
    distance_12=Distance_2(averagePositions[1],averagePositions[2]);
    if(distance_01>distance_02&&distance_01>distance_12)
    {
        up_left_idx=2;
        other_idx_1=0;
        other_idx_2=1;
    }
    else if(distance_02>distance_12)
    {
        up_left_idx=1;
        other_idx_1=0;
        other_idx_2=2;
    }
    else
    {
        up_left_idx=0;
        other_idx_1=1;
        other_idx_2=2;
    }
    //cerr<<"up_left_center : "<<"("<<averagePositions[up_left_idx].x<<","<<averagePositions[up_left_idx].y<<")"<<endl;

    //变量定义
    //三个路标对应的中心点
    Point A=averagePositions[up_left_idx];//A代表左上角定位标轮廓的中心点
    Point B=averagePositions[other_idx_1];
    Point C=averagePositions[other_idx_2];
    //三个轮廓对应的辅助点
    Point temp_A_Bside,temp_A_Cside,temp_B,temp_C;
    //四边形的四个顶点
    Point corner_A,corner_B,corner_C,corner_D;
    float distance_min=9999.f;
    float distance_max=0.f;

    //temp_A_Bside,temp_A_Cside
    LINE vertical_2_AB=vertical_line(A,B);
    LINE center_AB(A,B);
    LINE vertical_2_AC=vertical_line(A,C);
    LINE center_AC(A,C);
    int idx_temp_A=0;
    distance_min=9999.f;
    for(int i=0;i<boxes[up_left_idx].size();i++)
    {
        Point temp=boxes[up_left_idx][i];
        if(vertical_2_AB.IsNormDirection(temp))
            continue;
        //To do 如果有大量点到直线距离公式的计算，可以只计算公式分子的部分
        //To do 或者可能的简单的方法就是只计算ax+by+c的值用来计较最大值
        float dis=center_AB.distance(temp);
        //cout<<"dis = "<<dis<<endl;
        if(dis<distance_min)
        {
            distance_min=dis;
            idx_temp_A=i;
        }
    }
    temp_A_Cside=boxes[up_left_idx][idx_temp_A];
    idx_temp_A=0;
    distance_min=9999.f;
    for(int i=0;i<boxes[up_left_idx].size();i++)
    {
        Point temp=boxes[up_left_idx][i];
        if(vertical_2_AC.IsNormDirection(temp))
            continue;
        float dis=center_AC.distance(temp);
        if(dis<distance_min)
        {
            distance_min=dis;
            idx_temp_A=i;
        }
    }
    temp_A_Bside=boxes[up_left_idx][idx_temp_A];
    //B对应的顶点corner_B,B轮廓中，离直线center_BC最近的那个点
    distance_min=9999.f;
    int idx_corner_B=0;
    LINE vertical_2_BC=vertical_line(B,C);
    LINE center_BC(B,C);
    for(int i=0;i<boxes[other_idx_1].size();i++)
    {
        Point temp=boxes[other_idx_1][i];
        if(vertical_2_BC.IsNormDirection(temp))
            continue;
        float dis=center_BC.distance(temp);
        if(dis<distance_min)
        {
            distance_min=dis;
            idx_corner_B=i;
        }
    }
    corner_B=boxes[other_idx_1][idx_corner_B];
    //C对应的顶点
    LINE vertical_2_CB=vertical_line(C,B);
    int idx_corner_C=0;
    distance_min=9999.f;
    for(int i=0;i<boxes[other_idx_2].size();i++)
    {
        Point temp=boxes[other_idx_2][i];
        if(vertical_2_CB.IsNormDirection(temp))
            continue;
        float dis=center_BC.distance(temp);
        if(dis<distance_min)
        {
            distance_min=dis;
            idx_corner_C=i;
        }
    }
    corner_C=boxes[other_idx_2][idx_corner_C];
    //A对应的顶点.通过temp_AB和temp_AC的交点得到corner_A
    LINE temp_AB(corner_B,temp_A_Bside);
    LINE temp_AC(corner_C,temp_A_Cside);
    //cout<<"temp_AB A,B,C : "<<temp_AB.A<<","<<temp_AB.B<<","<<temp_AB.C<<endl;
    //cout<<"temp_AC A,B,C : "<<temp_AC.A<<","<<temp_AC.B<<","<<temp_AC.C<<endl;
    corner_A=intersectionLines(temp_AB,temp_AC);
    //求temp_B,离直线BC距离最远且不在A点那侧的点
    LINE corner_BC(corner_B,corner_C);
    //使直线法相量朝着没有点corner_A的那一侧
    if(corner_BC.IsNormDirection(corner_A))
        corner_BC.inverse();
    int idx_temp_B=0;
    distance_max=0.f;
    for(int i=0;i<boxes[other_idx_1].size();i++)
    {
        Point temp=boxes[other_idx_1][i];
        if(!corner_BC.IsNormDirection(temp))
            continue;
        float dis=corner_BC.distance(temp);
        if(dis>distance_max)
        {
            distance_max=dis;
            idx_temp_B=i;
        }
    }
    temp_B=boxes[other_idx_1][idx_temp_B];
    //类似地求temp_C
    int idx_temp_C=0;
    distance_max=0.f;
    for(int i=0;i<boxes[other_idx_2].size();i++)
    {
        Point temp=boxes[other_idx_2][i];
        if(!corner_BC.IsNormDirection(temp))
            continue;
        float dis=corner_BC.distance(temp);
        if(dis>distance_max)
        {
            distance_max=dis;
            idx_temp_C=i;
        }
    }
    temp_C=boxes[other_idx_2][idx_temp_C];
    //通过ABC三点求点D
    //corner_D=parallelogram(A,B,C);
    //打印四个中心点的信息
    //cout<<"A : ("<<A.x<<","<<A.y<<")"<<endl;
    //cout<<"B : ("<<B.x<<","<<B.y<<")"<<endl;
    //cout<<"C : ("<<C.x<<","<<C.y<<")"<<endl;
    //cout<<"D : ("<<D.x<<","<<D.y<<")"<<endl;
    //D对应的顶点
    LINE temp_BD(corner_B,temp_B);
    LINE temp_CD(corner_C,temp_C);
    //cout<<"temp_BD A,B,C : "<<temp_BD.A<<","<<temp_BD.B<<","<<temp_BD.C<<endl;
    //cout<<"temp_CD A,B,C : "<<temp_CD.A<<","<<temp_CD.B<<","<<temp_CD.C<<endl;
    corner_D=intersectionLines(temp_BD,temp_CD);
    //cout<<"temp_A_Cside : ("<<temp_A_Cside.x<<","<<temp_A_Cside.y<<")"<<endl;
    //cout<<"temp_A_Bside : ("<<temp_A_Bside.x<<","<<temp_A_Bside.y<<")"<<endl;
    //cout<<"temp_B : ("<<temp_B.x<<","<<temp_B.y<<")"<<endl;
    //cout<<"temp_C : ("<<temp_C.x<<","<<temp_C.y<<")"<<endl;
    //cout<<"corner_A : ("<<corner_A.x<<","<<corner_A.y<<")"<<endl;
    //cout<<"corner_B : ("<<corner_B.x<<","<<corner_B.y<<")"<<endl;
    //cout<<"corner_C : ("<<corner_C.x<<","<<corner_C.y<<")"<<endl;
    //cout<<"corner_D : ("<<corner_D.x<<","<<corner_D.y<<")"<<endl;

    //构建四条边的直线方程
    LINE corner_AB(corner_A,corner_B);
    LINE corner_AC(corner_A,corner_C);
    LINE corner_BD(corner_B,corner_D);
    LINE corner_CD(corner_C,corner_D);
    //调整法相量，使得其朝向四边形内部
    if(!corner_AB.IsNormDirection(corner_D))
        corner_AB.inverse();
    if(!corner_AC.IsNormDirection(corner_B))
        corner_AC.inverse();
    if(!corner_BD.IsNormDirection(corner_C))
        corner_BD.inverse();
    if(!corner_CD.IsNormDirection(corner_A))
        corner_CD.inverse();
    //cout<<"corner_AB A,B,C : "<<corner_AB.A<<","<<corner_AB.B<<","<<corner_AB.C<<endl;
    //cout<<"corner_AC A,B,C : "<<corner_AC.A<<","<<corner_AC.B<<","<<corner_AC.C<<endl;
    //cout<<"corner_BD A,B,C : "<<corner_BD.A<<","<<corner_BD.B<<","<<corner_BD.C<<endl;
    //cout<<"corner_CD A,B,C : "<<corner_CD.A<<","<<corner_CD.B<<","<<corner_CD.C<<endl;
    //cout<<endl<<endl;
    //对范围进行扩张,向外扩张使得四边形的面积包含住二维码
    float ratio=0.5;
    float d_line=ratio*sqrt(pow(corner_A.x-corner_B.x,2)+pow(corner_A.y-corner_B.y,2));
    float extension=max(10.f,d_line);
    //cout<<"extension = "<<extension<<endl;
    corner_AB.back(extension);
    corner_AC.back(extension);
    corner_BD.back(extension);
    corner_CD.back(extension);

    //给qua_raw四个顶点赋值
    qua_raw.existence=true;
    qua_raw.A=corner_A;
    qua_raw.B=corner_B;
    qua_raw.C=corner_C;
    qua_raw.D=corner_D;

    //获取扩张后的四边形四个顶点
    qua_expand.existence=true;
    qua_expand.A=intersectionLines(corner_AB,corner_AC);
    qua_expand.B=intersectionLines(corner_AB,corner_BD);
    qua_expand.C=intersectionLines(corner_AC,corner_CD);
    qua_expand.D=intersectionLines(corner_BD,corner_CD);
}

double Distance_2(Point& A,Point& B)
{
    double dis=0.0;
    dis=pow(A.x-B.x,2)+pow(A.y-B.y,2);
    return dis;
}

float Distance(Point& A,Point& B)
{
    float dis=sqrt((float)pow(A.x-B.x,2)+(float)pow(A.y-B.y,2));
    return dis;
}

Point parallelogram(Point&A,Point& B,Point& C)
{
    Point D;
    D.x=B.x+C.x-A.x;
    D.y=B.y+C.y-A.y;
    return D;
}

LINE vertical_line(Point& p1,Point& p2)
{
    float a,b,c;
    a=(float)(p2.x-p1.x);
    b=(float)(p2.y-p1.y);
    c=-(float)(a*p1.x+b*p1.y);
    LINE line(a,b,c);
    return line;
}

Point intersectionLines(LINE l1,LINE l2)
{
    float x,y,z;
    x=l1.B*l2.C-l1.C*l2.B;
    y=-(l1.A*l2.C-l1.C*l2.A);
    z=l1.A*l2.B-l1.B*l2.A;
    if(fabs(z)<1e-7)
    {
        cout<<"error:two lines is parallel!"<<endl;
        return Point(-99999,-99999);
    }
    return Point(x/z,y/z);
}
void decodeImg(cv::Mat& img, vector<vector<Point>>& boxes, vector<string>& infos, vector<vector<Vec2f>>& positions
               ,vector<QuaVert>& quas_raw,vector<QuaVert>& quas_expand,bool ROI_SHOW)
{
    if(img.type() != CV_8UC1)
    {
        cout<<"please input a gray image!"<<endl;
        return;
    }

    infos.clear();
    positions.clear();
    quas_raw.clear();
    quas_expand.clear();

    //少于三个定位标，则没有二维码
    if(boxes.size()<3)
    {
        cout<<"size of boxes less than 3"<<endl;
        return;
    }

    cv::QRCodeDetector qr;
    //如果只有三个定位标,直接进行检测
    vector<Vec2f> points_3_boxes;
    string info_3_boxes;
    if(boxes.size()==3)
    {
        info_3_boxes=qr.detectAndDecode(img,points_3_boxes);
        cout<<"info_3_boxes = "<<info_3_boxes<<endl;
        if(info_3_boxes.length()>0)
        {
            infos.push_back(info_3_boxes);
            positions.push_back(points_3_boxes);
            QuaVert q;
            q.A.x=points_3_boxes[0][0];
            q.A.y=points_3_boxes[0][1];
            q.C.x=points_3_boxes[1][0];
            q.C.y=points_3_boxes[1][1];
            q.D.x=points_3_boxes[2][0];
            q.D.y=points_3_boxes[2][1];
            q.B.x=points_3_boxes[3][0];
            q.B.y=points_3_boxes[3][1];
            quas_raw.push_back(q);
            quas_expand.push_back(q);
        }
        return ;
    }

    vector<int> matched(boxes.size(),0);//表示定位标的匹配情况，0：未处理，1：匹配上了，2：处理了，但没有匹配成功
    vector<Point> centers;//每个轮廓的中心点
    for(int i=0;i<boxes.size();i++)
    {
        Point p_temp=centerPosition(boxes[i]);
        centers.push_back(p_temp);
    }
    for(int i=0;i<boxes.size()-2;i++)
    {
        if(matched[i]==1)
            continue;
        vector<pair<int,float>> dis;
        for(int j=0;j<boxes.size();j++)
        {
            if(j==i||matched[j]==1)
                continue;
            float distance=Distance(centers[i],centers[j]);
            dis.push_back(make_pair(j,distance));
        }
        sort(dis.begin(), dis.end(), cmp);//按照距离由小到大排序
        vector<vector<Point>> boxes_3;
        boxes_3.push_back(boxes[i]);
        boxes_3.push_back(boxes[dis[0].first]);
        boxes_3.push_back(boxes[dis[1].first]);
        //得到了三个定位标后检查这三个定位标的最大角度是否超过110度
        vector<Point> angle_test;
        angle_test.push_back(centers[i]);
        angle_test.push_back(centers[dis[0].first]);
        angle_test.push_back(centers[dis[1].first]);
        float angle=maxAngle(angle_test);
        bool Effective=(angle<110.f);
        //cout<<"angle = "<<angle<<" Effective = "<<Effective<<endl;
        QuaVert qua_raw,qua_expand;
        if(Effective)
        {
            getQuaVert(img,boxes_3,qua_raw,qua_expand);//找到这三个定位标可能包含的二维码信息
            //检测该四边形是否包含了其它的定位标
            vector<int> idxs_qua;
            idxs_qua.push_back(i);
            idxs_qua.push_back(dis[0].first);
            idxs_qua.push_back(dis[1].first);
            Effective=IsEffectiveQua(qua_expand,centers,idxs_qua);
        }
        
        if(!Effective||!qua_raw.existence)
        {
            matched[i]=1;
        }
        else
        {
            string info_temp;
            vector<Vec2f> points_temp;
            bool detected=decodeQua(img,qua_expand,info_temp,points_temp,ROI_SHOW);
            if(detected)
            {
                //cout<<"a QRcode detected!"<<endl;
                matched[i]=1;
                matched[dis[0].first]=1;
                matched[dis[1].first]=1;
                infos.push_back(info_temp);
                positions.push_back(points_temp);
                quas_raw.push_back(qua_raw);
                quas_expand.push_back(qua_expand);
            }
            else
                matched[i]=2;
        }

        //剩余的轮廓少于三个就停止检测
        int unmached=0;
        for(int idx_matched=0;idx_matched<matched.size();idx_matched++)
        {
            if(matched[idx_matched]!=1)
                unmached++;
        }
        //cout<<"unmached = "<<unmached<<endl;
        if(unmached<3)
            break;
    }
}

Point centerPosition(vector<Point>& points)
{
    float x=0.f;
    float y=0.f;
    float N=(float)points.size();
    for(int i=0;i<points.size();i++)
    {
        x+=(float)points[i].x/N;
        y+=(float)points[i].y/N;
    }
    Point p(x,y);
    return p;
}

//通过四边形的顶点的范围来检测二维码，这好个四边形应该是把二维码包含在内的，而不是刚刚
bool decodeQua(Mat& img,QuaVert qua_expand,string& info,vector<Vec2f>& points,bool ROI_SHOW)
{
    if(img.type() != CV_8UC1)
    {
        cout<<"please input a gray image!"<<endl;
        return false;
    }

    points.clear();

    vector<Point> points_for_rect;
    points_for_rect.push_back(qua_expand.A);
    points_for_rect.push_back(qua_expand.B);
    points_for_rect.push_back(qua_expand.D);
    points_for_rect.push_back(qua_expand.C);
    Rect rect_roi=findRect(points_for_rect,img.cols,img.rows);
    //cout<<"img.height = "<<img.rows<<" "<<"img.width = "<<img.cols<<endl;
    //cout<<"rect_roi: x,y,wid,height"<<rect_roi.x<<","<<rect_roi.y<<","<<rect_roi.width<<","<<rect_roi.height<<endl;
    Mat img_rect=img(rect_roi).clone();
    //cv::Ptr<cv::CLAHE> clahe=cv::createCLAHE(6.0,cv::Size(8,8));
    //clahe->apply(img_rect,img_rect);
    //normalize(img_rect,img_rect,0,255,NORM_MINMAX);
    //equalizeHist(img_rect,img_rect);
    //adaptiveBinaryImg(img_rect,img_rect,8);
    //blur(img_rect,img_rect,Size(3,3));

    QRCodeDetector qr;
    bool Detected=qr.detect(img_rect,points);
    if(Detected&&isValid(img_rect,points))//如果从灰度图检测到了定位标,进行解码
    {
        info=qr.detectAndDecode(img_rect,points);
    }
    //cout<<"info = "<<info<<endl;
    for(int i=0;i<points.size();i++)
    {
        points[i][0]+=(float)rect_roi.x;
        points[i][1]+=(float)rect_roi.y;
    }
    //cout<<"info detecting by rect area"<<info<<endl;

    //viewer
    //namedWindow("img_rect",WINDOW_NORMAL);
    //imshow("img_rect",img_rect);
    //cout<<"img_rect.rows,cols = "<<img_rect.rows<<","<<img_rect.cols<<endl;
    //后面都是为了显示ROI
    /**/
    if(ROI_SHOW)
    {
        //构建四条边的直线方程
        LINE corner_AB(qua_expand.A,qua_expand.B);
        LINE corner_AC(qua_expand.A,qua_expand.C);
        LINE corner_BD(qua_expand.B,qua_expand.D);
        LINE corner_CD(qua_expand.C,qua_expand.D);
        //调整法相量，使得其朝向四边形内部
        if(!corner_AB.IsNormDirection(qua_expand.D))
            corner_AB.inverse();
        if(!corner_AC.IsNormDirection(qua_expand.B))
            corner_AC.inverse();
        if(!corner_BD.IsNormDirection(qua_expand.C))
            corner_BD.inverse();
        if(!corner_CD.IsNormDirection(qua_expand.A))
            corner_CD.inverse();
        //显示提取到的四边形
        Mat img_clone=img.clone();
        for(int i=0;i<img_clone.rows;i++)
        {
            uchar* data=img_clone.ptr<uchar>(i);
            for(int j=0;j<img_clone.cols;j++)
            {
                Point pixel(j,i);
                if(     !corner_AB.IsNormDirection(pixel)
                        ||!corner_AC.IsNormDirection(pixel)
                        ||!corner_BD.IsNormDirection(pixel)
                        ||!corner_CD.IsNormDirection(pixel))
                    data[j]=0;
            }
        }
        namedWindow("ROI",WINDOW_NORMAL);
        cv::imshow("ROI",img_clone);
        cv::waitKey();
    }  

    if(info.length()!=0)
        return true;
    else return false;
}

void preProcessing(const cv::Mat& img_raw,cv::Mat& img_binary)
{
    int pieces=6;
    int blockSize=min(img_raw.cols/pieces,img_raw.rows/pieces);
    if(!(blockSize&0x01))
    blockSize++;
    cout<<"blockSize = "<<blockSize<<endl;

    cv::Mat img_binary_temp;
    equalizeHist(img_raw,img_binary_temp);
    //cv::imwrite("hist.jpg",img_binary_temp);
    bilateralFilter(img_binary_temp,img_binary,25,25*2,25/2);
    //cv::imwrite("bilateral_after.jpg",img_binary);
    adaptiveThreshold(img_binary,img_binary,255,cv::ADAPTIVE_THRESH_MEAN_C,cv::THRESH_BINARY,blockSize,0);
    //cv::imwrite("binary.jpg",img_binary);
}

void detect_decode(cv::Mat& img,vector<string>& infos,vector<vector<Vec2f>>& positions
                   ,vector<QuaVert>& quas_raw,vector<QuaVert>& quas_expand,bool DetailedInfo)
{
    //步骤一、图像预处理
    Mat img_binary;
    preProcessing(img,img_binary);
    //adaptiveBinaryImg(img_binary,img_binary,4);
    if(DetailedInfo){
        namedWindow("img_binary_whole",WINDOW_NORMAL);
        cv::imshow("img_binary_whole",img_binary);
        cv::waitKey();
    }

    //步骤二、提取轮廓并选出定位标轮廓
    //定义轮廓和层次结构
    vector<vector<Point>> contours;
    vector<Vec4i> hierachy;
    vector<int> idxs;
    //查找轮廓
    findContours(img_binary,contours,hierachy,RETR_TREE,CHAIN_APPROX_NONE);
    //查找回形的位置探测图形
    vector<vector<Point>> boxes;//存放探测图形的最外层轮廓
    //从轮廓中选择定位标的轮廓

#if 1
    namedWindow("轮廓",WINDOW_NORMAL);
    namedWindow("定位标",WINDOW_NORMAL);
#endif

    Mat img_temp=Mat::zeros(img.rows,img.cols,CV_8UC3);
    for(int i=0;i<contours.size();i++)
    {
        if(hierachy[i][3]!=-1)
        {
            //cerr<<"1"<<endl;
        }
        if(hierachy[i][2]!=-1&&hierachy[i][3]!=-1)//同时有父轮廓和子轮廓的才可能是定位标
        {
            Scalar color(rand()&255,rand()&255,rand()&255);
            drawContours(img_temp,contours,hierachy[i][3],color,1,8,hierachy);
            //imshow("轮廓",img_temp);
            int parentIdx=hierachy[i][3];
            bool isQr = IsQrPoint(contours[parentIdx], img_binary);
            //if(isQr)
            //cout<<"是一个QRPoint"<<endl;
            //cout<<"---------------"<<endl;
            //cv::waitKey();
            //img_temp=Mat::zeros(img.rows,img.cols,CV_8UC3);
            //namedWindow("定位标",WINDOW_NORMAL);
            //imshow("定位标",img_temp);
            //cv::waitKey();
            if(isQr)
            {
                boxes.push_back(contours[parentIdx]);
                idxs.push_back(parentIdx);
                //cout<<"QR Point"<<endl;
            }
            else
            {
                //cout<<"a contour but not a QR point"<<endl;
                continue;
            }
        }
    }
    cout<<"boxes.size() = "<<boxes.size()<<endl;
    //if(DetailedInfo)
    {
        Mat img_view_decoding=Mat::zeros(img.rows,img.cols,CV_8UC3);
        for(int i=0;i<boxes.size();i++)//给轮廓图注释相应的编号
        {
            string text=to_string(i);
            Scalar color(rand()&255,rand()&255,rand()&255);
            drawContours(img_view_decoding,contours,idxs[i],color,1,8,hierachy);
            putText(img_view_decoding,text,boxes[i][0],FONT_HERSHEY_SIMPLEX,1,Scalar(255,23,0),4,8);
            //cout<<"boxes["<<i<<"].size() = "<<boxes[i].size()<<endl;
        }
        
        imshow("定位标",img_view_decoding);
        imshow("轮廓",img_temp);
        //cv::imwrite("contours.jpg",img_temp);
        //cv::imwrite("QRCoutours.jpg",img_view_decoding);
        //cv::waitKey();
    }

    //步骤三、得到定位标轮廓后，进行鲁棒解码
    decodeImg(img_binary,boxes,infos,positions,quas_raw,quas_expand,DetailedInfo); 
}
bool IsEffectiveQua(QuaVert& qua,vector<Point>& centers,vector<int>& idxs)
{
    if(idxs.size()!=3)
    {
        cout<<"Error:idxs.size()! = 3"<<endl<<endl;
        return false;
    }
    LINE corner_AB(qua.A,qua.B);
    LINE corner_AC(qua.A,qua.C);
    LINE corner_BD(qua.B,qua.D);
    LINE corner_CD(qua.C,qua.D);
    //调整法相量，使得其朝向四边形内部
    if(!corner_AB.IsNormDirection(qua.D))
        corner_AB.inverse();
    if(!corner_AC.IsNormDirection(qua.B))
        corner_AC.inverse();
    if(!corner_BD.IsNormDirection(qua.C))
        corner_BD.inverse();
    if(!corner_CD.IsNormDirection(qua.A))
        corner_CD.inverse();
    for(int i=0;i<centers.size();i++)
    {
        if(i==idxs[0]||i==idxs[1]||i==idxs[2])
        {
            continue;
        }
        if(corner_AB.IsNormDirection(centers[i])
           &&corner_AC.IsNormDirection(centers[i])
           &&corner_BD.IsNormDirection(centers[i])
           &&corner_CD.IsNormDirection(centers[i]))
           return false;
    }
    return true;
}
float maxAngle(vector<Point> points)
{
    if(points.size()!=3)
    {
        cout<<"Error:points size not equal 3!"<<endl;
        return 9999;
    }
    float line_max,line1,line2;
    vector<float> lines;
    lines.push_back(Distance(points[0],points[1]));
    lines.push_back(Distance(points[0],points[2]));
    lines.push_back(Distance(points[1],points[2]));
    sort(lines.begin(),lines.end(),cmp_float);
    
    //debug
    //cout<<"lines : ";
    //for(int i=0;i<lines.size();i++)
    //{
    //    cout<<" "<<lines[i];
    //}
    //cout<<endl;

    //求解余弦值
    float cos_rad;
    cos_rad=(pow(lines[1],2)+pow(lines[2],2)-pow(lines[0],2))/(2.f*lines[1]*lines[2]);
    return 180.f*acos(cos_rad)/3.1415926;
}
void adaptiveBinaryImg(cv::Mat& src,cv::Mat& dst,int size)
{
    CV_Assert(src.type()==CV_8UC1);
    dst=src.clone();

    //将图像平均分成size×size块
    int pieces=size*size;
    int WIDTH=src.cols;
    int HEIGHT=src.rows;
    int frac_row=(double)HEIGHT/(double)size;
    int frac_col=(double)WIDTH/(double)size;
    //cout<<"frac_row,frac_col = "<<frac_row<<","<<frac_col<<endl;
    vector<double> averages(pieces,0.0);
    for(int i=0;i<size;i++)
    {
        for(int j=0;j<size;j++)
        {
            Rect rect(frac_col*j,frac_row*i,frac_col,frac_row);
            if(j==size-1){
                rect.width=WIDTH-frac_col*j;
            }
            if(i==size-1)
                rect.height=HEIGHT-frac_row*i;
            Mat img_temp=dst(rect);
            //imshow("img_temp",img_temp);
            //cv::waitKey();
            //medianBlur(img_temp,img_temp,3);
            //equalizeHist(img_temp,img_temp);
            //img_temp=img_temp>119;
            averageBinary(img_temp);
            //adaptiveThreshold(img_temp,img_temp,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,51,0);
#if 0
            //对每一块元素进行遍历获取平均值
            double sum=0.0;
            double N=(double)(frac_row*frac_col);
            int num=0;
            for(int k=frac_row*i;k<frac_row*(i+1)&&k<HEIGHT;k++)
            {
                uchar* data=src.ptr<uchar>(k);
                for(int m=frac_col*j;m<frac_col*(j+1)&&m<WIDTH;m++)
                {
                    sum+=data[m]/N;
                    num++;
                }
            }
            //cout<<"num = "<<num<<",N  = "<<N<<endl;
            
            //cout<<"sum = "<<sum<<endl;
            averages[i*size+j]=sum;

            for(int k=frac_row*i;k<frac_row*(i+1)&&k<HEIGHT;k++)
            {
                uchar* data=dst.ptr<uchar>(k);
                for(int m=frac_col*j;m<frac_col*(j+1)&&m<WIDTH;m++)
                {
                    data[m]=data[m]>0.5*sum?255:0;
                }
            }
#endif
        }
    }
    //cv::namedWindow("src",WINDOW_NORMAL);
    //cv::namedWindow("dst",WINDOW_NORMAL);
    //cv::imwrite("dst.png",dst);
    //imshow("src",src);
    //imshow("dst",dst);
    //cv::waitKey();
}

#if 0
void adaptiveBinaryImg(cv::Mat& src,cv::Mat& dst,int size)
{
    CV_Assert(src.type()==CV_8UC1);
    dst=src.clone();

    //将图像平均分成size×size块
    int pieces=size*size;
    int WIDTH=src.cols;
    int HEIGHT=src.rows;
    int frac_row=(double)HEIGHT/(double)size;
    int frac_col=(double)WIDTH/(double)size;
    //cout<<"frac_row,frac_col = "<<frac_row<<","<<frac_col<<endl;
    vector<double> averages(pieces,0.0);
    for(int i=0;i<size;i++)
    {
        for(int j=0;j<size;j++)
        {
            //对每一块元素进行遍历获取平均值
            double sum=0.0;
            double N=(double)(frac_row*frac_col);
            int num=0;
            for(int k=frac_row*i;k<frac_row*(i+1)&&k<HEIGHT;k++)
            {
                uchar* data=src.ptr<uchar>(k);
                for(int m=frac_col*j;m<frac_col*(j+1)&&m<WIDTH;m++)
                {
                    sum+=data[m]/N;
                    num++;
                }
            }
            //cout<<"num = "<<num<<",N  = "<<N<<endl;
            
            //cout<<"sum = "<<sum<<endl;
            averages[i*size+j]=sum;

            for(int k=frac_row*i;k<frac_row*(i+1)&&k<HEIGHT;k++)
            {
                uchar* data=dst.ptr<uchar>(k);
                for(int m=frac_col*j;m<frac_col*(j+1)&&m<WIDTH;m++)
                {
                    data[m]=data[m]>0.5*sum?255:0;
                }
            }
        }
    }
    cv::namedWindow("src",WINDOW_NORMAL);
    cv::namedWindow("dst",WINDOW_NORMAL);
    cv::imwrite("dst.png",dst);
    imshow("src",src);
    imshow("dst",dst);
    //cv::waitKey();
}
#endif
void averageBinary(cv::Mat& src)
{
    CV_Assert(src.type()==CV_8UC1);
    int WIDTH=src.cols;
    int HEIGHT=src.rows;
    double sum=0.0;
    for(int i=0;i<HEIGHT;i++){
        for(int j=0;j<WIDTH;j++){
            sum+=(double)src.at<uchar>(i,j);
        }
    }
    sum=sum/(double)(HEIGHT*WIDTH);
    int avg=(int)sum;
    for(int i=0;i<HEIGHT;i++){
        for(int j=0;j<WIDTH;j++){
            int num=src.at<uchar>(i,j);
            if(num>avg)
            src.at<uchar>(i,j)=255;
            else
            src.at<uchar>(i,j)=0;
        }
    }
}
bool isValid(cv::Mat& img,vector<Vec2f>& points)
{
    int row=img.rows,col=img.cols;
    for(auto x:points){
        if(x[0]<=0||x[0]<=0>row-1
        ||x[1]<=0||x[1]<=0>col-1)
        return false;
    }
    return true;
}