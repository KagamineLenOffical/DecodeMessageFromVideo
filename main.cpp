#include <iostream>
#include <opencv2/opencv.hpp>
#include <cstring>
extern "C"
{
    #include "zlib.h"
}
#define pow2(a) ((a)*(a))
#define ll long long

using namespace std;
using namespace cv;
bool result[800000];
bool comp_src[800000];
int cnt=0;
const int ROW=1000;
const int COL=1000;
const int LEN=10;

const int NUMBER_OF_POS=4;
/*int loc_point[9][9]={
        {1,1,1,1,1,1,1,0,0},
        {1,0,0,0,0,0,1,0,0},
        {1,0,1,1,1,0,1,0,0},
        {1,0,1,1,1,0,1,0,0},
        {1,0,1,1,1,0,1,0,0},
        {1,0,0,0,0,0,1,0,0},
        {1,1,1,1,1,1,1,0,0},
        {0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0}
};*/

/*-----------------------------------------*/
//压缩模块
void zip(char *PreZipBits,unsigned char *ZippedBits)
{
    z_stream stream;
    
    
    stream.next_in = (Bytef *)PreZipBits;
    stream.avail_in = (uInt)strlen(PreZipBits);
    
    stream.next_out = (Bytef *)ZippedBits;
    stream.avail_out = (uInt)1024;
    
    stream.zalloc = (alloc_func)0;
    stream.zfree = (free_func)0;
    stream.opaque = (voidpf)0;
    
    deflateInit(&stream, Z_DEFAULT_COMPRESSION);
    deflate(&stream,Z_FINISH);
    deflateEnd(&stream);     
}

//变换
int dis(Point a,Point b){
    //printf("%d %d %d\n",a.x-b.x,a.y-b.y,pow2(a.x-b.x)+pow2(a.y-b.y));
    return pow2(a.x-b.x)+pow2(a.y-b.y);
}
void WarpImage(vector<Point> res,Mat *SrcMat)    //输入定位点的vector和图像指针
{
    Mat DstMat;
    Point2f SrcPoints[4];//变换来源的四点


    //从res中取出定位点到SrcPoints
    for(int i=0;i<=NUMBER_OF_POS;i++)
    {
        SrcPoints[i]=res[i];
    }


    Point2f DstPoints[4];//变换目标的四点

    //初始化变换目标的四个点
    DstPoints[0] = Point2f(0, 0);
    DstPoints[1] = Point2f(0, ROW-2*LEN);
    DstPoints[2] = Point2f(COL-2*LEN, 0);
    DstPoints[3] = Point2f(COL-2*LEN, ROW-2*LEN);

    //变换目标对应顺序：左上 左下 右上 右下
    Mat TransBuffer = getPerspectiveTransform(SrcPoints,DstPoints);  //TransBuffer存放变换矩阵
    warpPerspective(*SrcMat, *SrcMat, TransBuffer, SrcMat->size());



}
int r=ROW/LEN-2,c=ROW/LEN-2;
int originpoint[ROW/LEN][COL/LEN];
/*-----------------------------------------*/
void decode(Mat img){
    for(int i=0;i<c;i++)
        for(int j=0;j<r;j++){
            originpoint[i][j]=-1;
        }
    for(int i=0;i<9;i++)
        for(int j=0;j<9;j++){
            int k=1;
            originpoint[i][j]=k;
            originpoint[c-i-1][j]=k;
            originpoint[i][r-j-1]=k;
            originpoint[c-i-1][r-j-1]=k;
        }
    for(int i=0;i<c;i++) {
        for (int j = 0; j < r; j++) {
            if (originpoint[i][j] == -1) {

                int x=i*LEN+LEN/2,y = j * LEN + LEN/2;

//                rectangle(img,Point(i*LEN,j*LEN),Point(i*LEN+10,j*LEN+10),Scalar(0,0,0));
                int b=img.at<Vec3b>(Point(x,y))[0];
                int g=img.at<Vec3b>(Point(x,y))[1];
                int r=img.at<Vec3b>(Point(x,y))[2];
                Scalar color(b,g,r);
//                rectangle(img,Point(i*LEN,j*LEN),Point(i*LEN+LEN/2,j*LEN+LEN/2),Scalar(0,0,0));
//                rectangle(img,Point(0,0),Point(80,80),color,FILLED);
                result[cnt+2]=b<145;
                result[cnt+1]=g<160;
                result[cnt]=r<140;
                if(result[cnt+2]!=comp_src[cnt+2]||result[cnt+1]!=comp_src[cnt+1]||
                    result[cnt]!=comp_src[cnt]){
                    rectangle(img,Point(i*LEN,j*LEN),Point(i*LEN+LEN,j*LEN+LEN),Scalar(0,0,0));
                    rectangle(img,Point(0,0),Point(80,80),color,FILLED);
                    printf("%d:%d %d %d\n",cnt,r,g,b);
                    namedWindow("ThresholdImage");
                    imshow("ThresholdImage",img);
                    waitKey(0);
                }
//                printf("%d %d %d",b,g,r);

                cnt+=3;
            }

        }
    }
//    namedWindow("ThresholdImage");
//    imshow("ThresholdImage",img);
//    waitKey(0);
}
bool CheckForQRpos(Mat img){
//    namedWindow("OriginalImage");
//    imshow("OriginalImage",img);
//    waitKey(0);
    Mat gray_img;
    cvtColor(img,gray_img,COLOR_BGR2GRAY);
    Mat threshold_img;
    threshold(gray_img,threshold_img,0,255,THRESH_BINARY|THRESH_OTSU);
//    namedWindow("ThresholdImage");
//    imshow("ThresholdImage",threshold_img);
//    waitKey(0);
    vector<vector<Point>>contours;
    vector<Vec4i>hierarchy;

    Mat dst=Mat::zeros(img.rows,img.cols,CV_8U);

    findContours(threshold_img,contours,hierarchy,RETR_TREE,CHAIN_APPROX_NONE,Point(0,0));
    int son=-1;
    int QRPoint;
    vector<vector<Point>>QRPosPoint;
    for(int i=0;i<contours.size();i++){
        if(hierarchy[i][2]!=-1){  //有子轮廓
            son=hierarchy[i][2];
            if(hierarchy[son][2]!=-1){
//                printf("%d\n",hierarchy[son][3]);
                QRPoint=i;
                RotatedRect rect;
                rect=minAreaRect(contours[i]);
                if(rect.size.width>LEN*15||rect.size.height>LEN*15)continue;//判断是否为定位点（按大小）
                QRPosPoint.push_back(contours[i]);

//                drawContours(dst,contours,i,Scalar(127,127,127));
//                namedWindow("ThresholdImage");
//                imshow("ThresholdImage",dst);
//                printf("%d\n",threshold_img.channels());
//                waitKey(0);    //中间过程显示
            }


        }
    }

    if(QRPosPoint.size()<4)return 0;
    Point tmp[4]={Point(0,0),Point(0,img.rows),Point(img.cols,0),Point(img.cols,img.rows)};
    vector<Point> res;
    res.resize(4);
    int d[4];
    for(int i=0;i<4;i++)d[i]=1e9;
    for(int i=0;i<QRPosPoint.size();i++)
        for(int j=0;j<QRPosPoint[i].size();j++){
            Point t=QRPosPoint[i][j];
            // printf("%d %d\n",t.x,t.y);
            for(int k=0;k<4;k++){
                if(dis(t,tmp[k])<d[k]){
                    d[k]=dis(t,tmp[k]);
                    res[k]=t;
                }
            }
        }
    for(int i=0;i<3;i++) {
        line(dst, res[i], res[i + 1], Scalar(255, 255, 255));
    }
    WarpImage(res,&img);
//    namedWindow("ThresholdImage");
//    imshow("ThresholdImage",img);
//    waitKey(0);
    //接下来进行信息读取
    decode(img);
    /*
    float minx=min(v[0].y,v[2].y),miny=min(v[0].x,v[2].x);
    float maxx=max(v[0].y,v[2].y),maxy=max(v[0].x,v[2].x);
//    printf("x:%f %f y:%f %f\n",minx,maxx,miny,maxy);
    float px=(maxx-minx)/(ROW/LEN),py=(maxy-miny)/(COL/LEN);
//    printf("px %f py %f\n",px,py);
    for (int i = 0; i < COL / LEN; i++)
        for (int j = 0; j < ROW / LEN; j++) {
            int x = int(minx+px*j), y = int(miny+py*i);
            if ((i == 0 && j == 0) || (i == COL / LEN - 1 && j == 0) || (i == 0 && j == ROW / LEN - 1)) {
                continue;
            }
            else if ((i == 0 && (j == 1 || j == ROW / LEN - 2)) ||
                       (i == 1 && (j == 0 || j == 1 || j == ROW / LEN - 2 || j == ROW / LEN - 1)) ||
                       (i == COL / LEN - 2 && (j == 0 || j == 1)) || (i == COL / LEN - 1 && j == 1)) { continue; }
            else{
                int tot=(int)px*(int)py;int avg_cnt=0;
//                printf("%d %d\n",i,j);
                namedWindow("thresholdImg");
                imshow("thresholdImg",threshold_img);
                waitKey(0);
                for(int k=0;k<(int)px;k++)
                    for(int l=0;l<(int)py;l++){
                        avg_cnt+=threshold_img.at<uchar>(x+k,y+l)>127?0:1;
                        threshold_img.at<uchar>(x+k,y+l)=0;
                    }
                float avg=(float)avg_cnt/tot;
                if(avg>0.5)result[cnt++]=1;
                else result[cnt++]=0;
            }
        }
    */
    return 1;
}
VideoCapture cap;
int main() {
    cap=VideoCapture("7.mp4");
    FILE *cmp_file=fopen("2.out","r");
    char ch;
    int _cnt=0;
    while((ch=fgetc(cmp_file))!=EOF){
        comp_src[_cnt++]=ch-'0';
    }
    int fps=30;
    Mat frame;
    while(1){
        cap>>frame;
        if(CheckForQRpos(frame))break;
    }
    memset(result,0,sizeof(result));
    for(int i=0;i<2;i++)cap>>frame; //跳过前面不稳定的几帧
    cnt=0;
    while(1){
        if(frame.empty())break;
        if(!CheckForQRpos(frame))break;
        for(int i=0;i<5;i++)cap>>frame;
    }
    FILE *fout=fopen("1.out","w+");

    for(int i=0;i<cnt;i++){
        fprintf(fout,"%d",result[i]);
    }
    fclose(fout);
    printf("\ntot bit:%d",cnt);
    cout<<endl;
    char s[100000];
    int now=0;
    memset(s,0,sizeof(s));
    while(1){
        for(int i=now*8+7;i>=now*8;i--)s[now]=(s[now]<<1)|result[i];
        if(s[now]==0)break;
        now++;
    }
    printf("%s",s);
    waitKey(0);
    return 0;

}
