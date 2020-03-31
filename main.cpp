#include <iostream>
#include <opencv2/opencv.hpp>
#include <cstring>
#include <bitset>
#define pow2(a) ((a)*(a))
#define ll long long
#define Data_Block_Size_By_Bit 256      //改变这个可以调节数据块大小。具体多大最合适和错误率挂钩，我没算。暂时不要改动，否则会出错。
#define Buffer_Size_By_Byte 1024*1024*100
#define Buffer_Size_By_Bit 1024*1024*50*8

using namespace std;
using namespace cv;

bool Raw_Data_Read_Buffer[Buffer_Size_By_Bit];
bool Processed_Data_Buffer[Buffer_Size_By_Bit];
bool Original_Raw_Data_Buffer[Buffer_Size_By_Bit];




int Total_Bits_Read=0;
int After_Process_Bits = 0;
const int ROW=1000;
const int COL=1800;
const int LEN=10;

char Input_Video_Path_Buffer[200];
char Output_Result_Path_Buffer[200];
char Output_Error_Tracker_Path_Buffer[200];

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

/*------------------预处理出错找这里----------------------*/
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
int Rows_Of_Block=ROW/LEN-2;
int Columns_Of_Block=COL/LEN-2;
int originpoint[COL/LEN][ROW/LEN];
/*-----------------------------------------------------*/




/*------------------检错码出错找这里----------------------*/
void Take_Out_Parity_Bits_And_Create_Error_Checking_Binary(bool *Pre_Processed_Data,bool *After_Processed_Data)  //奇校验
{
    int Marker_Pre_Processed = 0;
    int Marker_After_Processed = 0;
    int Parity_Number_Tracker = 0;
    int Parity_Current_Parity_Tracker = 0;
    FILE *fout = fopen(Output_Error_Tracker_Path_Buffer,"w+");
    int Block_Size_With_Parity_Bit = Data_Block_Size_By_Bit + 1 ;
    After_Process_Bits = Total_Bits_Read;



    while(Marker_Pre_Processed <= Total_Bits_Read)
    {
        After_Processed_Data[Marker_After_Processed] = Pre_Processed_Data[Marker_Pre_Processed];
        if(Pre_Processed_Data[Marker_Pre_Processed] == 1)
        {
            Parity_Current_Parity_Tracker ++;
        }
        Marker_Pre_Processed ++;

        Marker_After_Processed ++;

        Parity_Number_Tracker ++;
        if(Parity_Number_Tracker == Block_Size_With_Parity_Bit )
        {
            Marker_After_Processed--;//把刚才读到After数组里的校验位退回去。
            After_Process_Bits--;
            if(Parity_Current_Parity_Tracker % 2 == 1) //余1，就是奇数嘛，就是代表没错
            {
                //写32个字节
                for(int i = 0 ; i < (Data_Block_Size_By_Bit/8) ; i++)
                {
                    fputc(255,fout);   //255 = 11111111
                }

            }
            else //错了就写一堆0
            {
                for(int i = 0 ; i < (Data_Block_Size_By_Bit/8) ; i++)
                {
                    fputc(0,fout);   //0 = 00000000
                }
            }
            Parity_Current_Parity_Tracker = 0;
            Parity_Number_Tracker = 0;

        }

    }

    for(int i = 0 ; i < 32 ; i++)  //最后再多写32字节。
    {
        fputc(255,fout);   //255 = 11111111
    }


}
/*-----------------------------------------------------*/






void decode(Mat img)
{
    for(int i=0;i<Columns_Of_Block;i++)
        for(int j=0;j<Rows_Of_Block;j++)
        {
            originpoint[i][j]=-1;
        }
    for(int i=0;i<9;i++)
        for(int j=0;j<9;j++)
        {
            int k=1;
            originpoint[i][j]=k;
            originpoint[Columns_Of_Block-i-1][j]=k;
            originpoint[i][Rows_Of_Block-j-1]=k;
            originpoint[Columns_Of_Block-i-1][Rows_Of_Block-j-1]=k;
        }



    for(int i=0;i<Columns_Of_Block;i++)
    {
        for (int j = 0; j < Rows_Of_Block; j++)
        {
            if (originpoint[i][j] == -1)
            {

                int x=i*LEN+LEN/2,y = j * LEN + LEN/2;

                int b=img.at<Vec3b>(Point(x,y))[0];
                int g=img.at<Vec3b>(Point(x,y))[1];
                int r=img.at<Vec3b>(Point(x,y))[2];
                Scalar color(b,g,r);

                Raw_Data_Read_Buffer[Total_Bits_Read+2]=b<145;
                //<145为1 >=145为0
                Raw_Data_Read_Buffer[Total_Bits_Read+1]=g<160;
                Raw_Data_Read_Buffer[Total_Bits_Read]=r<140;
                rectangle(img, Point(i * LEN, j * LEN), Point(i * LEN + LEN, j * LEN + LEN), Scalar(0, 0, 0),1);



                /*这里是显示错误的程序*/
                if(Raw_Data_Read_Buffer[Total_Bits_Read+1]^Raw_Data_Read_Buffer[Total_Bits_Read])
                {
                    if (Raw_Data_Read_Buffer[Total_Bits_Read + 2] != Original_Raw_Data_Buffer[Total_Bits_Read + 2] || Raw_Data_Read_Buffer[Total_Bits_Read + 1] != Original_Raw_Data_Buffer[Total_Bits_Read + 1] ||
                        Raw_Data_Read_Buffer[Total_Bits_Read] != Original_Raw_Data_Buffer[Total_Bits_Read])
                    {

                        rectangle(img, Point(0, 0), Point(80, 80), color, FILLED);
                        printf("%d:%d %d %d\n", Total_Bits_Read, r, g, b);
//                        namedWindow("ThresholdImage");
//                        imshow("ThresholdImage", img);
//                        waitKey(0);
                    }
                }

                Total_Bits_Read+=3;
            }

        }
    }

}


bool CheckForQRpos(Mat img)
{

    Mat gray_img;
    cvtColor(img,gray_img,COLOR_BGR2GRAY);
    Mat threshold_img;
    threshold(gray_img,threshold_img,0,255,THRESH_BINARY|THRESH_OTSU);
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
            if(hierarchy[son][2]!=-1)
            {

                QRPoint=i;
                RotatedRect rect;
                rect=minAreaRect(contours[i]);
                if(rect.size.width>LEN*15||rect.size.height>LEN*15)continue;//判断是否为定位点（按大小）
                QRPosPoint.push_back(contours[i]);
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

    //接下来进行信息读取
    decode(img);

    return 1;
}


VideoCapture cap;

int main(int argc,char *argv[])
{
    strcpy(Input_Video_Path_Buffer,argv[1]);
    strcpy(Output_Result_Path_Buffer,argv[2]);
    strcpy(Output_Error_Tracker_Path_Buffer,argv[3]);


    cap=VideoCapture(Input_Video_Path_Buffer);

    FILE *cmp_file=fopen("Processed.out","r");  //改成对比Processed.out了。因为图片对比
    char ch;
    int _cnt=0;

    while((ch=fgetc(cmp_file))!=EOF)
    {
        Original_Raw_Data_Buffer[_cnt++]=ch-'0';
    }

    Mat frame;
    while(1){
        cap>>frame;
        if(CheckForQRpos(frame))break;
    }

//    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
//    cameraMatrix.at<double>(0, 0) = 1802.4891034442435;
//    cameraMatrix.at<double>(0, 2) = 969.7033764824771;
//    cameraMatrix.at<double>(1, 1) = 1800.940836843952;
//    cameraMatrix.at<double>(1, 2) = 546.1764101253999;
//    cameraMatrix.at<double>(2, 2) = 1.0;
//
//    Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
//    distCoeffs.at<double>(0, 0) = 0.22082425770962424;
//    distCoeffs.at<double>(1, 0) = -0.998223161784493;
//    distCoeffs.at<double>(2, 0) = 4.7000979984294e-05;
//    distCoeffs.at<double>(3, 0) = 0.0007672376802051042;
//    distCoeffs.at<double>(4, 0) = 0.9611339808360126;

    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0, 0) = 6000.4891034442435;
    cameraMatrix.at<double>(0, 2) = 1200.7033764824771;
    cameraMatrix.at<double>(1, 1) = 6000.940836843952;
    cameraMatrix.at<double>(1, 2) = 546.1764101253999;
    cameraMatrix.at<double>(2, 2) = 1.0;

    Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
    distCoeffs.at<double>(0, 0) = 0.22082425770962424;
    distCoeffs.at<double>(1, 0) = -0.998223161784493;
    distCoeffs.at<double>(2, 0) = 4.7000979984294e-05;
    distCoeffs.at<double>(3, 0) = 0.0007672376802051042;
    distCoeffs.at<double>(4, 0) = 0.9611339808360126;


    Mat view, rview, map1, map2;
    Size imageSize;
    imageSize = frame.size();
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
                            getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),imageSize, CV_16SC2, map1, map2);





    memset(Raw_Data_Read_Buffer,0,sizeof(Raw_Data_Read_Buffer));
    for(int i=0;i<2;i++)cap>>frame; //跳过前面不稳定的几帧
    Total_Bits_Read=0;

    while(1)
    {
        if(frame.empty())break;
        Mat frameCalibration;
        remap(frame, frameCalibration, map1, map2, INTER_LINEAR);
        if(!CheckForQRpos(frameCalibration))break;
        for(int i=0;i<5;i++)cap>>frame;
    }






    FILE *fout=fopen("Processed_Read.out","w+");


    for(int i = 0 ;i < Total_Bits_Read ; i++)
    {
        fprintf(fout,"%d",Raw_Data_Read_Buffer[i]);
    }

    fclose(fout);


    Take_Out_Parity_Bits_And_Create_Error_Checking_Binary(Raw_Data_Read_Buffer, Processed_Data_Buffer);

    fout = fopen(Output_Result_Path_Buffer,"wb+");

    int j = 1;
    unsigned char CharStorege = 0;
    for(int i = 0 ; i < After_Process_Bits ; i++)
    {
        if(Processed_Data_Buffer[i])
        {
            switch(j)
            {
                case 1:
                    CharStorege += 1;
                    break;
                case 2:
                    CharStorege += 2;
                    break;
                case 3:
                    CharStorege += 4;
                    break;
                case 4:
                    CharStorege += 8;
                    break;
                case 5:
                    CharStorege += 16;
                    break;
                case 6:
                    CharStorege += 32;
                    break;
                case 7:
                    CharStorege += 64;
                    break;
                case 8:
                    CharStorege += 128;
                    break;


            }
        }
        if( j == 8 )
        {
            j = 0;
            fputc(CharStorege,fout);
            CharStorege = 0;
        }

        j++;
    }

    fclose(fout);







    printf("\ntot bit:%d",Total_Bits_Read);
    cout<<endl;

    return 0;

}
