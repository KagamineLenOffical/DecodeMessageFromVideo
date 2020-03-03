#include <iostream>
#include <opencv2/opencv.hpp>
#include <cstring>
using namespace std;
using namespace cv;
bool result[10000];
int cnt=0;
const int ROW=1000;
const int COL=1000;
const int LEN=10;
bool CheckForQRpos(Mat img){
//    namedWindow("OriginalImage");
//    imshow("OriginalImage",img);
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

                drawContours(dst,contours,i,Scalar(127,127,127));
                namedWindow("ThresholdImage");
                imshow("ThresholdImage",dst);
//                printf("%d\n",threshold_img.channels());
                waitKey(0);    //中间过程显示
            }


        }
    }

    if(QRPosPoint.size()<4)return 0;
    vector<Point> res;
    for(int i=0;i<QRPosPoint.size();i++)
        for(int j=0;j<QRPosPoint[i].size();j++){
            res.push_back(QRPosPoint[i][j]);
        }
    RotatedRect rect;
    rect=minAreaRect(res);
    Point2f v[4];
    rect.points(v);
    for(int i=0;i<3;i++) {
        line(dst, v[i], v[i + 1], Scalar(127, 127, 127));
    }
    //接下来进行信息读取
    namedWindow("ThresholdImage");
    imshow("ThresholdImage",dst);
    waitKey(0);
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
    cap=VideoCapture("4.mp4");
    int fps=30;
    Mat frame;
    while(1){
        cap>>frame;
        if(CheckForQRpos(frame))break;
    }
    memset(result,0,sizeof(result));
    for(int i=0;i<7;i++)cap>>frame; //跳过前面不稳定的几帧
    cnt=0;
    while(1){
        if(frame.empty())break;
        if(!CheckForQRpos(frame))break;
        for(int i=0;i<15;i++)cap>>frame;
    }
    for(int i=0;i<cnt;i++){
        printf("%d",result[i]);
    }
    cout<<endl;
    char s[1000];
    int now=0;
    memset(s,0,sizeof(s));
    while(1){
        for(int i=now*8+7;i>=now*8;i--)s[now]=(s[now]<<1)|result[i];
        if(s[now]==0)break;
        now++;
    }
    printf("%s",s);
    //010000101010011001001110011101101001011010100110000001001100101010000110011101100010011010100110010011101100111000000100000101101000011011001110000001001100011010100110101101101010011001110110001011101010011000100110000001000001011010010110110011100000010011001110001011101000011000101110101011101100111000000100100001101100111000000100001011100001011010100110000001000010001010100110101101101111011011000110010011101000011000101110100101101100011000000100011001100100111011110110011101100010111010110100010011101010111001110110011101101010011001001110000001000010111011110110000001000010111010000110110101101010011000000100111101100111011000000100001000101111011001110110100001100011011000100110000001000010101001001110101011101011011000001110000001001001011001110110000001000111001011110110011011101010011010110110010001101010011001001110111001001100111000000100101010101100101000000100000011100100111010100110110011101001011000100110101001100111011000101110100101101000011000110110000001001010011000110110101001101100011000101110100101101111011001110110011101000001001010100110000001001001011011001110000001000000111001001110111101100101011010100110110001100010111010100110001001100000010000101110111101100000010011101110100101100111011000000100011100101010011001101110100001100010011010000110111001001100111000000100110001101000011010101110110001101010111011001110101001101100111000110100000001001000011001110110001001100000010010100110100001100100111000110110100111100000010001001110101001101100111010101110001101100010111011001110000001001100111010101110111001101110011010100110110011100010111000000100000101101010011000000100100101101100111000000100111101100111011000000100110001101111011010101110010011101100111010100110000001000110011011110110010011100000010010000110000001000011011010000110010011101110011010100110000001000110111010010110110001100010111011110110010011101001111001110100001010100001011010100110010011101010011000000100100101101100111000000100100001100000010000110110111101100111011011100110000001001110111010000110100111100000010000101110111101100000010011100110111101100011010000000100000101101111011011101110101001100110111010100110010011100011010000000100101011100111011000101110100101100011011000000100100001100000010001110110111101101011011010010110011101101010011010100110000001001001011011001110000001001100011011110110011101100110011010010110010011101011011010100110001001100111010010100010100001100100111000110110100111100000010001001110101001101100111010101110001101100010111011001110000001001000011000110110110011101111011000000100110011101010111011100110111001101010011011001110001011100000010001100110111101100100111010110110101001100100111000000100011011101001011011000110101001101011010000001110010011101010011011001110100101100010011010100110011101100010111000000100010100101111011010100110000001000100001010010110001001101010011001110110000001000001011010000110110011100000010000001110101001100100111001100110111101100100111010110110101001100010011000000100010001101010011000101110001011101010011001001110000001001001011001110110000001000111001010100110011011101000011000100110100001100000010000101110000101101000011001110110000001001001011001110110000001000010111000010110101001100000010011110110001011100001011010100110010011100000010000101110111011101111011000000100110011100010111010000110001011101010011011001110000001001110111000010110100101101100011000010110000001000001011010000110011011101010011000000100011011101111011000101110101001100010011000000100110011101111011000000100011001101000011001001110011101000001001010100110000001000001011010000110001001100000010010101110011101100010011010100110010011101110111000010110101001100011011010110110100101100111011011100110000001000100111010100110110011101010111000110110001011101100111000000100100101100111011000000100100100101111011011101110100001100000010010000110011101100010011000000100011100101010011011101110000001000001001010000110101101100000111011001110000101101001011001001110101001100111010000000100001010100001011011110110110011101010011000000100110011100010111010000110001011101010011011001110000001001101011010010110110001101101011010100110001001100000010011110110011001100110011000000100001011100001011010100110000001000110011011110110101011100100111010110100101101101111011001110110001011100001011000000100001101101111011001110110111001100000010000001110010011101001011010110110100001100100111010010110101001101100111000000100000011100100111011110110110001101010011011001110110011100011010000000100100101100111011000000100111011100001011010010110110001100001011000000100110001101000011001110110001001101001011000100110100001100010111010100110110011100000010010000110010011101010011000000100010101101111011011001110001011100011011010010110011101101110011000000100001011101111011000000100110001101111011001110110011011101001011001110110110001101010011000000100011011101111011000101110101001100100111011001110000001001110111000010110100111100000010000101110000101101010011010011110000001001000011001001110101001100000010000101110000101101010011000000100010001101010011011001110001011100000010011000110100001100111011000100110100101100010011010000110001011101010011000000100001011101111011000000100110001100001011010000110001101100011011010100110011101101110011010100110000001001011001001001110000001000010101001001110101011101011011000001110011101001700 2600
    //010000101010011001001110011101101001011010100110000001001100101010000110011101100010011010100110010011101100111000000100000101101000011011001110000001001100011010100110101101101010011001110110001011101010011000100110000001000001011010010110110011100000010011001110001011101000011000101110101011101100111000000100100001101100111000000100001011100001011010100110000001000010001010100110101101101111011011000110010011101000011000101110100101101100011000000100011001100100111011110110011101100010111010110100010011101010111001110110011101101010011001001110000001000010111011110110000001000010111010000110110101101010011000000100111101100111011000000100001000101111011001110110100001100011011000100110000001000010101001001110101011101011011000001110000001001001011001110110000001000111001011110110011011101010011010110110010001101010011001001110111001001100111000000100101010101100101000000100000011100100111010100110110011101001011000100110101001100111011000101110100101101000011000110110000001001010011000110110101001101100011000101110100101101111011001110110011101000001001010100110000001001001011011001110000001000000111001001110111101100101011010100110110001100010111010100110001001100000010000101110111101100000010011101110100101100111011000000100011100101010011001101110100001100010011010000110111001001100111000000100110001101000011010101110110001101010111011001110101001101100111000110000000001001000011001110110001001100000010010100110100001100100111000110110100111100000010001001110101001101100111010101110001101100010111011001110000001001100111010101110111001101110011010100110110011100010111000000100000101101010011000000100100101101100111000000100111101100111011000000100110001101111011010101110010011101100111010100110000001000110011011110110010011100000010010000110000001000011011010000110010011101110011010100110000001000110111010010110110001100010111011110110010011101001111001110100001010100001011010100110010011101010011000000100100101101100111000000100100001100000010000110110111101100111011011100110000001001110111010000110100111100000010000101110111101100000010011100110111101100011010000000100000101101111011011101110101001100110111010100110010011100011010000000100101011100111011000101110100101100011011000000100100001100000010001110110111101101011011010010110011101101010011010100110000001001001011011001110000001001100011011110110011101100110011010010110010011101011011010100110001001100111010010100010100001100100111000110110100111100000010001001110101001101100111010101110001101100010111011001110000001001000011000110110110011101111011000000100110011101010111011100110111001101010011011001110001011100000010001100110111101100100111010110110101001100100111000000100011011101001011011000110101001101011010000001110010011101010011011001110100101100010011010100110011101100010111000000100010100101111011010100110000001000100001010010110001001101010011001110110000001000001011010000110110011100000010000001110101001100100111001100110111101100100111010110110101001100010011000000100010001101010011000101110001011101010011001001110000001001001011001110110000001000111001010100110011011101000011000100110100001100000010000101110000101101000011001110110000001001001011001110110000001000010111000010110101001100000010011110110001011100001011010100110010011100000010000101110111011101111011000000100110011100010111010000110001011101010011011001110000001001110111000010110100101101100011000010110000001000001011010000110011011101010011000000100011011101111011000101110101001100010011000000100110011101111011000000100011001101000011001001110011101000001001010100110000001000001011010000110001001100000010010101110011101100010011010100110010011101110111000010110101001100011011010110110100101100111011011100110000001000100111010100110110011101010111000110110001011101100111000000100100101100111011000000100100100101111011011101110100001100000010010000110011101100010011000000100011100101010011011101110000001000001001010000110101101100000111011001110000101101001011001001110101001100111010000000100001010100001011011110110110011101010011000000100110011100010111010000110001011101010011011001110000001001101011010010110110001101101011010100110001001100000010011110110011001100110011000000100001011100001011010100110000001000110011011110110101011100100111010110100101101101111011001110110001011100001011000000100001101101111011001110110111001100000010000001110010011101001011010110110100001100100111010010110101001101100111000000100000011100100111011110110110001101010011011001110110011100011010000000100100101100111011000000100111011100001011010010110110001100001011000000100110001101000011001110110001001101001011000100110100001100010111010100110110011100000010010000110010011101010011000000100010101101111011011001110001011100011011010010110011101101110011000000100001011101111011000000100110001101111011001110110011011101001011001110110110001101010011000000100011011101111011000101110101001100100111011001110000001001110111000010110100111100000010000101110000101101010011010011110000001001000011001001110101001100000010000101110000101101010011000000100010001101010011011001110001011100000010011000110100001100111011000100110100101100010011010000110001011101010011000000100001011101111011000000100110001100000
    waitKey(0);
    return 0;

}
