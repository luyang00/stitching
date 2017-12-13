#include <opencv2/highgui/highgui.hpp>
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/legacy/legacy.hpp"
#include <iostream>
#include <Math.h>
using namespace cv;
using namespace std;

void OptimizeSeam(Mat& img1, Mat& trans, Mat& dst);

//畸变标定
//摄像头固定 安装角度
//ground truth 建立
//透视 变换
//3A 焦距 镜头歪 图像处理
//根据图像做摄像头位置估计
//图像融合
//视频采集。模拟实时
//移植到嵌入式优化

#define IMAGE_WIDTH 1250
#define IMAGE_HEIGHT 1650
#define FRONT_CAMERA cameras_pos[0]
#define BACK_CAMERA cameras_pos[1]
#define LEFT_CAMERA cameras_pos[2]
#define RIGHT_CAMERA cameras_pos[3]
typedef Point2f CameraPos;
int computeDistance(Point2f p1,Point2f p2){
    return sqrt(pow((p1.x-p2.x),2) + pow((p1.y-p2.y),2));
}
int computeMinDistance(Point2f pos,Point2f * cameras,int * dist_rank,float * mixed_factor){
    int closest_distance=1000000;
    int closest_camera = 0;
    int dist[2];
    for(int i=0;i<4;i++){
        int distance = MIN(closest_distance,computeDistance(pos,cameras[i]));
        if(distance < closest_distance ) {
            dist_rank[1] = closest_camera;
            dist[1] = closest_distance;
            dist_rank[0]= i;
            dist[0] = distance;
            closest_camera = i;
            closest_distance = distance;
            
        }
    }
    *mixed_factor = 1;//(float)(dist[1])/( (float)dist[1]+ (float)dist[0]);
    
    return closest_camera;
}
void stitching(Mat* images,Mat &image_stitching,Point2f * cameras){
    int dist_rank[2];
    float mixed_factor;
    for(int i=0;i<image_stitching.rows;i++){
        uchar *p_stitching = image_stitching.ptr<uchar>(i);
        for(int j=0;j<image_stitching.cols;j++){
            
            computeMinDistance(Point2f(j,i),cameras,dist_rank,&mixed_factor);
            //cout<<mixed_factor<<endl;
            p_stitching[j*3] = images[dist_rank[0]].ptr<uchar>(i)[j*3]*mixed_factor;//+
                                //images[dist_rank[1]].ptr<uchar>(i)[j*3]*(1-mixed_factor);
            p_stitching[j*3+1] = images[dist_rank[0]].ptr<uchar>(i)[j*3+1]*mixed_factor;//+
                                //images[dist_rank[1]].ptr<uchar>(i)[j*3+1]*(1-mixed_factor);
            p_stitching[j*3+2] = images[dist_rank[0]].ptr<uchar>(i)[j*3+2]*mixed_factor;//+
                                //images[dist_rank[1]].ptr<uchar>(i)[j*3+2]*(1-mixed_factor);
        }
    }
    //cout<<computeDistance(Point2f(8,4),Point2f(1,1))<<endl;
    
}
void loadColoredImage(Mat * images){
    images[0] = Mat(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8UC3,Scalar(255,255,255));
    images[1] = Mat(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8UC3,Scalar(255/4,255/4,255/4));
    images[2] = Mat(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8UC3,Scalar(255*2/4,255*2/4,255*2/4));
    images[3] = Mat(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8UC3,Scalar(255*3/4,255*3/4,255*3/4));
}
void loadImages(Mat * images){
    images[0] = imread("./input/front_perspectiveImage.png");
    images[1] = imread("./input/back_perspectiveImage.png");
    images[2] = imread("./input/left_perspectiveImage.png");
    images[3] = imread("./input/right_perspectiveImage.png");
    
}
//Given camera position compute perpendicular bisector paramater k and b
Point2f calculateLineParam(const CameraPos c1,const CameraPos c2, float & k, float & b){
    float x1 = c1.x;
    float y1 = c1.y;
    float x2 = c2.x;
    float y2 = c2.y;
    float x0 = (x1+x2)*0.5;
    float y0 = (y1+y2)*0.5;
    
    float k1 = (y2-y1)/(x2-x1);
    k = -1/k1;
    b = (y1+y2)*0.5 - k *(x1+x2)*0.5;
}
Point2f computeROI( Point2f * cameras_pos, Mat & image_stitching,int fusion_width){
    Point2f p;
    float x,y;
    
    //左前交点
    Point2f p0;
    float k02,b02;
    calculateLineParam(FRONT_CAMERA,LEFT_CAMERA,k02,b02);
    x=0;
    y = k02 * x + b02;
    p0.x = x;
    p0.y = y;
    circle(image_stitching,p0,5,Scalar(255,0,255),5);
    //上下
    Point2f p0_f(p0.x,p0.y+fusion_width);
    circle(image_stitching,p0_f,5,Scalar(255,0,255),5);
    Point2f p0_b(p0.x,p0.y-fusion_width);
    circle(image_stitching,p0_b,5,Scalar(255,0,255),5);
    
    //右前交点
    Point2f p1;
    float k03,b03;
    calculateLineParam(FRONT_CAMERA,RIGHT_CAMERA,k03,b03);
    x=1250;
    y = k03 * x + b03;
    p1.x = x;
    p1.y = y;
    circle(image_stitching,p1,5,Scalar(255,0,255),5);
    //上下
    Point2f p1_f(p1.x,p1.y+fusion_width);
    circle(image_stitching,p1_f,5,Scalar(255,0,255),5);
    Point2f p1_b(p1.x,p1.y-fusion_width);
    circle(image_stitching,p1_b,5,Scalar(255,0,255),5);
    
    
    
    //上左右交点
    Point2f p2;
    float intersect_x,intersect_y;
    intersect_x =  (b02-b03)/(k03-k02);
    intersect_y = k02 * intersect_x + b02;
    p2.x = intersect_x;
    p2.y = intersect_y;
    circle(image_stitching,p2,5,Scalar(255,0,255),5);
    //上下
    Point2f p2_f(p2.x,p2.y+fusion_width);
    circle(image_stitching,p2_f,5,Scalar(255,0,255),5);
    Point2f p2_b(p2.x,p2.y-fusion_width);
    circle(image_stitching,p2_b,5,Scalar(255,0,255),5);
    
    
    //上下
    /*
    Point2f p2_f(p2.x,p2.y+fusion_width);
    circle(image_stitching,p2_f,5,Scalar(255,0,255),5);
    Point2f p2_b(p2.x,p2.y-fusion_width);
    circle(image_stitching,p2_b,5,Scalar(255,0,255),5);
    */
    //左后交点
    Point2f p3;
    float k12,b12;
    calculateLineParam(BACK_CAMERA,LEFT_CAMERA,k12,b12);
    x=0;
    y = k12 * x + b12;
    p3.x = x;
    p3.y = y;
    circle(image_stitching,p3,5,Scalar(255,0,255),5);
    
    //右后交点
    Point2f p4;
    float k13,b13;
    calculateLineParam(BACK_CAMERA,RIGHT_CAMERA,k13,b13);
    x=1250;
    y = k13 * x + b13;
    p4.x = x;
    p4.y = y;
    circle(image_stitching,p4,5,Scalar(255,0,255),5);
    
    //下左右交点
    Point2f p5;
    intersect_x =  (b12-b13)/(k13-k12);
    intersect_y = k12 * intersect_x + b12;
    p5.x = intersect_x;
    p5.y = intersect_y;
    circle(image_stitching,p5,5,Scalar(255,0,255),5);
    
    return p;
    
}

void saveMapping(const Mat &image_stitching){
    Mat MappingImage(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8UC1);
    imwrite("./output/result.jpg", image_stitching);
    cvtColor(image_stitching,MappingImage,CV_BGR2GRAY);
    
    imwrite("./output/mapping.jpg", MappingImage);
    
}
int main(int argc, char *argv[])
{
    //fixed camera position
    Point2f cameras_pos[4];
    cameras_pos[0] = Point2f(625,500);//front
    cameras_pos[1] = Point2f(625,1300);//back
    cameras_pos[2]= Point2f(500,800);//left
    cameras_pos[3] = Point2f(750,800);//right

    Mat images[4];
    loadColoredImage(images);
    //loadImages(images);
    
    Mat image_stitching(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8UC3,Scalar(0,0));
    
    cout<<"load images"<<endl;
    cout<<"input image size:"<<images[0].cols<<","<<images[0].rows <<endl;
    cout<<"stitching size:"<<image_stitching.cols<<","<<image_stitching.rows <<endl;
    
    //resize image for display
    Mat scaled_front,scaled_right,scaled_left,scaled_back,scaled_stitching;
    //stitching image
    stitching(images,image_stitching,cameras_pos);
   
    computeROI( cameras_pos,image_stitching,100);
   
    saveMapping(image_stitching);
    //resize & display
    resize(images[0],scaled_front,Size(),0.5,0.5);
    resize(images[1],scaled_back,Size(),0.5,0.5);
    resize(images[2],scaled_left,Size(),0.5,0.5);
    resize(images[3],scaled_right,Size(),0.5,0.5);
    resize(image_stitching,scaled_stitching,Size(),0.5,0.5);
    imshow("front",scaled_front);
    imshow("left",scaled_left);
    imshow("back",scaled_back);
    imshow("right",scaled_right);
    imshow("stitching",scaled_stitching);
    
    waitKey(0);
    
    return 0;
}



