#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/legacy/legacy.hpp"
#include <iostream>

//for compute time
#include <time.h>
#include <iomanip>

using namespace cv;
using namespace std;

void OptimizeSeam(Mat& img1, Mat& trans, Mat& dst);


#define IMAGE_WIDTH 1250
#define IMAGE_HEIGHT 1650

#define TOTAL_ROI 8
/*camera position*/
#define FRONT_CAMERA 0
#define BACK_CAMERA 1
#define LEFT_CAMERA 2
#define RIGHT_CAMERA 3
#define FUSION_WIDTH 100
struct ROI{
    vector<Point> vertices;
};

typedef Point2f CameraPos;

Mat masks_my12doom[8];

#ifdef WIN32
#ifdef DEBUG
#pragma comment(lib, "opencv_highgui243d.lib")
#pragma comment(lib, "opencv_core243d.lib")
#pragma comment(lib, "opencv_video243d.lib")
#pragma comment(lib, "opencv_imgproc243d.lib")
#pragma comment(lib, "opencv_stitching243d.lib")
#else
#pragma comment(lib, "opencv_highgui243.lib")
#pragma comment(lib, "opencv_core243.lib")
#pragma comment(lib, "opencv_video243.lib")
#pragma comment(lib, "opencv_imgproc243.lib")
#pragma comment(lib, "opencv_stitching243.lib")
#endif
#endif

void loadColoredImage(Mat * images){
    images[0] = Mat(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8UC3,Scalar(255,255,255));
    images[1] = Mat(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8UC3,Scalar(255/4,255/4,255/4));
    images[2] = Mat(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8UC3,Scalar(255*2/4,255*2/4,255*2/4));
    images[3] = Mat(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8UC3,Scalar(255*3/4,255*3/4,255*3/4));
}
void loadImages(Mat * images_8UC3,Mat * images_32FC3 ){
    images_8UC3[0] = imread("front_perspectiveImage.jpg");
    images_8UC3[1] = imread("left_perspectiveImage.png");
    images_8UC3[2] = imread("back_perspectiveImage.png");
    images_8UC3[3] = imread("right_perspectiveImage.png");
    
    images_8UC3[0].convertTo( images_32FC3[0], CV_32FC3, 1.0/255);
    images_8UC3[1].convertTo( images_32FC3[1], CV_32FC3, 1.0/255);
    images_8UC3[2].convertTo( images_32FC3[2], CV_32FC3, 1.0/255);
    images_8UC3[3].convertTo( images_32FC3[3], CV_32FC3, 1.0/255);
    
}

//Given camera position compute perpendicular bisector paramater k and b
void calculateLineParam(const CameraPos c1,const CameraPos c2, float & k, float & b){
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


void makeROI(ROI &roi,Point2f p0, Point2f p1,Point2f p2,Point2f p3,Point2f p4){
    roi.vertices.push_back(p0);
    roi.vertices.push_back(p1);
    roi.vertices.push_back(p2);
    roi.vertices.push_back(p3);
    roi.vertices.push_back(p4);
    
}
void makeROI(ROI &roi,Point2f p0, Point2f p1,Point2f p2,Point2f p3){
    roi.vertices.push_back(p0);
    roi.vertices.push_back(p1);
    roi.vertices.push_back(p2);
    roi.vertices.push_back(p3);
    
}
Point2f computeROI( ROI * roi,Point2f * cameras_pos,int fusion_width){
    
    Point2f p;
	Point2f p2;
    float x,y;
    //左前交点
    Point2f p0;
    float k02,b02;
    calculateLineParam(cameras_pos[FRONT_CAMERA],cameras_pos[LEFT_CAMERA],k02,b02);
    x=0;
    y = k02 * x + b02;
    p0.x = x;
    p0.y = y;
    ////circle(image_stitching,p0,5,Scalar(255,0,255),5);
    //上下
    Point2f p0_f(p0.x,p0.y-fusion_width);
    ////circle(image_stitching,p0_f,5,Scalar(255,0,255),5);
    Point2f p0_b(p0.x,p0.y+fusion_width);
    ////circle(image_stitching,p0_b,5,Scalar(255,0,255),5);
    //右前交点
    Point2f p1;
    float k03,b03;
    calculateLineParam(cameras_pos[FRONT_CAMERA],cameras_pos[RIGHT_CAMERA],k03,b03);
    x=IMAGE_WIDTH;
    y = k03 * x + b03;
    p1.x = x;
    p1.y = y;
    //circle(image_stitching,p1,5,Scalar(255,0,255),5);
    //上下
    Point2f p1_f(p1.x,p1.y-fusion_width);
    //circle(image_stitching,p1_f,5,Scalar(255,0,255),5);
    Point2f p1_b(p1.x,p1.y+fusion_width);
    //circle(image_stitching,p1_b,5,Scalar(255,0,255),5);
    //上左右交点
    float intersect_x,intersect_y;
    intersect_x =  (b02-b03)/(k03-k02);
    intersect_y = k02 * intersect_x + b02;
    p2.x = intersect_x;
    p2.y = intersect_y;
    //circle(image_stitching,p2,5,Scalar(255,0,255),5);
    //上下
    Point2f p2_f(p2.x,p2.y-fusion_width);
    //circle(image_stitching,p2_f,5,Scalar(255,0,255),5);
    Point2f p2_b(p2.x,p2.y+fusion_width);
    //circle(image_stitching,p2_b,5,Scalar(255,0,255),5);
    //左后交点
    Point2f p3;
    float k12,b12;
    calculateLineParam(cameras_pos[BACK_CAMERA],cameras_pos[LEFT_CAMERA],k12,b12);
    x=0;
    y = k12 * x + b12;
    p3.x = x;
    p3.y = y;
    //circle(image_stitching,p3,5,Scalar(255,0,255),5);
    Point2f p3_f(p3.x,p3.y-fusion_width);
    //circle(image_stitching,p3_f,5,Scalar(255,0,255),5);
    Point2f p3_b(p3.x,p3.y+fusion_width);
    //circle(image_stitching,p3_b,5,Scalar(255,0,255),5);
    //右后交点
    Point2f p4;
    float k13,b13;
    calculateLineParam(cameras_pos[BACK_CAMERA],cameras_pos[RIGHT_CAMERA],k13,b13);
    x=IMAGE_WIDTH;
    y = k13 * x + b13;
    p4.x = x;
    p4.y = y;
    //circle(image_stitching,p4,5,Scalar(255,0,255),5);
    Point2f p4_f(p4.x,p4.y-fusion_width);
    //circle(image_stitching,p4_f,5,Scalar(255,0,255),5);
    Point2f p4_b(p4.x,p4.y+fusion_width);
    //circle(image_stitching,p4_b,5,Scalar(255,0,255),5);
    //下左右交点
    Point2f p5;
    intersect_x =  (b12-b13)/(k13-k12);
    intersect_y = k12 * intersect_x + b12;
    p5.x = intersect_x;
    p5.y = intersect_y;
    //circle(image_stitching,p5,5,Scalar(255,0,255),5);
    
    Point2f p5_f(p5.x,p5.y-fusion_width);
    //circle(image_stitching,p5_f,5,Scalar(255,0,255),5);
    Point2f p5_b(p5.x,p5.y+fusion_width);
    //circle(image_stitching,p5_b,5,Scalar(255,0,255),5);
    
    
    //Make 8 ROI
    makeROI(roi[0],Point2f(0,0),p0_f,p2_f,p1_f,Point2f(IMAGE_WIDTH,0));
    makeROI(roi[1],p0_f,p0_b,p2_b,p2_f);
    makeROI(roi[2],p0_b,p3_f,p5_f,p2_b);
    makeROI(roi[3],p3_f,p3_b,p5_b,p5_f);
    makeROI(roi[4],p3_b,Point2f(0,IMAGE_HEIGHT),Point2f(IMAGE_WIDTH,IMAGE_HEIGHT),p4_b,p5_b);
    makeROI(roi[5],p5_f,p5_b,p4_b,p4_f);
    makeROI(roi[6],p2_b,p5_f,p4_f,p1_b);
    makeROI(roi[7],p2_f,p2_b,p1_b,p1_f);
    
    
    
    
    return p;
    
}

void makeMask(ROI &roi,Mat & mask){
    mask = Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1,Scalar(0));


    vector<Point> ROI_Poly;

	double oops[9999] = {0};
    
    //approximate precision paramater
    approxPolyDP(roi.vertices, ROI_Poly, 1.0, true);
    // Fill polygon white
    fillConvexPoly(mask, &ROI_Poly[0], ROI_Poly.size(), 255, 8, 0);
    imshow("fff",mask);
   
}
/*Alpha blending*/
void make3DMask(ROI &roi,Mat & p_masks_fusion,Mat & n_masks_fusion,int fusion_width){
    //generate positive mask_fusion
    p_masks_fusion = Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_32FC3,Scalar(0,0,0));
    vector<Point> ROI_Poly,vertices;
    //make sure not overflow
    float y_start = (roi.vertices[0].y - 2.0 > 0)?(roi.vertices[0].y-2.0):0;
    float y_end =(roi.vertices[3].y + 2.0 < IMAGE_HEIGHT)?(roi.vertices[3].y + 2.0):IMAGE_HEIGHT;
    vertices.push_back(Point(roi.vertices[0].x,y_start));
    vertices.push_back(Point(roi.vertices[0].x,y_start+4));
    vertices.push_back(Point(roi.vertices[3].x,y_end));
    vertices.push_back(Point(roi.vertices[3].x,y_end-4));
    
    fusion_width =fusion_width*2;
    
    for(int i=0;i<fusion_width;i++){
        vertices[0].y+=1;
        vertices[1].y = vertices[0].y + 1;
        vertices[3].y+=1;
        vertices[2].y = vertices[3].y + 1;
        approxPolyDP(vertices, ROI_Poly, 1.0, true);
        //tricky way to fill with flexible weight
        fillConvexPoly(p_masks_fusion, &ROI_Poly[0], ROI_Poly.size(), Scalar(1-i/(float)fusion_width,1-i/(float)fusion_width,1-i/(float)fusion_width), 8, 0);
        
        //cout<<1-i/(float)fusion_width<<endl;
    }
    //generate negative mask_fusion = 1-positive mask_fusion
    Mat mask = Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_32FC3,Scalar(0,0,0));
    approxPolyDP(roi.vertices, ROI_Poly, 1.0, true);
    // Fill polygon white
    fillConvexPoly(mask, &ROI_Poly[0], ROI_Poly.size(), Scalar(1.0,1.0,1.0), 8, 0);
    
    subtract(mask,p_masks_fusion,n_masks_fusion);
    /*
    Mat scaled,scaled1,scaled2;
    resize(mask,scaled,Size(),0.5,0.5);
    resize(p_masks_fusion,scaled1,Size(),0.5,0.5);
    resize(n_masks_fusion,scaled2,Size(),0.5,0.5);
    
    imshow("mask",scaled);
    imshow("pmask",scaled1);
    imshow("nmask",scaled2);
    */
    
}


void makeMasks(Mat * masks,Mat * masks_fusion,CameraPos * cameras_pos,int fusion_width){
    ROI roi[TOTAL_ROI];
    computeROI(roi, cameras_pos,fusion_width);
    
    for(int i=0;i<TOTAL_ROI;i++){
        makeMask(roi[i],masks[i]);
    }
    //fusion ROI
    make3DMask(roi[1],masks_fusion[0],masks_fusion[4],fusion_width);
    make3DMask(roi[3],masks_fusion[1],masks_fusion[5],fusion_width);
    make3DMask(roi[5],masks_fusion[2],masks_fusion[6],fusion_width);
    make3DMask(roi[7],masks_fusion[3],masks_fusion[7],fusion_width);
   
}

void saveMapping(const Mat &image_stitching){
    Mat MappingImage(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8UC1);
    imwrite("result.jpg", image_stitching);
    cvtColor(image_stitching,MappingImage,CV_BGR2GRAY);
    
    imwrite("mapping.jpg", MappingImage);
    
}

#ifdef WIN32
#include <Windows.h>
#include <intrin.h>
#include <tmmintrin.h>
#else
#include <x86intrin.h>
#include <tmmintrin.h>
#endif

void stiching_my12doom(Mat *images, Mat *masks, Mat &image_out)
{
	memset(image_out.data, 0, image_out.rows * image_out.cols * 3);
	for(int i=0; i<4; i++)
	{
		if (!(masks[i].type() == 16 && images[i].type() == 16 && image_out.type() == 16))
		{
			printf("arguments error\n");
			return;
		}

		int count = (images->rows * images->cols * 3) / 16;
		__m128i zero = _mm_set1_epi8(0);
		__m128i *pimage = (__m128i *)images[i].data;
		__m128i *pmask = (__m128i *)masks[i].data;
		__m128i *pout = (__m128i *)image_out.data;
		for(int j=0; j<count; j++)
		{
			__m128i image = _mm_loadu_si128(pimage);
			__m128i mask = _mm_loadu_si128(pmask);
			__m128i out = _mm_loadu_si128(pout);
			__m128i image1, image2;
			__m128i mask1, mask2;

			image1 = _mm_unpackhi_epi8(image, zero);
			image2 = _mm_unpacklo_epi8(image, zero);
			mask1 = _mm_unpackhi_epi8(mask, zero);
			mask2 = _mm_unpacklo_epi8(mask, zero);

			image1 = _mm_mullo_epi16(image1, mask1);
			image2 = _mm_mullo_epi16(image2, mask2);

			image1 = _mm_srai_epi16(image1, 8);
			image2 = _mm_srai_epi16(image2, 8);
			image = _mm_packs_epi16(image2, image1);
			
// 			image = _mm_blendv_epi8(zero, image, mask);
			out = _mm_add_epi8(out, image);

			_mm_storeu_si128(pout, out);

			pimage ++;
			pmask ++;
			pout ++;
		}
	}
}

void stitching(const Mat * images_32FC3,const Mat * masks,const Mat * masks_fusion,Mat &image_stitching){
    
    
    
   
    //There is no need of fusion for ROI 0,2,4,6
    for(int i=0;i<TOTAL_ROI;i+=2){
        //add(image_stitching,images_32FC3[i/2],image_stitching,masks[i]);
        
        images_32FC3[i/2].copyTo(image_stitching,masks[i]);/*copyTo is totaly 10ms faster than add*/
    }
    
    
    //ROI1
    Mat layer1,layer2,mixed_layer;
    multiply(images_32FC3[0],masks_fusion[0],layer1);
    multiply(images_32FC3[1],masks_fusion[4],layer2);
    add(layer1,layer2,mixed_layer,masks[1]);
    mixed_layer.copyTo(image_stitching,masks[1]);
   
    //ROI3
   
    multiply(images_32FC3[1],masks_fusion[1],layer1);
    multiply(images_32FC3[2],masks_fusion[5],layer2);
    add(layer1,layer2,mixed_layer,masks[3]);
    mixed_layer.copyTo(image_stitching,masks[3]);
    //ROI5
  
    multiply(images_32FC3[3],masks_fusion[2],layer1);
    multiply(images_32FC3[2],masks_fusion[6],layer2);
    add(layer1,layer2,mixed_layer,masks[5]);
    mixed_layer.copyTo(image_stitching,masks[5]);
    //ROI7
   
    multiply(images_32FC3[0],masks_fusion[3],layer1);
    multiply(images_32FC3[3],masks_fusion[7],layer2);
    add(layer1,layer2,mixed_layer,masks[7]);
    mixed_layer.copyTo(image_stitching,masks[7]);
    
    //imshow("mixed",mixed_layer);
    
	imshow("masks_fusion[0]",masks_fusion[0]);
	imshow("masks_fusion[4]",masks_fusion[4]);
   
}
void saveConfiguration(Mat * masks,Mat * masks_fusion,CameraPos * cameras_pos,int fusion_width){
    
}
void loadConfiguration(Mat * masks,Mat * masks_fusion,CameraPos * cameras_pos,int fusion_width){
    
}

#include <stdint.h>

int test()
{
	uint8_t da[16] = {1,2,3,4,5,6,7,8,9};
	__m128i p = _mm_loadu_si128((__m128i*)da);

	__m128i zero = _mm_set1_epi8(0);
	__m128i tmp = _mm_unpacklo_epi8(p, zero);
	__m128i tmp2 = _mm_unpackhi_epi8(p, zero);
	tmp = _mm_mullo_epi16(tmp, tmp);

	return 0;
}

int main(int argc, char *argv[])
{
  test();
    //fixed camera position
    Point2f cameras_pos[4];
    cameras_pos[FRONT_CAMERA] = Point2f(625,500);//front
    cameras_pos[BACK_CAMERA] = Point2f(625,1300);//back
    cameras_pos[LEFT_CAMERA]= Point2f(500,800);//left
    cameras_pos[RIGHT_CAMERA] = Point2f(750,800);//right

    Mat masks[8];
    Mat masks_fusion[8];
    makeMasks(masks,masks_fusion,cameras_pos,FUSION_WIDTH);
    
    saveConfiguration(masks,masks_fusion,cameras_pos,FUSION_WIDTH);
    
    Mat images_8UC3[4],images_32FC3[4];//currently not use image 8UC3 because of the need of matrix multiply
    
    cout<<"load images"<<endl;
    //loadColoredImage(images);
    loadImages(images_8UC3,images_32FC3);
    
    Mat image_stitching(IMAGE_HEIGHT,IMAGE_WIDTH,CV_32FC3,Scalar(0,0,0));
    
    
    cout<<"load configuration"<<endl;
    saveConfiguration(masks,masks_fusion,cameras_pos,FUSION_WIDTH);
   
    //cout<<"input image size:"<<images[0].cols<<","<<images[0].rows <<endl;
    //cout<<"stitching size:"<<image_stitching.cols<<","<<image_stitching.rows <<endl;
    
   
    //stitching image
    //stitching2(images,image_stitching,cameras_pos);

	Mat tmp;
	int tbl[4][2] = {{0,3}, {4,1}, {5,6}, {2,7}};
	for(int i=0; i<4; i++)
	{
		add(masks_fusion[tbl[i][0]], masks_fusion[tbl[i][1]], tmp);
		cvtColor(tmp, tmp, CV_BGR2GRAY);
		tmp.convertTo(tmp, CV_8U, 255);
		add(tmp, masks[i*2], masks_my12doom[i]);
		cvtColor(masks_my12doom[i], masks_my12doom[i], CV_GRAY2BGR);
	}
	imshow("m", masks_my12doom[0]);
	imshow("i", images_8UC3[0]);
	printf("type:%d\n", masks_my12doom[0].type());


    clock_t begin = clock();
    
	image_stitching = Mat(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8UC3,Scalar(0,0,0));
    for(int i=0;i<1000;i++){
//         image_stitching = Mat(IMAGE_HEIGHT,IMAGE_WIDTH,CV_32FC3,Scalar(0,0,0));
//         stitching(images_32FC3,masks,masks_fusion,image_stitching);
		stiching_my12doom(images_8UC3, masks_my12doom, image_stitching);
    }
    clock_t end = clock();
   
	float t = ((double)(end-begin)*1000/CLOCKS_PER_SEC)/1000.0;
    printf("Average Running time: %.3fms\n", t);
   
    
    
    saveMapping(image_stitching);
    //resize & display
    Mat scaled_front,scaled_right,scaled_left,scaled_back,scaled_stitching;
    resize(images_8UC3[0],scaled_front,Size(),0.5,0.5);
    resize(images_8UC3[1],scaled_back,Size(),0.5,0.5);
    resize(images_8UC3[2],scaled_left,Size(),0.5,0.5);
    resize(images_8UC3[3],scaled_right,Size(),0.5,0.5);
    resize(image_stitching,scaled_stitching,Size(),0.5,0.5);
    /*
    imshow("front",scaled_front);
    imshow("left",scaled_left);
    imshow("back",scaled_back);
    imshow("right",scaled_right);
    */
    imshow("stitching",scaled_stitching);
    
    //testkFill();
    waitKey(0);
    
    return 0;
}




