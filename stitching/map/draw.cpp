// fisheye.cpp : 定义控制台应用程序的入口点。
//

#include <opencv2\opencv.hpp>
#include <fstream>
using namespace std;
using namespace cv;

int main()
{
	Mat map = imread("c:/test/1218/map0.png", CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
	Mat testImage = imread("c:/test/1218/u0.jpg");
	Mat mapx, mapy,output;
	vector<Mat> ch3;
	vector<Mat> ch2(2);


	//split channel
	split(map, ch3);
	ch2[0] = ch3[0].clone();
	ch2[1] = ch3[1].clone();
	mapy = ch3[2].clone();
	merge(&ch2[0],2,mapx);

	//16U to 16S
	mapx.convertTo(mapx, CV_16SC2);

	cv::remap(testImage, output, mapx, mapy, INTER_LINEAR);
	imshow("result", output);
	waitKey(0);

	return 0;
}

