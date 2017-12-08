#include "stdafx.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
void drawOnImage(Mat &image,
	const std::vector<Point> &points,
	Scalar color = Scalar(255, 255, 255),
	int radius = 3,
	int thickness = 2) {
	std::vector<Point>::const_iterator it = points.begin();
	while (it != points.end()) {
		circle(image, *it, radius, color, thickness);
		++it;
	}
	std::cout << points << std::endl;
}
int main()
{
	// get original image.
	cv::Mat originalImage = cv::imread("c:/test/TestOutput.jpg");
	cv::namedWindow("xx", cv::WINDOW_AUTOSIZE);

	// perspective image.
	cv::Mat perspectiveImage;

	// perspective transform
	cv::Point2f objectivePoints[4], imagePoints[4];
	std::vector<Point> points;

	// original image points.
// 	imagePoints[0].x = 483.0; imagePoints[0].y = 517.0;
// 	imagePoints[1].x = 739.0; imagePoints[1].y = 513.0;
// 	imagePoints[2].x = 560.0; imagePoints[2].y = 396.0;
// 	imagePoints[3].x = 690.0; imagePoints[3].y = 394.0;


	imagePoints[0].x = 1025.0; imagePoints[0].y = 966.0;
	imagePoints[1].x = 1437.0; imagePoints[1].y = 956.0;
	imagePoints[2].x = 1357.0; imagePoints[2].y = 768.0;
	imagePoints[3].x = 1147.0; imagePoints[3].y = 770.0;


	points.push_back(imagePoints[0]);
	points.push_back(imagePoints[1]);
	points.push_back(imagePoints[2]);
	points.push_back(imagePoints[3]);
	drawOnImage(originalImage, points);
	cv::imshow("xx", originalImage);
	cv::waitKey(0);

	// objective points of perspective image.
	// move up the perspective image : objectivePoints.y - value .
	// move left the perspective image : objectivePoints.x - value.
	double moveValueX = 650.0;
	double moveValueY = 50.0;

	objectivePoints[0].x = 0.0 + moveValueX; objectivePoints[0].y = 0.0 + moveValueY;
	objectivePoints[1].x = 120.0 + moveValueX; objectivePoints[1].y = 0.0 + moveValueY;
	objectivePoints[2].x = 120.0 + moveValueX; objectivePoints[2].y = 80.0 + moveValueY;
	objectivePoints[3].x = 0.0 + moveValueX; objectivePoints[3].y = 80.0 + moveValueY;

	std::vector<Point> points_perspective;
	points_perspective.push_back(objectivePoints[0]);
	points_perspective.push_back(objectivePoints[1]);
	points_perspective.push_back(objectivePoints[2]);
	points_perspective.push_back(objectivePoints[3]);

	cv::Mat transform = cv::getPerspectiveTransform(objectivePoints, imagePoints);
	std::cout << transform << std::endl;
	// perspective.
	cv::warpPerspective(originalImage,
		perspectiveImage,
		transform,
		cv::Size(originalImage.rows, originalImage.cols),
		cv::INTER_LINEAR | cv::WARP_INVERSE_MAP);

	// cv::imshow("perspective image", perspectiveImage);
	// cvWaitKey(0);

	cv::imwrite("perspectiveImage1.png", perspectiveImage);
	drawOnImage(perspectiveImage, points_perspective);
	cv::imshow("result", perspectiveImage);
	cv::waitKey(0);
	
	return 0;
}