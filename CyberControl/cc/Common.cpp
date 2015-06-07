#include "stdafx.h"
#include "Common.h"

#include <iostream>
using namespace cv;

void Common::rotateImage(Mat &frame, int angle){
	Point2f center(frame.cols / 2.0F, frame.rows / 2.0F);
	angle = 180;												// на 60 градусов по часовой стрелке
	double scale = 1;											// масштаб
	Mat rot_matrix = getRotationMatrix2D(center, angle, 1.0);
	Mat rotated_img(Size(frame.size().height, frame.size().width), frame.type());

	warpAffine(frame, frame, rot_matrix, frame.size());		// выполняем вращение
}

void Common::mergeImages(Mat &appended, Mat image1, Mat image2){
	/* Briefly see how our original images look like */
	appended = Mat(image1.rows, image1.cols + image1.cols, image1.type());
	image1.copyTo(Mat(appended, Rect(0, 0, image1.cols, image1.rows)));
	image2.copyTo(Mat(appended, Rect(image1.cols, 0, image2.cols, image2.rows)));
	//resize(appended, appended, Size(), 0.3f, 0.3f);
}