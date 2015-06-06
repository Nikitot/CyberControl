#include "stdafx.h"
#include "Common.h"

#include <iostream>

void Common::rotateImage(Mat &frame, int angle){
	Point2f center(frame.cols / 2.0F, frame.rows / 2.0F);
	angle = 180;												// на 60 градусов по часовой стрелке
	double scale = 1;											// масштаб
	Mat rot_matrix = getRotationMatrix2D(center, angle, 1.0);
	Mat rotated_img(Size(frame.size().height, frame.size().width), frame.type());

	warpAffine(frame, frame, rot_matrix, frame.size());		// выполняем вращение

}