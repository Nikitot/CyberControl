#include "stdafx.h"
#include "SimpleViewer.h"

class StrucutreFromMotion
{
	GLfloat yRotated;
	cv::Point3f camera_position;

public :
	void calculation_SFM_SVD(cv::Mat &image1, cv::Mat &image2
		, vector <cv::Point2f> found_opfl_points, vector <cv::Point2f> prev_opfl_points);
	void calculation_simple_Z(cv::Mat img1, cv::Mat img2
		, vector <cv::Point2f> found_opfl_points, vector <cv::Point2f> prev_opfl_points);
	void simpleDrawGL();
	void initDrawGL(int argc, char** argv);
};