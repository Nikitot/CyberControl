#include "stdafx.h"
#include "SimpleViewer.h"
#pragma once

class StrucutreFromMotion
{
	GLfloat yRotated;
	Point3f camera_position;

public :
	void init();
	void draw_point_cloud();
	void animation();
	void reshape(int x, int y);
	void calculation_SFM_SVD_old(vector <Point2f> found_opfl_points, vector <Point2f> prev_opfl_points);
	void calculation_SFM_SVD(Mat &image1, Mat &image2, vector <Point2f> found_opfl_points, vector <Point2f> prev_opfl_points);
	void calculation_simple_Z(Mat &img1, Mat &img2, vector <Point2f> found_opfl_points, vector <Point2f> prev_opfl_points);
};

