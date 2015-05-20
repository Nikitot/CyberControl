#include "stdafx.h"
#pragma once
class StrucutreFromMotion
{
public:
	void init();
	static void draw_point_cloud();
	static void animation();
	static void reshape(int x, int y);
	void calculation_SFM_SVD_old(vector <Point2f> found_opfl_points, vector <Point2f> prev_opfl_points);
	void calculation_SFM_SVD(Mat &image1, Mat &image2, vector <Point2f> found_opfl_points, vector <Point2f> prev_opfl_points);
	void calculation_simple_Z(Mat &img1, Mat &img2, vector <Point2f> found_opfl_points, vector <Point2f> prev_opfl_points);
};

