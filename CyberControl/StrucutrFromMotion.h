#pragma once
class StrucutrFromMotion
{
public:
	StrucutrFromMotion();
	~StrucutrFromMotion();
	void StrucutrFromMotion::calculation_SFM_SVD(Mat &depth_map, vector <Point2f> found_opfl_points, vector <Point2f> prev_opfl_points);
};

