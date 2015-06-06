#pragma once
class CapturesMatching
{
public:
	CvCapture* cameraFrame(int);
	IplImage* createRemap(CvSize);
	void openMatrix(char *);
	void correctivePerspective(Mat &);
};

