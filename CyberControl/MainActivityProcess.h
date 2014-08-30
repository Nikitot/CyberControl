#pragma once
class MainActivityProcess
{
public:
	MainActivityProcess();
	~MainActivityProcess();
	int mainActivity(int _argc, char* _argv[]);
	int CAPTURE_0 = 0,
		CAPTURE_1 = 2;
private:
	Mat source;
	int stereoPreFilterSize,
		stereoPreFilterCap,
		stereoSADWindowSize,
		stereoMinDisparity,
		stereoNumDisparities,
		stereoTextureThreshold,
		stereoUniquenessRatio;

	void createDepthMapFSCBM(IplImage *img0, IplImage *img1, CvMat *disp_visual);
	void mergeDisps(CvMat* dispVisual1, CvMat* dispVisual2, CvSize size);


	
};