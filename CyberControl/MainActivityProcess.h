#pragma once
class MainActivityProcess
{
public:
	MainActivityProcess();
	~MainActivityProcess();
	int mainActivity(int _argc, char* _argv[]);
	int CAPTURE_0 = 0,
		CAPTURE_1 = 1;
	Mat source;
	bool remapFlag = false;
	bool typeStereo = false;

	KeysImage keysImage[2];

	int stereoPreFilterSize,
		stereoPreFilterCap,
		stereoSADWindowSize,
		stereoMinDisparity,
		stereoNumDisparities,
		stereoTextureThreshold,
		stereoUniquenessRatio;

	void createDepthMapFSCBM(IplImage *img0, IplImage *img1, CvMat *disp_visual);
	void mergeDisps(CvMat* dispVisual1, CvMat* dispVisual2, CvSize size);
	void getCameraFlow(int CAPTURE, VideoCapture cap);
	//private:


};