#pragma once
class MainActivityProcess
{
	Common::KeysImage keysImage[2];
	bool remapFlag = false;
	Mat source;
	vector<Point2f>			good_points[2];
	Mat						frame[2];
	Mat						gray_frame[2];

	struct opt_flow_parametrs {
		Size win_size = Size(11, 11);
		int max_level = 10;
		TermCriteria term_crit = TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03);
		int deriv_lamda = 0;
		int lk_flags = 0;
		double min_eig_threshold = 0.01;
	} opf_parametrs;

	struct feature_detect_parametrs {
		Size win_size = Size(5, 5);
		int max_ñorners = 1000;
		double quality_level = 0.001;
		double min_distance = 15;
		int block_size = 3;
		double k = 0.05;
	} fd_parametrs;

	int CAPTURE_0 = 0,	CAPTURE_1 = 1;
		
	bool typeStereo = false;

	int stereoPreFilterSize,
		stereoPreFilterCap,
		stereoSADWindowSize,
		stereoMinDisparity,
		stereoNumDisparities,
		stereoTextureThreshold,
		stereoUniquenessRatio;

	void createDepthMapFSCBM(IplImage *img0, IplImage *img1, CvMat *disp_visual);
	void mergeDisps(CvMat* dispVisual1, CvMat* dispVisual2, CvSize size);
	int waitCameras();
	void setCalibration(int _argc, char* _argv[]);

	void drawOptFlowMap(Mat flow, Mat &dst, int step);

	void getReconstuctionFlow();
	void getCameraFramesFlow(int CAPTURE);


	void impositionOptFlow(Mat &dst, Mat &frame0, Mat &frame1);
	void impositionOptFlowLK(vector<Point2f> &prev_features, vector<Point2f> &found_features, Mat prevgray, Mat gray
		, vector<float> &error, vector<uchar> &status);

public:

	MainActivityProcess();
	~MainActivityProcess();

	int mainActivity(int _argc, char* _argv[]);
	
};

class FlowsSynch
{
	
	bool	synch_flag[3];
public:

	bool	camera_status[2];
	void lock_capture_flow(VideoCapture cap){
		while(((synch_flag[0] != synch_flag[1]) || synch_flag[2] == true) && cap.isOpened());
	}

	void lock_reconstruction_flow(){
		while (synch_flag[2] == false);
	}

	void unlock_capture_flow(int CAPTURE){
		synch_flag[CAPTURE] = !synch_flag[CAPTURE];
		synch_flag[2] = true;
	}

	void unlock_reconstruction_flow(){
		synch_flag[2] = false;
	}
};