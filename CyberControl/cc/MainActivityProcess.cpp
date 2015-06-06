#include "stdafx.h"
#include "Common.h"
#include "ChessCalibration.h"
#include "CapturesMatching.h"
#include "StereoPairCalibration.h"
#include "MainActivityProcess.h"
#include "StrucutreFromMotion.h"

#include <time.h>

using namespace std;
using namespace cv;
using namespace cv::cuda;


ChessCalibration		*chessCalibration;
CapturesMatching		*capturesMatching;
StereoPairCalibration	*stereoPairCalibration;
StrucutreFromMotion		*strucutreFromMotion;
Common::DistortionMap	*distortionMap;
FlowsSynch synch;


MainActivityProcess::MainActivityProcess() {
	chessCalibration = new ChessCalibration();
	capturesMatching = new CapturesMatching();
	stereoPairCalibration = new StereoPairCalibration();

	distortionMap = new Common::DistortionMap();

	stereoPreFilterSize = 9;
	stereoPreFilterCap = 10;
	stereoSADWindowSize = 21;
	stereoMinDisparity = 0;
	stereoNumDisparities = 4;
	stereoTextureThreshold = 0;
	stereoUniquenessRatio = 30;
}

MainActivityProcess::~MainActivityProcess() {
	delete chessCalibration;
	delete capturesMatching;
	delete stereoPairCalibration;
	delete distortionMap;
}

void MainActivityProcess::createDepthMapFSCBM(IplImage *img0, IplImage *img1, CvMat *disp_visual) {
	CvStereoBMState *state = cvCreateStereoBMState(CV_STEREO_BM_BASIC, 64);
	state->preFilterType = CV_STEREO_BM_NORMALIZED_RESPONSE;
	state->preFilterSize = stereoPreFilterSize;
	state->preFilterCap = stereoPreFilterCap;
	state->SADWindowSize = stereoSADWindowSize;
	state->minDisparity = stereoMinDisparity * 16;
	state->numberOfDisparities = stereoNumDisparities * 16;
	state->textureThreshold = stereoTextureThreshold;
	state->uniquenessRatio = stereoUniquenessRatio;

	state->speckleRange = 18;
	state->disp12MaxDiff = 1;

	CvSize size = cvGetSize(img0);

	IplImage *img0_g = cvCreateImage(size, IPL_DEPTH_8U, 1);
	IplImage *img1_g = cvCreateImage(size, IPL_DEPTH_8U, 1);
	CvMat* disp = cvCreateMat(size.height, size.width, CV_16S);

	IplImage *img0r = cvCreateImage(size, IPL_DEPTH_8U, 1);
	IplImage  *img1r = cvCreateImage(size, IPL_DEPTH_8U, 1);

	cvCvtColor(img0, img0_g, CV_BGR2GRAY);
	cvCvtColor(img1, img1_g, CV_BGR2GRAY);

	cvFindStereoCorrespondenceBM(img0_g, img1_g, disp, state);
	cvNormalize(disp, disp_visual, 0, 255, CV_MINMAX);					//примен€ем полученную матрицу к диапазону 0 - 255 (сам€ €рка€ точка стала = 254, темна€ = 0)

	double framemin = 0;
	double framemax = 0;
	CvPoint p;
	cvMinMaxLoc(disp_visual, &framemin, &framemax, 0, &p);

	double *d = cvGet2D(disp_visual, p.y, p.x).val;

	cvReleaseMat(&disp);
	cvReleaseStereoBMState(&state);
	cvReleaseImage(&img0r);
	cvReleaseImage(&img1r);
	cvReleaseImage(&img0_g);
	cvReleaseImage(&img1_g);

}

void MainActivityProcess::mergeDisps(CvMat* dispVisual1, CvMat* dispVisual2, CvSize size) {
	for (int i = 0; i < size.height; i++) {
		for (int j = 0; j < size.width; j++) {
			CvScalar value1 = cvGet2D(dispVisual1, i, j);
			CvScalar value2 = cvGet2D(dispVisual2, i, j);

			value1.val[0] = value1.val[0] / 2;
			value1.val[1] = value1.val[1] / 2;
			value1.val[2] = value1.val[2] / 2;

			value2.val[0] = 127 + (255 - value2.val[0]) / 2;
			value2.val[1] = 127 + (255 - value2.val[1]) / 2;
			value2.val[2] = 127 + (255 - value2.val[2]) / 2;

			if (value1.val[0]) {
				cvSet2D(dispVisual2, i, j, value2);
				cvSet2D(dispVisual2, i, j, value1);
			}
		}
	}
}

void MainActivityProcess::drawOptFlowMap(Mat flow, Mat &dst, int step)
{
	for (int y = 0; y < flow.rows / step; y++) {
		for (int x = 0; x < flow.cols / step; x++)
		{
			const Point& fxy = flow.at<Point2i>(y*step, x*step);

			Point p1(x*step, y*step);

			Point p2(x*step + fxy.x, y*step + fxy.y);

			double length_fxy = pow(pow(fxy.x, 2) + pow(fxy.y, 2), 0.5);
			double length_xy = pow(pow(step, 2) + pow(step, 2), 0.5);

			double color = length_fxy * 20;
			line(dst, Point(p1.x * 2, p1.y * 2), Point(p2.x * 2, p2.y * 2), CV_RGB(255, 255, 255));
			//circle(dst, Point(p1.x * 2, p1.y * 2), 2, Scalar(int(color), int(color), int(color)));
		}
	}
}

void MainActivityProcess::impositionOptFlow(Mat &dst, Mat &frame0, Mat &frame1) {
	//if (!synch.camera_status[0] || !synch.camera_status[1])
	//	return;

	Mat flow, gray0, gray1;
	cvtColor(frame0, gray0, COLOR_BGR2GRAY);
	resize(gray0, gray0, Size(frame0.cols / 2, frame0.rows / 2));

	cvtColor(frame1, gray1, COLOR_BGR2GRAY);
	resize(gray1, gray1, Size(frame1.cols / 2, frame1.rows / 2));

	calcOpticalFlowFarneback(gray0, gray1, flow, 0.5, 7, 10, 3, 7, 1.5, OPTFLOW_FARNEBACK_GAUSSIAN);
	drawOptFlowMap(flow, dst, 7);
}

void MainActivityProcess::impositionOptFlowLK(vector<Point2f> &prev_features, vector<Point2f> &found_features, Mat prevgray, Mat gray) {
	vector<float> error;
	vector<uchar> status;
	if (synch.get_camera_status(0) && synch.get_camera_status(1) && prev_features.size()) {
		calcOpticalFlowPyrLK(prevgray, gray, prev_features, found_features, status, error
			, opf_parametrs.win_size, opf_parametrs.max_level, opf_parametrs.term_crit, opf_parametrs.lk_flags, opf_parametrs.min_eig_threshold);
	}
}

//void MainActivityProcess::impositionOptFlowLK_GPU(gpu::GpuMat &prevPts, gpu::GpuMat &nextPts, Mat prevgray, Mat gray
//	, gpu::GpuMat &status, gpu::GpuMat &err) {
//	if (synch.camera_status[0] && synch.camera_status[1] && !prevPts.empty()) {
//
//		//GPU using
//		gpu::GpuMat gpuGrayImgA(prevgray);
//		gpu::GpuMat gpuGrayImgB(gray);
//		gpu::PyrLKOpticalFlow    d_pyrLK;
//
//		d_pyrLK.winSize = opf_parametrs.win_size;
//		d_pyrLK.maxLevel = opf_parametrs.max_level;
//		d_pyrLK.iters = 20;
//		d_pyrLK.derivLambda = 0.3;
//		d_pyrLK.useInitialFlow = false;
//
//		d_pyrLK.sparse(gpuGrayImgA, gpuGrayImgB, prevPts, nextPts, status, &err);
//	}
//}

void MainActivityProcess::getReconstuctionFlow() {

	Mat depth_color = imread("color.jpg");
	int depth_max = depth_color.rows;
	
	vector <Point2f> found_opfl_points[2];

	double RAD = 57.2957795;
	
	while (!frame[0].empty() && !frame[1].empty()) {
		while (!synch.is_get_frames());

		Mat local_frame[2] = { frame[0], frame[1] };		
		Mat drawRes = (local_frame[0] + local_frame[1]) / 2;

		vector <Point2f> good_opfl_points = this->good_points[0];

		this->impositionOptFlowLK(good_opfl_points, found_opfl_points[0], gray_frame[0], gray_frame[1]);

		if (good_opfl_points.size() > 0) {

			for (unsigned int i = 0; i < found_opfl_points[0].size(); i++) {
				try {

					double dy = found_opfl_points[0].at(i).y - good_opfl_points.at(i).y;
					double dx = found_opfl_points[0].at(i).x - good_opfl_points.at(i).x;

					double delta = pow(pow(dx, 2) + pow(dy, 2), 0.5);
					double angle = atan(dy / dx) * RAD;

					if (((angle < 10 && angle > -10) || (angle > 170 && angle < -170) || delta < 10)
						&& found_opfl_points[0].at(i).x >= 0 
						&& found_opfl_points[0].at(i).x <= local_frame[0].cols) {
						found_opfl_points[0].at(i).y = good_opfl_points.at(i).y;

						int depth_average = (depth_max / 2);
						int koeff_dif_cameras = 7;
						double i_color = (delta * (depth_average * koeff_dif_cameras / ((local_frame[0].cols))));
						if (i_color > depth_average) i_color = depth_average;

						Scalar color;

						if (dx < 0)
						{
							//i_color *= 10;
							//color = CV_RGB(0, 255 - i_color, 255);
							i_color = depth_average - i_color;
						}
						else
						{
							//color = CV_RGB(255, 255 - i_color, 255 - i_color);
							i_color = depth_average + i_color;
						}

						Vec3b bgrPixel = depth_color.at<Vec3b>(i_color, 0);
						color = Scalar(bgrPixel.val[0], bgrPixel.val[1], bgrPixel.val[2]);
						
						if (dx < 0){
							circle(drawRes, found_opfl_points[0].at(i), 1, color, 2, 8, 0);
							line(drawRes, good_opfl_points.at(i), found_opfl_points[0].at(i), color);
						}
						else{
							circle(drawRes, found_opfl_points[0].at(i), 1, color, 2, 8, 0);
							line(drawRes, good_opfl_points.at(i), found_opfl_points[0].at(i), color);
						}
					}
					else
					{
						//circle(drawRes, found_opfl_points[0].at(i), 1, CV_RGB(200, 0, 0), 2, 8, 0);
						found_opfl_points[0].erase(found_opfl_points[0].begin() + i);
						good_opfl_points.erase(good_opfl_points.begin() + i);
						i--;
					}
				}
				catch (...) {}
			}

			imshow("result", drawRes);

			if (waitKey(33) == 13)
				//strucutreFromMotion->calculation_SFM_SVD(frame[0], frame[1], found_opfl_points[0], good_opfl_points);
				strucutreFromMotion->calculation_simple_Z(frame[0], frame[1], found_opfl_points[0], good_opfl_points);
		}		

		if (waitKey(33) == 27)
			break;

	}
}

void MainActivityProcess::getCameraFramesFlow() {
	
	VideoCapture cap[] = { VideoCapture(CAPTURE_0), VideoCapture(CAPTURE_1) };

	while (cap[0].isOpened() && cap[0].isOpened()) {
		if (waitKey(33) == 27) break; 

		synch.lock_capture_flow();

		for (int i = 0; i < 2; i++){
			cap[i] >> frame[i];

			if (frame[i].cols == 0 || frame[i].rows == 0){
				cout << "[ERR] Frame value is null" << endl;
				synch.set_camera_status(i, false);
				break;
			}
			else
				synch.set_camera_status(i, true);

			
		}

		Common::rotateImage(frame[1], 180);								//переворачиваем изображение второй камеры
		capturesMatching->correctivePerspective(frame[0]);				//корректировка перспективы первой камеры

		cvtColor(frame[0], gray_frame[0], COLOR_BGR2GRAY);
		cvtColor(frame[1], gray_frame[1], COLOR_BGR2GRAY);

		//Mat res(frame[CAPTURE].cols, frame[CAPTURE].rows, frame[CAPTURE].type());
		//resize(frame[CAPTURE], res, Size((int)(res.rows * 1.5), (int) res.cols * 1.5));
		//frame[CAPTURE] = res;

		//if (this->remapFlag) {
		//	this->source = frame[0];
		//	remap(this->source, frame[0], Mat(distortionMap->mapx0), Mat(distortionMap->mapy0), CV_INTER_CUBIC);					// Undistort image
		//}

		imshow("frame0", frame[0]);
		imshow("frame1", frame[1]);

		this->good_points[0].clear();

		Rect rect_gray = Rect(50, 50, 640 - 100, 480 - 100);
		Mat rect_gray_frame = gray_frame[0](rect_gray);
		
		goodFeaturesToTrack(rect_gray_frame, this->good_points[0], this->fd_parametrs.max_сorners, this->fd_parametrs.quality_level, this->fd_parametrs.min_distance, Mat(), this->fd_parametrs.block_size, 0, this->fd_parametrs.k);

		for (int i = 0; i < this->good_points[0].size(); i++){
			good_points[0].at(i).x += rect_gray.x;
			good_points[0].at(i).y += rect_gray.y;
		}

		//gpu::GpuMat gpuGrayImgACopyROI(gray_frame[CAPTURE]);		
		//int maxPtNb = 5000;
		//float qualityLevel = 0.01f;
		//int minimumDistance = 4;
		//
		//// extract features
		//gpu::GoodFeaturesToTrackDetector_GPU detector(maxPtNb, qualityLevel, minimumDistance);		
		//detector(gpuGrayImgACopyROI, gpuGoodCorners[CAPTURE]);

		synch.unlock_capture_flow();

	}
	
	cout << "[INF] cameras is turned off" << endl;
}

void MainActivityProcess::setCalibration(int _argc, char* _argv[]) {
	for (int i = 1; i < _argc; i++) {
		if (_argv[i][1] == 'c') {
			CvSize size = cvSize(frame[0].cols, frame[0].rows);										//update to c++
			distortionMap->mapx0 = capturesMatching->createRemap(size);
			distortionMap->mapy0 = cvCloneImage(distortionMap->mapx0);
			distortionMap->mapx1 = capturesMatching->createRemap(size);
			distortionMap->mapy1 = cvCloneImage(distortionMap->mapx1);

			cout << "[INF] The construction undistortion maps of #0 camera started" << endl;
			chessCalibration->calibration(distortionMap->mapx0, distortionMap->mapy0, this->CAPTURE_0, 3, 3, true);
			cout << "[INF] Construction undistortion maps of #1 camera camera started" << endl;
			chessCalibration->calibration(distortionMap->mapx1, distortionMap->mapy1, this->CAPTURE_1, 3, 3, false);

			bool remapFlag = true;
		}

		if (_argv[i][1] == 's') {
			cout << "[INF] Calibration stereo pair started" << endl;
			stereoPairCalibration->calibration(this->CAPTURE_0, this->CAPTURE_1);								//update to c++
		}
		if (_argv[i][1] == 'D') {
			this->typeStereo = false;
		}
		if (_argv[i][1] == 'B') {
			this->typeStereo = true;
		}
	}
	capturesMatching->openMatrix("./matrix.ini");
}

int MainActivityProcess::waitCameras() {

	while (frame[0].cols == 0 || frame[1].cols == 0);

	if (frame[0].cols == 0 || frame[1].cols == 0)
	{
		cout << "[ERR] Image of one of the cameras is null" << endl;
		return 1;
	}
		if( frame[0].cols != frame[1].cols
		|| frame[0].rows != frame[1].rows) {
		cout << "[ERR] different resilution of cameras" << endl;
		cout << "\t camera #0: w = " << frame[0].cols << "h = " << frame[0].rows << endl;
		cout << "\t camera #1: w = " << frame[1].cols << "h = " << frame[1].rows << endl;
		//return 1;
	}

	cout << "[INF] the flow of images from both cameras started" << endl;

	return 0;
}

int MainActivityProcess::mainActivity(int _argc, char* _argv[]){

	this->setCalibration(_argc, _argv);

	thread cameraFlow = thread(&MainActivityProcess::getCameraFramesFlow, this);
	cameraFlow.detach();

	if (this->waitCameras() == 1) return 1;
	
	thread reconstructionFlow(&MainActivityProcess::getReconstuctionFlow, this);
	reconstructionFlow.detach();

	while (1){
		if (waitKey(33) == 27) break;
	}

	cout << "[INF] shuting down..." << endl;
	cvDestroyAllWindows();
	return 0;
}

int main(int argc, char* argv[]) {
	MainActivityProcess map;
	return map.mainActivity(argc, argv);
}