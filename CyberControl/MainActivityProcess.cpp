#include "stdafx.h"
#include "Common.h"
#include "ChessCalibration.h"
#include "CapturesMatching.h"
#include "StereoPairCalibration.h"
#include "MainActivityProcess.h"


ChessCalibration		*chessCalibration;
CapturesMatching		*capturesMatching;
StereoPairCalibration	*stereoPairCalibration;

Captures				*captures;
DistortionMap			*distortionMap;
Mat						frame[2] = { Mat(480, 640, 0), Mat(480, 640, 0) };


MainActivityProcess::MainActivityProcess(){
	chessCalibration = new ChessCalibration();
	capturesMatching = new CapturesMatching();
	stereoPairCalibration = new StereoPairCalibration();

	captures = new Captures();
	distortionMap = new DistortionMap();

	stereoPreFilterSize = 9;
	stereoPreFilterCap = 10;
	stereoSADWindowSize = 21;
	stereoMinDisparity = 0;
	stereoNumDisparities = 4;
	stereoTextureThreshold = 0;
	stereoUniquenessRatio = 30;
}

MainActivityProcess::~MainActivityProcess(){
	delete chessCalibration;
	delete capturesMatching;
	delete stereoPairCalibration;

	delete captures;
	delete distortionMap;
}

void MainActivityProcess::createDepthMapFSCBM(IplImage *img0, IplImage *img1, CvMat *disp_visual){
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


	///*********************Establish storage matrix*********************/
	//CvMat* Q = cvCreateMat(4, 4, CV_32FC3);
	//
	//CvMat* rit3dimage = cvCreateMat(disp->height, disp->width, CV_16SC3); //CV_8U   CV_16SC1
	////Call necessary function with input matrix, output matrix, 
	//cvReprojectImageTo3D(disp_visual, rit3dimage, Q);
	//cvShowImage("ReprojectedTo3D", rit3dimage);
	///*****************************************************************/

	double framemin = 0;
	double framemax = 0;
	CvPoint p;
	cvMinMaxLoc(disp_visual, &framemin, &framemax, 0, &p);

	//cvCircle(img0, p, 20, CV_RGB(0, 255, 0), 5);

	//cvShowImage(name, disp_visual);
	//cvSaveImage("reslt.jpg", disp_visual);

	double *d = cvGet2D(disp_visual, p.y, p.x).val;

	cvReleaseMat(&disp);
	cvReleaseStereoBMState(&state);
	cvReleaseImage(&img0r);
	cvReleaseImage(&img1r);
	cvReleaseImage(&img0_g);
	cvReleaseImage(&img1_g);

}

void MainActivityProcess::mergeDisps(CvMat* dispVisual1, CvMat* dispVisual2, CvSize size){
	for (int i = 0; i < size.height; i++){
		for (int j = 0; j < size.width; j++){
			CvScalar value1 = cvGet2D(dispVisual1, i, j);
			CvScalar value2 = cvGet2D(dispVisual2, i, j);

			value1.val[0] = value1.val[0] / 2;
			value1.val[1] = value1.val[1] / 2;
			value1.val[2] = value1.val[2] / 2;

			value2.val[0] = 127 + (255 - value2.val[0]) / 2;
			value2.val[1] = 127 + (255 - value2.val[1]) / 2;
			value2.val[2] = 127 + (255 - value2.val[2]) / 2;

			if (value1.val[0]){
				cvSet2D(dispVisual2, i, j, value2);
				cvSet2D(dispVisual2, i, j, value1);
			}
		}
	}
}

void getCameraFlow(int CAPTURE, VideoCapture *cap, MainActivityProcess *mp){
	cap = new VideoCapture(CAPTURE);

	if (cap->isOpened()){
		cout << "camera # " << CAPTURE << "is statred" << endl;
	}

	while (cap->isOpened()){
		*cap >> frame[CAPTURE];

		if (CAPTURE == 1)
			stereoPairCalibration->common->rotateImage(&frame[1], 180);		//переворачиваем изображение левой камеры
		else
			capturesMatching->correctivePerspective(&frame[0]);				//корректировка перспективы первой камеры


		if (mp->remapFlag){
			mp->source = frame[0];
			remap(mp->source, frame[0], Mat(distortionMap->mapx0), Mat(distortionMap->mapy0), CV_INTER_CUBIC);					// Undistort image
		}

		stereoPairCalibration->common->extractDescriptorsSURF(&mp->keysImage[CAPTURE], frame[CAPTURE]); //Break если все близко, исправить

	}


	cout << "camera # " << CAPTURE << "is turned off" << endl;
}



int main(int _argc, char* _argv[]){
	MainActivityProcess *mp = new MainActivityProcess();
	VideoCapture cap0, cap1;

	thread cameraFlow0(&getCameraFlow, mp->CAPTURE_0, &cap0, mp);
	thread cameraFlow1(&getCameraFlow, mp->CAPTURE_1, &cap1, mp);

	cameraFlow0.detach();
	cameraFlow1.detach();

	if (!frame[0].cols || !frame[1].cols || frame[0].cols != frame[1].cols
		|| frame[0].rows != frame[1].rows){
		cout << "Error: different resilution of cameras" << endl;
		Sleep(300);
		return -1;
	}

	int width = frame[0].cols,
		height = frame[0].rows;

	Mat dispVisual0 = frame[0];
	Mat dispVisual1 = frame[1];



	for (int i = 1; i < _argc; i++){
		if (_argv[i][1] == 'c'){
			CvSize size = cvSize(width, height);										//update to c++
			distortionMap->mapx0 = capturesMatching->createRemap(size);
			distortionMap->mapy0 = cvCloneImage(distortionMap->mapx0);
			distortionMap->mapx1 = capturesMatching->createRemap(size);
			distortionMap->mapy1 = cvCloneImage(distortionMap->mapx1);

			cout << "The construction undistortion maps of FIRST camera started..." << endl;
			chessCalibration->calibration(distortionMap->mapx0, distortionMap->mapy0, mp->CAPTURE_0, 3, 3, true);
			cout << "The construction undistortion maps of SECOND camera started..." << endl;
			chessCalibration->calibration(distortionMap->mapx1, distortionMap->mapy1, mp->CAPTURE_1, 3, 3, false);

			bool remapFlag = true;
		}

		if (_argv[i][1] == 's'){
			cout << "Calibration stereo pair started..." << endl;
			stereoPairCalibration->calibration(captures);								//update to c++
		}
		if (_argv[i][1] == 'D'){
			mp->typeStereo = false;
		}
		if (_argv[i][1] == 'B'){
			mp->typeStereo = true;
		}
	}
	capturesMatching->openMatrix("C:\\Users\\Nikita\\Source\\Repos\\CyberControl\\Debug\\matrix.ini");



	while (1){

		imshow("frame0", frame[0]);
		imshow("frame1", frame[1]);

		if (!_argv[1]) _argv[1] = "C:\\noname\\";

		cout << mp->keysImage[0].descriptors.cols << " " << mp->keysImage[1].descriptors.cols << endl;
		if (mp->keysImage[0].descriptors.cols && mp->keysImage[0].descriptors.rows && mp->keysImage[1].descriptors.cols && mp->keysImage[1].descriptors.cols)
			stereoPairCalibration->common->matchDescriptorsToStereo(&mp->keysImage[0], &mp->keysImage[1], frame);		//не будет работать без синхронизации камер

		//if (typeStereo){
		//	
		//}
		//else{
		//	imshow("fr0", frames->frame0);
		//	imshow("fr1", frames->frame1);

		//mp->createDepthMapFSCBM(&(IplImage)frame[1], &(IplImage)frame[0], &(CvMat)dispVisual1);			//вызов функции расчета карты глубины в близи
		//	//createDepthMapFSCBM(&(IplImage)frames->frame0, &(IplImage)frames->frame1, &(CvMat)dispVisual2);			//вызов функции расчета карты глубины на рассто€нии
		//	cvShowImage("dispVisual1", &(CvMat)dispVisual1);
		//	//cvShowImage("dispVisual2", dispVisual2);
		//	//mergeDisps(&(CvMat)dispVisual1, dispVisual2, size);

		//}

		char c = cvWaitKey(33);
		if (c == 27)		{
			cout << "Shuting down..." << endl;
			break;
		}
	}
	cap0.release();
	cap1.release();
	cvDestroyAllWindows();
	delete mp;
	return 0;
}