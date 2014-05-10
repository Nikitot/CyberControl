#include "stdafx.h"
#include "ChessCalibration.h"
#include "CapturesMatching.h"

int nStereoPreFilterSize;
int nStereoPreFilterCap;
int nStereoSADWindowSize;
int nStereoMinDisparity;
int nStereoNumDisparities;
int nStereoTextureThreshold;
int nStereoUniquenessRatio;


CvPoint lastP;
ChessCalibration *chessCalibration = new ChessCalibration();
CapturesMatching *capturesMatching = new CapturesMatching();

void depthMap(IplImage *img0, IplImage *img1, char *name){

	CvStereoBMState *state = cvCreateStereoBMState(CV_STEREO_BM_BASIC, 64);

	state->preFilterType = CV_STEREO_BM_NORMALIZED_RESPONSE;
	state->preFilterSize = nStereoPreFilterSize;
	state->preFilterCap = nStereoPreFilterCap;
	state->SADWindowSize = nStereoSADWindowSize;
	state->minDisparity = nStereoMinDisparity * 16;
	state->numberOfDisparities = nStereoNumDisparities * 16;
	state->textureThreshold = nStereoTextureThreshold;
	state->uniquenessRatio = nStereoUniquenessRatio;

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
	CvMat* disp_visual = cvCreateMat(size.height, size.width, CV_8U);

	cvNormalize(disp, disp_visual, 0, 255, CV_MINMAX);					//применяем полученную матрицу к диапазону 0 - 255 (самя яркая точка стала = 254, темная = 0)



	////Establish storage matrix
	//CvMat* Q = cvCreateMat(4, 4, CV_32FC3);
	//
	//CvMat* rit3dimage = cvCreateMat(disp->height, disp->width, CV_16SC3); //CV_8U   CV_16SC1
	////Call necessary function with input matrix, output matrix, 
	//cvReprojectImageTo3D(disp_visual, rit3dimage, Q);
	//cvShowImage("ReprojectedTo3D", rit3dimage);



	double framemin = 0;
	double framemax = 0;
	CvPoint p;
	cvMinMaxLoc(disp_visual, &framemin, &framemax, 0, &p);

	//cvCircle(img0, p, 20, CV_RGB(0, 255, 0), 5);

	cvShowImage(name, disp_visual);
	//cvSaveImage(name, disp_visual, 0);

	double *d = cvGet2D(disp_visual, p.y, p.x).val;

	cvReleaseMat(&disp);
	cvReleaseStereoBMState(&state);
	cvReleaseMat(&disp_visual);
	cvReleaseImage(&img0r);
	cvReleaseImage(&img1r);
	cvReleaseImage(&img0_g);
	cvReleaseImage(&img1_g);

}

void initParams(){
	nStereoPreFilterSize = 9;
	nStereoPreFilterCap = 29;
	nStereoSADWindowSize = 21;
	nStereoMinDisparity = 0;
	nStereoNumDisparities = 4;
	nStereoTextureThreshold = 0;
	nStereoUniquenessRatio = 20;
}

//Поиск дескрипторов и вывод изображений с найденными дескрипторами
Mat findDescriptors(IplImage *image, char* name){
	//-- Этап 1. Нахождение ключевых точек.

	//FeatureDetector * detector = new GFTTDetector();
	//FeatureDetector * detector = new FastFeatureDetector();
	//FeatureDetector * detector = new DenseFeatureDetector(1, 1, 0.1, 10,0, true, false);
	//FeatureDetector * detector = new StarFeatureDetector();	
	//FeatureDetector * detector = new BRISK();
	//FeatureDetector * detector = new MSER();
	//FeatureDetector * detector = new ORB();
	//FeatureDetector * detector = new SURF(600);
	FeatureDetector *detector = new SIFT(600);

	vector<KeyPoint> keypoints;
	detector->detect(image, keypoints);
	Mat img_keypoints, descriptors_object;
	drawKeypoints(image, keypoints, img_keypoints, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

	//-- Этап 2. Вычисление дескрипторов.
	SiftDescriptorExtractor extractor;
	extractor.compute(image, keypoints, descriptors_object);

	imshow(name, img_keypoints);
	return descriptors_object;
}

int main(int argc, char* argv[])
{
	CvCapture *capture0 = 0, *capture1 = 0;
	IplImage *mapx0 = 0, *mapy0 = 0, *mapx1 = 0, *mapy1 = 0;
	IplImage *frame0 = 0, *frame1 = 0, *result = 0;
	IplImage *img = 0, *img1 = 0, *imgContours = 0;


	initParams();
	capturesMatching->openMatrix("C:\\matrix.txt");

	capture0 = capturesMatching->cameraFrame(1);
	frame0 = cvQueryFrame(capture0);	//получение изображения камеры
	capture1 = capturesMatching->cameraFrame(2);
	frame1 = cvQueryFrame(capture1);
	if (!frame0 || !frame1){
		cout << "Error: error connecting to captures" << endl;
		return -1;
	}

	mapx0 = capturesMatching->createRemap(cvGetSize(frame0));
	mapy0 = cvCloneImage(mapx0);
	mapx1 = capturesMatching->createRemap(cvGetSize(frame1));
	mapy1 = cvCloneImage(mapx1);
	char* argum = argv[2];


	//начинаем калибровку
	if (argc > 2)
	if (argum[1] == 'c'){
		chessCalibration->calibration(mapx0, mapy0, 0, 10, 10);
		chessCalibration->calibration(mapx1, mapy1, 1, 10, 10);
	}


	lastP = cvPoint(320, 240);
	int rememberShift = -1;

	while (1){
		frame0 = cvQueryFrame(capture0);	//получение изображения камеры
		frame1 = cvQueryFrame(capture1);

		capturesMatching->correctivePerspective(frame0);		//корректировка перспективы одной из камер для более точного результата

		IplImage *t0 = cvCloneImage(frame0);
		IplImage *t1 = cvCloneImage(frame1);
		if (argc > 2){
			if (argum[1] == 'd' || argum[2] == 'd'){
				findDescriptors(frame0, "frame0");
				findDescriptors(frame1, "frame1");
			}
			if (argum[1] == 'c'){
				cvRemap(t0, frame0, mapx0, mapy0); // Undistort image
				cvRemap(t1, frame1, mapx1, mapy1); // Undistort image
			}
		}


		cvReleaseImage(&t0);
		cvReleaseImage(&t1);

		cvShowImage("fr0", frame0);
		cvShowImage("fr1", frame1);

		if (!argv[1]) argv[1] = "C:\\noname\\";

		depthMap(frame1, frame0, argv[1]);			//вызов функции расчета карты глубины

		//IplImage *dst = 0;
		//dst = cvCreateImage(cvGetSize(frame0),frame0->depth, frame0->nChannels);
		//cvAddWeighted(frame0, 0.5, frame1, 0.5, 0.0, dst);
		//cvShowImage("fr0-fr1", dst);
		//cvReleaseImage(&dst);

		char c = cvWaitKey(33);
		if (c == 27)		{
			cvReleaseImage(&img);
			cvReleaseImage(&imgContours);
			break;
		}
		cvReleaseImage(&img);
		cvReleaseImage(&imgContours);
	}

	cvDestroyAllWindows();
	cvReleaseCapture(&capture0);
	cvReleaseCapture(&capture1);
	return 0;
}