#include "stdafx.h"
#include "CapturesMatching.h"

float matrix[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

CvCapture *CapturesMatching::cameraFrame(int number){
	CvCapture *capture = 0;
	capture = cvCreateCameraCapture(number);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 640);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 480);
	return capture;
}

IplImage *CapturesMatching::createRemap(CvSize size){
	IplImage *map = cvCreateImage(size, IPL_DEPTH_32F, 1);
	return map;
}

void CapturesMatching::openMatrix(char *path){
	ifstream file;
	file.open(path);
	if (!file){
		cout << "[ERR] Opening matrix perspective correction a failure" << endl;
	}
	else
		cout << "[INF] Opening matrix perspective correction successfully completed" << endl;

	int cit = 0;
	while (!file.eof()){
		file >> matrix[cit];
		cout << matrix[cit] << "\t";
		switch (cit){
		case 1: cout << ": Top left" << endl;
			break;
		case 3: cout << ": Top right" << endl;
			break;
		case 5: cout << ": Bottom left" << endl;
			break;
		case 7: cout << ": Bottom right" << endl;
			return;
		}
		cit++;
	}
}

//корректируем перспективу первого изображения в соответствии со вторым
void CapturesMatching::correctivePerspective(Mat &frame0){
	int width = frame0.cols;
	int height = frame0.rows;

	// точки
	Point2f srcQuad[4], dstQuad[4];	

	// задаём точки
	srcQuad[0].x = (float)0;					srcQuad[0].y = (float)0;					//src Top left
	srcQuad[1].x = (float)width - 1;			srcQuad[1].y = (float)0;					//src Top right
	srcQuad[2].x = (float)0;					srcQuad[2].y = (float)height - 1;			//src Bottom left
	srcQuad[3].x = (float)width - 1;			srcQuad[3].y = (float)height - (float)0;	//src Bot right
	//- - - - - - - - - - - - - -//
	dstQuad[0].x = -matrix[0];			dstQuad[0].y = matrix[1];				//dst Top left       
	dstQuad[1].x = width + matrix[2];	dstQuad[1].y = matrix[3];				//dst Top right        
	dstQuad[2].x = -matrix[4];			dstQuad[2].y = height + matrix[5];		//dst Bot left          
	dstQuad[3].x = width + matrix[6];	dstQuad[3].y = height + matrix[7];		//dst Bot right
	Mat& transform_matrix = getPerspectiveTransform(srcQuad, dstQuad);
	// получаем матрицу преобразования
	getPerspectiveTransform(srcQuad, dstQuad);
	// преобразование перспективы
	warpPerspective(frame0, frame0, transform_matrix, frame0.size());
}


