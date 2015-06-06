#pragma once
class StereoPairCalibration
{

public:

	Common *common;
	/*
		 алибровка
		*/
	void calibration(int CAPTURE_0, int CAPTURE_1);
private:
	/*
		находим координаты первого и последнего квадрата шахматной доски
		*/
	int *minCoordinats(CvPoint2D32f *corners, int board_w, int board_h);

	/*
		корректируем перспективу первого изображени€ в соответствии со вторым
		*/
	void correctivePerspective(IplImage *frame0);

	/*
		мен€ем перспективу, пока dif != 0,
		dif - разность кооринат между шахматной доской на двух изображени€х
		*/
	void shiftImage(int dif, int k1, int k2);

	/*
		ѕоиск шахматной доски
		*/
	bool findChess(IplImage *image1, IplImage *image2, char *name);
};

