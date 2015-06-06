#pragma once
class StereoPairCalibration
{

public:

	Common *common;
	/*
		����������
		*/
	void calibration(int CAPTURE_0, int CAPTURE_1);
private:
	/*
		������� ���������� ������� � ���������� �������� ��������� �����
		*/
	int *minCoordinats(CvPoint2D32f *corners, int board_w, int board_h);

	/*
		������������ ����������� ������� ����������� � ������������ �� ������
		*/
	void correctivePerspective(IplImage *frame0);

	/*
		������ �����������, ���� dif != 0,
		dif - �������� �������� ����� ��������� ������ �� ���� ������������
		*/
	void shiftImage(int dif, int k1, int k2);

	/*
		����� ��������� �����
		*/
	bool findChess(IplImage *image1, IplImage *image2, char *name);
};

