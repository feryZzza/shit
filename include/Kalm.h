#pragma once
typedef struct {
	float filterValue;
	float kalmanGain;
	float A;
	float H;
	float Q;
	float R;
	float P;
	float B;
	float u;
	float predictValue;
}KalmanInfo;
class Kalm
{
public:
	void kalmanFilterFun(float new_value);
	void initKalmanFilter(KalmanInfo *info);
	Kalm();
	KalmanInfo info;
};

