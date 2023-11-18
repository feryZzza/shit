#include "Kalm.h"
void Kalm::kalmanFilterFun(float x)
{
	float predictValue = info.A*info.filterValue + info.B*info.u;
	info.P = info.A*info.A*info.P + info.Q;
	info.kalmanGain = info.P * info.H / (info.P * info.H * info.H + info.R);
	info.filterValue = predictValue + (x - predictValue)*info.kalmanGain;
	info.predictValue= x + (x - predictValue)*info.kalmanGain;
	info.P = (1 - info.kalmanGain* info.H)*info.P;
	
}


void Kalm::initKalmanFilter(KalmanInfo *info)
{
	info->A = 1;
	info->H = 1;
	info->P = 0.1;
	info->Q = 0.05;
	info->R = 0.1;
	info->B = 0.1;
	info->u = 0;
	info->filterValue = 0;
	info->predictValue = 0;
}

Kalm::Kalm()
{

	info.A = 1;
	info.H = 1;
	info.P = 0.1;
	info.Q = 0.05;
	info.R = 0.1;
	info.B = 0.1;
	info.u = 0;
	info.filterValue = 0;
	info.predictValue = 0;
}
