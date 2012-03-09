/*
	HPRApp is an applicative use of the hpr classes
	it only instanciate functions not classes
	callable from main with a minimal commands dependence
	The commands of this file can be converted to HPRcmd mapped from Phython or script

*/




#ifndef __HPRAPP__
#define __HPRAPP__


#include "mcvGeneral.h"
#include "S3DEnv.h"


//View
void AnimViewer(stringmap& Config,int Start,int Last,int msHold = 10);
void AnimMultiViewer(stringmap& Config,int Start,int Last);
void MultiVideos(S3DEnv_c S3DEnv,const std::string ChanName = "");
void StreamPlay(S3DEnv_c &S3DEnv,std::string ChanName);

//File Channels processing
void Voxellize(std::string &CfgFile,float VoxelSize,int Start,int Last,		bool isProcess2DToVox = true,
																			bool isProcessResp3D = true);
void GenerateTrainResp3D(std::string &CfgFile,int Start,int Last,			bool isProcessTrain3D = true,
																			bool isProcessResp3D = true);
void GenerateTrainResp(std::string &CfgFile,int Start,int Last);
void LoadChannelsAndTrainModel(std::string &CfgFile,int Start,int Last,		bool isFilterEveryOne = false,
																			bool isFilterTotal = false);
void LoadAndTrainModel(std::string &CfgFile);
void PredictCompare(std::string &Config,int Start,int Last,					bool isPredict = true,
																			bool isCompare = true,
																			bool isMedianFilter = true,
																			bool isGetCenters = true,
																			bool isCompareCenters = true);

//In Mem
void DepthLoopAllinMem(std::string &CfgFile,int StartLearn,int LastLearn,int StartTest,int LastTast);
void ROC_TrainCurve_Param(std::string &CfgFile,int Start,int Last);
//---------------------------------------------------------------------------------------
char Hold(int msHold = 10);
void ROC_PredictCurve_Param();
void CompareAllClasses();
void EnvChannelsViewer();



#endif __HPRAPP__