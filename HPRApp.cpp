
#include "HPRApp.h"

//---------------------------------------------------------------------------------------

using namespace mcv;
using namespace mrg;
using namespace std;

//---------------------------------------------------------------------------------------
//				Viewing
//---------------------------------------------------------------------------------------
void MultiViewer_Chan_Grab(stringmap& Config)
{
	//nedded Flags : "CfgFile" "Channels"

	S3DEnv_c MyEnv(Config["CfgFile"]);
	S3DMultiCamViewer_c Viewer(&MyEnv);
	Viewer.SetUpGrabber(Config);//"CfgFile" "Channels"

	char Key = cv::waitKey(10);
	while(Key!='q')
	{
		Viewer.GrabFrame();
		Viewer.Render("Anim");
		Key = cv::waitKey(30);
	}
	printf("\n");

	cout << "Closing..." << endl;
	//Viewer.Grabber.Views[0].NetStream.Close();

}
//---------------------------------------------------------------------------------------
void MultiViewer_Chan_Index(stringmap& Config,int Start,int Last)
{
	S3DEnv_c MyEnv(Config["CfgFile"]);
	S3DMultiCamViewer_c Viewer(&MyEnv);
	Viewer.SetUpGrabber(Config);//"CfgFile" "Channels"

	char Key = cv::waitKey(10);
	int i=Start;
	while(Key!='q')
	{
		Viewer.GrabFrameIndex(i);
		Viewer.Render("Anim");
		Key = cv::waitKey(30);
		i++;
		if(i>Last)i=Start;
	}
	printf("\n");

	cout << "Closing..." << endl;
}
//---------------------------------------------------------------------------------------
void MultiViewer_JointsVoxChan_Index(stringmap& Config,int Start,int Last)//3
{
	//nedded Flags : "CfgFile" "Node"
	//optional Flags : "ViewerJoints" "ViewerVox" "ViewerChannels"
	bool isAnim = false;
	bool isVoxel = false;
	stringmap LocalC;
	BodyJoints_c JointsAnim;
	if(!Config["ViewerJoints"].empty())
	{
		LocalC["CfgFile"]	= Config["CfgFile"];
		LocalC["Node"]		= Config["Node"];
		LocalC["Tag"]		= Config["ViewerJoints"];
		JointsAnim.SetupGrabber(LocalC);
		isAnim = true;
	}

	g3d::VoxelSpace_c Vox;
	//re use of Config - same CfgFile and Node 
	if(Config.find("ViewerVox")!=Config.end())
	{
		LocalC["CfgFile"]	= Config["CfgFile"];
		LocalC["Node"]		= Config["Node"];
		LocalC["Tag"] = Config["ViewerVox"];//"Voxels"
		Vox.SetupGrabber(LocalC);
		isVoxel = true;
	}

	S3DEnv_c MyEnv(Config["CfgFile"]);
	S3DMultiCamViewer_c Viewer(&MyEnv);
	if(Config.find("ViewerChannels")!=Config.end())
	{
		LocalC["CfgFile"]	= Config["CfgFile"];
		LocalC["Channels"] = Config["ViewerChannels"];//"BKG"
		Viewer.SetUpGrabber(LocalC);//"CfgFile" "Channels"
	}
	if(isAnim)
	{
		Viewer.AddToGrab(&JointsAnim);
		Viewer.AddToRender(&JointsAnim);
	}
	if(isVoxel)
	{
		Viewer.AddToGrab(&Vox);
		Viewer.AddToRender(&Vox);
	}
	char Key = cv::waitKey(10);
	int i=Start;
	bool isGrab = true;
	while(Key!='q')
	{
		if(isGrab)
		{
			printf("%d,",i);
			Viewer.GrabFrameIndex(i++);
			isGrab = false;
		}
		Viewer.Render("Anim");
		Key = cv::waitKey(10);
		if(Key == 'g')
		{
			isGrab = true;
		}
		if(i==Last+1)i=Start;
	}
	printf("\n");
}
//---------------------------------------------------------------------------------------
char Hold(int msHold)
{
	char Key = cv::waitKey(msHold);
	if(Key!=-1)
	{
		printf("Hold, press Key to resume...\n");
		Key = cv::waitKey();
	}
	return Key;
}
//---------------------------------------------------------------------------------------
void FlyViewer_JointsVoxChan_Index(stringmap& Config,int Start,int Last,int msHold)//3
{
	//nedded Flags : "CfgFile" "Node"
	//optional Flags : "ViewerJoints" "ViewerJoints2" "ViewerVox"
	bool isAnim = false;
	bool isAnimres = false;
	bool isVoxel = false;
	bool isGrab = false;
	stringmap LocalC;
	int vmsHold = msHold;
	BodyJoints_c JointsAnim,JointsAnimres;
	if(!Config["ViewerJoints"].empty())
	{
		LocalC["CfgFile"]	= Config["CfgFile"];
		LocalC["Node"]		= Config["Node"];
		LocalC["Tag"]		= Config["ViewerJoints"];
		JointsAnim.SetupGrabber(LocalC);
		isAnim = true;
	}
	if(!Config["ViewerJoints2"].empty())
	{
		LocalC["CfgFile"]	= Config["CfgFile"];
		LocalC["Node"]		= Config["Node"];
		LocalC["Tag"]		= Config["ViewerJoints2"];
		JointsAnimres.SetupGrabber(LocalC);
		isAnimres = true;
	}

	g3d::VoxelSpace_c Vox;
	if(!Config["ViewerVox"].empty())
	{
		LocalC["CfgFile"]	= Config["CfgFile"];
		LocalC["Node"]		= Config["Node"];
		LocalC["Tag"] = Config["ViewerVox"];//"Voxels"
		Vox.SetupGrabber(LocalC);
		isVoxel = true;
	}

	S3DEnv_c MyEnv(Config["CfgFile"]);
	S3DFlyCamViewer_c Viewer(&MyEnv,"Anim");
	//Viewer.EnableExport("G:\\Data\\vid\\Seq1_%04d.png");
	//Viewer.isExportEnabled = false;

	if(isAnim)
	{
		Viewer.AddToGrab(&JointsAnim);
		Viewer.AddToRender(&JointsAnim);
	}
	if(isAnimres)
	{
		Viewer.AddToGrab(&JointsAnimres);
		Viewer.AddToRender(&JointsAnimres);
	}
	if(isVoxel)
	{
		Viewer.AddToGrab(&Vox);
		Viewer.AddToRender(&Vox);
	}
	char Key = cv::waitKey(10);
	int i=Start;
	Viewer.GrabFrameIndex(i++);
	while(Key!='q')
	{
		vmsHold-=10;
		Key = cv::waitKey(10);
		Viewer.Render("Anim");
		if(vmsHold<=0)
		{
			if(isGrab)
			{
				printf("%d,",i);
				Viewer.GrabFrameIndex(i++);
				//isGrab = false;
			}
			vmsHold = msHold;
		}
		//Viewer.Cam.RotateY(0.02f);
		//Key = Hold(msHold);
		if(Key == 'e')
		{
			Viewer.isExportEnabled = !Viewer.isExportEnabled;
			cout << "Export " << Viewer.isExportEnabled << endl;
		}
		if(Key == 'g')
		{
			isGrab = !isGrab;
		}
		if(Key == 'v')
		{
			Vox.isVisible = !Vox.isVisible;
		}
		if(i==Last+1)i=Start;
	}
	printf("\n");
}
//---------------------------------------------------------------------------------------
//				Processing Data
//---------------------------------------------------------------------------------------
void Voxellize(std::string &CfgFile,float VoxelSize,int Start,int Last)
{
	printf("=======================================================================================\n");
	printf("GenerateVoxels(%d to %d) on (%s) \n",Start,Last,CfgFile.c_str());
	printf("=======================================================================================\n");
	//----------------------------------------------------------Detectors BPR needs S3DEnv
	S3DEnv_c S3DVirtS(CfgFile);
	hpr::DetectorBPR_c	MyBPR(&S3DVirtS,CfgFile);
	double t;
	TStart(t);
	//------------------------------------------Data Generation
	MyBPR.Process2DToVox(Start,Last,VoxelSize,"BKG","Voxels");
	TStop(t,"Voxellize()==================================================");
}
//---------------------------------------------------------------------------------------
void GenerateTrainResp3D(std::string &CfgFile,int Start,int Last,bool isProcessTrain3D,bool isProcessResp3D)
{
	printf("=======================================================================================\n");
	printf("GenerateVoxels(%d to %d) on (%s) \n",Start,Last,CfgFile.c_str());
	printf("=======================================================================================\n");
	//----------------------------------------------------------Detectors BPR needs S3DEnv
	S3DEnv_c S3DVirtS(CfgFile);
	hpr::DetectorBPR_c	MyBPR(&S3DVirtS,CfgFile);
	//MyBPR.Voxelliser.SetUpDescSign("Desc3DSign");//Automatically loaded
	double t;
	TStart(t);
	//------------------------------------------Data Generation
	if(isProcessTrain3D)MyBPR.ProcessTrain3D(Start,Last,"Voxels","Desc3DTrain");
	if(isProcessResp3D)MyBPR.ProcessResp3D(Start,Last,"Voxels","Joints","Parts3DResp","VoxelsParts");
	TStop(t,"GenerateVoxels()==================================================");
}
//---------------------------------------------------------------------------------------
void GenerateTrainResp(std::string &CfgFile,int Start,int Last)
{
	printf("=======================================================================================\n");
	printf("GenerateTrainResp(%s)\n",CfgFile.c_str());
	printf("=======================================================================================\n");
	//----------------------------------------------------------Detectors BPR needs S3DEnv
	S3DEnv_c S3DVirtS(CfgFile);
	hpr::DetectorBPR_c	MyBPR(&S3DVirtS,CfgFile);
	printf("Processing Poses (%d,%d):\n",Start,Last);
	double t;
	TStart(t);
	//------------------------------------------Data Generation
	//MyBPR.ProcessTrain2D(Start,Last,"BKG","Desc2DTrain");//Yes All 0 - 850 : "BKG","Desc2DTrain"
	//MyBPR.ProcessRespLabels(Start,Last,"BKG","BPR","LabelResp");// All : "BKG","BPR","LabelResp"
	MyBPR.ProcessTrainDepth(Start,Last,"BKG","Depth","DepthTrain");// All : "BKG","Depth","DepthTrain"
	//MyBPR.ProcessRespLabels2Imgs(Start,Last,"BKG","LabelRespres","BPRres");
	//MyBPR.ProcessRespLabels2Imgs(Start,Last,"BKG","LabelResp","BPRres");//Debug Check
	TStop(t,"GenerateTrainResp()==============================");
}
//---------------------------------------------------------------------------------------
//				Machine Learning
//---------------------------------------------------------------------------------------
void LoadChannelsAndTrainModel(std::string &CfgFile,int Start,int Last,bool isFilterEveryOne,bool isFilterTotal)//5Tags
{
	printf("=======================================================================================\n");
	printf("LoadChannelLeranTrainModel(%s)\n",CfgFile.c_str());
	printf("=======================================================================================\n");
	//----------------------------------------------------------Detectors BPR needs S3DEnv
	printf("Processing Poses (%d,%d):\n",Start,Last);
	double t;
	mrg::MLData_c MLData(CfgFile,"MLD1");//MLData1 CfgFile to be able to load Channels - "MLData1" to save load Train-Resp Tables
	mrg::MLModel_c MLModel(CfgFile,"MLM1");//MLModel1

	TStart(t);
	//----------------------------------------------------------MLData
	mrg::MLData_c MLDatafiltred;//MLData1
	for(int i=Start;i<=Last;i++)
	{
		printf("Pose (%d)------------\n",i);
		double t;
		TStart(t);
		//MLData.loadTrainChannel(i,i,"Views","DepthTrain");//"Desc2DTrain"//Load Train
		MLData.loadTrainChannel(i,i,"Poses","Desc3DTrain");
		MLData.loadRespChannel(i,i,"Poses","Parts3DResp");//LabelResp	//Load Resp
		TStop(t,"Loading()");
		if(isFilterEveryOne)MLData.Filter_RemoveDuplicates();
		MLDatafiltred.CopyAppend(MLData);
		printf("Nb Samples in Mem: %d  Train.Data.size(%d)\n",MLDatafiltred.Resp.size(),MLDatafiltred.Train.Data.size());
	}
	if(isFilterTotal)MLDatafiltred.Filter_RemoveDuplicates();//--------------------------------------double Filtering !!!!!!

	MLModel.Learn(MLDatafiltred);//or not filtered
	MLModel.Train(mrg::Model_ANN);//Default ANN
	MLModel.saveModel();//Default ANN - 3 files : .Train .Resp .ann

	TStop(t,"LoadChannelsAndTrainModel()==============================");
}
//---------------------------------------------------------------------------------------
void ExportChannels(std::string &CfgFile,int Start,int Last)
{
	printf("=======================================================================================\n");
	printf("ExportChannel(%s)\n",CfgFile.c_str());
	printf("=======================================================================================\n");
	//----------------------------------------------------------Detectors BPR needs S3DEnv
	printf("Exporting Poses (%d,%d):\n",Start,Last);
	double t;
	mrg::MLData_c MLData(CfgFile,"MLD1");//MLData1 CfgFile to be able to load Channels - "MLData1" to save load Train-Resp Tables
	for(int i=Start;i<=Last;i++)
	{
		printf("Pose (%d)------------\n",i);
		double t;
		TStart(t);
		MLData.Train.clear();
		MLData.Resp.clear();
		MLData.loadTrainChannel(i,i,"Poses","Desc3DTrain");
		MLData.loadRespChannel(i,i,"Poses","Parts3DResp");//LabelResp	//Load Resp
		TStop(t,"Loading()");
		char FileName[512];
		sprintf(FileName,"G:\\PosturesDB\\temp\\Train%04d.csv",i);
		MLData.Train.Export(FileName);
		sprintf(FileName,"G:\\PosturesDB\\temp\\Resp%04d.csv",i);
		MLData.Resp.Export(FileName);
	}

}
//---------------------------------------------------------------------------------------
void LoadAndTrainModel(std::string &CfgFile)//no big interest
{
	printf("=======================================================================================\n");
	printf("LoadAndTrainModel(%s)\n",CfgFile.c_str());
	printf("=======================================================================================\n");
	//----------------------------------------------------------Detectors BPR needs S3DEnv
	double t;
	TStart(t);
	mrg::MLData_c MLData(CfgFile,"MLD1");//MLData1 CfgFile to be able to load Channels - "MLData1" to save load Train-Resp Tables
	mrg::MLModel_c MLModel(CfgFile,"MLM1");//MLModel1

	MLData.load("Train","Resp");
	MLModel.Learn(MLData);
	MLModel.Train(mrg::Model_ANN);//Default ANN
	MLModel.saveModel();//Default ANN - 3 files : .Train .Resp .ann
	TStop(t,"LoadAndTrainModel()==========================================");
}
//---------------------------------------------------------------------------------------
void PredictCompare(std::string &Config,int Start,int Last,bool isPredict,bool isCompare,bool isMedianFilter,bool isGetCenters,bool isCompareCenters)//5 Tags
{
	printf("=======================================================================================\n");
	printf("PredictCompare(%s)\n",Config.c_str());
	printf("=======================================================================================\n");
	//----------------------------------------------------------Detectors BPR needs S3DEnv
	double t;
	TStart(t);
	S3DEnv_c S3DVirt(Config);
	hpr::DetectorBPR_c	MyBPR(&S3DVirt,Config);
	printf("Processing Poses (%d,%d):\n",Start,Last);
	//----------------------------------------------------------MLModel
	mrg::MLModel_c MLModel(Config,"MLM1");//MLModel1 - MLMDepth1
	if(isPredict)
	{
		MLModel.loadModel();
		//------------------------------------------Data Generation
		MLModel.Predict(Start,Last,"Poses","Desc3DTrain","Parts3DRespres");//"Views","DepthTrain","LabelRespres"
		//"VoxelsParts" generated by GenerateTrainResp3D()
		MyBPR.ProcessResp3DToVox(Start,Last,"VoxelsParts","Parts3DRespres","VoxelsPartsres");
	}
		//MyBPR.ProcessRespLabels2Imgs(Start,Last,"BKG","LabelRespres","BPRres");
		//MyBPR.ProcessResp3DToVox(Start,Last,"Voxels","Parts3DRespres","VoxelsPartsres");
	
	if(isMedianFilter)MyBPR.VoxMedianFilter(Start,Last,"VoxelsPartsres","Parts3DRespres");

	if(isCompare)MLModel.CompareClass(Start,Last,"Poses","Parts3DResp","Parts3DRespres",true);//with ,true optional can display per image info

	if(isGetCenters)MyBPR.ProcessVoxParts2Centers(Start,Last,"VoxelsPartsres","Jointsres");
	if(isGetCenters)MyBPR.ProcessVoxParts2Centers(Start,Last,"VoxelsParts","RefJointsres");
	printf("----------------------'Joints','Jointsres'------------------");
	if(isCompareCenters)MyBPR.CompareCenters(Start,Last,"Joints","Jointsres");
	printf("----------------------'RefJointsres','Jointsres'------------------");
	if(isCompareCenters)MyBPR.CompareCenters(Start,Last,"RefJointsres","Jointsres");
	TStop(t,"PredictCompare()==========================================");
}
//---------------------------------------------------------------------------------------
void DepthLoopAllinMem(std::string &CfgFile,int StartLearn,int LastLearn,int StartTest,int LastTast)
{
	double t;TStart(t);
	printf("=======================================================================================\n");
	printf("DepthLoopAllinMem(%s - Learn :(%d-%d) Test(%d-%d))\n",CfgFile.c_str(),StartLearn,LastLearn,StartTest,LastTast);
	printf("=======================================================================================\n");
	//----------------------------------------------------------Detectors BPR needs S3DEnv
	S3DEnv_c S3DEnv(CfgFile);
	hpr::DetectorBPR_c	MyBPR(&S3DEnv,CfgFile);
	int NbParams = 4;
	std::vector<double> T1(NbParams);
	std::vector<double> T2(NbParams);
	std::vector<double> T3(NbParams);
	std::vector<double> T4(NbParams);
	std::vector<double> T5(NbParams);
	std::vector<float> Params(NbParams);
	std::vector<float> ResCurve(NbParams);
	Params[0] = 1000;
	Params[1] = 5000;
	Params[2] = 20000;
	Params[3] = 100000;

	//for(int i=0;i<Params.size();i++)
	for(int i=0;i<1;i++)
	{
		//MyBPR.SetROCParam(Params[i]);
		mrg::MLData_c LearnData;
		mrg::MLData_c TestData;
		mrg::VectorsTable_c TestRefResp;
		mrg::MLModel_c MLModel;
		printf("PixSign_c::PixSignsDepth() BKG Depth = Param(%d) = %f\n",i,Params[i]);
		TStart(T1[i]);//-----------------------------ProcessLearn
		MyBPR.ProcessTrainDepth(StartLearn,LastLearn,"BKG","Depth",LearnData.Train);// All : "BKG","Depth","DepthTrain"
		MyBPR.ProcessRespLabels(StartLearn,LastLearn,"BKG","BPR",LearnData.Resp);//Could be just loaded - always the same
		TStop(T1[i]);		TStart(T2[i]);//-----------------------------CopyTrain
		MLModel.Learn(LearnData);
		MLModel.Train(mrg::Model_ANN);//Default ANN
		TStop(T2[i]);		TStart(T3[i]);//-----------------------------ProcessTest
		MyBPR.ProcessTrainDepth(StartTest,LastTast,"BKG","Depth",TestData.Train);// All : "BKG","Depth","DepthTrain"
		MyBPR.ProcessRespLabels(StartTest,LastTast,"BKG","BPR",TestRefResp);//Could be just loaded - always the same
		TStop(T3[i]);		TStart(T4[i]);//-----------------------------
		MLModel.Predict(TestData);// From Train to Resp
		TStop(T4[i]);		TStart(T5[i]);//-----------------------------Compare
		ResCurve[i] = TestData.Resp.CompareClasses(TestRefResp,20,true,true);
		TStop(T5[i]);//-----------------------------
		printf("==============================================================================================\n");
	}

	mcv::TabPrintf(Params,"ParamCurve");
	mcv::TabPrintf(ResCurve,"ROCCurve");
	printf("\n");
	mcv::TabPrintf(T1,"TProcessLearn ms",true,',',1);
	mcv::TabPrintf(T2,"TCopyTrain ms",true,',',1);
	mcv::TabPrintf(T3,"TProcessTest ms",true,',',1);
	mcv::TabPrintf(T4,"TPredict ms",true,',',1);
	mcv::TabPrintf(T5,"TCompare ms",true,',',1);
	printf("\n");
	TStop(t,"DepthLoopAllinMem()");
}
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void ROC_TrainCurve_Param(std::string &CfgFile,int Start,int Last)//8 Tags + Paramcurve
{
	printf("=======================================================================================\n");
	printf("ROC_TrainCurve_Param( KMeans branching test on (%d,%d))\n",Start,Last);
	printf("=======================================================================================\n");
//---------------------------------------------------------------------
	float BestPerf = 0;
	mrg::MLModel_c MLModel(CfgFile,"DM100");

	mrg::MLData_c MLData(CfgFile,"DD100");
	MLData.load("Train","Resp");
	MLModel.Learn(MLData);

	float BestAvgParts = 0;
	int NbParams = 7;
	std::vector<float> ROCCurve(NbParams);
	std::vector<float> ParamCurve(NbParams);
	ParamCurve[0] = 2.0f;
	ParamCurve[1] = 3.0f;
	ParamCurve[2] = 4.0f;
	ParamCurve[3] = 5.0f;
	ParamCurve[4] = 6.0f;
	ParamCurve[5] = 7.0f;
	ParamCurve[6] = 8.0f;
	std::vector<double> TimesBuild(NbParams);
	std::vector<double> TimesPredict(NbParams);
	//for(float Param = 0.1f;Param <=1.0f;Param+=0.1f)	{
	for(int i=0;i<NbParams;i++)	{
		TStart(TimesBuild[i]);
		//MLModel.ROC_Param = ParamCurve[i];//
		MLModel.Train(mrg::Model_ANN);
		TStop(TimesBuild[i]);
		TStart(TimesPredict[i]);
		MLModel.Predict(Start,Last,"Views","DepthTrain","LabelRespres");//Desc2DTrain
		TStop(TimesPredict[i]);
		float AvgParts = MLModel.CompareClass(Start,Last,"Views","LabelResp","LabelRespres");//Ref, Test 38 is bugous
		ROCCurve[i] = AvgParts;
		//ParamCurve.push_back(Param);
		if(AvgParts > BestAvgParts)
		{
			BestAvgParts = AvgParts;
			printf(">>>>Param %f performs better: %f\n",ParamCurve[i],AvgParts);
			//MLModel.saveModel();
		}
	}
	mcv::TabPrintf(ParamCurve,"ParamCurve");
	mcv::TabPrintf(ROCCurve,"ROCCurve");
	printf("TimesBuild: ");for(int i=0;i<NbParams;i++)printf("%1.2f ms,",TimesBuild[i]);
	printf("TimesPredict: ");for(int i=0;i<NbParams;i++)printf("%1.2f ms,",TimesPredict[i]);
}
//---------------------------------------------------------------------------------------
void ROC_PredictCurve_Param()
{
	std::string CfgFile = "G:\\PosturesDB\\VirtualDB\\All10Seq\\S3D.env";
	printf("=======================================================================================\n");
	printf("ROC_PredictCurve_Param()\n");
	printf("=======================================================================================\n");
//---------------------------------------------------------------------
	float BestPerf = 0;
	mrg::MLModel_c MLModel(CfgFile,"ML_ANN_KMeans_3p5m_1");
	MLModel.loadModel();

	float BestAvgParts = 0;
	int NbParams = 1;//21
	std::vector<float> ROCCurve(NbParams);
	std::vector<float> ParamCurve(NbParams);
	for(int i=0;i<NbParams;i++)
	{
		ParamCurve[i] = 13;//i+1;//1->21
	}
	std::vector<double> TimesPredict(NbParams);
	for(int i=0;i<NbParams;i++)	
	{
		MLModel.ROC_Param = ParamCurve[i];printf("---------------Prediction with K == %1.0f -----------\n",ParamCurve[i]);
		TStart(TimesPredict[i]);
		MLModel.Predict(0,1,"Views","Desc2DTrain","LabelRespres");
		TStop(TimesPredict[i]);
		float AvgParts = MLModel.CompareClass(0,1,"Views","LabelResp","LabelRespres");//Ref, Test 38 is bugous
		ROCCurve[i] = AvgParts;
	}
	mcv::TabPrintf(ParamCurve,"ParamCurve",true,',',0);
	mcv::TabPrintf(ROCCurve,"ROCCurve");
	printf("TimesPredict: ");for(int i=0;i<NbParams;i++)printf("%1.2f ms,",TimesPredict[i]);
}
//---------------------------------------------------------------------------------------
void CompareAllClasses()
{
	std::string CfgFile = "G:\\PosturesDB\\VirtualDB\\All10Seq\\S3D.env";
	printf("=======================================================================================\n");
	printf("CompareAllClasses()\n");
	printf("=======================================================================================\n");
//---------------------------------------------------------------------
	mrg::MLModel_c MLModel(CfgFile,"MLModel2");
	MLModel.CompareClass(37,38,"Views","LabelResp","LabelRespres");//Ref, Test 38 is bugous
}
//---------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void StreamPlay(S3DEnv_c &S3DEnv,std::string ChanName)//rather use MultiViewer_Chan_Index
{
	MultiCamStream Streams;
	int NbStreams = S3DEnv.GetStreams(Streams,ChanName.c_str());
	if(NbStreams !=0)
	{
		printf("NbStreams = %d\n",NbStreams);
		std::vector<cv::Mat> Images(NbStreams);
		char Key = cv::waitKey(10);
		do
		{
		
			Streams.GetFrames(Images);
			mcv::imNShow(Images,"Posture");
			Key = cv::waitKey(10);
			if(Key != -1)
			{
				printf("press Key to resume...\n");
				if(Key!='q')cv::waitKey();
			}
		}while(Key !='q');
	}
	else
	{
		printf("Channel '%s' is not found on the Path (%s)\n",ChanName,S3DEnv.SMainPath.c_str());
	}
}
//---------------------------------------------------------------------------------------
void DisplayHelp(bool clear = true)
{
	if(clear)system("cls");
	printf("\n\n_____________________________________________________________________\n");
	printf("s:	MLModel.saveModel()\n");
	printf("k:	Video Skel Play BKG\n");
	printf("\n");
	printf("DB : 0 -Virtual DB;    1 -Human Eva;   2 -Virt Smooth\n");
	printf("Channel : 3 -Depth;  4 -Any;  5 -Gray;  6 -COLOR;  7 -BPR;  8 -BKG;  9 -DepthC; / -BPRres\n");
	printf("\n");
	printf("q: quit\n");
	printf("\n");
}
//---------------------------------------------------------------------------------------
void EnvChannelsViewer()
{
	std::string Cfg_Virt = "G:\\PosturesDB\\VirtualDB\\All10Seq\\Virt.env";
	std::string Cfg_Real = "G:\\PosturesDB\\HumanEva_Sync\\Jog_1_BKG_100\\Real.env";
	std::string Cfg_VirtSoft = "G:\\PosturesDB\\VirtualDB\\SoftMotion\\Virtsoft.env";

	printf("=======================================================================================\n");
	printf("Main()\n");
	printf("=======================================================================================\n");
	S3DEnv_c S3DVirt(Cfg_Virt);
	hpr::DetectorBPR_c	MyBPR(&S3DVirt,Cfg_Virt);
	//MyBPR.BPR_2DMV.Parts2D[0].PxSign.

	//hpr::DetectorHSC_c	MyHSC(MainPath,&MyDXS3D);
	S3DEnv_c S3DHE(Cfg_Real);
	S3DEnv_c S3DVirtSmooth(Cfg_VirtSoft);
	hpr::DetectorBPR_c	RealBPR(&S3DHE,Cfg_Real);


	//Params to select
	S3DEnv_c *pS3D = &S3DVirt;
	//pS3D->Render("Posture");
	std::string ChanName = "Any";

	DisplayHelp(false);
	char Key;
	do
	{
		Key = cv::waitKey(50);
		switch(Key)
		{
			case 'h':										DisplayHelp();			break;

			case 'k':	MultiVideos(*pS3D,ChanName);	DisplayHelp(false);		break;

			case '0':	pS3D = &S3DVirt;printf("Virtual DB\n");						break;
			case '1':	pS3D = &S3DHE;printf("Human Eva DB\n");						break;
			case '2':	pS3D = &S3DVirtSmooth;printf("Virt Smooth\n");				break;

			case '/':	StreamPlay(*pS3D,"BPRres");						break;
			case '9':	StreamPlay(*pS3D,"DepthC");						break;
			case '7':	StreamPlay(*pS3D,"BPR");						break;
			case '8':	StreamPlay(*pS3D,"BKG");						break;
			case '6':	StreamPlay(*pS3D,"COLOR");						break;
			case '5':	StreamPlay(*pS3D,"Gray");						break;
			case '4':	StreamPlay(*pS3D,"Any");						break;
			case '3':	StreamPlay(*pS3D,"Depth");						break;
		}
		

	}while(Key!='q');
	printf("Quitting... Bye bye\n");
	printf("Destruction Might take time...");
}
//---------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------
void MultiVideos(S3DEnv_c S3DEnv,std::string ChanName)
{
	MultiCamStream Streams;
	int NbStreams = S3DEnv.GetStreams(Streams,ChanName.c_str());
	std::vector<cv::Mat> Images(NbStreams);
	char Key = cv::waitKey(10);
	int StreamIndex = Streams.ImgsStream[0].FirstImage;
	int ModelIndex = 0;
	if(NbStreams ==S3DEnv.Cams.size())
	{
		do
		{
			//S3DEnv.WModel.SetAnimByIndex(ModelIndex++,S3DEnv.Motion_isPostion);
			Streams.GetFramesByIndex(Images,StreamIndex++);
			if(StreamIndex > Streams.ImgsStream[0].LastImage)
			{
				printf("Restarting, press Key to continue...");cv::waitKey();
				StreamIndex = Streams.ImgsStream[0].FirstImage;
				ModelIndex = 0;
			}
			//clearBuffers
			S3DEnv.ClearBuffers();
			//S3DEnv.FillBuffers(Grabber.GetFrames());//Will look for every configured Channel Type
			S3DEnv.FillBuffers(Images);
			S3DEnv.DrawReference(g3d::MatrixIdentity(),50,-1);
			//S3DEnv.Render("Porsture",false);
			//S3DEnv.Render("Motion");
			Key = cv::waitKey(10);
			if(Key != -1)
			{
				printf("press Key to resume...\n");
				if(Key!='q')Key = cv::waitKey();
			}
		}while(Key !='q');
	}
	else
	{
		printf("Env Nb Views (%d) different from Channel Nb Streams (%d) don't have labels to associate them\n",S3DEnv.Cams.size(),NbStreams);
	}
}
