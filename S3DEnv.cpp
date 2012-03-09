
#include "S3DEnv.h"

#include <iostream>
#include <fstream>
#include "mcvGeneral.h"

#include "S3DGeom.h"

//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//										ViewChannel_c
//--------------------------------------------------------------------------------------------------------------
/*int ViewChannel_c::GetType()
{
	int res = NotSpecified;
	if(Label.compare("COLOR"))	res = COLOR;
	if(Label.compare("BKG"))	res = BKG;
	if(Label.compare("BPR"))	res = BPR;
	if(Label.compare("Gray"))	res = Gray;
	if(Label.compare("Depth"))	res = Depth;
	return res;
}*/
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//										Soft3DEnv_c
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
S3DEnv_c::S3DEnv_c(const std::string &vCfgFileName,int vNbCams)
{
	Cams.resize(vNbCams);
	NbCams = vNbCams;
	NbModels = 0;
	MaxCams = 10;
	MaxModels = 4;
	//isSurfaceInitialized = false;

	Vector3f LookAt(0,0,0);
	Vector3f Pos;
	for(int i=0;i<NbCams;i++)
	{
		Cams[i].Fx = 1140;
		Cams[i].Fy = 1140;
		Cams[i].Cx = 320;
		Cams[i].Cy = 320;
		Cams[i].mProjection = g3d::S3DMatrixProjection(Cams[i].Fx,Cams[i].Fy,Cams[i].Cx,Cams[i].Cy);
	}
	Pos << 500,200,500;
	Cams[0].mView = g3d::S3DMatrixView(Pos,LookAt);
	Pos << -500,200,500;
	Cams[1].mView = g3d::S3DMatrixView(Pos,LookAt);
	Pos << 500,200,-500;
	Cams[2].mView = g3d::S3DMatrixView(Pos,LookAt);
	Pos << -500,200,-500;
	Cams[3].mView = g3d::S3DMatrixView(Pos,LookAt);


	Cams[0].mProjView = Cams[0].mProjection * Cams[0].mView;
	Cams[1].mProjView = Cams[1].mProjection * Cams[1].mView;
	Cams[2].mProjView = Cams[2].mProjection * Cams[2].mView;
	Cams[3].mProjView = Cams[3].mProjection * Cams[3].mView;

	CfgFileName = vCfgFileName;
	SMainPath = mcv::GetMainPath(CfgFileName);

	load(CfgFileName);
}
//--------------------------------------------------------------------------------------------------------------
void S3DEnv_c::save(const std::string &vCfgFileName,bool isSaveModel)//sample save then manual modif
{
	//char FileName[512];
	//sprintf(FileName,"%sS3D.env",MainPath);
	cv::FileStorage Fs(vCfgFileName,cv::FileStorage::WRITE);

#ifdef HUMAN_EVA
	assert(NbCams == 7);
	Fs << "Views" << "[";
		Fs << "{" << "Gray" << "stream00\\motion%03d.png" << "Calib" << "Cam00.cam" << "}";
		Fs << "{" << "Gray" << "stream01\\motion%03d.png" << "Calib" << "Cam01.cam" << "}";
		Fs << "{" << "Gray" << "stream02\\motion%03d.png" << "Calib" << "Cam02.cam" << "}";
		Fs << "{" << "Gray" << "stream03\\motion%03d.png" << "Calib" << "Cam03.cam" << "}";
		Fs << "{" << "Color" << "stream04\\motion%03d.png" << "Calib" << "Cam04.cam" << "}";
		Fs << "{" << "Color" << "stream05\\motion%03d.png" << "Calib" << "Cam05.cam" << "}";
		Fs << "{" << "Color" << "stream06\\motion%03d.png" << "Calib" << "Cam06.cam" << "}";
	Fs << "]";
	Fs << "ModelFileName" << "S2_Jog1_WModel.skel";
	Fs << "MotionFileName" << "Jog_1";
#endif
	if(NbCams>0)
	{
		Fs << "Views" << "[";
	}
	for(int i=0;i<NbCams;i++)
	{
		char CamName[32];
		sprintf(CamName,"Cam%02d.cam",i);
		Fs << "{" << "Calib" << CamName << "}";
	}
	if(NbCams>0)
	{
		Fs << "]";
	}
	Fs << "ModelFileName" << "Model1.skel";
	Fs << "MotionFileName" << "All10Seq";

	//------------------------------------------------------Now Save The dependant Files
	std::string CfgMainPath = mcv::GetMainPath(vCfgFileName);
	std::string ModelFileName = CfgMainPath;
	ModelFileName += "Model1.skel";
	//sprintf(ModelFileName,"%sModel1.skel",MainPath);
	WModel.save(ModelFileName.c_str());
	//---------------------------------------------------------------------save the cameras
	for(int i=0;i<NbCams;i++)
	{
		char CamFileName[512];
		sprintf(CamFileName,"%sCam%02d.cam",CfgMainPath.c_str(),i);
		Cams[i].Save(CamFileName,true,true,false);
	}
}
//--------------------------------------------------------------------------------------------------------------
int S3DEnv_c::GetStreams(MultiCamStream &Streams,const char * ChannelName)//From Views
{
	int FirstImage = 0;
	if(FirstStreamsIndex!=0)
	{
		FirstImage = FirstStreamsIndex;
	}
	int LastImage = 0;
	if(LastStreamsIndex!=0)
	{
		LastImage = LastStreamsIndex;
	}
	
	cv::FileNode FNode = Fs["Views"];
	Streams = MultiCamStream((int)FNode.size(),SMainPath.c_str(),"%03d.png",FirstImage,LastImage);
	assert(NbCams == (int)FNode.size());

	std::string StreamFileName;
	int NbStreamsFound = 0;
	int RightChannel = 0;
	for(int i=0;i<NbCams;i++)
	{

		if(!FNode[i][ChannelName].empty())
		{
			std::string Val = (std::string)FNode[i][ChannelName];
			Streams.SetStreamFileName(NbStreamsFound,Val.c_str());
			Streams.ImgsStream[NbStreamsFound].ChannelTypeName = ChannelName;
			NbStreamsFound++;
		}
	}
	Streams.ImgsStream.resize(NbStreamsFound);//Initialized with Views.size() then can be lowered

	return NbStreamsFound;
}
//--------------------------------------------------------------------------------------------------------------
void S3DEnv_c::load(const std::string &vCfgFileName)
{
	SMainPath = mcv::GetMainPath(vCfgFileName);
	Fs = cv::FileStorage(vCfgFileName,cv::FileStorage::READ);
	if (Fs.isOpened())
	{
		cv::FileNode FNode = Fs["Views"];
		assert(MaxCams >= (int)FNode.size());
		NbCams = (int)FNode.size();
		Cams.resize(NbCams);
		for(int i=0;i<NbCams;i++)
		{
			std::string CamFileName = SMainPath;
			CamFileName += (std::string)FNode[i]["Calib"];
			Cams[i].Load(CamFileName.c_str(),true,true);
			Cams[i].Id = i;
		}

		if(!Fs["ModelFileName"].empty())
		{
			std::string ModelFileName = (std::string)Fs["ModelFileName"];
			ModelFileName = SMainPath + ModelFileName;
			WModel.load(ModelFileName.c_str());
			//-----------------------------------------------------------------load Motion if Available
			if(!Fs["MotionFileName"].empty())
			{
				std::string MotionFileName = (std::string)Fs["MotionFileName"];
				WModel.Motion.MotionName = MotionFileName;
				MotionFileName = SMainPath + MotionFileName;
				WModel.Motion.load(MotionFileName.c_str());
			}
		}

		if(!Fs["LastStreamsIndex"].empty())
		{
			Fs["LastStreamsIndex"] >> LastStreamsIndex;
		}
		else
		{
			LastStreamsIndex = 0;
		}

		if(!Fs["FirstStreamsIndex"].empty())
		{
			Fs["FirstStreamsIndex"] >> FirstStreamsIndex;
		}
		else
		{
			FirstStreamsIndex = 0;
		}
		if(!Fs["Motion_isPosition"].empty())
		{
			int isPostionInt;
			Fs["Motion_isPosition"] >> isPostionInt;
			if(isPostionInt)
			{
				Motion_isPostion = true;
			}
			else
			{
				Motion_isPostion = false;
			}
			
		}
		else
		{
			Motion_isPostion = false;
		}
		
	}
	else
	{
		printf("File(%s) not found\n",vCfgFileName.c_str());
		printf("Creating File (%s)\n",vCfgFileName.c_str());
		save(vCfgFileName);
	}
}
//--------------------------------------------------------------------------------------------------------------
void S3DEnv_c::ClearBuffers()
{
	for(int i=0;i<Cams.size();i++)
	{
		Cams[i].ClearBuffers();
	}
}
//--------------------------------------------------------------------------------------------------------------
void S3DEnv_c::InitSurfaces()
{
	for(int i=0;i<Cams.size();i++)
	{
		if(!Cams[i].isSurfaceInitialized)
		{
			Cams[i].InitSurface();
		}
	}
}
//--------------------------------------------------------------------------------------------------------------
void S3DEnv_c::FillBuffers(std::vector<cv::Mat>&Imgs)
{
	assert(Imgs.size() >= Cams.size());
	InitSurfaces();//only is not Initialized
	for(int i=0;i<Cams.size();i++)
	{
		if(Imgs[i].type() == CV_8UC3)
		{
			Imgs[i].copyTo(Cams[i].ImgRender);
		}
		else if(Imgs[i].type() == CV_8UC1)
		{
			cv::Mat ImgGrayBGR;
			cv::cvtColor(Imgs[i],ImgGrayBGR,cv::COLOR_GRAY2BGR);
			ImgGrayBGR.copyTo(Cams[i].ImgRender);
		}
	}
}
//--------------------------------------------------------------------------------------------------------------
void S3DEnv_c::GetBuffers(std::vector<cv::Mat>&Imgs)
{
	assert(Imgs.size() >= Cams.size());
	for(int i=0;i<Cams.size();i++)
	{
		Cams[i].ImgRender.copyTo(Imgs[i]);
	}
}
//--------------------------------------------------------------------------------------------------------------
void S3DEnv_c::Render(bool IsClearBuffers)
{
	for(int i=0;i<(int)Cams.size();i++)
	{
		if(IsClearBuffers)Cams[i].ClearBuffers();
		WModel.Draw(Cams[i]);
	}
}
//--------------------------------------------------------------------------------------------------------------
void S3DEnv_c::Render(const char*WinName, bool IsClearBuffers)
{
	Render(IsClearBuffers);
	if(WinName)
	{
		Display(WinName);
	}
}
//--------------------------------------------------------------------------------------------------------------
void S3DEnv_c::SaveRendered(const char*FileName)
{
	cv::imwrite(FileName,
		mcv::im3Show(NULL,Cams[0].ImgRender,Cams[1].ImgRender,Cams[2].ImgRender)
		);
}
//--------------------------------------------------------------------------------------------------------------
void S3DEnv_c::Draw(std::vector<cv::Mat> &ImgsDrawOn,const char*WinName,float Light)
{
	for(int i=0;i<3;i++)WModel.Draw(ImgsDrawOn[i],Cams[i],Light);
	if(WinName)mcv::im3Show(WinName,ImgsDrawOn[0],ImgsDrawOn[1],ImgsDrawOn[2]);
}
//--------------------------------------------------------------------------------------------------------------
int S3DEnv_c::EvalFilt(std::vector<cv::Mat> &ImgsBkg)//Bad, to rework !!! no support for variable Nb Cams
{
	int Val = 0;
	for(int i=0;i<3;i++)
		Val += WModel.EvalFit(ImgsBkg[i],Cams[i]);
	return Val;
}
//--------------------------------------------------------------------------------------------------------------
Eigen::Vector3f S3DEnv_c::PointsToRaysIntersection(cv::Point P1,cv::Point P2,cv::Point P3)
{
	Vector3f Pos;

	EigenRay Ray1 = Cams[0].GetRay(P1);
	EigenRay Ray2 = Cams[1].GetRay(P2);
	EigenRay Ray3 = Cams[2].GetRay(P3);
	Vector3f I12A,I12B,I13A,I13B,I23A,I23B;
	I12A = g3d::RayRayIntersect(Ray1,Ray2);
	I12B = g3d::RayRayIntersect(Ray2,Ray1);
	I13A = g3d::RayRayIntersect(Ray1,Ray3);
	I13B = g3d::RayRayIntersect(Ray3,Ray1);
	I23A = g3d::RayRayIntersect(Ray2,Ray3);
	I23B = g3d::RayRayIntersect(Ray3,Ray2);

	Pos = (I12A + I12B + I13A + I13B + I23A + I23B) / 6;
	return Pos;
}
//--------------------------------------------------------------------------------------------------------------
Eigen::Vector3f S3DEnv_c::PointsToRaysIntersectionDist(cv::Point P1,cv::Point P2,cv::Point P3,float &dist)
{
	Vector3f Pos;
	dist = 0;

	EigenRay Ray1 = Cams[0].GetRay(P1);
	EigenRay Ray2 = Cams[1].GetRay(P2);
	EigenRay Ray3 = Cams[2].GetRay(P3);
	Vector3f I12A,I12B,I13A,I13B,I23A,I23B;
	I12A = g3d::RayRayIntersect(Ray1,Ray2);
	I12B = g3d::RayRayIntersect(Ray2,Ray1);
	dist += ((I12A-I12B).norm());
	I13A = g3d::RayRayIntersect(Ray1,Ray3);
	I13B = g3d::RayRayIntersect(Ray3,Ray1);
	dist += ((I13A-I13B).norm());
	I23A = g3d::RayRayIntersect(Ray2,Ray3);
	I23B = g3d::RayRayIntersect(Ray3,Ray2);
	dist += ((I23A-I23B).norm());

	Pos = (I12A + I12B + I13A + I13B + I23A + I23B) / 6;
	return Pos;
}
//--------------------------------------------------------------------------------------------------------------
void S3DEnv_c::DrawPoint3D(Vector3f Pos,std::vector<cv::Mat> &Imgs,cv::Scalar Color)
{
	for(int i=0;i<Cams.size();i++)
	{
		cv::Point CCenter = Cams[i].Project(Pos);
		cv::circle(Imgs[i],CCenter,5,Color,3);
	}
}
//--------------------------------------------------------------------------------------------------------------
void S3DEnv_c::DrawPoint3D(const Vector4f &Pos,cv::Scalar Color)
{
	for(int i=0;i<Cams.size();i++)
	{
		Cams[i].DrawPoint3D(Pos,Color);
	}
}
//--------------------------------------------------------------------------------------------------------------
void S3DEnv_c::DrawReference(Matrix4f &mCurrent,float VSize,float charsizeRef)
{
	for(int i=0;i<Cams.size();i++)
	{
		Cams[i].DrawReference(mCurrent,VSize,charsizeRef);
	}
}
//--------------------------------------------------------------------------------------------------------------
void S3DEnv_c::DrawDoublePoint3D(Vector3f Pos,std::vector<cv::Mat> &Imgs,cv::Scalar Color1,cv::Scalar Color2)
{
	for(int i=0;i<Cams.size();i++)
	{
		cv::Point CCenter = Cams[i].Project(Pos);
		cv::circle(Imgs[i],CCenter,8,Color2,8);
		cv::circle(Imgs[i],CCenter,3,Color1,3);
	}
}
//--------------------------------------------------------------------------------------------------------------
void S3DEnv_c::DrawLine3D(const Vector4f &Point1,const Vector4f &Point2,cv::Scalar Color)
{
	for(int i=0;i<Cams.size();i++)
	{
		Cams[i].DrawLine3D(Point1,Point2,Color);
	}
}
//--------------------------------------------------------------------------------------------------------------
void S3DEnv_c::DrawRay(EigenRay Ray,float Length,cv::Scalar color)
{
	for(int i=0;i<Cams.size();i++)
	{
		Cams[i].DrawRay(Ray,Length,color);
	}
}
//--------------------------------------------------------------------------------------------------------------
void S3DEnv_c::DrawPoint3DRect(Vector3f Pos,std::vector<cv::Mat> &Imgs,float RSize)
{
	for(int i=0;i<Cams.size();i++)
	{
		cv::Point CCenter = Cams[i].Project(Pos);
		int Size = (int)Cams[i].ProjectSize(Pos,RSize);
		cv::Rect HeadRect(CCenter.x-Size/2,CCenter.y-Size/2,Size,Size);
		cv::rectangle(Imgs[i],HeadRect,cv::Scalar(155,250,145),2);
		//cv::circle(Imgs[i],CCenter,Size,cv::Scalar(250,250,150),5);
	}
}
//--------------------------------------------------------------------------------------------------------------
void S3DEnv_c::DrawBox(const EigenBox3D &Box)
{
	for(int i=0;i<(int)Cams.size();i++)
	{
		Cams[i].DrawBox(Box);
	}
}
//--------------------------------------------------------------------------------------------------------------
void S3DEnv_c::DrawModel(const char*WinName,float Light)
{
	for(int i=0;i<Cams.size();i++)
	{
		WModel.Draw(Cams[i]);
	}
}
//--------------------------------------------------------------------------------------------------------------
void S3DEnv_c::DrawMoCapPos(int FrameIndex,cv::Scalar Color)
{
	if(MoCap.BodyParts.size()>0)
	{
		if(FrameIndex>=MoCap.BodyParts[0].PartAnim.size())
		{
			return;
		}
	}
	else
	{
		return;
	}

	for(int i=0;i<MoCap.BodyParts.size();i++)
	{
		DrawPoint3D(V3To4(MoCap.BodyParts[i].PartAnim[FrameIndex]),Color);
	}
}
//--------------------------------------------------------------------------------------------------------------
void S3DEnv_c::DrawJointsPos(BodyJointsPose_c Pose,cv::Scalar Color)
{
	for(int i=0;i<Pose.BodyParts.size();i++)
	{
		DrawPoint3D(V3To4(Pose.BodyParts[i]),Color);
	}
}
//--------------------------------------------------------------------------------------------
void S3DEnv_c::Cvt_MoCap2SkelDim(int iFrame)
{
	int MId = 0;
	WModel.hBody = MoCap.GetLength(MC_Pelvis,MC_Thorax,iFrame);
	WModel.wShoulders = MoCap.GetLength(MC_LeftShoulder,MC_RightShoulder,iFrame);
	WModel.wWaist = MoCap.GetLength(MC_LeftHip,MC_RightHip,iFrame);
	WModel.Arm = MoCap.GetLength(MC_RightShoulder,MC_RightElbow,iFrame);
	WModel.Forearm = MoCap.GetLength(MC_RightElbow,MC_RightWrist,iFrame);
	WModel.Leg = MoCap.GetLength(MC_RightHip,MC_RightKnee,iFrame);
	WModel.Calf = MoCap.GetLength(MC_RightKnee,MC_FootRight,iFrame);
	WModel.Neck = MoCap.GetLength(MC_Thorax,MC_Head,iFrame);//We got an issue on the head's pos ??? Do we ?

	WModel.UpdateDefault();//Sets The Default positions using the Body Length Params, hBody...
	//MySoft3D.WModel.FillNodesCacheTables();//useless doesn't work
	printf("hBody: %1.2f\n",WModel.hBody);
	printf("wShoulders: %1.2f\n",WModel.wShoulders);
	printf("wWaist: %1.2f\n",WModel.wWaist);
	printf("Arm: %1.2f\n",WModel.Arm);
	printf("Forearm: %1.2f\n",WModel.Forearm);
	printf("Leg: %1.2f\n",WModel.Leg);
	printf("Calf: %1.2f\n",WModel.Calf);
	printf("Neck: %1.2f\n",WModel.Neck);
}
//--------------------------------------------------------------------------------------------
void S3DEnv_c::Cvt_MoCap2Posture(int iFrame)
{
	Matrix4f S3DMatrix;
	Vector3f S3DVect,VectAL,VectAR,VectTL,VectTR,VPelvis;
	//Pelvis----------------------------------------------------------------------------------
	VPelvis = MoCap.BodyParts[MC_Pelvis].PartAnim[iFrame];
	VectAL = MoCap.BodyParts[MC_LeftShoulder].PartAnim[iFrame];
	VectAR = MoCap.BodyParts[MC_RightShoulder].PartAnim[iFrame];
	VectTL = MoCap.BodyParts[MC_LeftHip].PartAnim[iFrame];
	VectTR = MoCap.BodyParts[MC_RightHip].PartAnim[iFrame];

	WModel.MoveTo(VPelvis,VectAL,VectAR,VectTL,VectTR);// Pelvis->mLocal -> Must MoveTo as Pos to Adjust are Global
	S3DMatrix = WModel.pPelvis->mLocal;

	bool isDrawRef = false;
	if(isDrawRef)
	{
		g3d::printfMatrix4f(S3DMatrix);
		DrawReference(S3DMatrix,40);
	}

	float p1 = 0.04f;
	float p2 = 0.0016f;
	/*
	float precision = 0.1;
	Vector4f AbsolutePos = V3To4(MoCap.BodyParts[MC_FootLeft].PartAnim[iFrame]);
	float minLength = FLT_MAX;
	float minFx,minFy;
	float StartLimit = 0.0f;
	float StopLimit = 1.05f;
	for(float vfy=StartLimit;vfy<StopLimit;vfy+=precision)
	for(float vfx=StartLimit;vfx<StopLimit;vfx+=precision)
	{
		Vector4f CPos = WModel.pLKnee->GetChildPosFromParams(vfx,vfy);
		Vector3f VectLength = V4To3(AbsolutePos - CPos);
		float VLength = VectLength.norm();
		cv::Scalar Color = cv::Scalar(0,70-VLength*5,VLength*5);
		DrawPoint3D(CPos,Color);
		if (VLength < minLength) 
		{
			minLength = VLength;
			minFx = vfx;
			minFy = vfy;
		}
	}
	WModel.pLKnee->MoveTo(minFx,minFy);
	Vector4f CPos = WModel.pLKnee->GetChildPosFromParams(minFx,minFy);
	DrawReference(g3d::MatrixTranslation(CPos),20);
	DrawReference(g3d::MatrixTranslation(AbsolutePos),20);
	*/
	
	WModel.pRShoulder->MoveToAbsoluteCheckAllDouble(V3To4(MoCap.BodyParts[MC_RightElbow].PartAnim[iFrame]),p1,p2);
	WModel.pLShoulder->MoveToAbsoluteCheckAllDouble(V3To4(MoCap.BodyParts[MC_LeftElbow].PartAnim[iFrame]),p1,p2);
	WModel.pRElbow->MoveToAbsoluteCheckAllDouble(V3To4(MoCap.BodyParts[MC_RightWrist].PartAnim[iFrame]),p1,p2);
	WModel.pLElbow->MoveToAbsoluteCheckAllDouble(V3To4(MoCap.BodyParts[MC_LeftWrist].PartAnim[iFrame]),p1,p2);
	WModel.pRThigh->MoveToAbsoluteCheckAllDouble(V3To4(MoCap.BodyParts[MC_RightKnee].PartAnim[iFrame]),p1,p2);
	WModel.pLThigh->MoveToAbsoluteCheckAllDouble(V3To4(MoCap.BodyParts[MC_LeftKnee].PartAnim[iFrame]),p1,p2);
	WModel.pRKnee->MoveToAbsoluteCheckAllDouble(V3To4(MoCap.BodyParts[MC_RightAnkle].PartAnim[iFrame]),p1,p2);
	WModel.pLKnee->MoveToAbsoluteCheckAllDouble(V3To4(MoCap.BodyParts[MC_LeftAnkle].PartAnim[iFrame]),p1,p2);
	
}
//--------------------------------------------------------------------------------------------------------------
BodyJointsPose_c S3DEnv_c::Cvt_MoCap2JointsPos(int iFrame)
{
	BodyJointsPose_c Pose;
	Pose.BodyParts.resize(19);
	Pose.BodyParts[0] = MoCap.BodyParts[MC_Pelvis].PartAnim[iFrame];// 0 - Pelvis
	Pose.BodyParts[1] = MoCap.BodyParts[MC_LeftHip].PartAnim[iFrame];
	Pose.BodyParts[2] = MoCap.BodyParts[MC_LeftKnee].PartAnim[iFrame];
	Pose.BodyParts[3] = MoCap.BodyParts[MC_LeftAnkle].PartAnim[iFrame];
	Pose.BodyParts[4] = MoCap.BodyParts[MC_FootLeft].PartAnim[iFrame];
	Pose.BodyParts[5] = MoCap.BodyParts[MC_RightHip].PartAnim[iFrame];
	Pose.BodyParts[6] = MoCap.BodyParts[MC_RightKnee].PartAnim[iFrame];
	Pose.BodyParts[7] = MoCap.BodyParts[MC_RightAnkle].PartAnim[iFrame];
	Pose.BodyParts[8] = MoCap.BodyParts[MC_FootRight].PartAnim[iFrame];
	Pose.BodyParts[9] = MoCap.BodyParts[MC_Thorax].PartAnim[iFrame];
	Pose.BodyParts[10] = MoCap.BodyParts[MC_LeftShoulder].PartAnim[iFrame];
	Pose.BodyParts[11] = MoCap.BodyParts[MC_LeftElbow].PartAnim[iFrame];
	Pose.BodyParts[12] = MoCap.BodyParts[MC_LeftWrist].PartAnim[iFrame];
	Pose.BodyParts[13] = MoCap.BodyParts[MC_HandLeft].PartAnim[iFrame];
	Pose.BodyParts[14] = MoCap.BodyParts[MC_RightShoulder].PartAnim[iFrame];
	Pose.BodyParts[15] = MoCap.BodyParts[MC_RightElbow].PartAnim[iFrame];
	Pose.BodyParts[16] = MoCap.BodyParts[MC_RightWrist].PartAnim[iFrame];
	Pose.BodyParts[17] = MoCap.BodyParts[MC_HandRight].PartAnim[iFrame];
	Pose.BodyParts[18] = MoCap.BodyParts[MC_Head].PartAnim[iFrame];

	return Pose;
}
//--------------------------------------------------------------------------------------------------------------
void S3DEnv_c::Display(const char*WinName)
{
	std::vector<cv::Mat> Imgs(Cams.size());
	for(int i=0;i<Cams.size();i++)
	{
		Imgs[i] = Cams[i].ImgRender;
	}
	mcv::imNShow(Imgs,WinName);
}
//--------------------------------------------------------------------------------------------------------------
//Approximate box caus we project the silhouette on the blob center plane
EigenBox3D S3DEnv_c::BlobToBox(std::vector<cv::Mat> &ImgsBkg)
{
//#define DEBUG_BLOB_TO_BOX
#ifdef	DEBUG_BLOB_TO_BOX
	ClearBuffers();
	FillBuffers(ImgsBkg);
#endif

	bool isDebug = true;
	assert(ImgsBkg.size() == Cams.size());
	std::vector<cv::Mat> Bkgs(ImgsBkg.size());
	for(UINT i=0;i<ImgsBkg.size();i++)
	{
		ImgsBkg[i].copyTo(Bkgs[i]);
	}

	//printf("This Function Spoils the images has to make a copy\n");
	int NbImgs = (int)Bkgs.size();
	VPoints Contour;
	V2DPoints contours;
	std::vector<EigenBlob2DExtremas> Boxes(NbImgs);
	EigenBox3D Box3D;
	std::vector<EigenRay> BlobRays(NbImgs);
	for(int i=0;i<NbImgs;i++)
	{
		//cv::findContours(Bkgs[i],contours,CV_RETR_LIST,CV_CHAIN_APPROX_NONE);//CV_RETR_EXTERNAL
		cv::findContours(Bkgs[i],contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);//
#ifdef	DEBUG_BLOB_TO_BOX
		cv::drawContours(this->Cams[i].ImgRender,contours,-1,cv::Scalar(0,0,255),2);
#endif
		//printf("NbContours : %d\n",contours.size());
		Contour = mcv::ContoursFilter(contours,-1);//Mode -1 : Keep the longest
		Boxes[i] = g3d::ContourToExtremas(Contour);//box would be invalid if Contour's size is 0
	}
	//--------------------------------------Here we already should display the 2D Boxes Extremas
	for(int i=0;i<NbImgs;i++)
	{
		if(Boxes[i].isValid)
		{
			int		x = (int)(( Boxes[i].Right.x() + Boxes[i].Left.x() ) / 2),
					y = (int)(( Boxes[i].Top.y() + Boxes[i].Bottom.y() ) / 2);
			BlobRays[i] = Cams[i].GetRay(cv::Point(x,y));
			BlobRays[i].Weight = 1;
		}
		else//Not valid here
		{
			int		x = ( Cams[i].SurfWidth ) / 2,
					y = ( Cams[i].SurfWidth ) / 2;			//Screen center
			BlobRays[i] = Cams[i].GetRay(cv::Point(x,y));
			BlobRays[i].Weight = 0.00000001f;					//Weight looses the ray
		}
	}
	Vector3f BlobCenter = g3d::RaysNWeighted(BlobRays);
	for(int i=0;i<NbImgs;i++)
	{
		if(Boxes[i].isValid)
		{
			Eigen::Vector3f Cam2BCenter = BlobCenter - Cams[i].GetPos3D();
			float Dist = Cam2BCenter.norm();
			Vector3f P3DTop    = Cams[i].GetRayPointDist(PF2I(Boxes[i].Top),Dist);
			Vector3f P3DBottom = Cams[i].GetRayPointDist(PF2I(Boxes[i].Bottom),Dist);
			Vector3f P3DLeft   = Cams[i].GetRayPointDist(PF2I(Boxes[i].Left),Dist);
			Vector3f P3DRight  = Cams[i].GetRayPointDist(PF2I(Boxes[i].Right),Dist);
			
			Box3D.Add(P3DTop);
			Box3D.Add(P3DBottom);
			Box3D.Add(P3DLeft);
			Box3D.Add(P3DRight);
		}
	}

#ifdef	DEBUG_BLOB_TO_BOX
	std::vector<EigenRay> TestRays(NbImgs);
	for(int i=0;i<NbImgs;i++)
	{
		//TestRays[i] = Cams[i].GetRay(cv::Point(Cams[i].SurfWidth/2,Cams[i].SurfHeight/2));
		//DrawRay(TestRays[i],900,mcv::clGreen);
	}

	for(int i=0;i<BlobRays.size();i++)
	{
		//BlobRays[i].printf("BlobRays[i]");
		DrawRay(BlobRays[i],900,mcv::clBlue);
	}
	DrawBox(Box3D);
	Display("BKGViews");
	cv::waitKey();
#endif
	return Box3D;
}
//--------------------------------------------------------------------------------------------------------------
Vector3f S3DEnv_c::BlobCenter(const std::vector<cv::Mat> &ImgsBkg)
{
	assert(ImgsBkg.size() == Cams.size());
	std::vector<cv::Mat> Bkgs(ImgsBkg.size());
	for(UINT i=0;i<ImgsBkg.size();i++)
	{
		ImgsBkg[i].copyTo(Bkgs[i]);
	}

	//printf("This Function Spoils the images has to make a copy\n");
	int NbImgs = (int)Bkgs.size();
	VPoints Contour;
	V2DPoints contours;
	std::vector<EigenBlob2DExtremas> Boxes(NbImgs);
	EigenBox3D Box3D;
	std::vector<EigenRay> BlobRays(NbImgs);
	for(int i=0;i<NbImgs;i++)
	{
		//cv::findContours(Bkgs[i],contours,CV_RETR_LIST,CV_CHAIN_APPROX_NONE);//CV_RETR_EXTERNAL
		//cv::imshow("Image to find contour on",Bkgs[i]);
		cv::findContours(Bkgs[i],contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);//
		//printf("NbContours : %d\n",contours.size());
		//cv::waitKey();
		Contour = mcv::ContoursFilter(contours,-1);//Mode -1 : Keep the longest
		Boxes[i] = g3d::ContourToExtremas(Contour);//box would be invalid if Contour's size is 0
	}
	//--------------------------------------Here we already should display the 2D Boxes Extremas
	for(int i=0;i<NbImgs;i++)
	{
		if(Boxes[i].isValid)
		{
			int		x = (int)(( Boxes[i].Right.x() + Boxes[i].Left.x() ) / 2),
					y = (int)(( Boxes[i].Top.y() + Boxes[i].Bottom.y() ) / 2);
			BlobRays[i] = Cams[i].GetRay(cv::Point(x,y));
			BlobRays[i].Weight = 1;
		}
		else//Not valid here
		{
			int		x = ( Cams[i].SurfWidth ) / 2,
					y = ( Cams[i].SurfWidth ) / 2;			//Screen center
			BlobRays[i] = Cams[i].GetRay(cv::Point(x,y));
			BlobRays[i].Weight = 0.00000001f;					//Weight looses the ray
		}
	}
	Vector3f VRes = g3d::RaysNWeighted(BlobRays);
	return VRes;
}
//--------------------------------------------------------------------------------------------------------------
void S3DEnv_c::Undistort(const std::vector<cv::Mat> &Imgs,std::vector<cv::Mat> &UndistImgs)
{

	for(int i=0;i<(int)Cams.size();i++)
	{
		cv::Mat CamIntrinsicMat(3,3,CV_32FC1);
		cv::Mat DistCoeffs(1,5,CV_32FC1);
		CamIntrinsicMat.setTo(cv::Scalar(0));
		DATA32FC1(CamIntrinsicMat,0) = Cams[i].Fx;
		DATA32FC1(CamIntrinsicMat,4) = Cams[i].Fy;
		DATA32FC1(CamIntrinsicMat,2) = Cams[i].Cx;
		DATA32FC1(CamIntrinsicMat,5) = Cams[i].Cy;
		DATA32FC1(CamIntrinsicMat,8) = 1;
		for(int j=0;j<5;j++)
		{

			DATA32FC1(DistCoeffs,j) = Cams[i].Dist_coeffs.data.fl[j];
			//float f1 = Cams[i].Dist_coeffs->data.fl[j];
			//printf("%f ",f1);
		}
		//printf("\n");cv::waitKey();
		cv::undistort(Imgs[i],UndistImgs[i],CamIntrinsicMat,DistCoeffs);
	}
}

//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//											View_c
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
bool View_c::Grab(int Index)
{
	Buffer = Stream.GetFrameByIndex(Index);
	return true;
}
//--------------------------------------------------------------------------------------------------------------
cv::Mat View_c::GetFrameByIndex(int Index)
{
	Grab(Index);
	return Buffer;
}
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//											S3DGrabber_c
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
void S3DGrabber_c::SetupGrabber(stringmap &Config)//Parses "CfgFile" and Adds "Channels" "BKG,BPR...."
{
	cv::FileStorage Fs(Config["CfgFile"],cv::FileStorage::READ);

	if(!Config["Channels"].empty())//All Views Same channel initialisation
	{
		Views.resize(Fs["Views"].size());
		//--------------------------------------------------------------------------------------"Channels" : "BKG"
		for(size_t i=0;i<Fs["Views"].size();i++)
		{
			Views[i].ChanType = Config["Channels"];
			Views[i].Source = "IMGFileStream";
			Views[i].Stream.Init(Config["CfgFile"],"Views",Views[i].ChanType,i);//use of SeqIndex
			std::string CamCaliFileName = (std::string)Fs["Views"][i]["Calib"];
			CamCaliFileName = mcv::GetMainPath(Config["CfgFile"]) + CamCaliFileName;
			Views[i].Cam.Load(CamCaliFileName.c_str());//So that we have the width and height
		}
	}
	else//Every View has a specific channel for initialisation
	{
		//--------------------------------------------------------------------------------------("View00","BKG")("View01","BPR")
		char ViewName[10];
		bool isViewListed = true;
		int index=0;
		do
		{
			sprintf(ViewName,"View%02d",index);
			if(!Config[ViewName].empty())
			{
				Views.resize(index+1);
				Views[index].ChanType = Config[ViewName];
				Views[index].Source = "IMGFileStream";
				Views[index].Stream.Init(Config["CfgFile"],"Views",Views[index].ChanType);
				std::string CamCaliFileName = (std::string)Fs["Views"][index]["Calib"];
				Views[index].Cam.Load(CamCaliFileName.c_str());//So that we have the width and height
			}
			else
			{
				isViewListed = false;
			}
			index++;
		}while(isViewListed);
	}
	printf("GrabberConfig:\n");
	for(size_t i=0;i<Views.size();i++)
	{
		printf(" - View(%02d) '%s' (%dx%d)\n",i,Views[i].ChanType.c_str(),Views[i].Cam.SurfWidth,Views[i].Cam.SurfHeight);
	}
	//int NbViews = index;
	//Buffers.resize(NbViews);
}
//--------------------------------------------------------------------------------------------------------------
//void S3DGrabber_c::SetChannels(std::string &ChannelTag)//"Any" will take the First of every channels list
//--------------------------------------------------------------------------------------------------------------
void S3DGrabber_c::GrabFrameIndex(int Index)
{
	Buffers.resize(Views.size());
	for(size_t i=0;i<Views.size();i++)
	{
		Buffers[i] = Views[i].GetFrameByIndex(Index);
	}
}
//--------------------------------------------------------------------------------------------------------------
std::vector<cv::Mat> S3DGrabber_c::GetFrames(int Index)
{
	GrabFrameIndex(Index);
	return Buffers;
}
//--------------------------------------------------------------------------------------------------------------
void S3DGrabber_c::Render(Soft3DCamera_c &Cam)//Takes the Cam Id and fill it with the corresponding Channel
{
	assert(Cam.SurfHeight == Views[Cam.Id].Cam.SurfHeight);
	assert(Cam.SurfWidth == Views[Cam.Id].Cam.SurfWidth);
	if(Buffers[Cam.Id].channels() ==1)
	{
		cv::cvtColor(Buffers[Cam.Id],Cam.ImgRender,cv::COLOR_GRAY2BGR);
	}
	else
	{
		Buffers[Cam.Id].copyTo(Cam.ImgRender);
	}
}
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//------------ S3DBaseViewer_c ---------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
S3DBaseViewer_c::S3DBaseViewer_c(S3DEnv_c *pvS3D,const char*vWinName)
{
	pS3D = pvS3D;
	WinName = vWinName;
	isExportEnabled = false;
}
//--------------------------------------------------------------------------------------------------------------
void S3DBaseViewer_c::AddToRender(Renderable_c* pToRender)
{
	RenderList.push_back(pToRender);
}
//--------------------------------------------------------------------------------------------------------------
void S3DBaseViewer_c::AddToGrab(mcv::Grabbable_c* pToGrab)
{
	GrabList.push_back(pToGrab);
}
//--------------------------------------------------------------------------------------------------------------
void S3DBaseViewer_c::GrabFrameIndex(int Index)
{
	for(int i=0;i<GrabList.size();i++)
	{
		GrabList[i]->GrabFrameIndex(Index);
	}
}
//--------------------------------------------------------------------------------------------------------------
void S3DBaseViewer_c::EnableExport(const std::string &FileName)
{
	isExportEnabled = true;
	StreamOut.Init(FileName.c_str(),0,10000);
}

//--------------------------------------------------------------------------------------------------------------
void onMouse(int event, int x, int y, int flags, void* param)
{
	S3DFlyCamViewer_c *pViewer = (S3DFlyCamViewer_c*)param;
	//--------------------------------------------------------------------------Events
	if((event & cv::EVENT_LBUTTONDOWN) && (flags & cv::EVENT_FLAG_LBUTTON))
	{
		pViewer->Mouse.LDown = cv::Point(x,y);
		//pViewer->Cam.RollAround(Vector3f(0,0,0),0,0);
	}
	if((event & cv::EVENT_RBUTTONDOWN) && (flags & cv::EVENT_FLAG_RBUTTON))
	{
		pViewer->Mouse.RDown = cv::Point(x,y);
	}
	if((event & cv::EVENT_MBUTTONDOWN) && (flags & cv::EVENT_FLAG_MBUTTON))
	{
		pViewer->Mouse.MDown = cv::Point(x,y);
	}
	//--------------------------------------------------------------------------Flags
	if(flags & CV_EVENT_FLAG_LBUTTON)//Roll around object
	{
		cv::Point PMove = cv::Point(x,y) - pViewer->Mouse.LDown;
		float FPS = 30;
		float DeltaX = (100.0/FPS) * (((float)PMove.x)/10.0);
		float DeltaY = -(100.0/FPS) * (((float)PMove.y)/10.0);
		pViewer->Cam.TranslateRelative(Vector3f(DeltaX,DeltaY,0));
		//Start rolling on origin
		//pViewer->Cam.RollAround(Vector3f(0,0,0),DeltaX,DeltaY);
	}
	if(flags & CV_EVENT_FLAG_RBUTTON)//Rotate
	{
			cv::Point PMove = cv::Point(x,y) - pViewer->Mouse.RDown;
			float FPS = 30;
		if(abs(PMove.x) > abs(PMove.y))
		{
			pViewer->Cam.RotateYRelative((PI_div_2/FPS) * (((float)PMove.x)/500.0));
		}
		else
		{
			pViewer->Cam.RotateXRelative((PI_div_2/FPS) * (((float)PMove.y)/500.0));
		}
	}
	if(flags & cv::EVENT_FLAG_MBUTTON)//Forward backward
	{
		cv::Point PMove = cv::Point(x,y) - pViewer->Mouse.MDown;
		float FPS = 30;
		float DeltaY = -(300.0/FPS) * (((float)PMove.y)/100.0);
		pViewer->Cam.TranslateRelative(Vector3f(0,0,DeltaY));
	}
	
}
//--------------------------------------------------------------------------------------------------------------
//------------ S3DBaseViewer_c ---------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
S3DFlyCamViewer_c::S3DFlyCamViewer_c(S3DEnv_c *pvS3D,const char*vWinName):S3DBaseViewer_c(pvS3D,vWinName)
{
	//pS3D = pvS3D; done on S3DBaseViewer_c(pvS3D)
	if(!pS3D->Cams.empty())
	{
		Cam = pS3D->Cams[0];
	}
	Cam.Id = -1;//The FlyCam
	cv::setMouseCallback(vWinName,onMouse,this);
}
//--------------------------------------------------------------------------------------------------------------
void S3DFlyCamViewer_c::Render(const char*WINNAME,bool DoClear)
{
	bool isRenderCams = true;
	bool isRenderReference = true;
	if(DoClear)
	{
		Cam.ClearBuffers();
	}
	for(int i=0;i<RenderList.size();i++)
	{
		RenderList[i]->Render(Cam);
	}
	if(isRenderCams)
	{
		for(size_t i=0;i<pS3D->Cams.size();i++)
		{
			pS3D->Cams[i].Render(Cam);
		}
	}
	if(isRenderReference)
	{
		Cam.DrawReference(g3d::MatrixIdentity(),50,0.4);
	}
	if(isExportEnabled)
	{
		StreamOut.WriteFrame(Cam.ImgRender);
	}
	if(WINNAME!=NULL)
	{
		Display(WINNAME);
	}
}
//--------------------------------------------------------------------------------------------------------------
void S3DFlyCamViewer_c::Display(const char*WINNAME)
{
	Cam.Display(WINNAME);
}
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
S3DMultiCamViewer_c::S3DMultiCamViewer_c(S3DEnv_c *pvS3D):S3DBaseViewer_c(pvS3D)
{
}
//--------------------------------------------------------------------------------------------------------------
void S3DMultiCamViewer_c::SetUpGrabber(stringmap &Config)//will call SetupGrabber() with the same map
{
	Grabber.SetupGrabber(Config);
	AddToGrab(&Grabber);
	AddToRender(&Grabber);
}
//--------------------------------------------------------------------------------------------------------------
void S3DMultiCamViewer_c::Render(const char*WINNAME,bool DoClear)//Render the RenderList for every pS3D->Cam
{
	//we render in the pS3D->Cams.ImgRender
	bool isRenderReference = true;
	if(DoClear)
	{
		for(size_t i=0;i<pS3D->Cams.size();i++)
		{
			pS3D->Cams[i].ClearBuffers();
		}
	}
	for(int r=0;r<RenderList.size();r++)
	{
		for(size_t c=0;c<pS3D->Cams.size();c++)
		{
			RenderList[r]->Render(pS3D->Cams[c]);
			if(isRenderReference)
			{
				pS3D->Cams[c].DrawReference(g3d::MatrixIdentity(),50,0.4);
			}
		}
	}
	if(WINNAME!=NULL)
	{
		Display(WINNAME);
	}
}
//--------------------------------------------------------------------------------------------------------------
void S3DMultiCamViewer_c::Display(const char*WINNAME)
{
	//overload mcv::imNShow with std::vector<cv::Mat&> or diplay in any other way
	//mcv::imNShow(Grabber.Buffers,WINNAME);
	//As the render has gone in the S3DEnv
	pS3D->Display(WINNAME);
}
//--------------------------------------------------------------------------------------------------------------
