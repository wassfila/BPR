/*
	S3DModel is a division of Soft3DWireModel into and Model and Env Files
	Started on Sept 29 2011


	Soft3DWireModel.h file started on March 25th 2011
	The objective is to have a purely soft 3D wire model that will be lightweight
	and subject to multithreading acceleration.
	Its light weight will allow not only points projections but also curve trajectory
	computation that would help finding optimal points on the curve, thus even higher
	dimentionnal space coverage
*/
#pragma once
#ifndef __S3MODEL__
#define __S3MODEL__

#include "cv.h"
#include "highgui.h"
#include <Eigen\Dense>
#include<Eigen/StdVector>


#include "mcvGeneral.h"
#include "S3DGeom.h"
#include "S3DCamera.h"
#include "mrgRegression.h"
using namespace mrg;


#define PF2I(P)	cv::Point((int)P.x(),(int)P.y())


//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
class BodyJointsPose_c
{
public:
	std::vector<Vector3f> BodyParts;//x,y,z  Body Joints Centers
public:
	//BodyJointsPose_c(){}
	//BodyJointsPose_c(int NbJoints){BodyParts.resize(NbJoints);}
	int Compare(BodyJointsPose_c OtherPose);
	std::vector<float> Get();
	void Set(std::vector<float> &Vect);
	int GetClosestJointIndex(const Vector4f& Pos);
	int GetClosestJointIndex_10(const Vector4f& Pos);

	//--------------------------------------------------------
	std::string Serialize();
	void Deserialize(std::string &Blob);
};
//--------------------------------------------------------------------------------------------------------------
class BodyJoints_c:public Renderable_c,public mcv::Grabbable_c
{
public:
	std::vector<std::string>		BJointsNames;//same indexing as in every Anim[]
	std::vector<int>				BJointsParts;//optionnal - might be identity
	std::vector<std::string>		BPartsNames;//optionnal - use Joints Names
	std::vector<ScalarC>			BPartsColors;
	std::vector<BodyJointsPose_c>	Poses;
	//bool							isConfigured;
	int	CurrentPoseIndex;
	int FirstIndex;
public:
	BodyJoints_c(){clear();}
	BodyJoints_c(stringmap &Config);
	void load(const std::string &FileName); //loads a JointsPosDesc Channel File with FileStorage
	void save(const std::string &FileName);
	void outputPartsColors(YAML::Emitter& Yout);

	void ToVectTab(mrg::VectorsTable_c &VectTab);
	void FromVectTab(mrg::VectorsTable_c &VectTab);
	void clear();
	void loadData(const std::string &FileName); //To be Deprecated - for temporary compatibility "JPos1.JPos.csv"
	void push(BodyJointsPose_c Pose);

	//--------------------------------------------------------------------------------mcv::Grabbable_c
	void SetupGrabber(stringmap &Config);//Loads "Joints" node from "Poses" Tag of "CfgFile"
	void GrabFrameIndex(int Index);
	int JointToPart(int i);
	//--------------------------------------------------------------------------------Renderable_c
	void Render(Soft3DCamera_c &Cam);//Takes the Cam Id and fill it with the corresponding Channel
	//utilities
	int GetClass(ScalarC Color);
	ScalarC GetColor(int Index);
	//--------------------------------------------------------------------------------compare
	float Compare(BodyJoints_c& OtherBCenters,int StartIndex,int LastIndex);
};

//--------------------------------------------------------------------------------------------------------------
//			For The MoCap Only
//--------------------------------------------------------------------------------------------------------------

#define MC_Pelvis			0
#define MC_LeftHip			1
#define MC_LeftKnee			2
#define MC_LeftAnkle		3
#define MC_RightHip			4
#define MC_RightKnee		5
#define MC_RightAnkle		6
#define MC_Thorax			7
#define MC_LeftShoulder		8
#define MC_LeftElbow		9
#define MC_LeftWrist		10
#define MC_RightShoulder	11
#define MC_RightElbow		12
#define MC_RightWrist		13
#define MC_Head				14
#define MC_HeadLeftBack		15
#define MC_HeadRightBack	16
#define MC_HeadLeftFront	17
#define MC_HeadRightFront	18
#define MC_FootLeft			19
#define MC_FootRight		20
#define MC_HandLeft			21
#define MC_HandRight		22

//--------------------------------------------------------------------------------------------------------------
struct BodyPartTrajectory
{
	char Name[256];
	std::vector<Vector3f> PartAnim;
};
//--------------------------------------------------------------------------------------------------------------
#define MAX_BODY_PARTS 35
class BodyMoCap_c
{
public:
	std::vector<BodyPartTrajectory> BodyParts;
public:
	void load(const char*FileName);
	float GetLengthBody(int iFrame);
	float GetLength(int iPart1,int iPart2,int iFrame);
};

#endif /*__S3DMODEL__*/
