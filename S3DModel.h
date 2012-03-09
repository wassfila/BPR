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
#ifndef __S3MODEL__
#define __S3MODEL__

#include "cv.h"
#include "highgui.h"
#include <Eigen\Dense>

#include "mcvGeneral.h"
#include "S3DCamera.h"
#include "mrgRegression.h"
using namespace mrg;


using Eigen::Matrix4f;
using Eigen::Vector4f;
using Eigen::Vector3f;
using Eigen::Vector2f;
using Eigen::Quaternionf;


#define S_Root		0
#define S_Pelvis	1	
#define S_LThigh	2	
#define S_RThigh	3	
#define S_LCalf		4
#define S_RCalf		5
#define S_LFoot		6
#define S_RFoot		7
#define S_Neck		8
#define S_LArm		9
#define S_RArm		10
#define S_LFArm		11
#define S_RFArm		12
#define S_LHand		13
#define S_RHand		14
#define S_Head		15


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

/*
static const COLORREF g_JointColorTable[NUI_SKELETON_POSITION_COUNT] = 
{
    RGB(169, 176, 155), // NUI_SKELETON_POSITION_HIP_CENTER
    RGB(169, 176, 155), // NUI_SKELETON_POSITION_SPINE
    RGB(168, 230, 29), // NUI_SKELETON_POSITION_SHOULDER_CENTER
    RGB(200, 0,   0), // NUI_SKELETON_POSITION_HEAD
    RGB(79,  84,  33), // NUI_SKELETON_POSITION_SHOULDER_LEFT
    RGB(84,  33,  42), // NUI_SKELETON_POSITION_ELBOW_LEFT
    RGB(255, 126, 0), // NUI_SKELETON_POSITION_WRIST_LEFT
    RGB(215,  86, 0), // NUI_SKELETON_POSITION_HAND_LEFT
    RGB(33,  79,  84), // NUI_SKELETON_POSITION_SHOULDER_RIGHT
    RGB(33,  33,  84), // NUI_SKELETON_POSITION_ELBOW_RIGHT
    RGB(77,  109, 243), // NUI_SKELETON_POSITION_WRIST_RIGHT
    RGB(37,   69, 243), // NUI_SKELETON_POSITION_HAND_RIGHT
    RGB(77,  109, 243), // NUI_SKELETON_POSITION_HIP_LEFT
    RGB(69,  33,  84), // NUI_SKELETON_POSITION_KNEE_LEFT
    RGB(229, 170, 122), // NUI_SKELETON_POSITION_ANKLE_LEFT
    RGB(255, 126, 0), // NUI_SKELETON_POSITION_FOOT_LEFT
    RGB(181, 165, 213), // NUI_SKELETON_POSITION_HIP_RIGHT
    RGB(71, 222,  76), // NUI_SKELETON_POSITION_KNEE_RIGHT
    RGB(245, 228, 156), // NUI_SKELETON_POSITION_ANKLE_RIGHT
    RGB(77,  109, 243) // NUI_SKELETON_POSITION_FOOT_RIGHT
};
*/

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
//-- Serialisation ---------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//-- Gradually Get Deprecated ----------------------------------------------------------------------------------
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

//--------------------------------------------------------------------------------------------------------------
typedef enum S3DJointType_e
{
	S3DJT_Ball, 
	S3DJT_Revolute,
	S3DJT_Fixed,	//for some uses cases of Pelvis-Chest...
	S3DJT_Free		//for the Root-Pelvis
}S3DJointType;

struct CBallCahche_s
{
public:
	Vector2f	Pos;
	Quaternionf Quat;
	Vector3f	Vect3D;
};
class CBallCahche_c
{
public:
	int NbSteps;
	std::vector<std::vector<CBallCahche_s>> Table;
public:
	void PosToIndex(const Vector2f vPos,int &i,int &j);
	Vector2f IndexToPos(int i,int j);
	Vector2f Find(const Vector3f &vVect);
	void Printf();
};


class S3DJTConfigBall
{
public:
	S3DJTConfigBall();
	//To get the default angle, set Angle to 0
	//To get the default axis, select the point that corresponds to the identity Quaternion
	Quaternionf		QTop,QBottom,QRight,QLeft;
	Vector2f		PTop,PBottom,PRight,PLeft;//By default in the [0,1][0,1] square
	Vector2f		Pos;
	Vector3f		DefaultRotAxis;//Initial rotation relative to, Axis
	float			Angle;
	float			MaxAngle,MinAngle;
	
	CBallCahche_c Cache;
public:
	void TestDrawPQ();
	Matrix4f GetLocalFromConf();
	void SetConfFromLocal(Matrix4f &mLocal);
	void GeomProject_P2Q(const Vector2f &P, Quaternionf &Q);
	void GeomProject_Q2P(const Quaternionf &QLocal,Vector2f &PRes);
	void Optimise_Q2P(const Quaternionf &Qref,Vector2f &PosRes);
	void Optimise_P2Q(const Vector2f &vPos, Quaternionf &Qresult);
	void DoubleSlerp_P2Q(const Vector2f &vPos, Quaternionf &Qresult);

	char Optim_OutOfRange(Vector2f &TrackP);
	char Optim_CalcGrad(const Vector2f &RefP,const Vector2f &TrackP,const float &DeltaGrad,Vector2f &Grad);
	float Optim_PQDist(const Vector2f &TrackP,const Quaternionf &QTrack);

	void FillCache();

	void SetParam_Angle(float paramFrom0To1);
	void GetParam_Angle(float &paramFrom0To1);
};

class S3DJTConfigRev
{
public:
	Vector3f		RotAxis;
	float			Pos;// [0,1] will be mapped on Angle from Min to Max
	float			Angle;
	float			MaxAngle,MinAngle;
public:
	Matrix4f GetLocalFromConf();
	void SetParam(float param);
	void GetParam(float &param);
};

struct S3DJointConfig_s
{
	S3DJointType		JT;
	//union here
	S3DJTConfigBall		ConfBall;
	S3DJTConfigRev		ConfRev;
	//No config for Free Joint Type, just for the Root-Pelvis
};

struct LocalCache_s
{
	Vector4f	V;
	float		fx,fy;
};
//--------------------------------------------------------------------------------------------------------------
struct MeanShiftCylinder_s
{
	float scale;
	Matrix4f	M;
};
//--------------------------------------------------------------------------------------------------------------
class ModelNode_s
{
public:
	//Direct Control Params
	float				fx,fy;
	int					XIndex,YIndex;//-1 if the Joint Type Doesn't allow it
	Matrix4f			mDefault;//relative to parent
	Matrix4f			mLocal;//Local movement relative to the default pos
	Matrix4f			mCurrent;//The final pos concatenation of mParent*mDefault*mLocal
	S3DJointConfig_s	Joint;
	Vector4f			Pos;//Current Last Transformed Pos
	ModelNode_s*		pParent;
	ModelNode_s*		pFirstChild;//used just for the Limbs
	char				Name[32];
	cv::Scalar			Color;

	MeanShiftCylinder_s MS;
public:
	ModelNode_s();
	//--------------------------------------------------------------------------------------------
	void UpdateCurrent();
	void UpdateCurrentHierarchy();
	void GetParams(float &vfx,float &vfy);
	void SetParams(float fx,float fy);
	void MoveTo(float fx,float fy);
	void MoveToAbsolute(const Vector4f &AbsolutePos);
	Vector4f			GetRelativePosToDefault(const Vector4f &WorldPos);
	void				MoveToAbsoluteCheckAll(const Vector4f &AbsolutePos,float precision = 0.05f);
	void				MoveToAbsoluteCheckAllDouble(const Vector4f &AbsolutePos,float precision1,float precision2);
	Vector4f			GetRelativePos(const Vector4f &WorldPos);//returns the Node relative Pos
	void				Translate(const Vector4f &VTr);
	void				Rotate(float alpha);
	void				UpdateLocalFromConfig();
	void				UpdateConfigFromLocalMat(const Matrix4f &vMat);
	Vector4f			GetChildPosFromParams(float fx,float fy);
};
//--------------------------------------------------------------------------------------------------------------
class SkelPose_c
{
	std::vector<float> ConfigParams;
	std::vector<float> Position;
	std::vector<float> Orientation;
};
//--------------------------------------------------------------------------------------------------------------
class SkelMotion_c
{
public:
	std::string MotionName;
	int NbParams;
public:
	SkelMotion_c(int SMNbParams):Postures(SMNbParams),Position(3),Orientation(6){NbParams = SMNbParams;}
	VectorsTable_c Postures;
	VectorsTable_c Position;
	VectorsTable_c Orientation;

	void load(const char* BaseFileName);

	//-------------------------------------------------------To Be Developped
	void GetPose(int Index,SkelPose_c &Pose);
	void SetPose(int Index,const SkelPose_c &Pose);
	void PostureDistance(const SkelPose_c &Pose1,const SkelPose_c &Pose2,int DistanceType);//look old Detectors.h Detectors_c
};
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
class WireModel_c
{
private:
	float Scale;
public:
	static const int NbNodes = 17;
	static const int NbConfigParams = 16;
public:
	SkelMotion_c	Motion;
public:
	ModelNode_s	*pRoot,
				*pPelvis,
				*pLThigh,
				*pRThigh,
				*pLKnee,
				*pRKnee,
				*pLFoot,
				*pRFoot,
				*pChest,
				*pLShoulder,
				*pRShoulder,
				*pLElbow,
				*pRElbow,
				*pLHand,
				*pRHand,
				*pHead,
				*pNeck;
	std::vector<ModelNode_s> Nodes;//Table Access
	std::vector<ModelNode_s*> pNodes;//Table Access
	float		hBody,
				wShoulders,
				wWaist,
				Arm,
				Forearm,
				Leg,
				Calf,
				Neck;
	float*		pDimentions[7];
	//std::vector<float>	Params;
public:
	cv::Scalar	PointsColor;
	int			PointsSize;

private:
	void InitSkel();
	void InitNodes();
	void Init();//To be replaced by load
public:
	WireModel_c();
	~WireModel_c();
	void PrintNodes();
	void load(const char* FileName);
	void save(const char* FileName);

	void UpdateDefault();// From Dims to mDefault - only if Dims change
	void UpdateConfig(std::vector<float> &vParams);// From Params to Nodes Config
	void UpdateConfigDeprecated(std::vector<float> &vParams);// From Params to Nodes Config
	void ResetLocals();
	void UpdateLocals(bool isNoPelvisMove = true);// From Config or any to mLocals
	void UpdateCurrent();// From mLocals to mCurrent
	void GetPosture(std::vector<float> &Params);
	void SetPosture(std::vector<float> &vParams);//All Steps
	void SetPosture_Position(const std::vector<float> &Params3);
	void SetPosture_Orientation(const std::vector<float> &Params6);

	void Draw(cv::Mat &ImgDrawOn,Soft3DCamera_c &Cam,float Light=0.5);
	void Draw(Soft3DCamera_c &Cam,float Light=0.5);
	void Draw(std::vector<cv::Mat> &Imgs,Soft3DCamera_c *pCams,float Light=0.5);
	void Draw(Soft3DCamera_c *pCamsArray,float Light=0.5);
	void DrawReferences(Soft3DCamera_c &Cam);
	int EvalFit(cv::Mat &ImgBkg,Soft3DCamera_c &Cam);

	void UpdatePos(float x, float y,float Teta=0);
	void MoveTo(const Vector4f &VPos);
	void MoveTo(const Vector3f &Vect);
	void MoveTo(const Matrix4f &VMat);
	void MoveTo(const Matrix4f &PelvisMat,const Vector3f &LShoulder,const Vector3f &RShoulder,const Vector3f &LThigh,const Vector3f &RThigh);
	void MoveTo(const Vector3f &PelvisPos,const Vector3f &LShoulder,const Vector3f &RShoulder,const Vector3f &LThigh,const Vector3f &RThigh);
	void MoveToMat(const Matrix4f &PelvisMat,const Vector3f &LShoulder,const Vector3f &RShoulder);//This uses ZAxis of Pelvis Mat
	void SetOrientation(const Eigen::Matrix3f &VMat);
	void SetOrientation(const Matrix4f &PelvisMat,const Vector3f &LShoulder,const Vector3f &RShoulder);

	void GetPosition(Vector3f &vPos);
	void GetOrientation(Eigen::Matrix3f &MatOriantation);

	float GetDistance_Config(WireModel_c &WModel);
	float GetDistance_Degree(WireModel_c &WModel);
	float GetDistance_cm(WireModel_c &WModel);

	//------------------Animation
	void SetAnimByIndex(int AnimIndex,bool isPosition = true);

};


#endif /*__S3DMODEL__*/
