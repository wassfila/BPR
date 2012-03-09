/*
	File started on 03/02/2011
	Implementation of the Shape Context descriptor
	according to [A.Agarwal and B.Triggs PAMI 2006]
	included the variants of [Poppe FGR 2006] for
	reverse distance square soft voting and
	of [M.Barnard ACVIS 2008] for weighted radial
	distance of the descriptor circles

	FileName changed from "mcvShapeContext.h" to "DetectorHSC.h" on Oct 12th 2011
	The ex "Descriptor_c" class will be transfered here and renamed DetectorHSC_c
	with the ShapeContext_c class

*/

#ifndef __DETECTORBPR__
#define __DETECTORBPR__

#include "cv.h"
#include "highgui.h"

#include "opencv2/ml/ml.hpp"

#include "mrgRegression.h"

#include "S3DGeom.h"
#include "S3DVox.h"
#include "S3DEnv.h"
#include "S3DModel.h"

#define PART_HEAD		0
#define PART_BODY		1
#define PART_SHOULDERS	2
#define PART_ELBOW		3
#define PART_HANDS		4
#define PART_KNEE		5
#define PART_FEET		6
#define PART_BACKGROUND 7
#define PART_UNCKNOWNBODYPART 8
#define NB_PARTS	9

using namespace std;
using namespace g3d;
using namespace mrg;


namespace hpr
{


	const std::string BodyPartsLabels[9] = {"Head","Body","Shoulders","Elbow","Hands","Knee","Feet","BKG","Uncknown"};
	const uchar BPCol_B[9] = {0,	255,	150,	0,		255,	0,		150,	255,	0};
	const uchar BPCol_G[9] = {255,	255,	150,	150,	0,		0,		0,		255,	0};
	const uchar BPCol_R[9] = {0,	0,		0,		0,		0,		150,	150,	255,	0};
//-------------------------------------------------------------------------------------------------
class Sgn_s
{
public:
	// No more divs afeter 2Pi, nor after Pi/2 for Latitude
	//Either NbDiv or NbDivLat must be 1 else the points distance will be desequilibred
	int NbDiv;//		!!! Might be one thus becoming seemlessly a random descriptor
	float Distcm;
	float AngleOffset;	//Teta - [0,2Pi] - Goes on (x,y)
	float AngleLatitude;//Beta - [-Pi/2,Pi/2]
	int NbDivLat;
public:
	Sgn_s();
};

class BasicDescriptor_c
{
public:
	std::vector<Sgn_s>				SignsList;	//SgList
	std::vector<Eigen::Vector3f>	Vects;		//Vects in cm whether 2D (x,y) in Cam Plane or 3D (x,y,z)
	float MinVectsDist_cm;
	float							Size_cm;
	std::string						Name;
public:
	void clear();
	void SignsListToVects();	//Updates sizeR_cm
	float GetSize_cm();

	//inherited class saves by it's only serialisation is enough for base class
	void load(const std::string &FileName);//,const char *DescTag = "Desc2DSign");
};

//typedef std::map<std::string,std::string> MapStrings_t;
//-------------------------------------------------------------------------------------------------
//should be enhanced for Specific Vectors use, most significant vectors (0 or 1) and not significant vectors
//or significance rate
//this same descriptor can be used for depth with a depth value in stand of only 0 or 1
class PixSign_c: public BasicDescriptor_c
{
public:
	std::vector<int>		PixOffsets;
	//--------------------------------LastImageProcessSpecific Data
	int ImgW,ImgH;
	float SizeR_px;
	float Pxlpercm;
public:
	float BKG_Default;
public:
	PixSign_c();
	void clear();
	void ScaleToImage(float NewScaleVal,const cv::Mat &Image);//works for Gray and Depth Images

	void PixSigns2D(const cv::Mat &Img,int PixOffset,float*pVals);
	void PixSignsDepth(const cv::Mat &ImgSilhouette,const cv::Mat &Img,int PixOffset,float*pVals);

	void save(const std::string &FileName);//saves specific info but loads only General ones

	void PixSignsDepthDraw(const cv::Mat &DepthImg24,int PixOffset);

};
//-------------------------------------------------------------------------------------------------
class VoxSign_3D_c: public BasicDescriptor_c,public Renderable_c
{
public:
	g3d::VoxelSpace_c* pVox;
	std::vector<int>		VoxOffsets;//Hase the VectorsTable.VectSize
	//--------------------------------LastVoxProcessSpecific Data
	//float Size_cm;
	int Vx,Vy,Vz;
	//int LastVoxOffsetJustForRender;//Just rendered in center of its Voxe's box
public:
	VoxSign_3D_c();
	void clear();
	void FitOffsetsToVox(g3d::VoxelSpace_c* pvVox);//equivalent to the 2D Scale and fit

	void VoxSigns3D(int VoxOffset,float*pVals);//on the Vox Taken at FitOffsetsToVox
	void VoxSigns3D(int VoxOffset,unsigned char*pBitVals);

	void save(const std::string &FileName);//saves specific info but loads only General ones

	void Render(Soft3DCamera_c &Cam);//Renderable_c overload
};
//-------------------------------------------------------------------------------------------------
//Here we have a [Pixels] [ANN] [PixelSign] [Simple and Multi Views] Classifier
class PartsPixels2D_c
{
public:
	PixSign_c		PxSign;
	Soft3DCamera_c *pCam;
	BodyJoints_c	*pModel;//For GetColor() and GetClass() and To save the Names and colors on the File
public:
	//--------------------------------------------------------------------------------Clean API
	int ProcessTrain2D(const cv::Mat &ImgSilhouette,VectorsTable_c &TrainTable);
	int ProcessTrainDepth(const cv::Mat &ImgSilhouette,const cv::Mat &ImgDepthmm,VectorsTable_c &TrainDepthTable);
	int ProcessLabels(const cv::Mat &ImgSilhouette,const cv::Mat &ImgParts,VectorsTable_c &LabelsTable);
	int MatchLabelsToImg(const cv::Mat &ImgSilhouette,cv::Mat &ImgParts,VectorsTable_c &LabelsTable);

	void ScaleSignDepth(const cv::Mat &ImgSilhouette,const cv::Mat &ImgDepthmm);

	//--------------------------------------------------------------------------------overloads
	//These overloads add info about user header
	int ProcessTrain2D(const cv::Mat &ImgSilhouette,std::string &FileName);
	int ProcessTrainDepth(const cv::Mat &ImgSilhouette,const cv::Mat &ImgDepthmm,std::string &FileName);
	int ProcessLabels(const cv::Mat &ImgSilhouette,const cv::Mat &ImgParts,std::string &FileName);
	int MatchLabelsToImg(const cv::Mat &ImgSilhouette,cv::Mat &ImgParts,std::string &FileName);

	//void BlobToParts(std::vector<cv::Mat> &ImgsClasses,std::vector<cv::Mat> &ImgsBKG,std::vector<cv::Mat> &ImgsDrawOn);
};
//-------------------------------------------------------------------------------------------------
class PartsPixels2DMultiView_c
{
public:
	std::string CfgFileName;
	std::vector<PartsPixels2D_c>	Parts2D;
	S3DEnv_c		*pS3D;//Just for the cameras
	//MLData_c		*pML;
public:
	PartsPixels2DMultiView_c(S3DEnv_c	*pvS3DEnv,std::string vCfgFileName);//,MLData_c *pvML

	void ScaleSigns(std::vector<cv::Mat> &ImgsSilh);
	void SetUpDescSign(const char* TagName);

	void DebugDisplayPxSign(std::vector<cv::Mat> &ImgsSilh);
};
//-------------------------------------------------------------------------------------------------
class VoxDetector_c:public Voxelliser_c//and more
{
public:
	VoxSign_3D_c	VoxSign;
	std::string		CfgFileName;
	BodyJoints_c*	pModel;//Used to update the Vox Color Index Table

public:
	VoxDetector_c(S3DEnv_c	*pvS3D,const std::string& vCfgFileName):Voxelliser_c(pvS3D)
	{
		CfgFileName = vCfgFileName;
		SetUpDescSign("Desc3DSign");
	}
	//utilities
	void SetUpDescSign(const std::string& Tag);
	void UpdateVoxColorTable(VoxelSpace_c& Vox);
	int ProcessVoxPartsClosest(BodyJointsPose_c& Pose,VoxelSpace_c& Vox);//Set Colors on Vox
	int ProcessVoxPartsPropagation(BodyJointsPose_c& Pose,VoxelSpace_c& Vox);//Set Colors on Vox
	int ProcessVoxPartsSpatialCartography(BodyJointsPose_c& Pose,VoxelSpace_c& Vox);//Set Colors on Vox
	int ProcessVoxParts(BodyJointsPose_c& Pose,VoxelSpace_c& Vox);//Set Colors on Vox

	//Main Processing functions
	int ProcessTrain3D(VoxelSpace_c& Vox,VectorsTable_c &Train3D);
	int ProcessResp3DParts(BodyJointsPose_c& Pose,VoxelSpace_c& Vox,VectorsTable_c &Resp3D);//Set Colors on Vox
	int ProcessResp3DJVects(BodyJointsPose_c& Pose,VoxelSpace_c& Vox,VectorsTable_c &Resp3D);//Set Colors on Vox

	void ProcessVoxParts2Centers(VoxelSpace_c& Vox,std::vector<Vector3f>& BodyParts);//Starts from 2
};
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
class MeanShift3DPart_c
{
public:
	char					Name[30];
	cv::Scalar				Color;
	std::vector<Vector3f>	MeanShift3DRes;
	std::vector<float>		FillRate;
	bool					isPosFound;
	std::vector<Vector3f>	Pos;
	float					WindowSize;
	float					Step3DSize;
	int						NbGridPoints;
	//Methods
	MeanShift3DPart_c();
	bool					AddPos(Vector3f NewVect,float delta,float fill);
	void					ClearPos();
	Vector3f				GetPos();
	bool					GetPos(Vector3f &P1,Vector3f &P2);
};
//--------------------------------------------------------------------------------------------------------------
class DetectorBPR_c
{
	//enum { Sign2D=0, SignDepth=1, Voxels3D=2 };
public:
	PartsPixels2DMultiView_c		BPR_2DMV;
	VoxDetector_c					Voxelliser;
	BodyJoints_c					Model;//Used to update the Vox Color Index Table

	S3DEnv_c					*pS3DEnv;
	MLData_c					*pML;//Handled by the user, so that he can learn many DBs
	std::string					ConfigFileName;
	std::string					MainPath;

	void SetROCParam(float RCP);

public:
	void SetUpJoints(const std::string& Tag);//with usually private usage

	//--------------------------------------------------------------------------------------------------------------------------------------
	//---New API----------------------------------------------------------------------------------------------------------------------------
	//--------------------------------------------------------------------------------------------------------------------------------------
	void ProcessTrain2D(int Start,int Last,			const char*MStreamsBKG_In			= "BKG",
													const char*MStreamsD2DTrain_Out		= "Desc2DTrain");
	void ProcessTrainDepth(int Start,int Last,		const char*MStreamsBKG_In			= "BKG",
													const char*MStreamsDepth_In			= "Depth",
													const char*MStreamsDepthTrain_Out	= "DepthTrain");
	void Process2DToVox(int Start,int Last,float VoxSize,
													const char*Streams_In				= "BKG",//Multi
													const char*StreamVox_Out			= "Voxels");//Mono
	void ProcessTrain3D(int Start,int Last,			const char*StreamVox_In				= "Voxels",//Mono
													const char*Stream3DTrain_Out		= "Desc3DTrain");//Mono
	void ProcessRespLabels(int Start,int Last,		const char*MStreamsBKG_In			= "BKG",
													const char*MStreamsBPR_In			= "BPR",
													const char*MStreamsLabelsResp_Out	= "LabelResp");
	void ProcessResp3D(int Start,int Last,			const char*StreamVox_In				= "Voxels",		//Mono-Vox
													const char*StreamJCenters_In		= "Joints",		//Mono-vector
													const char*StreamResp3D_Out			= "Parts3DResp",//Mono-VectorsTable
													const char*ExportVoxParts = NULL);					//Mono-Vox Parts
	//Converting from result back to display
	//Thus Converting from indexed representation to dense representation
	void ProcessRespLabels2Imgs(int Start,int Last,	const char*MStreamsBKG_In			= "BKG",
													const char*MStreamsLabelsResp_In	= "LabelResp",
													const char*MStreamsBPR_Out			= "BPRres");
	void ProcessResp3D2Vox(int Start,int Last,		const char*StreamResp3D_In			= "J3DResp",//Mono-VectorsTable
													const char*StreamVox_Out			= "Voxels");//Mono-Vox

	void ProcessResp3DToVox(int Start,int Last,		const char*RefVox_In				= "VoxelsParts",
													const char*RespResVt_In				= "Parts3DRespres",
													const char*RespResVox_Out			= "VoxelsPartsres");
	void VoxMedianFilter(int Start,int Last,		const char*RefVox_InOut				= "VoxelsPartsres",
													const char*RespResVt_Out			= "Parts3DRespres");
	void ProcessVoxParts2Centers(int Start,int Last,const char*PartsResp_in				= "VoxelsPartsres",
													const char*JointsRes_Out			= "Jointsres");
	void CompareCenters(int Start,int Last,			const char*JointsRef_in				= "Joints",
													const char*JointsRes_in			= "Jointsres");
	//---------------------------------------------------------------------------------------------------------------------- Overloads -----
	void ProcessTrain2D(int Start,int Last,			const char*MStreamsBKG_In,VectorsTable_c &TrainTable);
	void ProcessRespLabels(int Start,int Last,		const char*MStreamsBKG_In,const char*MStreamsBPR_In,VectorsTable_c &LabelsTable);
	void ProcessTrainDepth(int Start,int Last,const char*MStreamsBKG_In,const char*MStreamsDepth_In,VectorsTable_c &TrainTable);
	
	void ProcessTrain3D(int Start,int Last,	const char*StreamVox_In,VectorsTable_c &Train3DTable);//append on Train3D


public:
	DetectorBPR_c(S3DEnv_c	*pvS3DEnv,std::string CfgFileName):BPR_2DMV(pvS3DEnv,CfgFileName),Voxelliser(pvS3DEnv,CfgFileName)
		{
			pS3DEnv = pvS3DEnv;
			ConfigFileName = CfgFileName;
			MainPath = mcv::GetMainPath(CfgFileName);
			SetUpJoints("Joints");
			Voxelliser.pModel = &Model;//Goes through reload as the Model is unique
		}

};



}/* end of namespace hpr */


//YAML Emit----------------------------------------------------------------------------------------
YAML::Emitter& operator << (YAML::Emitter& out, const std::vector<hpr::Sgn_s> SignsList);
YAML::Emitter& operator<<(YAML::Emitter& out, const hpr::BasicDescriptor_c& PxS);
YAML::Emitter& operator<<(YAML::Emitter& out, const hpr::PixSign_c& PxS);
//YAML Parse----------------------------------------------------------------
void operator >> (const YAML::Node& node, hpr::Sgn_s& Sign);
void operator >> (const YAML::Node& node, hpr::BasicDescriptor_c& PxS);
void operator >> (const YAML::Node& node, hpr::PixSign_c& PxS);
//ostream----------------------------------------------------------------
ostream& operator<<(ostream &out, const hpr::BasicDescriptor_c& PxS);
ostream &operator<<(ostream &out, const hpr::PixSign_c& PxS);
ostream& operator<<(ostream &out, const hpr::VoxSign_3D_c& VxS);

#endif /*__DETECTORBPR__*/
