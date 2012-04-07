/*
	Inherited from S3DGeom out on Saturday February 18th 2012 @ 13h10
	The S3DVox File got out of S3DGeom so that it can use a Cam Object while
	S3DCam is using the S3DGeom, to avoid cyclic referencing
	The S3DVox contains voxels Tables that are 3D Arrays of a <BlockType>
	The inherit from Renderable_c so that they can Render themselves on a Soft3DCamera_c
	
*/

#pragma once
#ifndef __S3DVox__
#define __S3DVox__
//#pragma message("------------------------------------S3DVox.h------------------------------------")

#include "S3DEnv.h"

#include "cv.h"
#include "highgui.h"

#include "mcvGeneral.h"
#include "MultiCamStream.h"
#include "mrgRegression.h"
#include "S3DGeom.h"
#include "S3DCamera.h"

#include <iostream>
#include <fstream>

namespace g3d
{
//----------------------------------------------------------------------------------------------------------

//Nice trial but compile link error, then when constructor is here: Err no appropriate const available
template<typename BlockType>
class VoxelSpace_t
{
public:
	EigenBox3D	Box;
	std::vector<BlockType>	Data;
	int Xv,Yv,Zv;
	int strideZ;
	float VoxelSize;// == 1 to start
public:
	VoxelSpace_t();
	VoxelSpace_t(EigenBox3D	&vBox);

	void SetUpVoxelData(float vVoxelSize = 1.0f);
	BlockType& at(int x,int y,int z);
	BlockType& operator()(const Vector3f &Pos);

};

//----------------------------------------------------------------------------------------------------------
//should be equivalent to the VectorsTable_c
class VoxelSpace_c:public Renderable_c,public mcv::Grabbable_c
{
public:
	EigenBox3D	Box;
	std::vector<unsigned char>	Data;
	std::vector<ScalarC>		ColorTable;//same indexing as in every Anim[]
	int NbFill;
	int Xv,Yv,Zv;
	int strideY,strideZ;
	float VoxelSize;// == 1 to start
	std::string TypeName;
	std::vector<std::string>	ClassesNames;//Optionnal Use
	int RenderStep;
	bool isVisible;
public:
	VoxelSpace_c(){isVisible=true;}
	VoxelSpace_c(const EigenBox3D	&vBox,float vVoxelSize = 1.0f);
	VoxelSpace_c(const std::string& FileToloadFrom);

	void Reset(const EigenBox3D	&vBox,float vVoxelSize = 1.0f);
	void Reset(float vVoxelSize);//Keep it's box but clears the data
	void Reset(Vector3f RefBoxLow, int vXv,int vYv,int vZv, float vVoxelSize);

	Vector4f Pos(int VoxOffset);						// Vox indexe to World Pos
	Vector4f Pos(int i,int j,int k);					// Vox indexes to World Pos
	unsigned char& operator()(int i,int j,int k);		// Vox indexes to content
	unsigned char& operator()(const Vector4f &Pos);		// World Pos to Vox content
	unsigned char& operator()(const Vector3f &Pos);		// World Pos to Vox content

	int VectToOffset(const Vector3f &RelativeVect);		// RelativeVect to Relative Offset

	void save(const std::string &FileName,const std::string &UserHeader = "");
	bool load(const std::string &FileName,YAML::Node &user_doc);
	bool load(const std::string &FileName);

	//------------------------------------------------------------------------------------------------------
	IMGFileStream	VoxFileStream;
	void SetupGrabber(stringmap &Config);//Tag in "Poses" Node - not very generic
	void GrabFrameIndex(int Index);//Overriding the Grabbable class
	//------------------------------------------------------------------------------------------------------
	void Render(Soft3DCamera_c &Cam);//Overriding the Renderable_c::Render

	//-------------------------------------------------------------------------------------------utilities--
	int countNonZero();
	int countHigherThan(unsigned char Val);
	//unsigned char& atClosestNZ(Vector3f& Pos);//could be derived from SetAtClosestNZ if needed
	bool SetAtClosestOne(Vector3f& Ref,unsigned char Val);
	int Propagate();
	int Replace(unsigned char oldVal,unsigned char NewVal);
	int MedianFilter(int MaxClasses);
	void ToVectTab(mrg::VectorsTable_c &VectTab);
	void FromVectTab(mrg::VectorsTable_c &VectTab);
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//-------------------------------------------------------------------------------------------------
class Voxelliser_c//and 3D Descriptor manager
{
public:
	std::string CfgFileName;
	S3DEnv_c		*pS3D;//Just for the cameras
public:
	//VoxelSpace_c	Vox;
public:
	Voxelliser_c(S3DEnv_c	*pvS3D)
	{
		pS3D = pvS3D;
	}
	
	//fixed config for pS3DEnv
	int Process2DToVox(std::vector<cv::Mat> BKGViews,VoxelSpace_c&Vox,float VoxSize = 1.0f);
	int Process2DToVox(std::vector<cv::Mat> BKGViews,std::string &FileName,float VoxSize);//Adds user header info

	//Variable config with some views from pS3D
	int ProcessVox(S3DGrabber_c* pGrabber,VoxelSpace_c &Vox,float VoxSize = 1.0f);//lol Yeah the grabber isn't ready yet
};

//----------------------------------------------------------------------------------------------------------

}/*end of namespace g3d*/

#endif /*__S3DVox__*/
