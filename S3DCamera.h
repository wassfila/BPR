
#pragma once
#ifndef __S3DCAMERA__
#define __S3DCAMERA__


#include "cv.h"
#include "highgui.h"
#include <Eigen\Dense>
#include<Eigen/StdVector>

#include "S3DGeom.h"
#include "mcvGeneral.h"


using Eigen::Matrix4f;
using Eigen::Vector4f;
using Eigen::Vector3f;
using Eigen::Vector2f;
using Eigen::Quaternionf;

struct Calib_s
{
	float	FocalLength[2];
	float	PrinciplePoint[2];
	float	SkewCoeff;
	CvMat * Dist_coeffs;
	CvMat * Cam_rot;
	CvMat * Cam_tr;
};

class S3DCam_c//We should symplify the Soft3DCamera_c - no not now
{
public:
	//OpenCV Camera params
	cv::Mat dist_coeffs;
	//General
	int SurfWidth,SurfHeight;
	//3D model
	Matrix4f	mView;
	Matrix4f	mProjection;
	float Fx,Fy,Cx,Cy;		//mProjection params
	Matrix4f	mProjView;
	ScalarC		Color;
public:
	S3DCam_c();
	void Load(const char* FileName,bool isPos=true,bool isFocal=true,bool isDist=true);
	void Save(const char* FileName,bool isPos=true,bool isFocal=true,bool isDist=true);
	void load(const char* GroundPosFile,const char* CamCalibFile);// OpenCV calibration file format
	void loadcal(const char* CalibFile);//Human Eva Dataset file format .cal

	cv::Point GetPoint(Vector4f &Point3D);
	cv::Point Project(Vector4f &Point3D);
	cv::Point Project(Vector3f &Point3D);
	//optimal performance with inline, and no data copy
	void ProjectZ(const Vector4f &Point3D,cv::Point &ScreenPos,float &ScreenDepth);
	float ProjectSize(Vector3f &Point3D,float Size);
	float ProjectSize(Vector4f &Point3D,float Size);
	Vector3f GetPos3D();
	EigenRay GetRay(cv::Point ScreenPoint);
	Vector3f GetRayPointDist(cv::Point ScreenPoint, float Dist);

	void Translate(float Tx,float Ty,float Tz);
	void RotateY(float Ry);
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class Soft3DCamera_c
{
public:
	int Id;//in the S3DEnv Cams List - so that the Grabber Becomes renderable in the corresponding Cam
	ScalarC		Color;
	cv::Mat ImgRender;
	bool isCamReverseRays;//glue fix
public:
	bool isSurfaceInitialized;
	cv::Mat ImgZBuffer;
	//Temporary variables do not use
	cv::Mat cam_rot;
	cv::Mat cam_tr;
	cv::Mat camera_mat;
	CvMat Cam_rot;
	CvMat Cam_tr;
	CvMat Camera_mat;
public:
	//OpenCV Camera params
	cv::Mat dist_coeffs;
	CvMat Dist_coeffs;
	//General
	int SurfWidth,SurfHeight;
	//3D model
	Matrix4f	mView;
	Matrix4f	mProjection;
	float Fx,Fy,Cx,Cy;		//mProjection params
	Matrix4f	mProjView;

public:
	Soft3DCamera_c();
	void Load(const char* FileName,bool isPos=true,bool isFocal=true,bool isDist=true);
	void Save(const char* FileName,bool isPos=true,bool isFocal=true,bool isDist=true);
	void load(const char* GroundPosFile,const char* CamCalibFile);// OpenCV calibration file format
	void loadcal(const char* CalibFile);//Human Eva Dataset file format .cal

	cv::Point GetPoint(Vector4f &Point3D);
	cv::Point Project(Vector4f &Point3D);
	cv::Point Project(Vector3f &Point3D);
	//optimal performance with inline, and no data copy
	void ProjectZ(const Vector4f &Point3D,cv::Point &ScreenPos,float &ScreenDepth);
	float ProjectSize(Vector3f &Point3D,float Size);
	float ProjectSize(Vector4f &Point3D,float Size);
	Vector3f GetPos3D();
	EigenRay GetRay(cv::Point ScreenPoint);
	Vector3f GetRayPointDist(cv::Point ScreenPoint, float Dist);
	void DrawRay(cv::Mat &ImgDrawOn,EigenRay Ray,cv::Scalar color);
	void DrawRay(EigenRay Ray,float Length,cv::Scalar color);

	void Translate(float Tx,float Ty,float Tz);
	void Translate(Vector3f& Vect);
	void RotateY(float Ry);
	void RotateXRelative(float Rx);
	void RotateYRelative(float Ry);
	void TranslateRelative(Vector3f& Vect);

	void RollAround(Vector3f& LookAt,float dX,float dY);

	//enriched with Drawing Functions and a Z buffered surface
	void SetSize(int width,int height);
	void InitSurface();
	void ClearBuffers();
	void ClearBuffers(cv::Scalar Color);
	void DrawPoint3D(const Vector4f &Point3D,cv::Scalar Color,bool isPutBlock = true);
	bool IsPoint3DVisible(Vector4f &Point3D);
	void DrawLine3D(const Vector4f &Point1,const Vector4f &Point2,cv::Scalar Color);
	void DrawLine3D(const Vector3f &Point1,const Vector3f &Point2,cv::Scalar Color);
	void DrawReference(Matrix4f &mCurrent,float VSize,float charsizeRef = 0.3f);

	void DrawBox(const EigenBox3D &Box,cv::Mat &Img);
	void DrawBox(const EigenBox3D &Box);

	void Display(const char*WinName);

	void Render(Soft3DCamera_c& RenderCam);
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
//--------------------------------------------------------------------------------------------------------------

//A Class that can be rendered by a Camera So that an Environment can use a list of them and render them all
//Every by it's own

class Renderable_c
{
public:
	virtual void Render(Soft3DCamera_c &Cam) = 0;//Pure Virtual
	//std::cout << "Renderable_c::Render()" << std::endl;
};

//--------------------------------------------------------------------------------------------------------------

#endif /*__S3DCAMERA__*/
