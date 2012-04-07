/*
	S3DEnv is the Camera and Environment parts of Soft3DWireModel.h


	Soft3DWireModel.h file started on March 25th 2011
	The objective is to have a purely soft 3D wire model that will be lightweight
	and subject to multithreading acceleration.
	Its light weight will allow not only points projections but also curve trajectory
	computation that would help finding optimal points on the curve, thus even higher
	dimentionnal space coverage
*/

#pragma once
#ifndef __S3ENV__
#define __S3ENV__
//#pragma message("------------------------------------S3DEnv.h------------------------------------")

#include "NetStream.h"

#include "cv.h"
#include "highgui.h"
#include <Eigen\Dense>
#include<Eigen/StdVector>


#include "MultiCamStream.h"
#include "mcvGeneral.h"
#include "S3DGeom.h"
#include "S3DCamera.h"
#include "S3DModel.h"

#include <iostream>
#include <fstream>


using Eigen::Matrix4f;
using Eigen::Vector4f;
using Eigen::Vector3f;
using Eigen::Vector2f;
using Eigen::Quaternionf;

Vector3f V4To3(const Vector4f &Vect);
Vector4f V3To4(const Vector3f &Vect);


//enum { NotSpecified=0, COLOR=1, BKG=2, BPR=3, Gray=4, Depth=5, Any=6, DescTrain=7, LabelResp=8, BPRres=9, DescTrainResp=10,DepthC=11 };

//--------------------------------------------------------------------------------------------------------------
class S3DEnv_c
{
public:
	int NbCams,NbModels;// Deprecated remove use gradually
	int MaxCams,MaxModels;

	std::string CfgFileName;
	cv::FileStorage					Fs;//what's the best representation of a file than the file itself No more "Views"
	std::string SMainPath;

	std::vector<Soft3DCamera_c,Eigen::aligned_allocator<Soft3DCamera_c>>		Cams;

	int							LastStreamsIndex;
	int							FirstStreamsIndex;
	bool						Motion_isPostion;// Deprecated
	//WireModel_c			WModel;

	BodyMoCap_c MoCap;//To re-Work

public:
    S3DEnv_c(const std::string &vCfgFileName,int vNbCams=10);
	void save(const std::string &vCfgFileName,bool isSaveModel=true);//MainPath to use it for Names Generation
	void load(const std::string &vCfgFileName);
	//---------------------------------------------------------Rendering and Drawing functions
	void SaveRendered(const char*FileName);

	//All deprecated drawing on images functions - use Camera Drawing functions handle depth buffer
	void DrawPoint3D(Vector3f Pos,std::vector<cv::Mat> &Imgs,cv::Scalar Color);
	void DrawPoint3D(const Vector4f &Pos,cv::Scalar Color);
	void DrawReference(Matrix4f &mCurrent,float VSize,float charsizeRef = -1.0f);
	void DrawDoublePoint3D(Vector3f Pos,std::vector<cv::Mat> &Imgs,cv::Scalar Color1,cv::Scalar Color2);
	void DrawLine3D(const Vector4f &Point1,const Eigen::Vector4f &Point2,cv::Scalar Color);
	void DrawRay(EigenRay Ray,float Length,cv::Scalar color);
	void DrawPoint3DRect(Vector3f Pos,std::vector<cv::Mat> &Imgs,float RSize);

	//------------------------------------------------------------------------------
	void DrawBox(const EigenBox3D &Box);//Cam.DrawBox()

	//------------------------------------------------------------------------------------------------All redundent Functions
	void DrawModel(const char*WinName=NULL,float Light=0.5);//Cam.DrawModel
	//void Render(const char*WinName=NULL, bool IsClearBuffers = true);//Draw on Unicolor Background
	//void Render(bool IsClearBuffers);
	void Draw(std::vector<cv::Mat> &ImgsDrawOn,const char*WinName=NULL,float Light=0.5);


	void	DrawMoCapPos(int FrameIndex,cv::Scalar Color);
	void	DrawJointsPos(BodyJointsPose_c Pose,cv::Scalar Color);
	void	Cvt_MoCap2SkelDim(int iFrame);
	void	Cvt_MoCap2Posture(int iFrame);
	BodyJointsPose_c Cvt_MoCap2JointsPos(int iFrame);


	EigenBox3D BlobToBox(std::vector<cv::Mat> &ImgsBkg);
	Vector3f BlobCenter(const std::vector<cv::Mat> &ImgsBkg);

	Eigen::Vector3f PointsToRaysIntersection(cv::Point P1,cv::Point P2,cv::Point P3);//P1=Cam1, P2=Cam2, P3=Cam3
	Eigen::Vector3f PointsToRaysIntersectionDist(cv::Point P1,cv::Point P2,cv::Point P3,float &dist);
	//---------------------------------------------------------
	int EvalFilt(std::vector<cv::Mat> &ImgsBkg);//Eval 3 of ImgsBkg with Cams[] projections on Models[0].EvalFit
	//---------------------------------------------------------
	//Here we will add functions for drawing on the default Env Surface to benefit of the Z buffering
	//---------------------------These should be managed by the Viewer, Renderer, Grabber
	void InitSurfaces();
	void ClearBuffers();
	void FillBuffers(std::vector<cv::Mat>&Imgs);
	void GetBuffers(std::vector<cv::Mat>&Imgs);
	void Display(const char*WinName);

	void Undistort(const std::vector<cv::Mat> &Imgs,std::vector<cv::Mat> &UndistImgs);//Should Be on Cam

	//int GetStreams(MultiCamStream &Streams,int EnumChannelType,const char* MainPath = NULL,int NbImgs = 0);
	int GetStreams(MultiCamStream &Streams,const char * ChannelName);
};

//--------------------------------------------------------------------------------------------------------------
class View_c//Abstract Class of the Views - The inherited Classes should Grab From anywhere - Files, Cams, Network
{
public:
	std::string		ChanType;//"BPR" "BKG" "Depth" ... Can handle a single channel as the S3DGrabber_c is the controller of Channels affectation
	std::string		Source;//"FileStream","net","rosnet"   could be a counter indexed FileName (.png) or a Camera source config File (FireWire,Network)

	//----------------------------
	IMGFileStream	Stream;
	sck::Client		NetStream;//can push or pull, from or into the Buffer, once initialized
	//ros::Client	RosNetStream;

	Soft3DCamera_c Cam;//No render here - should be Simple Cam (S3DCam_c)
	cv::Mat Buffer;//This is the Grabbing only buffer no render here
public:

	bool Grab();
	cv::Mat GetFrame();

	bool Grab(int Index);
	cv::Mat GetFrameByIndex(int Index);
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
//--------------------------------------------------------------------------------------------------------------
//The Main advantage of a Grabber over a direct MultiCamStream is that :
// - it can handle different Channels on different Views !!
// - it will handle different Grab ways : FireWireCams, Network (Maybe through inhancing View_c)
class S3DGrabber_c:public Renderable_c,public mcv::Grabbable_c
{
public:
	//S3DEnv_c *pS3D;//For the Camera's List ?

	//std::string CfgFileName;//The Cfg File contains N Channels in "Views"
	std::vector<View_c,Eigen::aligned_allocator<View_c>> Views;//Every Image will contain the corresponding View[id] - Same size as the Config File
	std::vector<cv::Mat>Buffers;//Points the Buffer of every View !!!
public:
	//SetChannel() : Set the Found channels on the corresponding Views
	//If not All Views Are Found the Buffer is initialized anyway using the "Calib" info for Image size else default(640,480)
	//Call to this function can be and should to be concatenated till all Channels are configured
	//void SetChannels(std::string &ChannelTag);//"Any" will take the First of every channels list
	//std::vector<cv::Mat> GetFrames();
	std::vector<cv::Mat> GetFrames(int Index);

	//--------------------------------------------------------------------------------mcv::Grabbable_c
	void SetupGrabber(stringmap &Config);//Parses "CfgFile" and Adds "Channels" "BKG,BPR...."
	void GrabFrameIndex(int Index);
	void GrabFrame();
	//--------------------------------------------------------------------------------Renderable_c
	void Render(Soft3DCamera_c &Cam);//Takes the Cam Id and fill it with the corresponding Channel
};
//--------------------------------------------------------------------------------------------------------------
class S3DCalibrator_c//Live Calibrator - Grabs the channels, process the interest points, Adjust the calibration
{
public:
	S3DEnv_c *pS3D;				// - modify The Cameras params will be updated
	S3DGrabber_c	*pGrabber;//read only

};
//--------------------------------------------------------------------------------------------------------------
class S3DBaseViewer_c//Half Abstract - Rather Renderer than Viewer
{
public:
	S3DEnv_c *pS3D;//For the Camera's Geometry Pos-Calib...
	std::string WinName;
	IMGFileStream StreamOut;
	bool	isExportEnabled;
	//S3DGrabber_c	Grabber;
	//RenderableObjectsList with function : Render(Soft3DCamera_c Cam);
	std::vector<Renderable_c*> RenderList;
	std::vector<mcv::Grabbable_c*> GrabList;
public:
	S3DBaseViewer_c(S3DEnv_c *pvS3D,const char*vWinName = "");
	void AddToGrab(mcv::Grabbable_c* pToGrab);
	void AddToRender(Renderable_c* pToRender);
	//virtual void SetUpViewer(stringmap &Config) = 0;
	void EnableExport(const std::string &FileName);

	void GrabFrame();
	void GrabFrameIndex(int Index);
	virtual void Render(const char*WINNAME = NULL,bool DoClear = true) = 0;
	virtual void Display(const char*WINNAME) = 0;
	//virtual void save() = 0;
	//virtual void load()= 0;
};

struct MouseParams_t
{
	bool		isLeft;
	cv::Point	LDown;
	cv::Point	RDown;
	cv::Point	MDown;
};
//--------------------------------------------------------------------------------------------------------------
class S3DFlyCamViewer_c:public S3DBaseViewer_c
{
public:
	Soft3DCamera_c Cam;//For the "MonoCam" Mode - Must have an index different from the pS3D Cams ones
	MouseParams_t	Mouse;
public:
	S3DFlyCamViewer_c(S3DEnv_c *pvS3D,const char*vWinName);

	//void SetUpViewer(stringmap &Config);
	void Render(const char*WINNAME = NULL,bool DoClear = true);
	void Display(const char*WINNAME);
	//void save(){}//Just the Cam Config and using mode
	//void load(){}
};
//--------------------------------------------------------------------------------------------------------------
class S3DMultiCamViewer_c:public S3DBaseViewer_c
{
public:
	S3DGrabber_c	Grabber;//The Grabber is the First Element in the RenderList !!!
public:
	S3DMultiCamViewer_c(S3DEnv_c *pvS3D);

	void SetUpGrabber(stringmap &Config);//will call SetupGrabber() with the same map
	void Render(const char*WINNAME = NULL,bool DoClear = true);//Render the RenderList for every pS3D->Cam
	void Display(const char*WINNAME);
	//void save();//Just the Cam Config and using mode
	//void load();
};
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
// DO NOT USE - Re STUDY FIRST
/*
class S3DViewer_c//does not inherit but posees all kinds, Light weight no need for memory efficiency
{
public:

	std::string	Mode;//"Mode" : Could be "FlyCam", "TopView" , "CamsViews"
	S3DFlyCamViewer_c FlyCam;
	S3DMultiCamViewer_c MultiViews;

	void SetUpViewer(stringmap &Config);// "Mode"
	void AddObject(Renderable_c* pToRender);
	void Render(const char*WINNAME = NULL,bool DoClear = true);
	void Display(const char*WINNAME);
	void save();
	void load();
}
*/

#endif /*__S3ENV__*/

