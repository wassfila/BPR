#include "S3DCamera.h"

#include "mcvGeneral.h"
#include <iostream>
#include <fstream>

using namespace mcv;

void RodriguesTes(Soft3DCamera_c *pCam)
{
	g3d::printfMatrix4f(pCam->mView,"mView Before convert");
	g3d::printfMatrix4f(pCam->mProjView,"mProjView before mult");
				
	//save
	g3d::S3DmView2RotMat(&pCam->Camera_mat,&pCam->Cam_tr,pCam->mView);
	printf("Camera_mat:");TabPrintf(pCam->Camera_mat.data.fl,9);
	printf("Cam_rot:");TabPrintf(pCam->Cam_rot.data.fl,3);
	cvRodrigues2(&pCam->Camera_mat,&pCam->Cam_rot);
	cvRodrigues2(&pCam->Cam_rot,&pCam->Camera_mat);
	printf("cvRodrigues2 cvRodrigues2 \n");
	printf("Camera_mat:");TabPrintf(pCam->Camera_mat.data.fl,9);
	printf("Cam_rot:");TabPrintf(pCam->Cam_rot.data.fl,3);
	// => pCam->Cam_rot , pCam->Cam_tr

	//Load
	pCam->mView = g3d::S3DMatrixView(&pCam->Cam_rot,&pCam->Cam_tr);//updates mView - File orientation issue Rot(-PI/2)
	pCam->mProjView = pCam->mProjection * pCam->mView;

	g3d::printfMatrix4f(pCam->mView,"mView After convert");
	g3d::printfMatrix4f(pCam->mProjView,"mProjView After mult");
}
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//										Soft3DCamera_c
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------

Soft3DCamera_c::Soft3DCamera_c()
{
	isSurfaceInitialized = false;
	SetSize(640,480);//Default

	cam_rot = cv::Mat( 3, 1, CV_32FC1 );
	cam_tr = cv::Mat( 3, 1, CV_32FC1 );
	camera_mat = cv::Mat( 3, 3, CV_32FC1 );
	dist_coeffs = cv::Mat( 5, 1, CV_32FC1 );

	camera_mat.setTo(cv::Scalar(0));

	Dist_coeffs = dist_coeffs;
	Cam_rot = cam_rot;
	Camera_mat = camera_mat;
	Cam_tr = cam_tr;

	Camera_mat.data.fl[8] = 1.0f;

	isCamReverseRays = false;
}
//-------------------------------------------------------------------------------------------------------
void Soft3DCamera_c::SetSize(int width,int height)
{
	SurfWidth = width;
	SurfHeight = height;
	if(isSurfaceInitialized)
	{
		InitSurface();
	}
}
//-------------------------------------------------------------------------------------------------------
void Soft3DCamera_c::InitSurface()
{
	ImgRender = cv::Mat(SurfHeight,SurfWidth,CV_8UC3);
	ImgZBuffer = cv::Mat(SurfHeight,SurfWidth,CV_32FC1);
	isSurfaceInitialized = true;
}
//-------------------------------------------------------------------------------------------------------
void Soft3DCamera_c::ClearBuffers()
{
	ClearBuffers(mcv::clWhite);
}
//-------------------------------------------------------------------------------------------------------
void Soft3DCamera_c::ClearBuffers(cv::Scalar Color)
{
	if(!isSurfaceInitialized)InitSurface();
	ImgRender.setTo(Color);
	ImgZBuffer.setTo(cv::Scalar(-FLT_MAX));
}
//-------------------------------------------------------------------------------------------------------
//Main Save to use
void Soft3DCamera_c::Save(const char* FileName,bool isPos,bool isFocal,bool isDist)
{
	cv::FileStorage Fs(FileName, cv::FileStorage::WRITE);

	if(Fs.isOpened())
	{
		if(isPos)//Camera_mat for Pos
		{
			g3d::S3DmView2RotMat(&Camera_mat,&Cam_tr,mView);//Update Camera_mat and Cam_tr from mView
			cvRodrigues2(&Camera_mat,&Cam_rot);
			//printf("rot vect: %1.4f %1.4f %1.4f\n",Cam_rot.data.fl[0],Cam_rot.data.fl[1],Cam_rot.data.fl[2]);
			Fs << "rot_mat" << camera_mat;
			//Fs << "rot_vect" << cam_rot;
			Fs << "tr_vect" << cam_tr;
		}
		if(isFocal)//Camera_mat for Focal
		{
			assert(camera_mat.type() == CV_32FC1);
			camera_mat.setTo(cv::Scalar(0));
			Camera_mat = camera_mat;
			Camera_mat.data.fl[0] = Fx;
			Camera_mat.data.fl[4] = Fy;
			Camera_mat.data.fl[2] = Cx;
			Camera_mat.data.fl[5] = Cy;
			Camera_mat.data.fl[8] = 1.0f;
			//printf("Fx Fy Cx Cy: %1.4f %1.4f %1.4f %1.4f\n",Fx,Fy,Cx,Cy);
			Fs << "vcamera" << camera_mat;
		}
		if(isDist)
		{
			Fs << "vdist_coeffs" << dist_coeffs;
		}

		Fs << "ImageWidth" << SurfWidth;
		Fs << "ImageHeight" << SurfHeight;

		Fs << "Color" << "{";
		Fs << "b" << Color.val[0];
		Fs << "g" << Color.val[1];
		Fs << "r" << Color.val[2];
		Fs << "}";


		printf("Camera File (%s) saved\n",FileName);
	}
	else
	{
		printf("Couldn't Save File (%s)\n",FileName);
	}
}
//-------------------------------------------------------------------------------------------------------
//Main Load to use
void Soft3DCamera_c::Load(const char* FileName,bool isPos,bool isFocal,bool isDist)
{
	//Ground Pos -> view_mat
	bool isVerbose = false;
	cv::FileStorage Fs(FileName,cv::FileStorage::READ);
	if(Fs.isOpened())
	{
		if(isPos)
		{
			Fs["tr_vect"] >> cam_tr;
			Cam_tr = cam_tr;
			isPos = false;
			if(!Fs["rot_mat"].empty())
			{
				Fs["rot_mat"] >> camera_mat;
				Camera_mat = camera_mat;
				mView = g3d::S3DMatrixViewRotMat(&Camera_mat,&Cam_tr);
				isPos = true;
			}
			else if(!Fs["rot_vect"].empty())
			{
				Fs["rot_vect"] >> cam_rot;
				Cam_rot = cam_rot;
				mView = g3d::S3DMatrixView(&Cam_rot,&Cam_tr);
				isPos = true;
			}
			//Yeah, this correction was dues to the bias introduced by the rodrigues conversion
			//mView = g3d::S3DMatrixView(&Cam_rot,&Cam_tr) * g3d::MatrixRotationY(-PI/2);//updates mView - File orientation issue Rot(-PI/2)
		}
		//Cam Calib -> proj_mat
		if(isFocal)
		{
			Fs["vcamera"] >> camera_mat;
			Camera_mat = camera_mat;
			assert(camera_mat.type() == CV_32FC1);
			Fx = (float)Camera_mat.data.fl[0];
			Fy = (float)Camera_mat.data.fl[4];
			Cx = (float)Camera_mat.data.fl[2];
			Cy = (float)Camera_mat.data.fl[5];
			mProjection = g3d::S3DMatrixProjection(Fx,Fy,Cx,Cy);
		}
		if(isDist && !Fs["vdist_coeffs"].empty())
		{
			Fs["vdist_coeffs"] >> dist_coeffs;
			Dist_coeffs = dist_coeffs;
		}
		if(isPos || isFocal)//then update mProjView
		{
			mProjView = mProjection * mView;
		}

		int FWidth,FHeight;
		Fs["ImageWidth"] >> FWidth;
		Fs["ImageHeight"] >> FHeight;
		if((FWidth!=0) && (FHeight!=0))
		{
			SetSize(FWidth,FHeight);
		}
		//RotateYRelative(PI);//to correct the HEBug

		if(!Fs["Color"].empty())
		{
			Fs["Color"]["b"] >> Color.val[0];
			Fs["Color"]["g"] >> Color.val[1];
			Fs["Color"]["r"] >> Color.val[2];
		}
		else
		{
			Color = ScalarC(0,0,255,0);
		}
		if(!Fs["ReverseRays"].empty())
		{
			isCamReverseRays = true;
		}
		else
		{
			isCamReverseRays = false;
		}

		if(isVerbose)printf("Camera File (%s) Loaded\n",FileName);
	}
	else
	{
		printf("Couldn't Load File (%s) - Keeping Default cam config\n",FileName);
	}
}
//-------------------------------------------------------------------------------------------------------
//old load of separate files, deprecated
void Soft3DCamera_c::load(const char* GroundPosFile,const char* CamCalibFile)
{
	//Ground Pos -> view_mat
	cv::FileStorage Fs(GroundPosFile,cv::FileStorage::READ);
	if(Fs.isOpened())
	{

		Fs["rot_vect"] >> cam_rot;
		Fs["tr_vect"] >> cam_tr;
		mView = g3d::S3DMatrixView(&Cam_rot,&Cam_tr) * g3d::MatrixRotationX(-PI/2);//updates mView - File orientation issue Rot(-PI/2)
	}
	else
	{
		printf("Couldn't Load File (%s)\n",GroundPosFile);
	}

	//mView = S3DMatrixTranslation(0,0,5);//updates mView

	//Cam Calib -> proj_mat
	cv::FileStorage Fs2(CamCalibFile,cv::FileStorage::READ);
	if(Fs2.isOpened())
	{
		Fs2["vcamera"] >> camera_mat;
		Fs2["vdist_coeffs"] >> dist_coeffs;
		Camera_mat = camera_mat;
		Dist_coeffs = dist_coeffs;
		switch(camera_mat.type())
		{
		case CV_32FC1:
			Fx = (float)Camera_mat.data.fl[0];
			Fy = (float) Camera_mat.data.fl[4];
			Cx = (float) Camera_mat.data.fl[2];
			Cy = (float) Camera_mat.data.fl[5];
			break;
		case CV_64FC1:
			Fx = (float)Camera_mat.data.db[0];
			Fy = (float) Camera_mat.data.db[4];
			Cx = (float) Camera_mat.data.db[2];
			Cy = (float) Camera_mat.data.db[5];
			break;
		}
		mProjection = g3d::S3DMatrixProjection(Fx,Fy,Cx,Cy);

		mProjView = mProjection * mView;
	}
	else
	{
		printf("Couldn't Load File (%s)\n",CamCalibFile);
	}
}
//-------------------------------------------------------------------------------------------------------
//human Eva .cal File format
void Soft3DCamera_c::loadcal(const char* CalibFile)
{
	std::string line;
	std::ifstream myfile(CalibFile);
	if (myfile.is_open())
	{
		Fx		= readFloatLine(myfile);
		Fy		= readFloatLine(myfile);
		Cx		= readFloatLine(myfile);
		Cy		= readFloatLine(myfile);
		float skew = readFloatLine(myfile);
		assert(camera_mat.type() == CV_32FC1);
		assert(cam_tr.type() == CV_32FC1);
		assert(dist_coeffs.type() == CV_32FC1);
		for(int i=0;i<5;i++)Dist_coeffs.data.fl[i] = readFloatLine(myfile);
		for(int i=0;i<9;i++)Camera_mat.data.fl[i] = readFloatLine(myfile);
		for(int i=0;i<3;i++)Cam_tr.data.fl[i] = (readFloatLine(myfile)/10);//unit from mm to cm
	}
	bool isDebug = true;
	if(isDebug)
	{
		printf("Fx %1.3f  Fy %1.3f  Cx %1.3f  Cy %1.3f\n",Fx,Fy,Cx,Cy);
		printf("distCoeffs:");for(int i=0;i<5;i++)printf(" %1.3f ",Dist_coeffs.data.fl[i]);;printf("\n");
		printf("Camera_mat:\n");
		for(int i=0;i<3;i++)printf(" %1.3f ",Camera_mat.data.fl[i]);printf("\n");
		for(int i=3;i<6;i++)printf(" %1.3f ",Camera_mat.data.fl[i]);printf("\n");
		for(int i=6;i<9;i++)printf(" %1.3f ",Camera_mat.data.fl[i]);printf("\n");
		printf("Cam_tr:");for(int i=0;i<3;i++)printf(" %1.3f ",Cam_tr.data.fl[i]);printf("\n");
		printf("\n");
	}
	//Cam_rot = rodrigues(Camera_mat);
	//mView = g3d::S3DMatrixViewRotMat(Camera_mat,Cam_tr) * g3d::MatrixRotationX(PI/2);//Because we have a ground of (X,Z) and not (X,Y)
	mView = g3d::S3DMatrixViewRotMat(&Camera_mat,&Cam_tr);//Keep Coord as is

	//Cam Calib -> proj_mat
	mProjection = g3d::S3DMatrixProjection(Fx,Fy,Cx,Cy);

	mProjView = mProjection * mView;

}
//-------------------------------------------------------------------------------------------------------
//-------------------------- Cam Geometry ---------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void Soft3DCamera_c::Translate(Vector3f& Vect)
{
	mView *= g3d::MatrixTranslation(Vect.x(),Vect.y(),Vect.z());//updates mView - File orientation issue Rot(-PI/2)
	mProjView = mProjection * mView;
}
//-------------------------------------------------------------------------------------------------------
void Soft3DCamera_c::Translate(float Tx,float Ty,float Tz)
{
	mView *= g3d::MatrixTranslation(Tx,Ty,Tz);//updates mView - File orientation issue Rot(-PI/2)
	mProjView = mProjection * mView;
}
//-------------------------------------------------------------------------------------------------------
void Soft3DCamera_c::TranslateRelative(Vector3f& Vect)
{
	EigenRay RUp = GetRay(cv::Point(SurfWidth/2,0));
	EigenRay RDown = GetRay(cv::Point(SurfWidth/2,SurfHeight-1));
	Vector3f PUp = RUp.GetPoint(1.0);
	Vector3f PDown = RDown.GetPoint(1.0);
	Vector3f UpDown = PUp - PDown;

	EigenRay RLeft = GetRay(cv::Point(0,SurfHeight/2));
	EigenRay RRight = GetRay(cv::Point(SurfWidth-1,SurfHeight/2));
	Vector3f PLeft = RLeft.GetPoint(1.0);
	Vector3f PRight = RRight.GetPoint(1.0);
	Vector3f RightLeft = PRight - PLeft;

	EigenRay RCenter = GetRay(cv::Point(SurfWidth/2,SurfHeight/2));
	Vector3f FrontBack = RCenter.Vector;

	Vector3f Translation = Vect.x()*RightLeft + Vect.y()*UpDown + Vect.z()*FrontBack;
	Translate(Translation);
}
//-------------------------------------------------------------------------------------------------------
void Soft3DCamera_c::RotateYRelative(float Ry)
{
	EigenRay RUp = GetRay(cv::Point(SurfWidth/2,0));
	EigenRay RDown = GetRay(cv::Point(SurfWidth/2,SurfHeight-1));
	Vector3f PUp = RUp.GetPoint(1.0);
	Vector3f PDown = RDown.GetPoint(1.0);
	Vector3f UpDown = PUp - PDown;


	Vector3f CamPos = GetPos3D();
	mView = g3d::Matrix3fTo4f(g3d::Matrix4fTo3f(mView));
	mView *= g3d::Matrix3fTo4f(	Eigen::Matrix3f(Eigen::AngleAxisf(	Ry,UpDown)));
	Vector4f NewPos = mView * V3To4(-CamPos);
	g3d::matrixSetTrVector(mView,NewPos);

	mProjView = mProjection * mView;
}
//-------------------------------------------------------------------------------------------------------
void Soft3DCamera_c::RotateXRelative(float Rx)
{
	EigenRay RLeft = GetRay(cv::Point(0,SurfHeight/2));
	EigenRay RRight = GetRay(cv::Point(SurfWidth-1,SurfHeight/2));
	Vector3f PLeft = RLeft.GetPoint(1.0);
	Vector3f PRight = RRight.GetPoint(1.0);
	Vector3f RightLeft = PRight - PLeft;

	Vector3f CamPos = GetPos3D();
	mView = g3d::Matrix3fTo4f(g3d::Matrix4fTo3f(mView));
	mView *= g3d::Matrix3fTo4f(	Eigen::Matrix3f(Eigen::AngleAxisf(	Rx,RightLeft)));
	Vector4f NewPos = mView * V3To4(-CamPos);
	g3d::matrixSetTrVector(mView,NewPos);

	mProjView = mProjection * mView;
}
//-------------------------------------------------------------------------------------------------------
void Soft3DCamera_c::RotateY(float Ry)
{
//	mView *= g3d::MatrixRotationY(Ry);//updates mView - File orientation issue Rot(-PI/2)
//	mProjView = mProjection * mView;
	Vector3f CamPos = GetPos3D();
	mView = g3d::Matrix3fTo4f(g3d::Matrix4fTo3f(mView));
	mView *= g3d::MatrixRotationY(Ry);
	Vector4f NewPos = mView * V3To4(-CamPos);
	g3d::matrixSetTrVector(mView,NewPos);

	mProjView = mProjection * mView;
}
//-------------------------------------------------------------------------------------------------------
void Soft3DCamera_c::RollAround(Vector3f& LookAt,float dX,float dY)
{
	Vector3f InitPos = GetPos3D();
	Vector3f Backward = InitPos - LookAt;
	Vector3f Up = g3d::MatrixGetYVector(g3d::Matrix4fTo3f(mView));
	Vector3f Right = Up.cross(Backward);
	Up = Backward.cross(Right);
	mView = g3d::Matrix3fTo4f(g3d::MatrixAxis(Right,Up,Backward));

	//Vector4f NewPos = mView * V3To4(-InitPos);
	//g3d::matrixSetTrVector(mView,NewPos);
}
//-------------------------------------------------------------------------------------------------------
cv::Point Soft3DCamera_c::GetPoint(Vector4f &Point3D)
{
	cv::Point Res;
	CvMat* TrackObj3D = cvCreateMat( 1, 1, CV_32FC3 );
	CvMat* TrackObj2D = cvCreateMat( 1, 1, CV_32FC2 );
	TrackObj3D->data.fl[0] = Point3D.x();
	TrackObj3D->data.fl[1] = Point3D.y();
	TrackObj3D->data.fl[2] = Point3D.z();
	cvProjectPoints2(TrackObj3D,&Cam_rot,&Cam_tr,&Camera_mat,&Dist_coeffs,TrackObj2D);
	//cv::projectPoints(
	Res.x = (int)TrackObj2D->data.fl[0];
	Res.y = (int)TrackObj2D->data.fl[1];
	return Res;
}
//-------------------------------------------------------------------------------------------------------
cv::Point Soft3DCamera_c::Project(Vector3f &Point3D)
{
	Vector4f P4F;
	P4F << Point3D.x(), Point3D.y(), Point3D.z(), 1;
	return Project(P4F);
}
//-------------------------------------------------------------------------------------------------------
cv::Point Soft3DCamera_c::Project(Vector4f &Point3D)
{
	Vector4f RVect;
	RVect =  mProjView * Point3D;// x1M = 12.00 ms
	//SpeedUp over : RVect =  mProjection * mView * Point3D;// x1M = 40.00 ms
	//SpeedUp over : RVect =  mProjection * (mView * Point3D);// x1M = 25.33 ms
	return cv::Point(	(int)(RVect.x()/RVect.z()),
						(int)(RVect.y()/RVect.z())
					);
}
//-------------------------------------------------------------------------------------------------------
INLINE void Soft3DCamera_c::ProjectZ(const Vector4f &Point3D,cv::Point &ScreenPos,float &ScreenDepth)
{
	Vector4f RVect;
	RVect =  mProjView * Point3D;// x1M = 12.00 ms
	ScreenPos.x = (int)(RVect.x()/RVect.z());
	ScreenPos.y = (int)(RVect.y()/RVect.z());
	if(isCamReverseRays)
	{
		ScreenDepth = -RVect.z();
	}
	else
	{
		ScreenDepth = RVect.z();
	}
}
//-------------------------------------------------------------------------------------------------------
float Soft3DCamera_c::ProjectSize(Vector3f &Point3D,float Size)
{
	Vector4f P4F;
	P4F << Point3D.x(), Point3D.y(), Point3D.z(), 1;
	Vector4f RVect;
	RVect =  mProjView * P4F;
	return abs((Size * (Fx+Fy)/2 )/RVect.z());
}
//-------------------------------------------------------------------------------------------------------
float Soft3DCamera_c::ProjectSize(Vector4f &Point3D,float Size)
{
	Vector4f RVect;
	RVect =  mProjView * Point3D;
	return abs((Size * (Fx+Fy)/2 )/RVect.z());
}
//-------------------------------------------------------------------------------------------------------
Vector3f Soft3DCamera_c::GetPos3D()
{
	Vector3f resPoint;
	Vector4f CamFocal;
	CamFocal << 0, 0, 0, 1;
	CamFocal = mProjView.inverse() * CamFocal;
	resPoint << CamFocal.x(), CamFocal.y(), CamFocal.z();
	return resPoint;
}
//-------------------------------------------------------------------------------------------------------
EigenRay Soft3DCamera_c::GetRay(cv::Point ScreenPoint)
{
	EigenRay res;
	Vector4f CamFocal,Point3D,vect;
	CamFocal << 0, 0, 0, 1;
	CamFocal = mProjView.inverse() * CamFocal;
	Point3D << (float)ScreenPoint.x,	(float)ScreenPoint.y,	1.f,	1.f;
	Point3D = mProjView.inverse() * Point3D;
	res.Point << CamFocal.x(), CamFocal.y(), CamFocal.z();
	//this is the bug that wasn't understood
	//corrected with workaround glue - might be due to the definition of the Camera reference in the calib file
	//Z axis looking front or back
	if(isCamReverseRays)
	{
		vect = -CamFocal + Point3D;
	}
	else
	{
		vect = CamFocal - Point3D;
	}
	res.Vector << vect.x(), vect.y(), vect.z();
	//printf("x,y : %d %d\n",ScreenPoint.x,ScreenPoint.y);
	//printf("vect : %f\n",res.Vector.norm());
	res.Vector.normalize();
	return res;
}
//-------------------------------------------------------------------------------------------------------
Vector3f Soft3DCamera_c::GetRayPointDist(cv::Point ScreenPoint, float Dist)
{
	EigenRay ERay = GetRay(ScreenPoint);
	return (ERay.Point + ERay.Vector * Dist);
}
//-------------------------------------------------------------------------------------------------------
//------------------------ Cam Drawing ------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void Soft3DCamera_c::DrawRay(EigenRay Ray,float Length,cv::Scalar color)
{
	Vector4f P1,P2;
	P1 = V3To4(Ray.Point);
	P2 = V3To4(Ray.Point + Length * Ray.Vector);
	DrawLine3D(P1,P2,color);
}
//-------------------------------------------------------------------------------------------------------
void Soft3DCamera_c::DrawRay(cv::Mat &ImgDrawOn,EigenRay Ray,cv::Scalar color)
{
	Vector3f Pt;
	for(int i=0;i<1000;i++)
	{
		Pt = Ray.Point + ((float)i) * Ray.Vector;
		cv::circle(ImgDrawOn,Project(Pt),2,color);
	}
}
//-------------------------------------------------------------------------------------------------------
void Soft3DCamera_c::DrawPoint3D(const Vector4f &Point3D,cv::Scalar Color,bool isPutBlock)
{
	if(!isSurfaceInitialized)InitSurface();
	cv::Point ScreenPos;
	float ScreenDepth,*pZbufPix;
	ProjectZ(Point3D,ScreenPos,ScreenDepth);
	//Nasty Trick to avoid Block out of Image array
	SurfWidth-=3;SurfHeight-=3;
	bool isInsideSurface = true;
	int BlockStep,BlockStepF;
	BlockStep = (int)(ImgRender.step - 9);
	BlockStepF = (int)((ImgZBuffer.step/4) - 3);
	if(ScreenPos.x < 0)isInsideSurface = false;
	else if(ScreenPos.x >= SurfWidth)isInsideSurface = false;
	if(ScreenPos.y < 0)isInsideSurface = false;
	else if(ScreenPos.y >= SurfHeight)isInsideSurface = false;
	if(isInsideSurface)
	{
		if(ScreenDepth<0)//not behind the screen
		{
			pZbufPix = &PIXEL32FC1(ImgZBuffer,ScreenPos.x,ScreenPos.y);
			uchar* pImg = &PIXEL8UC3_1(ImgRender,ScreenPos.x,ScreenPos.y);
			if(ScreenDepth > (*pZbufPix))//then Draw
			{
	#ifndef PIXEL_POINT_DRAW
				if(isPutBlock)
				{
					PutBlock(pImg,BlockStep,Color);
					PutBlockF(pZbufPix,BlockStepF,ScreenDepth);
				}
				else
				{
					(*pZbufPix) = ScreenDepth;//Update Z Buffer
					(*pImg++)	= (uchar)Color[0];
					(*pImg++)	= (uchar)Color[1];
					(*pImg)		= (uchar)Color[2];
				}
	#else
				cv::circle(ImgRender,ScreenPos,2,Color,2);
				cv::circle(ImgZBuffer,ScreenPos,2,cv::Scalar(ScreenDepth),2);//does this work ?
	#endif
			}
		}
	}
	SurfWidth+=3;SurfHeight+=3;
}
//--------------------------------------------------------------------------------------------------------------
bool Soft3DCamera_c::IsPoint3DVisible(Vector4f &Point3D)
{
	bool IsVisible = false;
	if(!isSurfaceInitialized)
	{
		printf("testing visibility on unInitialised surface\n");
		return IsVisible;
	}
	cv::Point ScreenPos;
	float ScreenDepth,*pZbufPix;
	ProjectZ(Point3D,ScreenPos,ScreenDepth);
	bool isInsideSurface = true;
	if(ScreenPos.x < 0)isInsideSurface = false;
	else if(ScreenPos.x >= SurfWidth)isInsideSurface = false;
	if(ScreenPos.y < 0)isInsideSurface = false;
	else if(ScreenPos.y >= SurfHeight)isInsideSurface = false;
	if(isInsideSurface)
	{
		pZbufPix = &PIXEL32FC1(ImgZBuffer,ScreenPos.x,ScreenPos.y);
		if(ScreenDepth < (*pZbufPix))//then Draw
		{
			IsVisible = true;
		}
	}
	return IsVisible;
}
//--------------------------------------------------------------------------------------------------------------
void Soft3DCamera_c::DrawLine3D(const Vector3f &Point1,const Vector3f &Point2,cv::Scalar Color)
{
	DrawLine3D(V3To4(Point1),V3To4(Point2),Color);
}
//--------------------------------------------------------------------------------------------------------------
void Soft3DCamera_c::DrawLine3D(const Vector4f &Point1,const Vector4f &Point2,cv::Scalar Color)
{
	if(!isSurfaceInitialized)InitSurface();
	cv::Point ScPoint1,ScPoint2;
	float ScDepth1,ScDepth2;
	ProjectZ(Point1,ScPoint1,ScDepth1);
	ProjectZ(Point2,ScPoint2,ScDepth2);
	Vector3f V3D = V4To3(Point2 - Point1);
	float V3DLength = V3D.norm();
	float Lw,Lh;
	Lw = (float)(ScPoint2.x - ScPoint1.x);
	Lh = (float)(ScPoint2.y - ScPoint1.y);
	float LineLength = sqrt(Lw*Lw + Lh*Lh);
	int NbPoints = (int)LineLength;//A point every pixels
	float stepLine3D = V3DLength/(NbPoints+1);
	for(int i=0;i<NbPoints;i++)
	{
		float v = ((float)i)/((float)(NbPoints));//between 0 and 1
		Vector3f DrawPoint = V4To3(Point1) + v * V3D;
		DrawPoint3D(V3To4(DrawPoint),Color);
	}
}
//--------------------------------------------------------------------------------------------------------------
void Soft3DCamera_c::DrawReference(Matrix4f &mCurrent,float VSize,float charsizeRef)
{
	Vector4f Center,XAxis,YAxis,ZAxis;
	Center	= mCurrent * Vector4f(0,0,0,1);
	XAxis	= mCurrent * Vector4f(VSize,0,0,1);
	YAxis	= mCurrent * Vector4f(0,VSize,0,1);
	ZAxis	= mCurrent * Vector4f(0,0,VSize,1);
	DrawLine3D(Center,XAxis,mcv::clRed);
	DrawLine3D(Center,YAxis,mcv::clGreen);
	DrawLine3D(Center,ZAxis,mcv::clBlue);
	float CharSize = ProjectSize(XAxis,charsizeRef);
	cv::Point Px = Project(XAxis);
	if(charsizeRef>0)cv::putText(ImgRender,"x",Px,0,CharSize,mcv::clRed,2);
	CharSize = ProjectSize(YAxis,charsizeRef);
	cv::Point Py = Project(YAxis);
	if(charsizeRef>0)cv::putText(ImgRender,"y",Py,0,CharSize,mcv::clGreen,2);
	CharSize = ProjectSize(ZAxis,charsizeRef);
	cv::Point Pz = Project(ZAxis);
	if(charsizeRef>0)cv::putText(ImgRender,"z",Pz,0,CharSize,mcv::clBlue,2);
}
//--------------------------------------------------------------------------------------------------------------
void Soft3DCamera_c::Display(const char*WinName)
{
	cv::imshow(WinName,ImgRender);
}
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//-------------------- Not Good Design Objects should Draw Themselves with the Cam -----------------------------
//--------------------------------------------------------------------------------------------------------------
void Soft3DCamera_c::DrawBox(const EigenBox3D &Box,cv::Mat &Img)
{
	Vector3f	P3DxHyHzH,P3DxLyHzH,P3DxHyLzH,P3DxLyLzH,
				P3DxHyHzL,P3DxLyHzL,P3DxHyLzL,P3DxLyLzL;
	cv::Point	PxHyHzH,PxLyHzH,PxHyLzH,PxLyLzH,
				PxHyHzL,PxLyHzL,PxHyLzL,PxLyLzL;
	cv::Scalar Color(150,90,160);
	//Collect the 8 Points of the 3D Box
	P3DxHyHzH.x() = Box.High.x();		P3DxHyHzH.y() = Box.High.y();		P3DxHyHzH.z() = Box.High.z();	P3DxHyHzH.w() = 1;
	P3DxLyHzH.x() = Box.Low.x();		P3DxLyHzH.y() = Box.High.y();		P3DxLyHzH.z() = Box.High.z();	P3DxLyHzH.w() = 1;
	P3DxHyLzH.x() = Box.High.x();		P3DxHyLzH.y() = Box.Low.y();		P3DxHyLzH.z() = Box.High.z();	P3DxHyLzH.w() = 1;
	P3DxLyLzH.x() = Box.Low.x();		P3DxLyLzH.y() = Box.Low.y();		P3DxLyLzH.z() = Box.High.z();	P3DxLyLzH.w() = 1;

	P3DxHyHzL.x() = Box.High.x();		P3DxHyHzL.y() = Box.High.y();		P3DxHyHzL.z() = Box.Low.z();	P3DxHyHzL.w() = 1;
	P3DxLyHzL.x() = Box.Low.x();		P3DxLyHzL.y() = Box.High.y();		P3DxLyHzL.z() = Box.Low.z();	P3DxLyHzL.w() = 1;
	P3DxHyLzL.x() = Box.High.x();		P3DxHyLzL.y() = Box.Low.y();		P3DxHyLzL.z() = Box.Low.z();	P3DxHyLzL.w() = 1;
	P3DxLyLzL.x() = Box.Low.x();		P3DxLyLzL.y() = Box.Low.y();		P3DxLyLzL.z() = Box.Low.z();	P3DxLyLzL.w() = 1;
	//Project these points
	PxHyHzH = Project(P3DxHyHzH);
	PxLyHzH = Project(P3DxLyHzH);
	PxHyLzH = Project(P3DxHyLzH);
	PxLyLzH = Project(P3DxLyLzH);

	PxHyHzL = Project(P3DxHyHzL);
	PxLyHzL = Project(P3DxLyHzL);
	PxHyLzL = Project(P3DxHyLzL);
	PxLyLzL = Project(P3DxLyLzL);
	//Draw The Linking Lines
	//Higher Box
	cv::line(Img,PxHyHzH,PxLyHzH,Color,2);
	cv::line(Img,PxLyHzH,PxLyLzH,Color,2);
	cv::line(Img,PxLyLzH,PxHyLzH,Color,2);
	cv::line(Img,PxHyLzH,PxHyHzH,Color,2);
	//Lower Box
	cv::line(Img,PxHyHzL,PxLyHzL,Color,2);
	cv::line(Img,PxLyHzL,PxLyLzL,Color,2);
	cv::line(Img,PxLyLzL,PxHyLzL,Color,2);
	cv::line(Img,PxHyLzL,PxHyHzL,Color,2);
	//Sides
	cv::line(Img,PxHyHzH,PxHyHzL,Color,2);
	cv::line(Img,PxLyHzH,PxLyHzL,Color,2);
	cv::line(Img,PxHyLzH,PxHyLzL,Color,2);
	cv::line(Img,PxLyLzH,PxLyLzL,Color,2);

}
//--------------------------------------------------------------------------------------------------------------
void Soft3DCamera_c::DrawBox(const EigenBox3D &Box)
{
	Vector4f	P3DxHyHzH,P3DxLyHzH,P3DxHyLzH,P3DxLyLzH,
				P3DxHyHzL,P3DxLyHzL,P3DxHyLzL,P3DxLyLzL;
	cv::Scalar Color(150,90,160);
	//Collect the 8 Points of the 3D Box
	P3DxHyHzH.x() = Box.High.x();		P3DxHyHzH.y() = Box.High.y();		P3DxHyHzH.z() = Box.High.z();	P3DxHyHzH.w() = 1;
	P3DxLyHzH.x() = Box.Low.x();		P3DxLyHzH.y() = Box.High.y();		P3DxLyHzH.z() = Box.High.z();	P3DxLyHzH.w() = 1;
	P3DxHyLzH.x() = Box.High.x();		P3DxHyLzH.y() = Box.Low.y();		P3DxHyLzH.z() = Box.High.z();	P3DxHyLzH.w() = 1;
	P3DxLyLzH.x() = Box.Low.x();		P3DxLyLzH.y() = Box.Low.y();		P3DxLyLzH.z() = Box.High.z();	P3DxLyLzH.w() = 1;

	P3DxHyHzL.x() = Box.High.x();		P3DxHyHzL.y() = Box.High.y();		P3DxHyHzL.z() = Box.Low.z();	P3DxHyHzL.w() = 1;
	P3DxLyHzL.x() = Box.Low.x();		P3DxLyHzL.y() = Box.High.y();		P3DxLyHzL.z() = Box.Low.z();	P3DxLyHzL.w() = 1;
	P3DxHyLzL.x() = Box.High.x();		P3DxHyLzL.y() = Box.Low.y();		P3DxHyLzL.z() = Box.Low.z();	P3DxHyLzL.w() = 1;
	P3DxLyLzL.x() = Box.Low.x();		P3DxLyLzL.y() = Box.Low.y();		P3DxLyLzL.z() = Box.Low.z();	P3DxLyLzL.w() = 1;

	//Draw The Linking Lines
	DrawLine3D(P3DxHyHzH,P3DxLyHzH,Color);
	DrawLine3D(P3DxLyHzH,P3DxLyLzH,Color);
	DrawLine3D(P3DxLyLzH,P3DxHyLzH,Color);
	DrawLine3D(P3DxHyLzH,P3DxHyHzH,Color);
	//Lower Box
	DrawLine3D(P3DxHyHzL,P3DxLyHzL,Color);
	DrawLine3D(P3DxLyHzL,P3DxLyLzL,Color);
	DrawLine3D(P3DxLyLzL,P3DxHyLzL,Color);
	DrawLine3D(P3DxHyLzL,P3DxHyHzL,Color);
	//Sides
	DrawLine3D(P3DxHyHzH,P3DxHyHzL,Color);
	DrawLine3D(P3DxLyHzH,P3DxLyHzL,Color);
	DrawLine3D(P3DxHyLzH,P3DxHyLzL,Color);
	DrawLine3D(P3DxLyLzH,P3DxLyLzL,Color);
}
//--------------------------------------------------------------------------------------------------------------
void Soft3DCamera_c::Render(Soft3DCamera_c& RenderCam)
{
	EigenRay R1 = GetRay(cv::Point(0,0));
	EigenRay R2 = GetRay(cv::Point(0,SurfHeight));
	EigenRay R3 = GetRay(cv::Point(SurfWidth,SurfHeight));
	EigenRay R4 = GetRay(cv::Point(SurfWidth,0));
	float CamSize = 100.0f;
	Vector3f P1 = R1.GetPoint(CamSize);
	Vector3f P2 = R2.GetPoint(CamSize);
	Vector3f P3 = R3.GetPoint(CamSize);
	Vector3f P4 = R4.GetPoint(CamSize);
	Vector3f CamPos = GetPos3D();
	//printf("Cam(%d) Pos(%1.1f,%1.1f,%1.1f)\n",Id,CamPos.x(),CamPos.y(),CamPos.z());
	RenderCam.DrawLine3D(CamPos,P1,Color);
	RenderCam.DrawLine3D(CamPos,P2,Color);
	RenderCam.DrawLine3D(CamPos,P3,Color);
	RenderCam.DrawLine3D(CamPos,P4,Color);
	RenderCam.DrawLine3D(P1,P2,Color);
	RenderCam.DrawLine3D(P2,P3,Color);
	RenderCam.DrawLine3D(P3,P4,Color);
	RenderCam.DrawLine3D(P4,P1,Color);
}
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
