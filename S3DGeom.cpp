#include "S3DGeom.h"


#include <iostream>
#include <fstream>
#include "mcvGeneral.h"



#define EPSDIST	0.00001f

//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//							YAML to Streaming Utilities
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
YAML::Emitter& operator << (YAML::Emitter& out, const Vector3f& Vect3)
{
	out << YAML::Flow;
	out << YAML::BeginMap;
	out << YAML::Key << "x";
	out << YAML::Value << Vect3.x();
	out << YAML::Key << "y";
	out << YAML::Value << Vect3.y();
	out << YAML::Key << "z";
	out << YAML::Value << Vect3.z();
	out << YAML::EndMap;
	return out;
}
//--------------------------------------------------------------------------------------------------------------
std::ostream& operator<<(std::ostream &out, const Vector3f& Vect3)
{
	YAML::Emitter Emit;
	Emit << Vect3;
	out << Emit.c_str() << std::endl;
	return out;
}
//-------------------------------------------------------------------------------------------------
YAML::Emitter& operator<<(YAML::Emitter& out, const std::vector<Vector3f>& Vector)
{
	out << YAML::Flow;
	out << YAML::BeginSeq;
	for(int i=0;i<Vector.size();i++)
	{
		out << Vector[i];
	}
	out << YAML::EndSeq;
	return out;
}
//-------------------------------------------------------------------------------------------------
void operator >> (const YAML::Node& node, Vector3f& Vect3)
{
	node["x"] >> Vect3.x();
	node["y"] >> Vect3.y();
	node["z"] >> Vect3.z();
}
//--------------------------------------------------------------------------------------------------------------
YAML::Emitter& operator << (YAML::Emitter& out, const Vector4f& Vect4)
{
	out << YAML::Flow;
	out << YAML::BeginMap;
	out << YAML::Key << "x";
	out << YAML::Value << Vect4.x();
	out << YAML::Key << "y";
	out << YAML::Value << Vect4.y();
	out << YAML::Key << "z";
	out << YAML::Value << Vect4.z();
	out << YAML::Key << "w";
	out << YAML::Value << Vect4.w();
	out << YAML::EndMap;
	return out;
}
//--------------------------------------------------------------------------------------------------------------
std::ostream& operator<<(std::ostream &out, const Vector4f& Vect4)
{
	YAML::Emitter Emit;
	Emit << Vect4;
	out << Emit.c_str() << std::endl;
	return out;
}
//-------------------------------------------------------------------------------------------------
YAML::Emitter& operator<<(YAML::Emitter& out, const std::vector<Vector4f>& Vector)
{
	out << YAML::Flow;
	out << YAML::BeginSeq;
	for(int i=0;i<Vector.size();i++)
	{
		out << Vector[i];
	}
	out << YAML::EndSeq;
	return out;
}
//-------------------------------------------------------------------------------------------------
void operator >> (const YAML::Node& node, Vector4f& Vect4)
{
	node["x"] >> Vect4.x();
	node["y"] >> Vect4.y();
	node["z"] >> Vect4.z();
	node["w"] >> Vect4.w();
}
//--------------------------------------------------------------------------------------------------------------
YAML::Emitter& operator << (YAML::Emitter& out, EigenBox3D Box3D)
{
	assert(Box3D.HasPointAdded);//no HasPointAdded has a specific usage
	//out << YAML::Flow;
	out << YAML::BeginMap;
	out << YAML::Key << "High";
	out << YAML::Value << Box3D.High;
	out << YAML::Key << "Low";
	out << YAML::Value << Box3D.Low;
	out << YAML::Key << "Size";			
	out << YAML::Value << Box3D.size();
	out << YAML::Key << "Center";
	out << YAML::Value << Box3D.GetCenter();
	out << YAML::EndMap;
	return out;
}
//--------------------------------------------------------------------------------------------------------------
void operator >> (const YAML::Node& node, EigenBox3D& Box3D)
{
	node["High"] >> Box3D.High;
	node["Low"] >> Box3D.Low;
	Box3D.HasPointAdded = true;
}
//-------------------------------------------------------------------------------------------------------
//cout << EigenBox3D
std::ostream& operator<<(std::ostream &out, const EigenBox3D& Box3D)
{
	YAML::Emitter Yout;
	Yout << Box3D;
	out << Yout.c_str() << std::endl;
	return out;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//							fstream Utilities
//-------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
Quaternionf g3d::readQuaternionf(std::ifstream &File)
{
	float Val;
	std::string line;
	Quaternionf Quat;
	File >> Val;	getline(File,line);		Quat.x() = Val;
	File >> Val;	getline(File,line);		Quat.y() = Val;
	File >> Val;	getline(File,line);		Quat.z() = Val;
	File >> Val;	getline(File,line);		Quat.w() = Val;
	return Quat;
}
//--------------------------------------------------------------------------------------------------------------
Vector3f g3d::readVector3f(std::ifstream &File)
{
	float Val;
	std::string line;
	Vector3f Vect;
	File >> Val;	getline(File,line);		Vect.x() = Val;
	File >> Val;	getline(File,line);		Vect.y() = Val;
	File >> Val;	getline(File,line);		Vect.z() = Val;
	return Vect;
}
//--------------------------------------------------------------------------------------------------------------
Vector2f g3d::readVector2f(std::ifstream &File)
{
	float Val;
	std::string line;
	Vector2f Vect;
	File >> Val;	getline(File,line);		Vect.x() = Val;
	File >> Val;	getline(File,line);		Vect.y() = Val;
	return Vect;
}
//--------------------------------------------------------------------------------------------------------------
void g3d::writeQuaternionf(std::ofstream &File,Quaternionf &Quat)
{
	File << Quat.x() << std::endl;
	File << Quat.y() << std::endl;
	File << Quat.z() << std::endl;
	File << Quat.w() << std::endl;
}
//--------------------------------------------------------------------------------------------------------------
void g3d::writeVector3f(std::ofstream &File,Vector3f Vect)
{
	File << Vect.x() << std::endl;
	File << Vect.y() << std::endl;
	File << Vect.z() << std::endl;
}
//--------------------------------------------------------------------------------------------------------------
void g3d::writeVector2f(std::ofstream &File,Vector2f Vect)
{
	File << Vect.x() << std::endl;
	File << Vect.y() << std::endl;
}
//--------------------------------------------------------------------------------------------------------------
void g3d::printfVector2f(const Vector2f &vPos,const char *VarName,bool isReturn,int precision)
{
	char pFormat[32];
	char pFormatAll[128];
	sprintf(pFormat,"%%1.%df",precision);
	sprintf(pFormatAll,"%s(%s,%s)",VarName,pFormat,pFormat);
	printf(pFormatAll,VarName,vPos.x(),vPos.y());
	if(isReturn)printf("\n");
}
//-------------------------------------------------------------------------------------------------------
void g3d::printfVector3f(const Vector3f &vVect,const char *VarName,bool isReturn,int precision)
{
	char pFormat[32];
	char pFormatAll[128];
	sprintf(pFormat,"%%1.%df",precision);
	sprintf(pFormatAll,"%s(%s,%s,%s)",VarName,pFormat,pFormat,pFormat);
	printf(pFormatAll,vVect.x(),vVect.y(),vVect.z());
	if(isReturn)printf("\n");
}
//-------------------------------------------------------------------------------------------------------
void g3d::printfMatrix3f(const Matrix3f &vMat,const char *VarName,int precision)
{
	char pFormat[32];
	char pFormatAll[512];
	sprintf(pFormat,"%%1.%df",precision);
	sprintf(pFormatAll,"%s:\n%s %s %s\n%s %s %s\n%s %s %s\n",VarName,pFormat,pFormat,pFormat
																	,pFormat,pFormat,pFormat
																	,pFormat,pFormat,pFormat);
#ifdef TRANSPOSEPRINT
	printf(pFormatAll,	vMat(0,0),vMat(1,0),vMat(2,0),vMat(3,0)
						,vMat(0,1),vMat(1,1),vMat(2,1),vMat(3,1)
						,vMat(0,2),vMat(1,2),vMat(2,2),vMat(3,2)
						,vMat(0,3),vMat(1,3),vMat(2,3),vMat(3,3));
#else
	printf(pFormatAll,	vMat(0,0),vMat(0,1),vMat(0,2)
						,vMat(1,0),vMat(1,1),vMat(1,2)
						,vMat(2,0),vMat(2,1),vMat(2,2)
						);
#endif
}
//-------------------------------------------------------------------------------------------------------
void g3d::printfMatrix4f(const Matrix4f &vMat,const char *VarName,int precision)
{
	char pFormat[32];
	char pFormatAll[512];
	sprintf(pFormat,"%%1.%df",precision);
	sprintf(pFormatAll,"%s:\n%s %s %s %s\n%s %s %s %s\n%s %s %s %s\n%s %s %s %s\n",VarName,pFormat,pFormat,pFormat,pFormat
																							,pFormat,pFormat,pFormat,pFormat
																							,pFormat,pFormat,pFormat,pFormat
																							,pFormat,pFormat,pFormat,pFormat);
#ifdef TRANSPOSEPRINT
	printf(pFormatAll,	vMat(0,0),vMat(1,0),vMat(2,0),vMat(3,0)
						,vMat(0,1),vMat(1,1),vMat(2,1),vMat(3,1)
						,vMat(0,2),vMat(1,2),vMat(2,2),vMat(3,2)
						,vMat(0,3),vMat(1,3),vMat(2,3),vMat(3,3));
#else
	printf(pFormatAll,	vMat(0,0),vMat(0,1),vMat(0,2),vMat(0,3)
						,vMat(1,0),vMat(1,1),vMat(1,2),vMat(1,3)
						,vMat(2,0),vMat(2,1),vMat(2,2),vMat(2,3)
						,vMat(3,0),vMat(3,1),vMat(3,2),vMat(3,3));
#endif
}
//-------------------------------------------------------------------------------------------------------
void g3d::printfQuaternionf(const Quaternionf &vQuat,bool isReturn,const char *VarName,int precision)
{
	char pFormat[32];
	char pFormatAll[128];
	sprintf(pFormat,"%%1.%df",precision);
	sprintf(pFormatAll,"%s(%s,%s,%s,%s)",VarName,pFormat,pFormat,pFormat,pFormat);
	printf(pFormatAll,VarName,vQuat.x(),vQuat.y(),vQuat.z(),vQuat.w());
	if(isReturn)printf("\n");
}
//-------------------------------------------------------------------------------------------------------
float g3d::GetNearestIndex(const std::vector<Eigen::Vector3f> Vects,const Vector3f &V,int &Index)
{
	std::vector<float> Dists(Vects.size());
	for(size_t i=0;i<Vects.size();i++)
	{
		Dists[i] = (V-Vects[i]).norm();
	}
	Index = mcv::TabMin_Index(Dists);
	return Dists[Index];
}
//-------------------------------------------------------------------------------------------------------
int g3d::GetNearestDist(const std::vector<Eigen::Vector3f> Vects,const Vector3f &V,float &dist)
{
	std::vector<float> Dists(Vects.size());
	for(size_t i=0;i<Vects.size();i++)
	{
		Dists[i] = (V-Vects[i]).norm();
	}
	int Index = mcv::TabMin_Index(Dists);
	if(Index!=-1)
		dist = Dists[Index];
	return Index;
}
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//							Quaternions Spherical interpolation functions
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
float g3d::Point2DProject(Vector2f Vect1,Vector2f Vect2,Vector2f RefVector)
{
	float Res;
	Vector2f VDiff1,VDiff2;
	VDiff1 = Vect2 - Vect1;
	VDiff2 = RefVector - Vect1;
	float L1 = VDiff1.norm();
	if(L1<EPSDIST)//V1 == V2 ?? Bullshit
	{
		Res = 0;
	}
	else
	{
		Res = VDiff1.dot(VDiff2)/(L1*L1);//To get Unified [0,1] value
	}
	return Res;
}
//-------------------------------------------------------------------------------------------------------
Vector2f g3d::Point2DInterpolate(Vector2f P1,Vector2f P2,float P1v0)
{
	return (P2 - P1) * P1v0 + P1;
}
//-------------------------------------------------------------------------------------------------------
bool g3d::Point2DHalfPlane(Vector2f ThePoint,Vector2f TheProjection,Vector2f PSameHalf)
{
	Vector2f Vect1,Vect2;
	Vect1 = ThePoint - TheProjection;
	Vect2 = PSameHalf - TheProjection;
	float SameDir = Vect1.dot(Vect2);
	return (SameDir>=0);
}
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//							Matrix Utilities Functions
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
Vector3f V4To3(const Vector4f &Vect)
{
	Vector3f	RVect;
	RVect <<	Vect.x(), Vect.y(), Vect.z();
	return RVect;
}
//-------------------------------------------------------------------------------------------------------
Vector4f V3To4(const Vector3f &Vect)
{
	Vector4f	RVect;
	RVect <<	Vect.x(), Vect.y(), Vect.z(), 1;
	return RVect;
}
//-------------------------------------------------------------------------------------------------------
Eigen::Matrix3f g3d::Matrix4fTo3f(const Eigen::Matrix4f &Mat4)
{
	Eigen::Matrix3f MatOut;
	MatOut	<<		Mat4(0,0),	Mat4(0,1),	Mat4(0,2),
					Mat4(1,0),	Mat4(1,1),	Mat4(1,2),
					Mat4(2,0),	Mat4(2,1),	Mat4(2,2);
	return MatOut;
}
//-------------------------------------------------------------------------------------------------------
Matrix4f g3d::Matrix3fTo4f(const Eigen::Matrix3f &Mat3)
{
	Matrix4f MatOut;
	MatOut	<<		Mat3(0,0),	Mat3(0,1),	Mat3(0,2),	0,
					Mat3(1,0),	Mat3(1,1),	Mat3(1,2),	0,
					Mat3(2,0),	Mat3(2,1),	Mat3(2,2),	0,
					0,			0,			0,			1;
	return MatOut;
}
//-------------------------------------------------------------------------------------------------------
Matrix4f g3d::MatrixIdentity()
{
	Matrix4f matRes;
	matRes <<	1,	0,	0,	0,
				0,	1,	0,	0,
				0,	0,	1,	0,
				0,	0,	0,	1;
	return matRes;
}
//-------------------------------------------------------------------------------------------------------
Matrix4f g3d::MatrixRotationX(float a)
{
	Matrix4f matRes;
	matRes <<	1,	0,		0,		0,
				0,	cos(a),	-sin(a),0,
				0,	sin(a),	cos(a),	0,
				0,	0,		0,		1;
	return matRes;
}
//-------------------------------------------------------------------------------------------------------
Matrix4f g3d::MatrixRotationY(float a)
{
	Matrix4f matRes;
	matRes <<	cos(a),	0,	sin(a),	0,
				0,		1,	0,		0,
				-sin(a),0,	cos(a),	0,
				0,		0,	0,		1;
	return matRes;
}
//-------------------------------------------------------------------------------------------------------
Matrix4f g3d::MatrixRotationZ(float a)
{
	Matrix4f matRes;
	matRes <<	cos(a),	-sin(a),0,	0,
				sin(a),	cos(a),	0,	0,
				0,		0,		1,	0,
				0,		0,		0,	1;
	return matRes;
}
//-------------------------------------------------------------------------------------------------------
Vector3f g3d::MatrixGetXVector(const Matrix4f &vMat)
{
	return Vector3f(vMat(0,0),
					vMat(1,0),
					vMat(2,0)
					);
}
//-------------------------------------------------------------------------------------------------------
Vector3f g3d::MatrixGetYVector(const Matrix4f &vMat)
{
	return Vector3f(vMat(0,1),
					vMat(1,1),
					vMat(2,1)
					);
}
//-------------------------------------------------------------------------------------------------------
Vector3f g3d::MatrixGetZVector(const Matrix4f &vMat)
{
	return Vector3f(vMat(0,2),
					vMat(1,2),
					vMat(2,2)
					);
}
//-------------------------------------------------------------------------------------------------------
Vector3f g3d::MatrixGetXVector(const Matrix3f &vMat)
{
	return Vector3f(vMat(0,0),
					vMat(1,0),
					vMat(2,0)
					);
}
//-------------------------------------------------------------------------------------------------------
Vector3f g3d::MatrixGetYVector(const Matrix3f &vMat)
{
	return Vector3f(vMat(0,1),
					vMat(1,1),
					vMat(2,1)
					);
}
//-------------------------------------------------------------------------------------------------------
Vector3f g3d::MatrixGetZVector(const Matrix3f &vMat)
{
	return Vector3f(vMat(0,2),
					vMat(1,2),
					vMat(2,2)
					);
}
//-------------------------------------------------------------------------------------------------------
void g3d::matrixSetTrVector(Matrix4f& vMat,Vector4f& TrVect)
{
	vMat(0,3) = TrVect.x();
	vMat(1,3) = TrVect.y();
	vMat(2,3) = TrVect.z();
}
//-------------------------------------------------------------------------------------------------------
Vector4f g3d::TranslationVector(const Matrix4f &TranslationMatrix)
{
	//return TranslationMatrix.col(3);
	return Vector4f(TranslationMatrix(0,3),
					TranslationMatrix(1,3),
					TranslationMatrix(2,3),
					1);
}
//-------------------------------------------------------------------------------------------------------
Matrix3f g3d::MatrixAxis(Vector3f &XAxis,Vector3f &YAxis,Vector3f &ZAxis)
{
	Matrix3f matRes;
	matRes <<		XAxis.x(),	YAxis.x(),	ZAxis.x(),
					XAxis.y(),	YAxis.y(),	ZAxis.y(),
					XAxis.z(),	YAxis.z(),	ZAxis.z();
	return matRes;
}
//-------------------------------------------------------------------------------------------------------
Matrix4f g3d::MatrixAxisTr(const Vector3f &XAxis,const Vector3f &YAxis,const Vector3f &ZAxis,const Vector3f &TrVect)
{
	Matrix4f matRes;
	matRes <<		XAxis.x(),	YAxis.x(),	ZAxis.x(),	TrVect.x(),
					XAxis.y(),	YAxis.y(),	ZAxis.y(),	TrVect.y(),
					XAxis.z(),	YAxis.z(),	ZAxis.z(),	TrVect.z(),
					0,			0,			0,			1;
	return matRes;
}
//-------------------------------------------------------------------------------------------------------
Matrix4f g3d::MatrixTranslation(float Tx,float Ty,float Tz)
{
	Matrix4f matRes;
	matRes <<	1,	0,	0,	Tx,
				0,	1,	0,	Ty,
				0,	0,	1,	Tz,
				0,	0,	0,	1;
	return matRes;
}
//-------------------------------------------------------------------------------------------------------
void g3d::MatrixTranslation(Vector4f &TVect,Matrix4f &ResMatrix)
{
	ResMatrix <<	1,	0,	0,	TVect.x(),
					0,	1,	0,	TVect.y(),
					0,	0,	1,	TVect.z(),
					0,	0,	0,	1;
}
//-------------------------------------------------------------------------------------------------------
Matrix4f g3d::MatrixTranslation(const Vector4f &TVect)
{
	Matrix4f matRes;
	matRes <<	1,	0,	0,	TVect.x(),
				0,	1,	0,	TVect.y(),
				0,	0,	1,	TVect.z(),
				0,	0,	0,	1;
	return matRes;
}
//-------------------------------------------------------------------------------------------------------
Matrix4f g3d::S3DMatrixView(Vector3f &Pos,Vector3f &LookAt)
{
	Vector3f Tr = Pos - LookAt;
	Vector3f ZAxis = Tr;
	ZAxis.normalize();
	Vector3f YAxis;
	YAxis << 0,1,0;
	Vector3f XAxis = YAxis.cross(ZAxis);
	XAxis.normalize();
	YAxis = ZAxis.cross(XAxis);
	Matrix4f CamMatrix,ResMatrix;
	CamMatrix <<	XAxis.x(),	YAxis.x(),  ZAxis.x(), Pos.x(),
					XAxis.y(),	YAxis.y(),  ZAxis.y(), Pos.y(),
					XAxis.z(),	YAxis.z(),  ZAxis.z(), Pos.z(),
					0,			0,			0,			1;
	g3d::GetReverseTransform(ResMatrix,CamMatrix);
	ResMatrix(0,0) *=-1;
	ResMatrix(0,1) *=-1;
	ResMatrix(0,2) *=-1;
	return ResMatrix;
	//return S3DMatrixTranslation(0,0,400);
}
//-------------------------------------------------------------------------------------------------------
Matrix4f g3d::S3DMatrixView(CvMat * rot_vect,CvMat * tr_vect)
{
    Matrix4f mtView;
	CvMat *RotMat = cvCreateMat( 3, 3, CV_32FC1 );
	cvRodrigues2(rot_vect,RotMat);
	//assert(RotMat->type == CV_32FC1);
	mtView	<<	RotMat->data.fl[0],	RotMat->data.fl[1],	 RotMat->data.fl[2],	tr_vect->data.fl[0],
				RotMat->data.fl[3],	RotMat->data.fl[4],	 RotMat->data.fl[5],	tr_vect->data.fl[1],
				RotMat->data.fl[6],	RotMat->data.fl[7],	 RotMat->data.fl[8],	tr_vect->data.fl[2],
				0,					0,					0,						1;
	return mtView;
}
//-------------------------------------------------------------------------------------------------------
Matrix4f g3d::S3DMatrixViewRotMat(CvMat * RotMat,CvMat * tr_vect)
{
    Matrix4f mtView;
	//assert(RotMat->type == CV_32FC1);
	//assert(tr_vect->type == CV_32FC1);
	mtView	<<	RotMat->data.fl[0],	RotMat->data.fl[1],	 RotMat->data.fl[2],	tr_vect->data.fl[0],
				RotMat->data.fl[3],	RotMat->data.fl[4],	 RotMat->data.fl[5],	tr_vect->data.fl[1],
				RotMat->data.fl[6],	RotMat->data.fl[7],	 RotMat->data.fl[8],	tr_vect->data.fl[2],
				0,					0,					0,						1;
	return mtView;
}
//-------------------------------------------------------------------------------------------------------
void g3d::S3DmView2RotMat(CvMat * RotMat,CvMat * tr_vect,Matrix4f &mView)
{
	//assert(RotMat->type == CV_32FC1);
	RotMat->data.fl[0] = mView(0,0);	RotMat->data.fl[1] = mView(0,1);	 RotMat->data.fl[2] = mView(0,2);	tr_vect->data.fl[0] = mView(0,3);
	RotMat->data.fl[3] = mView(1,0);	RotMat->data.fl[4] = mView(1,1);	 RotMat->data.fl[5] = mView(1,2);	tr_vect->data.fl[1] = mView(1,3);
	RotMat->data.fl[6] = mView(2,0);	RotMat->data.fl[7] = mView(2,1);	 RotMat->data.fl[8] = mView(2,2);	tr_vect->data.fl[2] = mView(2,3);
}
//-------------------------------------------------------------------------------------------------------
//This function is not beeing used, just a sample of a DX like Proj mat
Matrix4f S3DMatrixProjectionDX(double Fovy, double ratio, float ZNear,float ZFar)
{
	double cy = 240.0;
	double h = Fovy / cy;
	double Q = ZFar/(ZFar - ZNear);
	double w = h * ratio;
	float FovY = (float)(2*atan(1/w));
	Matrix4f matRes;
	matRes <<	(float)w,	0,			0,					0,
				0,			(float)h,	0,					0,
				0,			0,			(float)Q,			1,
				0,			0,			(float)-Q*ZNear,	0;
	return matRes;
}
//--------------------------------------------------------------------------------------------------------------
Matrix4f g3d::S3DMatrixProjection(float Fx,float Fy,float Cx,float Cy)
{
	Matrix4f matRes;
	matRes <<	Fx,	0,	Cx,	0,
				0,	Fy,	Cy,	0,
				0,	0,	1,	0,
				0,	0,	0,	1;
	return matRes;
}
//--------------------------------------------------------------------------------------------------------------
void g3d::GetReverseTransform(Matrix4f &MatDest,const Matrix4f &MatSrc)
//To be tested for speed we hope it's quicker from just MatSrc.inverse();
{
	Matrix4f Rt;
	Rt = MatSrc.transpose();
	MatDest(0,0)  = Rt(0,0);	MatDest(0,1)  = Rt(0,1);	MatDest(0,2)  = Rt(0,2);	MatDest(0,3)  = 0;
	MatDest(1,0)  = Rt(1,0);	MatDest(1,1)  = Rt(1,1);	MatDest(1,2)  = Rt(1,2);	MatDest(1,3)  = 0;
	MatDest(2,0)  = Rt(2,0);	MatDest(2,1)  = Rt(2,1);	MatDest(2,2)  = Rt(2,2);	MatDest(2,3)  = 0;
	MatDest(3,0)  = 0;			MatDest(3,1)  = 0;			MatDest(3,2)  = 0;			MatDest(3,3)  = 1;

	Vector4f OldT;
	Vector4f NewT;
	OldT.x() = -MatSrc(0,3);
	OldT.y() = -MatSrc(1,3);
	OldT.z() = -MatSrc(2,3);
	OldT.w() = 1;
	NewT = MatDest * OldT;
	MatDest(0,3)  = NewT.x();
	MatDest(1,3)  = NewT.y();
	MatDest(2,3)  = NewT.z();
}
//-------------------------------------------------------------------------------------------------------
void g3d::S3DMatrixPrint(const Matrix4f& PrintThis)
{
	printf("S3DMatrix:\n");
	std::cout << PrintThis << std::endl << std::endl;
}
//--------------------------------------------------------------------------------------------------------------
//					Eigen Ray Class
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
EigenRay::EigenRay()
{
	Weight = 1;
}
//--------------------------------------------------------------------------------------------------------------
Vector3f EigenRay::GetPoint(float Distance)
{
	return Point + Vector * Distance;
}
//--------------------------------------------------------------------------------------------------------------
void EigenRay::printf(const char*VName)
{
	std::printf("%s:\n");
	g3d::printfVector3f(Point,"Point");
	g3d::printfVector3f(Vector,"Vector");
	std::printf("Weight:%f\n",Weight);
}
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//							Boxing Class
//--------------------------------------------------------------------------------------------------------------
EigenBox2D ContourToBox2D(VPoints Contour)
{
	EigenBox2D BoxRes;
	BoxRes.High = Vector2f((float)Contour[0].x,(float)Contour[0].y);
	BoxRes.Low  = Vector2f((float)Contour[0].x,(float)Contour[0].y);
	for(UINT i=1;i<Contour.size();i++)
	{
		float x = (float)Contour[i].x;
		float y = (float)Contour[i].y;
		if(x > BoxRes.High.x()) BoxRes.High.x() = x;
		if(y > BoxRes.High.y()) BoxRes.High.y() = y;
		if(x < BoxRes.Low.x()) BoxRes.Low.x() = x;
		if(y < BoxRes.Low.y()) BoxRes.Low.y() = y;
	}
	return BoxRes;
}
//--------------------------------------------------------------------------------------------------------------
EigenBlob2DExtremas g3d::ContourToExtremas(VPoints Contour)
{
	EigenBlob2DExtremas BlobExRes;
	if(Contour.size()==0)
	{
		BlobExRes.isValid = false;//Else it's true by default
		return BlobExRes;
	}
	BlobExRes.Top    = Vector2f((float)Contour[0].x,(float)Contour[0].y);
	BlobExRes.Bottom = Vector2f((float)Contour[0].x,(float)Contour[0].y);
	BlobExRes.Left   = Vector2f((float)Contour[0].x,(float)Contour[0].y);
	BlobExRes.Right  = Vector2f((float)Contour[0].x,(float)Contour[0].y);
	for(UINT i=1;i<Contour.size();i++)
	{
		float x = (float) Contour[i].x;
		float y = (float) Contour[i].y;
		if(x > BlobExRes.Right.x()) BlobExRes.Right << (float)Contour[i].x, (float)Contour[i].y;
		if(y > BlobExRes.Top.y()) BlobExRes.Top << (float)Contour[i].x, (float)Contour[i].y;
		if(x < BlobExRes.Left.x()) BlobExRes.Left << (float)Contour[i].x, (float)Contour[i].y;
		if(y < BlobExRes.Bottom.y()) BlobExRes.Bottom << (float)Contour[i].x, (float)Contour[i].y;
	}
	return BlobExRes;
}

//--------------------------------------------------------------------------------------------------------------
EigenBlob2DExtremas::EigenBlob2DExtremas()
{
	isValid = true;
}
//--------------------------------------------------------------------------------------------------------------
void EigenBlob2DExtremas::Draw(cv::Mat &Img)
{
	cv::Point P1,P2,P3,P4;
	P1 = cv::Point((int)Top.x(),(int)Top.y());
	P2 = cv::Point((int)Right.x(),(int)Right.y());
	P3 = cv::Point((int)Bottom.x(),(int)Bottom.y());
	P4 = cv::Point((int)Left.x(),(int)Left.y());
	cv::line(Img,P1,P2,cv::Scalar(20,80,190),2);
	cv::line(Img,P2,P3,cv::Scalar(20,80,190),2);
	cv::line(Img,P3,P4,cv::Scalar(20,80,190),2);
	cv::line(Img,P4,P1,cv::Scalar(20,80,190),2);
}
//--------------------------------------------------------------------------------------------------------------
EigenBox3D::EigenBox3D()
{
	HasPointAdded = false;
}
//--------------------------------------------------------------------------------------------------------------
void EigenBox3D::Reset()
{
	HasPointAdded = false;
}
//--------------------------------------------------------------------------------------------------------------
void EigenBox3D::Add(Vector3f P3D)
{
	if(HasPointAdded)
	{
		if(P3D.x() > High.x())High.x() = P3D.x();
		if(P3D.x() < Low.x()) Low.x() = P3D.x();
		if(P3D.y() > High.y())High.y() = P3D.y();
		if(P3D.y() < Low.y()) Low.y() = P3D.y();
		if(P3D.z() > High.z())High.z() = P3D.z();
		if(P3D.z() < Low.z()) Low.z() = P3D.z();
	}
	else
	{
		High = P3D;
		Low = P3D;
		HasPointAdded = true;
	}
}
//--------------------------------------------------------------------------------------------------------------
void EigenBox3D::SetCube(Vector3f Center,float BoxSide)
{
	Vector3f VHigh(BoxSide/2,BoxSide/2,BoxSide/2);
	Vector3f VLow(-BoxSide/2,-BoxSide/2,-BoxSide/2);
	Add(Center+VHigh);
	Add(Center+VLow);
}
//--------------------------------------------------------------------------------------------------------------
float EigenBox3D::MaxSide()
{
	Vector3f Diag = High - Low;
	float rVal;
	if(Diag.x()>Diag.y())
	{
		rVal = Diag.x();
	}
	else
	{
		rVal = Diag.y();
	}
	if(rVal<Diag.z())
	{
		rVal = Diag.z();
	}
	return rVal;
}
//--------------------------------------------------------------------------------------------------------------
Vector3f EigenBox3D::size()
{
	Vector3f Res = High - Low;
	return Res;
}
//--------------------------------------------------------------------------------------------------------------
bool EigenBox3D::isInside(Vector3f &Pos)
{
	bool isIn = true;
	if(Pos.x() < Low.x()) isIn = false;
	else if(Pos.x() > High.x()) isIn = false;
	if(Pos.y() < Low.y()) isIn = false;
	else if(Pos.y() > High.y()) isIn = false;
	if(Pos.z() < Low.z()) isIn = false;
	else if(Pos.z() > High.z()) isIn = false;
	return isIn;
}
//--------------------------------------------------------------------------------------------------------------
Vector4f EigenBox3D::GetBaseCenter()
{
	Vector3f Center;
	Vector4f Base;
	Center = (Low + High) / 2;
	Base.x() = Center.x();
	Base.y() = Low.y();
	Base.z() = Center.z();
	return Base;
}
//--------------------------------------------------------------------------------------------------------------
Vector3f EigenBox3D::GetCenter()
{
	return ((Low + High) / 2);
}
//--------------------------------------------------------------------------------------------------------------
void EigenBox3D::GetCenter(Vector4f& Vect)
{
	Vector3f Center;
	Center = (Low + High) / 2;
	Vect << Center.x(), Center.y(), Center.z(), 1;
}
//--------------------------------------------------------------------------------------------------------------
void EigenBox3D::printf(const char*BName)
{
	std::printf("%s:\n",BName);
	g3d::printfVector3f(High,"VHigh");
	g3d::printfVector3f(Low,"VLow");
	Vector3f VC = (High + Low)/2;
	g3d::printfVector3f(VC,"Center");
	float LDiag = (High-Low).norm();
	std::printf("W: %1.2f  H: %1.2f  D: %1.2f  Diag: %1.2f\n",(High.x() - Low.x()),(High.y() - Low.y()),(High.z() - Low.z()), LDiag);
}
//-------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//										geom namespace
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//see http://en.wikipedia.org/wiki/Line-plane_intersection
Vector3f g3d::PlaneRayIntersect(const EigenPlane &Plane,const EigenRay &Ray)
{
	Vector3f Pa,Pb,P0,P1,P2,Pa0,Ptuv;
	Pa = Ray.Point;
	Pb = Ray.Point + Ray.Vector;
	P0 = Plane.Point;
	P1 = Plane.Point + Plane.Vect1;
	P2 = Plane.Point + Plane.Vect2;
	Eigen::Matrix3f MOp;
	MOp <<	Pa.x() - Pb.x(),	P1.x() - P0.x(),	P2.x() - P0.x(),
			Pa.y() - Pb.y(),	P1.y() - P0.y(),	P2.y() - P0.y(),
			Pa.z() - Pb.z(),	P1.z() - P0.z(),	P2.z() - P0.z();
	Pa0 <<	Pa.x() - P0.x(),
			Pa.y() - P0.y(),
			Pa.z() - P0.z();
	Ptuv = MOp.inverse() * Pa0;
	float t = Ptuv.x();
	return (Pa + (Pb-Pa)*t);
}
//--------------------------------------------------------------------------------------------------------------
Vector3f g3d::Rays2Weighted(const EigenRay &Ray1,const EigenRay &Ray2)
{
	EigenPlane Plane1,Plane2;
	Plane1.Point = Ray1.Point;
	Plane1.Vect1 = Ray1.Vector;
	Plane1.Vect2 = Ray1.Vector.cross(Ray2.Vector);//This plane is perpendicular to the plane parallel to both rays
	Vector3f ResP2 = g3d::PlaneRayIntersect(Plane1,Ray2);

	Plane2.Point = Ray2.Point;
	Plane2.Vect1 = Ray2.Vector;
	Plane2.Vect2 = Ray2.Vector.cross(Ray1.Vector);//This plane is perpendicular to the plane parallel to both rays
	Vector3f ResP1 = g3d::PlaneRayIntersect(Plane2,Ray1);
	return ((ResP1 * Ray1.Weight + ResP2 * Ray2.Weight)/(Ray1.Weight + Ray2.Weight));
}
//--------------------------------------------------------------------------------------------------------------
Vector3f g3d::RayPointWeighted(const EigenRay &ERay,const Vector3f &EPoint,float PointWeight)
{
	EigenPlane EProjectPlane;
	EigenRay RayProject;
	EProjectPlane.Point = ERay.Point;
	EProjectPlane.Vect1 = ERay.Vector;
	Vector3f RayToPoint = EPoint - ERay.Point;
	EProjectPlane.Vect2 = ERay.Vector.cross(RayToPoint);//This plane is perpendicular to the plane parallel to both rays
	RayProject.Point = EPoint;
	RayProject.Vector = EProjectPlane.Vect1.cross(EProjectPlane.Vect2);//perpendicular to the plane
	Vector3f ProjectedPoint = g3d::PlaneRayIntersect(EProjectPlane,RayProject);

	return ((EPoint * PointWeight + ProjectedPoint * ERay.Weight)/(PointWeight + ERay.Weight));
}
//--------------------------------------------------------------------------------------------------------------
Vector3f g3d::RayRayIntersect(EigenRay &Ray1, EigenRay &Ray2)
{
	EigenPlane Plane;
	Plane.Point = Ray1.Point;
	Plane.Vect1 = Ray1.Vector;
	Plane.Vect2 = Ray1.Vector.cross(Ray2.Vector);//This plane is perpendicular to the plane parallel to both rays
	return g3d::PlaneRayIntersect(Plane,Ray2);
}
//--------------------------------------------------------------------------------------------------------------
Vector3f g3d::Rays3Intersetion(EigenRay &Ray1, EigenRay &Ray2, EigenRay &Ray3)
{
	Vector3f I12A,I12B,I13A,I13B,I23A,I23B;
	Vector3f Pos;
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
Vector3f g3d::Rays3IntersetionDist(EigenRay &Ray1, EigenRay &Ray2, EigenRay &Ray3,float &dist)
{
	Vector3f I12A,I12B,I13A,I13B,I23A,I23B;
	Vector3f Pos;
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
Vector3f g3d::RaysNWeighted(std::vector<EigenRay> &ERays)
{
	Vector3f InterAllP,Pos;
	assert(ERays.size() >= 2);
	Vector3f WPoint = Rays2Weighted(ERays[0],ERays[1]);
	float SumWeights = ERays[0].Weight + ERays[1].Weight;
	for(int i=2;i<(int)ERays.size();i++)
	{
		Vector3f NewPoint = RayPointWeighted(ERays[i],WPoint,SumWeights);
		SumWeights+=ERays[i].Weight;
		WPoint = NewPoint;
	}
	return WPoint;
}
//--------------------------------------------------------------------------------------------------------------
//Thsi function adds to Rays3Intersetion the Weight factor that allows to change the confidence we have in 
//every ray or eliminate it totally by a very low value, but NOT 0 for the case where all rays have low confidence
Vector3f g3d::Rays3WIntersetion(EigenRay &Ray1, EigenRay &Ray2, EigenRay &Ray3)
{
	Vector3f I12A,I12B,I13A,I13B,I23A,I23B;
	Vector3f Pos;
	I12A = g3d::RayRayIntersect(Ray1,Ray2);
	I12B = g3d::RayRayIntersect(Ray2,Ray1);
	I13A = g3d::RayRayIntersect(Ray1,Ray3);
	I13B = g3d::RayRayIntersect(Ray3,Ray1);
	I23A = g3d::RayRayIntersect(Ray2,Ray3);
	I23B = g3d::RayRayIntersect(Ray3,Ray2);
	float WeightsSum = 2 * (Ray1.Weight + Ray2.Weight + Ray3.Weight);
	Pos = (		I12A * Ray1.Weight + I12B * Ray2.Weight + 
				I13A * Ray1.Weight + I13B * Ray3.Weight + 
				I23A * Ray2.Weight + I23B * Ray3.Weight  ) / WeightsSum;
	return Pos;
}
//--------------------------------------------------------------------------------------------------------------
//Same as Rays3WIntersetion, but computes also the distance
Vector3f g3d::Rays3WIntersetionDist(EigenRay &Ray1, EigenRay &Ray2, EigenRay &Ray3,float &dist)
{
	Vector3f I12A,I12B,I13A,I13B,I23A,I23B;
	Vector3f Pos;
	dist = 0;
	I12A = g3d::RayRayIntersect(Ray1,Ray2);
	I12B = g3d::RayRayIntersect(Ray2,Ray1);
	dist += ((I12A-I12B).norm());
	I13A = g3d::RayRayIntersect(Ray1,Ray3);
	I13B = g3d::RayRayIntersect(Ray3,Ray1);
	dist += ((I13A-I13B).norm());
	I23A = g3d::RayRayIntersect(Ray2,Ray3);
	I23B = g3d::RayRayIntersect(Ray3,Ray2);
	dist += ((I23A-I23B).norm());
	float WeightsSum = 2 * (Ray1.Weight + Ray2.Weight + Ray3.Weight);
	Pos = (		I12A * Ray1.Weight + I12B * Ray2.Weight + 
				I13A * Ray1.Weight + I13B * Ray3.Weight + 
				I23A * Ray2.Weight + I23B * Ray3.Weight  ) / WeightsSum;
	return Pos;
}
//--------------------------------------------------------------------------------------------------------------
void g3d::DrawRay(cv::Mat &ImgDrawOn,EigenRay Ray,float scale)
{
	Vector3f Pt;
	int XCenter = 320,
		YCenter = 240;
	for(int i=0;i<2000;i++)
	{
		Pt = Ray.Point + ((float)i) * Ray.Vector;
		cv::Point center((int)(XCenter + scale*Pt.x()),(int)(YCenter + scale*Pt.z()));
		cv::circle(ImgDrawOn,center,1,cv::Scalar(250,100,200));
	}
}
