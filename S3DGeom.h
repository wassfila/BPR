/*
	Starter on Sept 29th 2011
	This S3DGeom file is the division of Soft3DWireModel, here are collected the
	geometry functions and different utilities

	Soft3DWireModel.h file started on March 25th 2011
	The objective is to have a purely soft 3D wire model that will be lightweight
	and subject to multithreading acceleration.
	Its light weight will allow not only points projections but also curve trajectory
	computation that would help finding optimal points on the curve, thus even higher
	dimentionnal space coverage
*/

#ifndef __S3DGEOM__
#define __S3DGEOM__

#include "cv.h"
#include "highgui.h"
#include <Eigen\Dense>

//for the VPoints
#include "mcvGeneral.h"

using Eigen::Matrix4f;
using Eigen::Matrix3f;
using Eigen::Vector4f;
using Eigen::Vector3f;
using Eigen::Vector2f;
using Eigen::Quaternionf;

typedef	cv::Mat S3DMatrix;


//Cannot keep this for generic std::vector cause we have the error:
//error C2593: 'operator <<' is ambiguous could be here or at
//\yaml-cpp\include\yaml-cpp/stlemitter.h(26): where they implement the same but the problem is
//that they won't recognise my specific type inside the std::vector<myType>  !!!!!!!!!!!!!!
//Emit any std::vector given that the serialisation function of the element Type is available


class EigenRay
{
public:
	Vector3f	Point;
	Vector3f	Vector;
	float		Weight;//used when one camera ray detects nothing or not sure to lower its impact on later intersection
public:
	EigenRay();
	Vector3f GetPoint(float Distance);
	void printf(const char*VName);
};

struct EigenPlane
{
	Vector3f	Point;
	Vector3f	Vect1;
	Vector3f	Vect2;
};

//--------------------------------------------------------------------------------------------------------------
class EigenBlob2DExtremas
{
public:
	Vector2f	Top,Bottom,Left,Right;
	//for the case of nothing is detected, keep trace that a Blob should not be considered
	//Default is True => means the Blob is Valid unless specified otherwise
	bool		isValid;
	EigenBlob2DExtremas();
	void Draw(cv::Mat &Img);
};

struct EigenBox2D
{
	Vector2f	High;
	Vector2f	Low;
};

//--------------------------------------------------------------------------------------------------------------
class EigenBox3D
{
public:
	Vector3f	High;
	Vector3f	Low;
	bool		HasPointAdded;//When the first use is Conditional
	EigenBox3D();
	void Add(Vector3f P3D);
	void SetCube(Vector3f Center,float BoxSide);
	void Reset();
	//---utilities
	float MaxSide();
	Vector3f size();
	bool	isInside(Vector3f& Pos);
	Vector4f GetBaseCenter();
	//By design This is less direct than the Vect3f one so the user is aware when using 4f, adjust w() to 1 afted operations
	void GetCenter(Vector4f& Vect);
	Vector3f GetCenter();
	//Vector4f BlobToBox(const std::vector<cv::Mat> &ImgsBkg,bool CopyImages=true);

	void printf(const char*BName);
};

namespace g3d
{

	float GetNearestIndex(const std::vector<Eigen::Vector3f> Vects,const Vector3f &V,int &Index);
	int GetNearestDist(const std::vector<Eigen::Vector3f> Vects,const Vector3f &V,float &dist);

	EigenBlob2DExtremas ContourToExtremas(VPoints Contour);

	Vector3f PlaneRayIntersect(const EigenPlane &Plane,const EigenRay &Ray);
	Vector3f RayRayIntersect(EigenRay &Ray1, EigenRay &Ray2);//The Intersection of One Ray with the Perp Plane of the Other
	Vector3f Rays3Intersetion(EigenRay &Ray1, EigenRay &Ray2, EigenRay &Ray3);
	Vector3f Rays3IntersetionDist(EigenRay &Ray1, EigenRay &Ray2, EigenRay &Ray3,float &dist);
	Vector3f Rays3WIntersetion(EigenRay &Ray1, EigenRay &Ray2, EigenRay &Ray3);
	Vector3f Rays2Weighted(const EigenRay &Ray1,const EigenRay &Ray2);
	Vector3f RayPointWeighted(const EigenRay &ERay,const Vector3f &EPoint,float PointWeight);
	Vector3f RaysNWeighted(std::vector<EigenRay> &ERays);
	Vector3f Rays3WIntersetionDist(EigenRay &Ray1, EigenRay &Ray2, EigenRay &Ray3,float &dist);
	void DrawRay(cv::Mat &ImgDrawOn,EigenRay Ray,float scale=1);

	Matrix4f Matrix3fTo4f(const Eigen::Matrix3f &Mat3);
	Eigen::Matrix3f Matrix4fTo3f(const Eigen::Matrix4f &Mat4);
	Matrix4f MatrixIdentity();

	Matrix4f MatrixTranslation(float Tx,float Ty,float Tz);
	Matrix4f MatrixTranslation(const Vector4f &TVect);
	void MatrixTranslation(Vector4f &TVect,Matrix4f &ResMatrix);

	Vector4f TranslationVector(const Matrix4f &TranslationMatrix);

	Vector3f MatrixGetXVector(const Matrix4f &vMat);
	Vector3f MatrixGetYVector(const Matrix4f &vMat);
	Vector3f MatrixGetZVector(const Matrix4f &vMat);

	Vector3f MatrixGetXVector(const Matrix3f &vMat);
	Vector3f MatrixGetYVector(const Matrix3f &vMat);
	Vector3f MatrixGetZVector(const Matrix3f &vMat);

	void matrixSetTrVector(Matrix4f& vMat,Vector4f& TrVect);

	Matrix4f MatrixRotationX(float a);
	Matrix4f MatrixRotationY(float a);
	Matrix4f MatrixRotationZ(float a);

	Matrix3f MatrixAxis(Vector3f &XAxis,Vector3f &YAxis,Vector3f &ZAxis);
	Matrix4f MatrixAxisTr(const Vector3f &XAxis,const Vector3f &YAxis,const Vector3f &ZAxis,const Vector3f &TrVect);

	void S3DMatrixPrint(Matrix4f PrintThis);

	Matrix4f S3DMatrixView(Vector3f &Pos,Vector3f &LookAt);
	Matrix4f S3DMatrixView(CvMat * rot_vect,CvMat * tr_vect);
	void S3DmView2RotMat(CvMat * Camera_mat,CvMat * Cam_tr,Matrix4f &mView);
	//Matrix4f S3DMatrixView(cv::Mat &rot_vect,cv::Mat &tr_vect);
	Matrix4f S3DMatrixViewRotMat(CvMat * RotMat,CvMat * tr_vect);
	Matrix4f S3DMatrixProjection(float Fx,float Fy,float Cx,float Cy);

	void GetReverseTransform(Matrix4f &MatDest,const Matrix4f &MatSrc);


	float Point2DProject(Vector2f Vect1,Vector2f Vect2,Vector2f RefVector);
	Vector2f Point2DInterpolate(Vector2f P1,Vector2f P2,float P1v0);
	bool Point2DHalfPlane(Vector2f ThePoint,Vector2f TheProjection,Vector2f PSameHalf);

	Quaternionf readQuaternionf(std::ifstream &File);
	Vector3f readVector3f(std::ifstream &File);
	Vector2f readVector2f(std::ifstream &File);
	void writeQuaternionf(std::ofstream &File,Quaternionf &Quat);
	void writeVector3f(std::ofstream &File,Vector3f Vect);
	void writeVector2f(std::ofstream &File,Vector2f Vect);

	void printfVector2f(const Vector2f &vPos,const char *VarName=NULL,bool isReturn=true,int precision=2);
	void printfVector3f(const Vector3f &vVect,const char *VarName=NULL,bool isReturn=true,int precision=2);
	void printfMatrix3f(const Matrix3f &vMat,const char *VarName=NULL,int precision=2);
	void printfMatrix4f(const Matrix4f &vMat,const char *VarName=NULL,int precision=2);
	void printfQuaternionf(const Quaternionf &vQuat,bool isReturn=false,const char *VarName=NULL,int precision=2);






}

Vector3f V4To3(const Vector4f &Vect);
Vector4f V3To4(const Vector3f &Vect);

//----------------------------------------------------------------------------------------------------------
//-- Serialisation -----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------

/*template<class myType>
YAML::Emitter& operator<<(YAML::Emitter& out, const std::vector<myType>& Vector)
{
	for(int i=0;i<Vector.size();i++)
	{
		out << Vector[i];//The class is free to end its text with endl or not
	}
	return out;
}*/

//namespace YAML{	class Emitter;}

//template <typename Emitter>
//Emitter& operator << (Emitter& out, const Vector3f& Vect3){}

//YAML << Vector3f - Flow: "x" "y" "z"
YAML::Emitter& operator << (YAML::Emitter& out, const Vector3f& Vect3);
//YAML << vector<Vector3f> Flow:
YAML::Emitter& operator<<(YAML::Emitter& out, const std::vector<Vector3f>& Vector);
//YAML >> Vector3f
void operator >> (const YAML::Node& node, Vector3f& Vect3);
//cout << Vector3f
std::ostream& operator<<(std::ostream &out, const Vector3f& Vect3);
//YAML << EigenBox3D
YAML::Emitter& operator << (YAML::Emitter& out, EigenBox3D Box3D);
//YAML >> EigenBox3D
void operator >> (const YAML::Node& node, EigenBox3D& Box3D);
//cout << EigenBox3D ===> Should specialize templates
std::ostream& operator<<(std::ostream &out, const EigenBox3D& Box3D);
//----------------------------------------------------------------------------------------------------------



#endif /*__S3DGEOM__*/
