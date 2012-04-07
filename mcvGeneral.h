
#pragma once
#ifndef __MCVGENERAL__
#define __MCVGENERAL__
//#pragma message("------------------------------------mcvGeneral.h------------------------------------")

#include "cv.h"
#include "highgui.h"

#include "yaml-cpp/yaml.h"

#define PI 3.1415926535897932384626433832795f
#define PI_div_2 1.5707963267948966192313216916398f
#define PI_div_12 0.26179938779914943653855361527329f
#define PIx2 6.283185307179586476925286766559f

#ifdef _MSC_VER
  #define INLINE __forceinline /* use __forceinline (VC++ specific) */
#else
  #define INLINE inline        /* use standard inline */
#endif

//-----------------------------------------------------------------------------------------------------
typedef cv::Scalar_<unsigned char> ScalarC;
//for contours
typedef std::vector<cv::Point> VPoints;
typedef std::vector<std::vector<cv::Point>> V2DPoints;

typedef std::vector<std::vector<float>> V2Data;

//------------------------------------------------------------------------------YAML serialisation
//--------------------------------------------------------------------------------------------------------------
//YAML << ScalarC
YAML::Emitter& operator<<(YAML::Emitter& out, const cv::Scalar_<unsigned char> &Scalar);
//YAML << vector<ScalarC>
YAML::Emitter& operator<<(YAML::Emitter& out, const std::vector<cv::Scalar_<unsigned char>>& ScalVect);
//YAML >> ScalarC
void operator >> (const YAML::Node& node, cv::Scalar_<unsigned char> &Scalar);
//YAML >> vector<ScalarC>
void operator >> (const YAML::Node& node, std::vector<cv::Scalar_<unsigned char>> &VScalars);
//cout << YAML::node
std::ostream& operator<<(std::ostream &out, const YAML::Node& node);
//YAML << cv::Scalar
YAML::Emitter& operator<<(YAML::Emitter& out, const cv::Scalar &Scalar);
//YAML << vector<cv::Scalar>
//YAML::Emitter& operator<<(YAML::Emitter& out, const std::vector<cv::Scalar>& ScalVect);
//YAML >> cv::Scalar
void operator >> (const YAML::Node& node, cv::Scalar &Scalar);

//YAML >> vector<cv::Scalar>
void operator >> (const YAML::Node& node, std::vector<cv::Scalar> &VScalars);

//cout << vector<myType>
template<class myType>
std::ostream& operator<<(std::ostream& out, const std::vector<myType>& Vector)
{
	for(int i=0;i<Vector.size();i++)
	{
		out << Vector[i];//The class is free to end its text with endl or not
	}
	return out;
}


//YAML >> vector<myType>
/*template<class myType>
void operator>>(const YAML::Node& node, std::vector<myType>& Vector)
{
	int NbVals = node.size();
	Vector.resize(NbVals);
	for(int i=0;i<NbVals;i++)
	{
		node >> Vector[i];
	}
}
*/


#define	PIXEL16UC1(Image,x,y) (((ushort*)(Image.data + Image.step*y))[x])
#define	PIXEL8UC1(Image,x,y)  (((uchar*)(Image.data + Image.step*y))[x])
#define	PIXEL8UC3_1(Image,x,y)  (((uchar*)(Image.data + Image.step*y))[x*3])
#define	PIXEL8UC3_2(Image,x,y)  (((uchar*)(Image.data + Image.step*y))[x*3+1])
#define	PIXEL8UC3_3(Image,x,y)  (((uchar*)(Image.data + Image.step*y))[x*3+2])

//this is used to return the 16 low bit data from a 24bit Image
//#define PIXEL24U16Low(Image,x,y)  *((unsigned short*)&(((uchar*)(Image.data + Image.step*y))[x*3+1]))
//#define DATA24U16Low(Image,x)  *((unsigned short*)&(((uchar*)(Image.data))[x*3+1]))
//#define DATA24U16Low(Image,x)  *((unsigned short*)&(((uchar*)(Image.data))[x*3+1]))
//#define DATA24U16Low_psi(Image,x)  ((unsigned short*)&(((uchar*)(Image.data))[x*3+1]))
//#define DATA24U16Low_p(Image,x)  &(((uchar*)(Image.data))[x*3+1])

#define	PIXEL8UC4_1(Image,x,y)  (((uchar*)(Image.data + Image.step*y))[x*4])
#define	PIXEL8UC4_2(Image,x,y)  (((uchar*)(Image.data + Image.step*y))[x*4+1])
#define	PIXEL8UC4_3(Image,x,y)  (((uchar*)(Image.data + Image.step*y))[x*4+2])

#define	PIXEL32SC1(Image,x,y)  (((int*)(Image.data + Image.step*y))[x])

#define	DATA8UC1(Image,x)  (((uchar*)(Image.data))[x])

#define	PIXEL32FC1(Image,x,y)  (((float*)(Image.data + Image.step*y))[x])
#define	DATA32FC1(Image,x)  (((float*)(Image.data))[x])

//yes this macro is used cause the endianness has to be switched
#define DATA24U16Low(Image,x,Val);		uchar* pMACROData = Image.datastart + x * 3 + 2;\
										uchar* pMACROVal = (uchar*) &Val;\
										(*pMACROVal++) = (*pMACROData--);\
										(*pMACROVal) = (*pMACROData);
//-------------------------------------------------------------------------------------------------------
//to be trated as inline
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
INLINE void PutBlock(uchar*pPix,int Step,cv::Scalar Color)
{
	(*pPix++)	= (uchar)Color[0];	(*pPix++)	= (uchar)Color[1];	(*pPix++)		= (uchar)Color[2];
	(*pPix++)	= (uchar)Color[0];	(*pPix++)	= (uchar)Color[1];	(*pPix++)		= (uchar)Color[2];
	(*pPix++)	= (uchar)Color[0];	(*pPix++)	= (uchar)Color[1];	(*pPix++)		= (uchar)Color[2];
	pPix+=Step;
	(*pPix++)	= (uchar)Color[0];	(*pPix++)	= (uchar)Color[1];	(*pPix++)		= (uchar)Color[2];
	(*pPix++)	= (uchar)Color[0];	(*pPix++)	= (uchar)Color[1];	(*pPix++)		= (uchar)Color[2];
	(*pPix++)	= (uchar)Color[0];	(*pPix++)	= (uchar)Color[1];	(*pPix++)		= (uchar)Color[2];
	pPix+=Step;
	(*pPix++)	= (uchar)Color[0];	(*pPix++)	= (uchar)Color[1];	(*pPix++)		= (uchar)Color[2];
	(*pPix++)	= (uchar)Color[0];	(*pPix++)	= (uchar)Color[1];	(*pPix++)		= (uchar)Color[2];
	(*pPix++)	= (uchar)Color[0];	(*pPix++)	= (uchar)Color[1];	(*pPix)		= (uchar)Color[2];
}
//-------------------------------------------------------------------------------------------------------
INLINE void PutBlockF(float*pPix,int Step,float Color)
{
	(*pPix++)	= Color;	(*pPix++)	= Color;	(*pPix++)		= Color;	
	pPix+=Step;
	(*pPix++)	= Color;	(*pPix++)	= Color;	(*pPix++)		= Color;	
	pPix+=Step;
	(*pPix++)	= Color;	(*pPix++)	= Color;	(*pPix)		= Color;	
}
//-------------------------------------------------------------------------------------------------------

#ifndef CONFIGMAPS
#define CONFIGMAPS
typedef std::map<std::string,std::string> stringmap;
typedef std::list<std::string> stringlist;
typedef std::map<std::string,stringlist> stringlistmap;//a map of lists, lists contains strings
#endif


namespace mcv
{

	class Grabbable_c//Abstract Grabbing Class - dangerous : None is mandatory but at least one of them is, else empty behavior
	{
	public:
		virtual void SetupGrabber(stringmap &Config){}//unnecessary as no high level will use it, just as indication to check practicality
		virtual void GrabFrame(){}//		GrabFrameIndex(NextIndexToGrab++);
		virtual void GrabFrameIndex(int Index){}//not mandatory - Will simply call the GrabFrame() if not overloaded
	};

	const int Cvt_BGR2HueGray = 1;
	const int Cvt_Gray2HueBGR = 2;
	const int Cvt_Short2HueBGR = 3;
	const int Cvt_Float2HueBGR = 4;
	const int Cvt_Float2Gray = 5;
	const int Cvt_Gray2Float = 6;
	const int Cvt_BayersToColorBayers = 7;

	const cv::Scalar clBlue		= cv::Scalar(255,0,0);
	const cv::Scalar clGreen	= cv::Scalar(0,255,0);
	const cv::Scalar clRed		= cv::Scalar(0,0,255);
	const cv::Scalar clBlack	= cv::Scalar(0,0,0);
	const cv::Scalar clGray		= cv::Scalar(128,128,128);
	const cv::Scalar clWhite	= cv::Scalar(255,255,255);

	const ScalarC ClBlue	= ScalarC(255,0,0);
	const ScalarC ClGreen	= ScalarC(0,255,0);
	const ScalarC ClRed		= ScalarC(0,0,255);
	const ScalarC ClBlack	= ScalarC(0,0,0);
	const ScalarC ClGray	= ScalarC(128,128,128);
	const ScalarC ClWhite	= ScalarC(255,255,255);

	int CheckSum(int* pData,int DataSizex4B);

	cv::Mat im2Show(const char	* WINNAME,cv::Mat Img1,cv::Mat Img2);
	cv::Mat im3Show(const char	* WINNAME,cv::Mat Img1,cv::Mat Img2,cv::Mat Img3);
	cv::Mat im3Show(const char	* WINNAME,std::vector<cv::Mat> Imgs);
	cv::Mat im4Show(const char	* WINNAME,cv::Mat Img1,cv::Mat Img2,cv::Mat Img3,cv::Mat Img4);
	cv::Mat im4Show(const char	* WINNAME,std::vector<cv::Mat> Imgs);
	cv::Mat im4CSplitShow(const char	* WINNAME,cv::Mat Img4C);
	cv::Mat im4CSplitNegShow(const char	* WINNAME,cv::Mat Img4C);

	void imNShow(std::vector<cv::Mat> Imgs,const char * WINNAME = "mcv::imNShow");
	void imNMultiShow(std::vector<cv::Mat> Imgs,const char * WINNAME = "mcv::imNMultiShow");

	void im3Write(const char *BaseFileName,std::vector<cv::Mat> ImgsSrc,int Index);
	void im3Write(const char *BaseFileName,cv::Mat ImgsSrc1,cv::Mat ImgsSrc2,cv::Mat ImgsSrc3,int Index);
	void imNWrite(const char *BaseFileName,std::vector<cv::Mat> ImgsSrc,int Index = -1);

	void imRGB2RGBX(cv::Mat &dest,cv::Mat src);
	void imRGBX2RGB(cv::Mat &dest,cv::Mat src);
	cv::Mat imGray2RGB(cv::Mat ImgGray);
	void CvtBGR2HueGray(cv::Mat ImgBGR,cv::Mat &ImgHueGray,int GrayColorThresh);
	void CvtBayersToColorBayers(cv::Mat ImgSrc,cv::Mat &ImgDest);
	void cvtColor(cv::Mat ImgSrc,cv::Mat &ImgDest,int Code);
	void CmpHSV(cv::Mat Img1HSV,cv::Mat Img2HSV,cv::Mat ImgMask);
	void ImgConcat(cv::Mat &ImgDest,std::vector<cv::Mat> ImgListSrc);

	void CvtFloat2GraySpecial(cv::Mat ImgSrc,cv::Mat &ImgDest);
	void CvtFloat2BGR(cv::Mat ImgSrc,cv::Mat &ImgDest);
	void CvtDepthmm2DepthDisplay(cv::Mat &DestBGRFun,const cv::Mat &SrcBGRmm);

	void saveMattxt(const char* FileName,cv::Mat Img);
	void loadMattxt(const char* FileName,cv::Mat &Img);
	void saveSVMdata(const char* FileName,cv::Mat trainData,cv::Mat responses);
	void saveMatData(const char* FileName,cv::Mat Img);
	bool loadMatData(const char* FileName,cv::Mat &Img);

	uchar max1Ch(cv::Mat Img);

	void minMaxLoc(cv::Mat centers,float *VMin,float *VMax);

	cv::Rect BoundingRect(cv::Mat Image);

	cv::Point GravityCenter(cv::Mat Img);

	VPoints ContoursFilter(V2DPoints Contours, int Mode = 0);


void TStart(double &t);
void TStop(double &t,const char *Text=NULL);

float DistL2(float* pF1,float* pF2,int Nb);
float TabDist(const std::vector<float> &Tab1,const std::vector<float> &Tab2);


void TabCpy(std::vector<float*> &Dest,std::vector<float*>Src,size_t Start,size_t NbCpy);
void TabCpy(std::vector<float> &Dest,std::vector<float>Src,size_t Start,size_t NbCpy);
void TabCpy(float *pDest,float* pSrc,size_t Start,size_t NbCpy);
void TabCpy(std::vector<cv::Mat> &Dest,const std::vector<cv::Mat>&Src);

int		TabIndex(std::vector<int> IntTable,int Element);

float TabMin(std::vector<float> Tab);

float TabMin(float*Tab,int Size);
int TabMin_Index(std::vector<float> Tab);

float TabMax(std::vector<float> Tab);

//int TabMax_Index(std::vector<int> Tab);
//int TabMax_Index(std::vector<float> Tab);
template <template<class,class> class STDVector,class Val_t,class Allocator_t> int TabMax_Index(STDVector<Val_t,Allocator_t> Tab);
template <template<class,class> class STDVector,class Val_t,class Allocator_t> int TabMax_Index(STDVector<Val_t,Allocator_t> Tab)
//int mcv::TabMax_Index(std::vector<int> Tab)
{
	Val_t maxVal;
	//int maxVal;
	int maxIndex = -1;
	if(!Tab.empty())
	{
		maxVal = Tab[0];
		maxIndex = 0;
	}
	for(int i=1;i<Tab.size();i++)
		if(maxVal<Tab[i])
		{
			maxVal = Tab[i];
			maxIndex = i;
		}
	return maxIndex;
}

float TabAverage(const std::vector<float> &Src);


void TabAdd(std::vector<float> &SrcDest,const std::vector<float> &AddSrc);

void TabMulInv(std::vector<float> &Dest,std::vector<float> InvSrc, float MulBy);
void TabAddMulInv(float*Dest,float*InvSrc, float MulBy, int Size);
void TabDiv(std::vector<float> &SrcDest,float Divisor);
void TabDiv(std::vector<float> &Dest,std::vector<float> Src,float Divisor);
void TabDiv(float*Dest,float*Src,float Divisor,int Size);
void TabPrintf(float* pVals,int NbVals);
//void TabPrintf(std::vector<float> DistsAll);
void TabPrintf(std::vector<unsigned char> DistsAll,const char *VarName = NULL,bool isReturn = true,const char separator = ',');
void TabPrintf(std::vector<int> DistsAll,const char *VarName = NULL,bool isReturn = true,const char separator = ',');
void TabPrintf(std::vector<float> DistsAll,const char *VarName = NULL,bool isReturn = true,const char separator = ',',int precision = 3);
void TabPrintf(std::vector<double> DistsAll,const char *VarName = NULL,bool isReturn = true,const char separator = ',',int precision = 3);
void TabPrintf(std::vector<std::string> Names,const char *VarName = NULL,const char separator = ',',bool isReturn = true);


void TabSet(std::vector<float> &TabVals,float Val);
void SampleAround(float * pDest,float * pSrc,float Scale,int Index=-1);
void ValRand(float &Dest,float minSample,float maxSample);
void TabRand(std::vector<float> &pDest,float minSample,float maxSample);
int TabConcat(const std::vector<std::vector<float>> &ArrayFloat2D,std::vector<float> &Dest);

bool TabEqual(const std::vector<float> &Vect1,const std::vector<float> &Vect2);
bool TabEqual(float *pVect1,float *pVect2,int VectSize);

void TabMostFrequent_Index(const std::vector<char> &DataVals,int &MostFrequentVal_Index,float &MostFrequent_Average);

float DistMax(float V1,float V2,float V3);


//File serializing
float readLineFloatLine(std::ifstream &File);
float readLineFloatLine_Debug(std::ifstream &File);
float readFloatLine(std::ifstream &File);

std::string GetMainPath(std::string Str);
std::string GetFileName(std::string Str);
std::string TakeParseTo(std::string &Str,char Separator);



struct DataInfo_t
{
	size_t DataOffset;	//Bytes
	size_t DataSizeBytes;	//32bits in bytes
	unsigned int CheckSum;//32bits
};

class MixedFileManager_c
{
public:
	DataInfo_t		DataInfo;
	std::string		FileName;
public:
	MixedFileManager_c();
	//In user load() : First Parse(), then LoadData()
	std::stringstream Parse(std::string FileName);		//Get all the info from the File, the most important beeing the DataSize
	void LoadData(char*	pAllocatedDest,size_t DataSizeBytes);//Load with memory ready to receive data as info is got from Parse
	
	//In user save() : Optionally Configure the MEmitter but the Text Header will be there anyway, 
	//Note the DataStartkey can Optionally be modified, then Save()
	void Save(std::string FileName,char*	pData,size_t DataSizeBytes,std::string UserHeader = "");	//YAML Emitter is configured optionally, and DataInfo obligatory
	
	//-------------------------------------------------------------------------------------------
};

}/* end of namespace mcv */

#endif /*__MCVGENERAL__*/
