
#include "mcvGeneral.h"
#include <iostream>
#include <fstream>

using namespace std;


//--------------------------------------------------------------------------------------------------------------
YAML::Emitter& operator<<(YAML::Emitter& out, const cv::Scalar_<unsigned char> &Scalar)
{
	out << YAML::Flow;
	out << YAML::BeginMap;
	out << YAML::Key << "b";
	out << YAML::Value << (int)(Scalar.val[0]);
	out << YAML::Key << "g";
	out << YAML::Value << (int)(Scalar.val[1]);
	out << YAML::Key << "r";
	out << YAML::Value << (int)(Scalar.val[2]);
	out << YAML::EndMap;
	return out;
}
//--------------------------------------------------------------------------------------------------------------
YAML::Emitter& operator<<(YAML::Emitter& out, const std::vector<cv::Scalar_<unsigned char>>& ScalVect)
{
	//out << YAML::Flow;
	out << YAML::BeginSeq;
	for(int i=0;i<ScalVect.size();i++)
	{
		out << ScalVect[i];
	}
	out << YAML::EndSeq;
	return out;
}
//--------------------------------------------------------------------------------------------------------------
void operator >> (const YAML::Node& node, cv::Scalar_<unsigned char> &Scalar)
{
	Scalar.val[0] = (unsigned char)node["b"].to<int>();
	Scalar.val[1] = (unsigned char)node["g"].to<int>();
	Scalar.val[2] = (unsigned char)node["r"].to<int>();
	Scalar.val[3] = 0;//Yes Not used and 0 for distance
}
//--------------------------------------------------------------------------------------------------------------
void operator >> (const YAML::Node& node, std::vector<cv::Scalar_<unsigned char>> &VScalars)
{
	int NbVals = node.size();
	VScalars.resize(NbVals);
	for(int i=0;i<NbVals;i++)
	{
		node[i] >> VScalars[i];
	}
}
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
std::ostream& operator<<(std::ostream &out, const YAML::Node& node)
{
	YAML::Emitter Emit;
	Emit << node;
	out << Emit.c_str() << std::endl;
	return out;
}
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
YAML::Emitter& operator<<(YAML::Emitter& out, const cv::Scalar &Scalar)
{
	out << YAML::Flow;
	out << YAML::BeginMap;
	out << YAML::Key << "x";
	out << YAML::Value << Scalar.val[0];
	out << YAML::Key << "y";
	out << YAML::Value << Scalar.val[1];
	out << YAML::Key << "z";
	out << YAML::Value << Scalar.val[2];
	out << YAML::Key << "w";
	out << YAML::Value << Scalar.val[3];
	out << YAML::EndMap;
	return out;
}
//--------------------------------------------------------------------------------------------------------------
YAML::Emitter& operator<<(YAML::Emitter& out, const std::vector<cv::Scalar>& ScalVect)
{
	//out << YAML::Flow;
	out << YAML::BeginSeq;
	for(int i=0;i<ScalVect.size();i++)
	{
		out << ScalVect[i];
	}
	out << YAML::EndSeq;
	return out;
}
//--------------------------------------------------------------------------------------------------------------
void operator >> (const YAML::Node& node, cv::Scalar &Scalar)
{
	node["x"] >> Scalar.val[0];
	node["y"] >> Scalar.val[1];
	node["z"] >> Scalar.val[2];
	node["w"] >> Scalar.val[3];
}
//--------------------------------------------------------------------------------------------------------------
void operator >> (const YAML::Node& node, std::vector<cv::Scalar> &VScalars)
{
	size_t NbVals = node.size();
	VScalars.resize(NbVals);
	for(size_t i=0;i<NbVals;i++)
	{
		node[i] >> VScalars[i];
	}
}
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
int mcv::CheckSum(int* pData,int DataSizex4B)
{
	int XorRes = 0;
	int *pDataAfterLast = pData + DataSizex4B;
	while(pData<pDataAfterLast)
	{
		XorRes+=(*pData++);
	}
	return XorRes;
}
//--------------------------------------------------------------------------------------------------------------
uchar mcv::max1Ch(cv::Mat Img)
{
	assert(Img.channels() == 1);
	int imagesize = Img.cols * Img.rows;
	uchar Max = 0;
	uchar *pImg = Img.data;
	for(int i=0;i<imagesize;i++)
	{
		if(Max < *pImg)Max = *pImg;
		pImg++;
	}
	return Max;
}
//--------------------------------------------------------------------------------------------------------------
void mcv::minMaxLoc(cv::Mat centers,float *VMin,float *VMax)
{
	if(centers.type() == CV_32FC1)
	{
		float * pF = (float *)centers.data;
		*VMin = *pF;
		*VMax = *pF;
		UINT Size = (UINT)(centers.dataend - centers.datastart)/4;
		for(UINT i=0;i<Size;i++)
		{
			if(*pF>*VMax)*VMax=*pF;
			if(*pF<*VMin)*VMin=*pF;
			pF++;
		}
	}
}
//--------------------------------------------------------------------------------------------------------------
cv::Mat mcv::im2Show(const char	* WINNAME,cv::Mat Img1,cv::Mat Img2)
{
	cv::Mat ImgDisp(480,1280,Img1.type(),cv::Scalar::all(0));
	cv::Size SizeS(640,480);

	cv::Mat Img1s(ImgDisp,cv::Rect(0,			0,	SizeS.width,SizeS.height));
	cv::Mat Img2s(ImgDisp,cv::Rect(SizeS.width,	0,	SizeS.width,SizeS.height));
	cv::resize(Img1,Img1s,SizeS);//INTER_NEAREST win about 1.5 to 2 ms
	cv::resize(Img2,Img2s,SizeS);

	if(WINNAME!=NULL)cv::imshow(WINNAME,ImgDisp);
	return ImgDisp;
}
//--------------------------------------------------------------------------------------------------------------
cv::Mat mcv::im3Show(const char	* WINNAME,std::vector<cv::Mat> Imgs)
{
	return mcv::im3Show(WINNAME,Imgs[0],Imgs[1],Imgs[2]);
}
//--------------------------------------------------------------------------------------------------------------
cv::Mat mcv::im3Show(const char	* WINNAME,cv::Mat Img1,cv::Mat Img2,cv::Mat Img3)
{

	cv::Mat Img1s;
	cv::Mat Img2s;
	cv::Mat Img3s;
	cv::Mat ImgDisp;
	cv::Size SizeS;

	assert(	(Img1.cols == Img2.cols) && (Img2.cols == Img3.cols) &&
			(Img1.rows == Img2.rows) && (Img2.rows == Img3.rows) );
	SizeS = cv::Size(Img1.cols/2,Img1.rows/2);
	ImgDisp = cv::Mat(SizeS.height*2,SizeS.width*2,Img1.type(),cv::Scalar::all(0));
	Img1s = cv::Mat(ImgDisp,cv::Rect(0,		0,		SizeS.width,SizeS.height));
	Img2s = cv::Mat(ImgDisp,cv::Rect(SizeS.width,	0,		SizeS.width,SizeS.height));
	Img3s = cv::Mat(ImgDisp,cv::Rect(0,		SizeS.height,	SizeS.width,SizeS.height));
	cv::resize(Img1,Img1s,SizeS);//INTER_NEAREST win about 1.5 to 2 ms
	cv::resize(Img2,Img2s,SizeS);
	cv::resize(Img3,Img3s,SizeS);

/*}
	else
	{
		SizeS = cv::Size(320,240);
		ImgDisp = cv::Mat(480,640,Img1.type(),cv::Scalar::all(0));
		Img1s = cv::Mat(ImgDisp,cv::Rect(0,			0,				SizeS.width,SizeS.height));
		Img2s = cv::Mat(ImgDisp,cv::Rect(SizeS.width,	0,				SizeS.width,SizeS.height));
		Img3s = cv::Mat(ImgDisp,cv::Rect(0,			SizeS.height,	SizeS.width,SizeS.height));
		cv::resize(Img1,Img1s,SizeS);//INTER_NEAREST win about 1.5 to 2 ms
		cv::resize(Img2,Img2s,SizeS);
		cv::resize(Img3,Img3s,SizeS);
	}
	*/
	if(WINNAME!=NULL)cv::imshow(WINNAME,ImgDisp);
	return ImgDisp;
}

//--------------------------------------------------------------------------------------------------------------
void mcv::im3Write(const char *BaseFileName,std::vector<cv::Mat> ImgsSrc,int Index)
{
	char FileName[512];
	for(int i=0;i<3;i++)
	{
		//sprintf_s(FileName,512,"%sM%dC%dBkg_Img%03d.png",BaseFileName,2,i,Index);//Models
		sprintf_s(FileName,512,"%s%02d.000%03d.png",BaseFileName,i,Index);//Images on ambiance3
		cv::imwrite(FileName,ImgsSrc[i]);
	}
}
//--------------------------------------------------------------------------------------------------------------
void mcv::im3Write(const char *BaseFileName,cv::Mat ImgsSrc1,cv::Mat ImgsSrc2,cv::Mat ImgsSrc3,int Index)
{
	char FileName[512];
	sprintf_s(FileName,512,"%s%02d.000%03d.png",BaseFileName,0,Index);//Images on ambiance3
	cv::imwrite(FileName,ImgsSrc1);
	sprintf_s(FileName,512,"%s%02d.000%03d.png",BaseFileName,1,Index);//Images on ambiance3
	cv::imwrite(FileName,ImgsSrc2);
	sprintf_s(FileName,512,"%s%02d.000%03d.png",BaseFileName,2,Index);//Images on ambiance3
	cv::imwrite(FileName,ImgsSrc3);
}
//--------------------------------------------------------------------------------------------------------------
void mcv::imNWrite(const char *BaseFileName,std::vector<cv::Mat> ImgsSrc,int Index)
{
	char FileName[512];
	for(int i=0;i<(int)ImgsSrc.size();i++)
	{
		if(Index!=-1)
		{
			sprintf_s(FileName,512,"%s%02d.000%03d.png",BaseFileName,i,Index);
		}
		else
		{
			sprintf_s(FileName,512,"%s%02d.png",BaseFileName,i);
		}
		cv::imwrite(FileName,ImgsSrc[i]);
	}
}
//--------------------------------------------------------------------------------------------------------------
//If Max Points == 0, then the contours are just concatenated into one list
//If Max Points == -1, then only the longest contour is returned
//Else the contours with less points than mode are returned
VPoints mcv::ContoursFilter(V2DPoints Contours, int Mode)
{
	VPoints ContourRes;
	if(Mode == 0)
	{
		for(int i=0;i<Contours.size();i++)
		{
			ContourRes.insert(ContourRes.end(),Contours[i].begin(),Contours[i].end());
		}
	}
	else if(Mode == -1)
	{
		int MaxContourPoints = 0,
			MaxContourPointsID = -1;
		for(int i=0;i<Contours.size();i++)
		{
			if(Contours[i].size() > MaxContourPoints)
			{
				MaxContourPoints = (int)Contours[i].size();
				MaxContourPointsID = i;
			}

		}
		if(MaxContourPointsID!=-1)
		{
			ContourRes = Contours[MaxContourPointsID];//
		}
	}
	else
	{
		for(int i=0;i<Contours.size();i++)
		{
			if(Contours[i].size() > Mode)
			{
				ContourRes.insert(ContourRes.end(),Contours[i].begin(),Contours[i].end());
			}
		}
	}
	return ContourRes;
}

//--------------------------------------------------------------------------------------------------------------


void CheckRatio(double ri,double r,int &H,int &W, int &X, int &Y)
{
	if(ri<r)//thinner
	{
		H = 240;
		W = (int) (H * ri);
		X = (320 - W)/2;
		Y = 0;
	}
	else//Thiker
	{
		W = 320;
		H = (int) (W / ri);
		Y = (240 - H)/2;
		X = 0;
	}
}
/* The Automatic Ratio Adaptation doesn't work yet
*/
cv::Mat mcv::im4Show(const char	* WINNAME,cv::Mat Img1,cv::Mat Img2,cv::Mat Img3,cv::Mat Img4)
{
	cv::Size SizeS;
	cv::Mat Img1s,Img2s,Img3s,Img4s;
	cv::Mat ImgDisp;

	//Ratio Checking
	if(			(Img1.cols == Img2.cols) && (Img2.cols == Img3.cols) && (Img3.cols == Img4.cols) &&
				(Img1.rows == Img2.rows) && (Img2.rows == Img3.rows) && (Img3.rows == Img4.rows) )
	{
		SizeS = cv::Size(Img1.cols/2,Img1.rows/2);
		ImgDisp = cv::Mat(SizeS.height*2,SizeS.width*2,Img1.type(),cv::Scalar::all(0));
		Img1s = cv::Mat(ImgDisp,cv::Rect(0,		0,		SizeS.width,SizeS.height));
		Img2s = cv::Mat(ImgDisp,cv::Rect(SizeS.width,	0,		SizeS.width,SizeS.height));
		Img3s = cv::Mat(ImgDisp,cv::Rect(0,		SizeS.height,	SizeS.width,SizeS.height));
		Img4s = cv::Mat(ImgDisp,cv::Rect(SizeS.width,	SizeS.height,	SizeS.width,SizeS.height));
		cv::resize(Img1,Img1s,SizeS);//INTER_NEAREST win about 1.5 to 2 ms
		cv::resize(Img2,Img2s,SizeS);
		cv::resize(Img3,Img3s,SizeS);
		cv::resize(Img4,Img4s,SizeS);
	}
	/*else if(	(Img1.cols == 644) && (Img2.cols == 644) && (Img3.cols == 644) && (Img4.cols == 644) &&
				(Img1.rows == 484) && (Img2.rows == 484) && (Img3.rows == 484) && (Img4.rows == 484) )
	{
		SizeS = cv::Size(320,240);
		ImgDisp = cv::Mat(480,640,Img1.type(),cv::Scalar::all(0));
		Img1s = cv::Mat(ImgDisp,cv::Rect(0,		0,		320,240));
		Img2s = cv::Mat(ImgDisp,cv::Rect(320,	0,		320,240));
		Img3s = cv::Mat(ImgDisp,cv::Rect(0,		240,	320,240));
		Img4s = cv::Mat(ImgDisp,cv::Rect(320,	240,	320,240));
		cv::resize(Img1,Img1s,SizeS);//INTER_NEAREST win about 1.5 to 2 ms
		cv::resize(Img2,Img2s,SizeS);
		cv::resize(Img3,Img3s,SizeS);
		cv::resize(Img4,Img4s,SizeS);
	}
	else if(	(Img1.cols == 656) && (Img2.cols == 656) && (Img3.cols == 656) && (Img4.cols == 656) &&
				(Img1.rows == 490) && (Img2.rows == 490) && (Img3.rows == 490) && (Img4.rows == 490) )
	{
		SizeS = cv::Size(328,245);
		ImgDisp = cv::Mat(SizeS.height*2,SizeS.width*2,Img1.type(),cv::Scalar::all(0));
		Img1s = cv::Mat(ImgDisp,cv::Rect(0,		0,		SizeS.width,SizeS.height));
		Img2s = cv::Mat(ImgDisp,cv::Rect(SizeS.width,	0,		SizeS.width,SizeS.height));
		Img3s = cv::Mat(ImgDisp,cv::Rect(0,		SizeS.height,	SizeS.width,SizeS.height));
		Img4s = cv::Mat(ImgDisp,cv::Rect(SizeS.width,	SizeS.height,	SizeS.width,SizeS.height));
		cv::resize(Img1,Img1s,SizeS);//INTER_NEAREST win about 1.5 to 2 ms
		cv::resize(Img2,Img2s,SizeS);
		cv::resize(Img3,Img3s,SizeS);
		cv::resize(Img4,Img4s,SizeS);
	}*/
	else
	{
		SizeS = cv::Size(320,240);
		ImgDisp = cv::Mat(480,640,Img1.type(),cv::Scalar::all(0));
		double r = 640.0/480.0;
		double r1 = (double)Img1.cols / (double)Img1.rows;
		double r2 = (double)Img2.cols / (double)Img2.rows;
		double r3 = (double)Img3.cols / (double)Img3.rows;
		double r4 = (double)Img4.cols / (double)Img4.rows;
		int H1 = Img1.cols,W1 = Img1.rows,X1=0,Y1=0;
		int H2 = Img2.cols,W2 = Img2.rows,X2=0,Y2=0;
		int H3 = Img3.cols,W3 = Img3.rows,X3=0,Y3=0;
		int H4 = Img4.cols,W4 = Img4.rows,X4=0,Y4=0;
		CheckRatio(r1,r,H1,W1,X1,Y1);
		CheckRatio(r2,r,H2,W2,X2,Y2);
		CheckRatio(r3,r,H3,W3,X3,Y3);
		CheckRatio(r4,r,H4,W4,X4,Y4);

		Img1s = cv::Mat(ImgDisp,cv::Rect(X1+0,			Y1+0,				W1,H1));
		Img2s = cv::Mat(ImgDisp,cv::Rect(X2+SizeS.width,Y2+0,				W2,H2));
		Img3s = cv::Mat(ImgDisp,cv::Rect(X3+0,			Y3+SizeS.height,	W3,H3));
		Img4s = cv::Mat(ImgDisp,cv::Rect(X4+SizeS.width,Y4+SizeS.height,	W4,H4));
		cv::resize(Img1,Img1s,SizeS);//INTER_NEAREST win about 1.5 to 2 ms
		cv::resize(Img2,Img2s,SizeS);
		cv::resize(Img3,Img3s,SizeS);
		cv::resize(Img4,Img4s,SizeS);
	}


	if(WINNAME!=NULL)cv::imshow(WINNAME,ImgDisp);
	return ImgDisp;
}
//--------------------------------------------------------------------------------------------------------------
cv::Mat mcv::im4Show(const char	* WINNAME,std::vector<cv::Mat> Imgs)
{
	assert((int)Imgs.size() >= 4);
	return im4Show(WINNAME,Imgs[0],Imgs[1],Imgs[2],Imgs[3]);
}
//--------------------------------------------------------------------------------------------------------------
cv::Mat mcv::im4CSplitShow(const char	* WINNAME,cv::Mat Img4C)
{
	std::vector<cv::Mat>Img4Channels;
	cv::split(Img4C,Img4Channels);
	return im4Show(WINNAME,Img4Channels[3],Img4Channels[2],Img4Channels[1],Img4Channels[0]);
}
//--------------------------------------------------------------------------------------------------------------
cv::Mat mcv::im4CSplitNegShow(const char	* WINNAME,cv::Mat Img4C)
{
	std::vector<cv::Mat>Img4Channels;
	cv::split(Img4C,Img4Channels);
	cv::bitwise_not(Img4Channels[0],Img4Channels[0]);
	cv::bitwise_not(Img4Channels[1],Img4Channels[1]);
	cv::bitwise_not(Img4Channels[2],Img4Channels[2]);
	cv::bitwise_not(Img4Channels[3],Img4Channels[3]);
	return im4Show(WINNAME,Img4Channels[3],Img4Channels[2],Img4Channels[1],Img4Channels[0]);
}
//--------------------------------------------------------------------------------------------------------------
void mcv::imNShow(std::vector<cv::Mat> Imgs,const char * WINNAME)
{
	char WName2[512];
	if((int)Imgs.size() > 4)
	{
		sprintf(WName2,"%s_(2)",WINNAME);
	}
	switch((int)Imgs.size())
	{
		case 0:									break;
		case 1:		cv::imshow(WINNAME,Imgs[0]);break;
		case 2:		mcv::im2Show(WINNAME,Imgs[0],Imgs[1]); break;
		case 3:		mcv::im3Show(WINNAME,Imgs); break;
		case 4:		mcv::im4Show(WINNAME,Imgs);break;
		case 5:		mcv::im4Show(WINNAME,Imgs);
					cv::imshow(WName2,Imgs[4]);break;
		case 6:		mcv::im4Show(WINNAME,Imgs);
					mcv::im2Show(WName2,Imgs[4],Imgs[5]);break;
		case 7:		mcv::im4Show(WINNAME,Imgs);
					mcv::im3Show(WName2,Imgs[4],Imgs[5],Imgs[6]);break;
		case 8:		mcv::im4Show(WINNAME,Imgs);
					mcv::im4Show(WName2,Imgs[4],Imgs[5],Imgs[6],Imgs[7]);break;
	};
}
//--------------------------------------------------------------------------------------------------------------
void mcv::imNMultiShow(std::vector<cv::Mat> Imgs,const char * WINNAME)
{
	char WName[512];
	assert(Imgs.size() < 10);//10 is too much
	for(int i=0;i<(int)Imgs.size();i++)
	{
		sprintf(WName,"%s_(%i)",WINNAME,i);
		cv::imshow(WName,Imgs[i]);
	}
}
//--------------------------------------------------------------------------------------------------------------
void mcv::imRGB2RGBX(cv::Mat &dest,cv::Mat src)
{
	std::vector<cv::Mat>Img3Channels;
	cv::Mat ChanX(src.size(),CV_8UC1);
	cv::split(src,Img3Channels);
	Img3Channels.push_back(ChanX);
	cv::merge(Img3Channels,dest);
}

//--------------------------------------------------------------------------------------------------------------
void mcv::imRGBX2RGB(cv::Mat &dest,cv::Mat src)
{
	std::vector<cv::Mat>Img4Channels;
	cv::split(src,Img4Channels);
	std::vector<cv::Mat>Img3Channels;
	Img3Channels.push_back(Img4Channels[0]);
	Img3Channels.push_back(Img4Channels[1]);
	Img3Channels.push_back(Img4Channels[2]);
	cv::merge(Img3Channels,dest);
}

cv::Mat mcv::imGray2RGB(cv::Mat ImgGray)
{
	cv::Mat ResColImg(ImgGray.rows,ImgGray.cols,CV_8UC3);
	std::vector<cv::Mat> Img3;
	Img3.push_back(ImgGray);
	Img3.push_back(ImgGray);
	Img3.push_back(ImgGray);
	cv::merge(Img3,ResColImg);
	return ResColImg;
}

void mcv::CvtBGR2HueGray(cv::Mat ImgBGR,cv::Mat &ImgHueGray,int GrayColorThresh)
{
	cv::Mat ImgHSV;
	cv::cvtColor(ImgBGR,ImgHSV,CV_BGR2HSV);
	std::vector<cv::Mat> ImgChans;
	cv::split(ImgHSV,ImgChans);
	cv::Mat I_Value;
	ImgChans[2].copyTo(I_Value);
	cv::Mat I_SatMask;
	cv::threshold(ImgChans[1],I_SatMask,GrayColorThresh,255,CV_THRESH_BINARY_INV);
	ImgChans[1].setTo(cv::Scalar(255));
	ImgChans[2].setTo(cv::Scalar(255));
	cv::merge(ImgChans,ImgHSV);
	cv::cvtColor(ImgHSV,ImgHueGray,CV_HSV2BGR);
	cv::split(ImgHueGray,ImgChans);
	I_Value.copyTo(ImgChans[0],I_SatMask);
	I_Value.copyTo(ImgChans[1],I_SatMask);
	I_Value.copyTo(ImgChans[2],I_SatMask);
	cv::merge(ImgChans,ImgHueGray);
}
//--------------------------------------------------------------------------------
void mcv::CvtBayersToColorBayers(cv::Mat ImgSrc,cv::Mat &ImgDest)
{
	assert(ImgSrc.channels() == 1);
	assert(ImgDest.channels() == 3);
	assert(ImgSrc.cols == ImgDest.cols);
	assert(ImgSrc.rows == ImgDest.rows);
	uchar* pSrc = ImgSrc.data;
	uchar* pBlue = ImgDest.data;
	uchar* pGreen = ImgDest.data+1;
	uchar* pRed = ImgDest.data+2;
	char C = 3;
	for(int j=0;j<ImgSrc.rows;j++)
	{
		if(C & 2) C &= 1;//mask bit0 to change bit 1
		else C |= 2;
		for(int i=0;i<ImgSrc.cols;i++)
		{

			if(C & 1) C &= 2;//mask bit 1 to change bit 0
			else C |= 1;
			switch(C)
			{//BGRG - GBGR - GRGB
				case 0:	*pBlue = 0;	*pGreen = *pSrc;		*pRed = 0;		break;//		G
				case 1:	*pBlue = 0;		*pGreen = 0;	*pRed = *pSrc;		break;//		R  - G ou B
				case 2:	*pBlue = *pSrc;		*pGreen = 0;		*pRed = 0;	break;//		B
				case 3:	*pBlue = 0;		*pGreen = *pSrc;	*pRed = 0;		break;//		G  - G ou B
			}
			pSrc++;
			pBlue+=3;
			pGreen+=3;
			pRed+=3;
		}
	}
}
//--------------------------------------------------------------------------------
void Cvt8U2HueBGR(cv::Mat ImgGray,cv::Mat &ImgHueBGR)
{
	std::vector<cv::Mat> ImgChans;
	ImgChans.resize(3);
	ImgChans[0] = cv::Mat(ImgGray);
	ImgChans[1] = cv::Mat(ImgGray.rows,ImgGray.cols,CV_8UC1);
	ImgChans[2] = cv::Mat(ImgGray.rows,ImgGray.cols,CV_8UC1);
	ImgChans[1].setTo(cv::Scalar(255));
	ImgChans[2].setTo(cv::Scalar(255));

	if((ImgHueBGR.type() != CV_8UC3) || (ImgHueBGR.cols != ImgGray.cols) || (ImgHueBGR.rows != ImgGray.rows) )
	{
		ImgHueBGR = cv::Mat(ImgGray.rows,ImgGray.cols,CV_8UC3);
	}
	cv::Mat ImgHSV(ImgGray.rows,ImgGray.cols,CV_8UC3);
	cv::merge(ImgChans,ImgHSV);
	cv::cvtColor(ImgHSV,ImgHueBGR,CV_HSV2BGR);
}
//--------------------------------------------------------------------------------
void Cvt16U2HueBGR(cv::Mat ImgShort,cv::Mat &ImgHueBGR)
{
	std::vector<cv::Mat> ImgChans;
	ImgChans.resize(3);
	ImgChans[0] = cv::Mat(ImgShort.rows,ImgShort.cols,CV_8UC1);
	ImgChans[1] = cv::Mat(ImgShort.rows,ImgShort.cols,CV_8UC1);
	ImgChans[2] = cv::Mat(ImgShort.rows,ImgShort.cols,CV_8UC1);
	ImgChans[1].setTo(cv::Scalar(128));
	ImgChans[2].setTo(cv::Scalar(128));

	cv::convertScaleAbs(ImgShort,ImgChans[0]);

	if((ImgHueBGR.type() != CV_8UC3) || (ImgHueBGR.cols != ImgShort.cols) || (ImgHueBGR.rows != ImgShort.rows) )
	{
		ImgHueBGR = cv::Mat(ImgShort.rows,ImgShort.cols,CV_8UC3);
	}
	cv::Mat ImgHSV(ImgShort.rows,ImgShort.cols,CV_8UC3);
	cv::merge(ImgChans,ImgHSV);
	cv::cvtColor(ImgHSV,ImgHueBGR,CV_HSV2BGR);
}
//--------------------------------------------------------------------------------
void CvtFloat2HueBGR(cv::Mat ImgFloat,cv::Mat &ImgDest)
{
	cv::Mat Img8U(ImgFloat.rows,ImgFloat.cols,CV_8UC1);
	cv::convertScaleAbs(ImgFloat,Img8U);
	Cvt8U2HueBGR(Img8U,ImgDest);
}
//--------------------------------------------------------------------------------
void CvtFloat2Gray(cv::Mat ImgSrc,cv::Mat &ImgDest)
{
	if(	(ImgDest.type() != CV_8UC1) || (ImgSrc.type() != CV_32FC1) || 
		(ImgDest.cols != ImgSrc.cols) || (ImgDest.rows != ImgSrc.rows) )
	{
		ImgDest = cv::Mat(ImgSrc.rows,ImgSrc.cols,CV_8UC1);
	}
	UINT NbPix = ImgSrc.cols * ImgSrc.rows;
	float * pF = (float*)ImgSrc.data;
	uchar * pU = ImgDest.data;
	for(UINT i=0;i<NbPix;i++)
	{
		if(*pF>255)*pU = 255;else *pU = (uchar) *pF;
		pF++;
		pU++;
	}
}
//--------------------------------------------------------------------------------
void mcv::CvtFloat2GraySpecial(cv::Mat ImgSrc,cv::Mat &ImgDest)
{
	if(	(ImgDest.type() != CV_8UC1) || (ImgSrc.type() != CV_32FC1) || 
		(ImgDest.cols != ImgSrc.cols) || (ImgDest.rows != ImgSrc.rows) )
	{
		ImgDest = cv::Mat(ImgSrc.rows,ImgSrc.cols,CV_8UC1);
	}
	UINT NbPix = ImgSrc.cols * ImgSrc.rows;
	float * pF = (float*)ImgSrc.data;
	uchar * pU = ImgDest.data;
	for(UINT i=0;i<NbPix;i++)
	{
		*pF = (*pF -0.9f) * 2550;
		if(*pF>255)*pU = 255;else *pU = (uchar) *pF;
		if(*pU == 255) *pU = 0;
		*pU *=10;
		pF++;
		pU++;
	}
}
//--------------------------------------------------------------------------------
void mcv::CvtFloat2BGR(cv::Mat ImgSrc,cv::Mat &ImgDest)
{
	assert(ImgSrc.type() == CV_32FC1);
	if(	(ImgDest.type() != CV_8UC3) || (ImgDest.cols != ImgSrc.cols) || (ImgDest.rows != ImgSrc.rows) )
	{
		ImgDest = cv::Mat(ImgSrc.rows,ImgSrc.cols,CV_8UC3);
	}
	UINT NbPix = ImgSrc.cols * ImgSrc.rows;
	float * pF = (float*)ImgSrc.data;
	uchar * pU = ImgDest.data;
	for(UINT i=0;i<NbPix;i++)
	{
		int RGB = (int)((*pF) * ((256 * 256 * 256) - 1));
		*pU++ =(RGB>>16) & 0xFF;
		*pU++ =(RGB>>8) & 0xFF;
		*pU++ =(RGB) & 0xFF;
		pF++;
	}
}
//--------------------------------------------------------------------------------
void CvtGray2Float(cv::Mat ImgSrc,cv::Mat &ImgDest)
{
	if(	(ImgDest.type() != CV_32FC1) || (ImgSrc.type() != CV_8UC1) || 
		(ImgDest.cols != ImgSrc.cols) || (ImgDest.rows != ImgSrc.rows) )
	{
		ImgDest = cv::Mat(ImgSrc.rows,ImgSrc.cols,CV_32FC1);
	}
	//else if Src != 8U ??!!
	UINT NbPix = ImgSrc.cols * ImgSrc.rows;
	uchar * pU = ImgSrc.data;
	float * pF = (float*)ImgDest.data;
	//printf("\n\nCvtGray2Float\n\n");
	for(UINT i=0;i<NbPix;i++)
	{
		*pF = (float) *pU;
		//printf("(%d %1.1f) ",*pU,*pF);
		pF++;
		pU++;
	}
}
//-------------------------------------------------------------------------------------------------------
void mcv::CvtDepthmm2DepthDisplay(cv::Mat &DestBGRFun,const cv::Mat &SrcBGRmm)
{
	assert(SrcBGRmm.channels() == 3);
	assert(DestBGRFun.channels() == 3);
	assert(SrcBGRmm.cols == DestBGRFun.cols);
	assert(SrcBGRmm.rows == DestBGRFun.rows);
	uchar* pSrcDH = SrcBGRmm.data;//High
	uchar* pSrcDM = SrcBGRmm.data+1;//Med
	uchar* pSrcDL = SrcBGRmm.data+2;//Low
	uchar* pBlue = DestBGRFun.data;
	uchar* pGreen = DestBGRFun.data+1;
	uchar* pRed = DestBGRFun.data+2;
	for(pSrcDH=SrcBGRmm.datastart;pSrcDH<SrcBGRmm.dataend;pSrcDH+=3)
	{
		//(*pSrcDH) = 1;		(*pSrcDM) = 0xFF;		(*pSrcDL) = 0x53;// To Debug
		//---------------------------
		int DataHigh16 = (*pSrcDH);
		DataHigh16<<=8;
		DataHigh16 |= (*pSrcDM);
		(*pGreen) = (*pSrcDL);
		(*pBlue) = 0;
		(*pRed) = 0;
		uchar MaskDown = 0x80;
		for(int MaskUp=1;MaskUp!=0x10000;MaskUp<<=1)
		{
			if(DataHigh16 & MaskUp)
			{
				(*pRed) = ((*pRed) | MaskDown);
			}
			MaskUp<<=1;
			if(DataHigh16 & MaskUp)
			{
				(*pBlue) = ((*pBlue) | MaskDown);
			}
			MaskDown>>=1;
		}
		//---------------------------
		pSrcDM+=3;
		pSrcDL+=3;
		pBlue+=3;
		pGreen+=3;
		pRed+=3;
	}
}
//--------------------------------------------------------------------------------

void mcv::cvtColor(cv::Mat ImgSrc,cv::Mat &ImgDest,int Code)
{
	switch(Code)
	{
	case mcv::Cvt_BGR2HueGray:
		CvtBGR2HueGray(ImgSrc,ImgDest,70);
		break;
	case mcv::Cvt_Gray2HueBGR:
		Cvt8U2HueBGR(ImgSrc,ImgDest);
		break;
	case mcv::Cvt_Short2HueBGR:
		Cvt16U2HueBGR(ImgSrc,ImgDest);
		break;
	case mcv::Cvt_Float2HueBGR:
		CvtFloat2HueBGR(ImgSrc,ImgDest);
		break;
	case mcv::Cvt_Float2Gray:
		CvtFloat2Gray(ImgSrc,ImgDest);
		break;
	case mcv::Cvt_Gray2Float:
		CvtGray2Float(ImgSrc,ImgDest);
		break;
	case mcv::Cvt_BayersToColorBayers:
		CvtBayersToColorBayers(ImgSrc,ImgDest);
		break;
	default:
		CvtBGR2HueGray(ImgSrc,ImgDest,70);
		break;
	}
}

void mcv::CmpHSV(cv::Mat Img1HSV,cv::Mat Img2HSV,cv::Mat ImgMask)
{int i,j;
uchar	*p1H,*p2H,*p1S,*p2S,*p1V,*p2V,*pMask;
uchar SatThresh = 60;
uchar HueCmpThresh = 2;
uchar ValCmpThresh = 20;

	p1H = Img1HSV.data;
	p1S = Img1HSV.data + 1;
	p1V = Img1HSV.data + 2;

	p2H = Img2HSV.data;
	p2S = Img2HSV.data + 1;
	p2V = Img2HSV.data + 2;

	pMask = ImgMask.data;

	for(j=0;j<Img1HSV.rows;j++)
	for(i=0;i<Img1HSV.cols;i++)
	{
		//Process Channels
		//(*pMask) = (*p1S);
		if( ((*p1S) > SatThresh) || ((*p2S) > SatThresh) )
		{
			if(  abs((*p1H)-(*p2H)) > HueCmpThresh )
				(*pMask) = 0;
			else
				(*pMask) = 255;
		}
		else// (S1 < T) && (S2 < T)
		{
			if(  abs((*p1H)-(*p2H)) > ValCmpThresh )
				(*pMask) = 0;
			else
				(*pMask) = 255;
		}
		//Update Pointers
		p1H += 3;p1S += 3;p1V += 3;
		p2H += 3;p2S += 3;p2V += 3;
		pMask++;
	}
}

/*
	Doesn't change nor chesk the size of Dest !!!
*/
void mcv::ImgConcat(cv::Mat &ImgDest,std::vector<cv::Mat> ImgListSrc)
{
	uchar *pDest,*pSrc;
	size_t Size;
	pDest = ImgDest.data;
	for(UINT i=0;i<ImgListSrc.size();i++)
	{
		pSrc = ImgListSrc[i].data;
		Size = (size_t)(ImgListSrc[i].dataend - ImgListSrc[i].data);
		memcpy(pDest,pSrc,Size);
		pDest += Size;
	}
}


//--------------------------------------------------------------------------------------------------------------
cv::Size getMatSize(const char* FileName)
{
	cv::Size MatSize;
	std::ifstream myfile(FileName,std::ios::in);
	int NbTabs=0;
	int NbLines=0;
	if (myfile.is_open())
	{
		std::string line;
		getline(myfile,line);
		for(int i=0;i<line.length();i++)
		{
			if(line[i] == '\t')
				NbTabs++;
		}
		while(!myfile.eof())
		{
			getline(myfile,line);
			NbLines++;
		}
	}
	myfile.close();
	MatSize.width = NbTabs+1;
	MatSize.height = NbLines;
	return MatSize;
}
//--------------------------------------------------------------------------------------------------------------
/*
 This is a function for saving specific known matrix format into a binary file
*/
void mcv::saveMatData(const char* FileName,cv::Mat Img)
{
	std::ofstream myfile;
	myfile.open(FileName,std::ios::out | std::ios::binary | std::ios::trunc);
	size_t Size = Img.dataend - Img.datastart;
	myfile.write((char*)Img.data,Size);
	myfile.close();
}
//--------------------------------------------------------------------------------------------------------------
void mcv::saveMattxt(const char* FileName,cv::Mat Img)
{
	std::ofstream myfile;
	myfile.open(FileName,std::ios::out | std::ios::trunc);
	for(int j=0;j<Img.rows;j++)
		for(int i=0;i<Img.cols;i++)
		{
			myfile << PIXEL32FC1(Img,i,j);
			if(i != Img.cols-1)
			{
				myfile << '\t';
			}
			else
			{
				myfile << std::endl;
			}
		}
	myfile.close();
}
//--------------------------------------------------------------------------------------------------------------
/*
 This is a function for loading specific known matrix format from a binary file
*/
bool mcv::loadMatData(const char* FileName,cv::Mat &Img)
{
	bool Loaded = false;
	std::ifstream myfile(FileName,std::ios::in | std::ios::binary);

	std::ifstream::pos_type size;
	size = myfile.tellg();

	size_t Size = Img.dataend - Img.datastart;
	if (myfile.is_open())
	{
	    //myfile.seekg (0, ios::beg);
		myfile.read((char*)Img.data,Size);
		myfile.close();
		Loaded = true;
	}
	return Loaded;
}

//--------------------------------------------------------------------------------------------------------------
void mcv::loadMattxt(const char* FileName,cv::Mat &Img)
{
	cv::Size MatSize = getMatSize(FileName);

	bool Loaded = false;
	std::ifstream myfile(FileName,std::ios::in);
	if( (Img.cols != MatSize.width) || (Img.rows != MatSize.height) || (Img.type() != CV_32FC1) )
		Img = cv::Mat(MatSize,CV_32FC1);

	if (myfile.is_open())
	{
		for(int j=0;j<Img.rows;j++)
		{
			std::string line;
			getline(myfile,line);
			std::stringstream ss;
			ss << line;
			for(int i=0;i<Img.cols;i++)
			{
				float val;
				ss >> val;
				PIXEL32FC1(Img,i,j) = val;
			}
		}
	}
	myfile.close();
}
//--------------------------------------------------------------------------------------------------------------

void mcv::saveSVMdata(const char* FileName,cv::Mat trainData,cv::Mat responses)
{
	std::ofstream myfile;
	myfile.open(FileName,std::ios::out | std::ios::trunc);
	for(int j=0;j<trainData.rows;j++)
	{
		myfile << DATA32FC1(responses,j)+1 << ' ';
		for(int i=0;i<trainData.cols;i++)
		{
			myfile << i+1 << ':' << PIXEL32FC1(trainData,i,j);
			if(i != trainData.cols-1)
			{
				myfile << ' ';
			}
			else
			{
				myfile << std::endl;
			}
		}
	}
	myfile.close();
}
//--------------------------------------------------------------------------------------------------------------
cv::Point mcv::GravityCenter(cv::Mat Img)
{
	cv::Point Res;
	cv::Moments Momts;
	Momts = cv::moments(Img);
	Res.x = (int) (Momts.m10 / Momts.m00);
	Res.y = (int) (Momts.m01 / Momts.m00);
	return Res;
}
//--------------------------------------------------------------------------------------------------------------

void mcv::TStart(double &t)
{
	t = (double)cv::getTickCount();
}

void mcv::TStop(double &t,const char *Text)
{
	t = ((double)cv::getTickCount() - t)*1000./cv::getTickFrequency();
	if(Text!=NULL)
	{
		printf("%s: %1.2f ms\n",Text,t);
	}
	
}
//-----------------------------------------------------------------------------------------------------------------------------

float mcv::DistL2(float* pF1,float* pF2,int Nb)
{
	float dist = 0;
	for(int i=0;i<Nb;i++)
	{
		float diff = pF1[i] - pF2[i];
		dist +=diff*diff;
	}
	//return sqrt(dist);
	return dist;
}
//--------------------------------------------------------------------------------------------------------------
float mcv::TabDist(const std::vector<float> &Tab1,const std::vector<float> &Tab2)
{
	assert(Tab1.size() == Tab2.size());
	float dist = 0;
	for(int i=0;i<(int)Tab1.size();i++)
	{
		float diff = Tab1[i] - Tab2[i];
		dist +=diff*diff;
	}
	//return sqrt(dist);
	return dist;
}
//--------------------------------------------------------------------------------------------------------------
void mcv::TabCpy(std::vector<float*> &Dest,std::vector<float*>Src,size_t Start,size_t NbCpy)
{
	for(int i=0;i<NbCpy;i++)
		Dest[Start+i] = Src[i];
}
//--------------------------------------------------------------------------------------------------------------
void mcv::TabCpy(std::vector<float> &Dest,std::vector<float>Src,size_t Start,size_t NbCpy)
{
	for(int i=0;i<NbCpy;i++)
		Dest[Start+i] = Src[i];
}
//--------------------------------------------------------------------------------------------------------------
void mcv::TabCpy(float *pDest,float* pSrc,size_t Start,size_t NbCpy)
{
	for(int i=0;i<NbCpy;i++)
		pDest[Start+i] = pSrc[i];
}
//--------------------------------------------------------------------------------------------------------------
void mcv::TabCpy(std::vector<cv::Mat> &Dest,const std::vector<cv::Mat>&Src)
{
	if(Dest.size() != Src.size())
	{
		Dest.resize(Src.size());
	}
	for(int i=0;i<(int)Src.size();i++)
	{
		Src[i].copyTo(Dest[i]);
	}
}
//--------------------------------------------------------------------------------------------------------------
int		mcv::TabIndex(std::vector<int> IntTable,int Element)//returns the first index starting from 0
{
	bool notFound = true;
	int i=0;
	int ResIndex = -1;
	while(notFound && (i<IntTable.size()))
	{
		if(Element == IntTable[i])
		{
			notFound = false;
			ResIndex = i;
		}
		i++;
	}
	return ResIndex;
}
//--------------------------------------------------------------------------------------------------------------
float mcv::TabMin(std::vector<float> Tab)
{
	float Resmin = FLT_MAX;
	for(int i=0;i<Tab.size();i++)if(Resmin>Tab[i])Resmin = Tab[i];
	return Resmin;
}
//--------------------------------------------------------------------------------------------------------------
float mcv::TabMin(float*Tab,int Size)
{
	float Resmin = FLT_MAX;
	for(int i=0;i<Size;i++)if(Resmin>Tab[i])Resmin = Tab[i];
	return Resmin;
}
//--------------------------------------------------------------------------------------------------------------
int mcv::TabMin_Index(std::vector<float> Tab)
{
	float minVal;
	int minIndex = -1;
	if(!Tab.empty())
	{
		minVal = Tab[0];
		minIndex = 0;
	}
	for(int i=1;i<Tab.size();i++)
		if(minVal>Tab[i])
		{
			minVal = Tab[i];
			minIndex = i;
		}
	return minIndex;
}
//--------------------------------------------------------------------------------------------------------------
float mcv::TabMax(std::vector<float> Tab)
{
	float Resmax = -FLT_MAX;// !!!
	for(int i=0;i<Tab.size();i++)if(Resmax<Tab[i])Resmax = Tab[i];
	return Resmax;
}
//--------------------------------------------------------------------------------------------------------------
/*template <template<class,class> class STDVector,class Val_t,class Allocator_t> int mcv::TabMax_Index(STDVector<Val_t,Allocator_t> Tab)
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
}*/
//--------------------------------------------------------------------------------------------------------------
/*void Testfunction()
{
	TabMax_Index
}*/
//--------------------------------------------------------------------------------------------------------------
/*int mcv::TabMax_Index(std::vector<float> Tab)
{
	//Val_t maxVal;
	float maxVal;
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
}*/
//--------------------------------------------------------------------------------------------------------------
void mcv::TabMulInv(std::vector<float> &Dest,std::vector<float> InvSrc, float MulBy)
{
	for(int i=0;i<Dest.size();i++)Dest[i] = MulBy / InvSrc[i];//1 downto 0
}
//--------------------------------------------------------------------------------------------------------------
void mcv::TabAdd(std::vector<float> &SrcDest,const std::vector<float> &AddSrc)
{
#ifndef IS_ITARATOR_FASTER
	int TabSize = SrcDest.size();
	for(int i=0;i<TabSize;i++)
	{
		SrcDest[i] += AddSrc[i];
	}
#else
	std::vector<float>::iterator itDest;
	std::vector<float>::iterator itSrc;
	for(itDest = SrcDest.begin(),itSrc = AddSrc.begin();	itDest<SrcDest.end() ;itDest++,itSrc++)
	{
		*itDes += *itSrc;
	}

#endif
}
//--------------------------------------------------------------------------------------------------------------
float mcv::TabAverage(const std::vector<float> &Src)
{
	double Sum = 0;
	for(int i=0;i<Src.size();i++)
	{
		Sum += Src[i];
	}
	if(Src.size() != 0)
	{
		Sum /= Src.size();
	}
	return (float)Sum;
}
//--------------------------------------------------------------------------------------------------------------
void mcv::TabAddMulInv(float*Dest,float*InvSrc, float MulBy, int Size)
{
	for(int i=0;i<Size;i++)Dest[i] += MulBy / InvSrc[i];
}

//--------------------------------------------------------------------------------------------------------------
void mcv::TabDiv(std::vector<float> &SrcDest,float Divisor)
{
	for(int i=0;i<SrcDest.size();i++)SrcDest[i]/=Divisor;
}
//--------------------------------------------------------------------------------------------------------------
void mcv::TabDiv(std::vector<float> &Dest,std::vector<float> Src,float Divisor)
{
	for(int i=0;i<Dest.size();i++)Dest[i]=Src[i]/Divisor;//Adjust max to 1
}
//--------------------------------------------------------------------------------------------------------------
void mcv::TabDiv(float*Dest,float*Src,float Divisor,int Size)
{
	for(int i=0;i<Size;i++)Dest[i]=Src[i]/Divisor;//Adjust max to 1
}
//--------------------------------------------------------------------------------------------------------------
void mcv::TabPrintf(float* pVals,int NbVals)
{
	for(int i=0;i<NbVals;i++)printf("%1.2f ",pVals[i]);printf("\n");
}
//--------------------------------------------------------------------------------------------------------------
void mcv::TabPrintf(std::vector<float> DistsAll,const char *VarName,bool isReturn,const char separator,int precision)
{
	char pFormat[32];
	char pFormatAll[128];
	sprintf(pFormat,"%%1.%df",precision);
	sprintf(pFormatAll,"%s%c",pFormat,separator);
	if(VarName)
	{
		printf("%s%c",VarName,separator);
	}
	for(int i=0;i<DistsAll.size();i++)printf(pFormatAll,DistsAll[i]);
	if(isReturn)printf("\n");
}
//--------------------------------------------------------------------------------------------------------------
void mcv::TabPrintf(std::vector<double> DistsAll,const char *VarName,bool isReturn,const char separator,int precision)
{
	char pFormat[32];
	char pFormatAll[128];
	sprintf(pFormat,"%%1.%df",precision);
	sprintf(pFormatAll,"%s%c",pFormat,separator);
	if(VarName)
	{
		printf("%s%c",VarName,separator);
	}
	for(int i=0;i<DistsAll.size();i++)printf(pFormatAll,DistsAll[i]);
	if(isReturn)printf("\n");
}
//--------------------------------------------------------------------------------------------------------------
void mcv::TabPrintf(std::vector<int> DistsAll,const char *VarName,bool isReturn,const char separator)
{
	for(int i=0;i<DistsAll.size();i++)printf("%d%c",DistsAll[i],separator);
	if(isReturn)printf("\n");
}
//--------------------------------------------------------------------------------------------------------------
void mcv::TabPrintf(std::vector<unsigned char> DistsAll,const char *VarName,bool isReturn,const char separator)
{
	for(int i=0;i<DistsAll.size();i++)printf("%d%c",DistsAll[i],separator);
	if(isReturn)printf("\n");
}
//--------------------------------------------------------------------------------------------------------------
void mcv::TabPrintf(std::vector<std::string> Names,const char *VarName,const char separator,bool isReturn)
{
	cout << VarName << separator;
	for(int i=0;i<Names.size();i++)
	{
		cout << Names[i] << separator;
	}
	if(isReturn)
	{
		cout << endl;
	}
}
//--------------------------------------------------------------------------------------------------------------
//void mcv::TabPrintf(std::vector<float> DistsAll)
//{	for(int i=0;i<DistsAll.size();i++)printf("%1.2f ",DistsAll[i]);printf("\n");}
//--------------------------------------------------------------------------------------------------------------
void mcv::TabSet(std::vector<float> &TabVals,float Val)
{
	for(int i=0;i<TabVals.size();i++)TabVals[i] = Val;
}
//--------------------------------------------------------------------------------------------------------------
int mcv::TabConcat(const std::vector<std::vector<float>> &ArrayFloat2D,std::vector<float> &Dest)
{
	int TotalSize = 0;
	int CopiedSize = 0;
	for(int i=0;i<ArrayFloat2D.size();i++)
	{
		TotalSize += (int)ArrayFloat2D[i].size();
	}
	if(Dest.size()!=TotalSize)Dest.resize(TotalSize);
	for(int i=0;i<ArrayFloat2D.size();i++)
	{
		TabCpy(Dest,ArrayFloat2D[i],CopiedSize,ArrayFloat2D[i].size());
		CopiedSize+=(int)ArrayFloat2D[i].size();
	}
	return CopiedSize;
}
//--------------------------------------------------------------------------------------------------------------
bool mcv::TabEqual(const std::vector<float> &Vect1,const std::vector<float> &Vect2)
{
	bool Res = true;
	if(Vect1.size() != Vect2.size())
	{
		Res = false;
	}
	else
	{
		for(int i=0;(i<(int)Vect1.size())&Res;i++)
		{
			if(Vect1[i]!=Vect2[i])
			{
				Res = false;
			}
		}
	}
	return Res;
}
//--------------------------------------------------------------------------------------------------------------
bool mcv::TabEqual(float *pVect1,float *pVect2,int VectSize)
{
	bool Res = true;
	for(int i=0;(i<VectSize)&Res;i++)
	{
		if(pVect1[i]!=pVect2[i])
		{
			Res = false;
		}
	}
	return Res;
}
//--------------------------------------------------------------------------------------------------------------
void mcv::TabMostFrequent_Index(const std::vector<char> &DataVals,int &MostFrequentVal_Index,float &MostFrequent_Average)
{
	std::vector<int> IndexesCount(DataVals.size());
	for(int i=0;i<DataVals.size();i++)
	{
		IndexesCount[i] = 0;
		for(int j=0;j<DataVals.size();j++)
		{
			if(DataVals[j] == DataVals[i])
			{
				IndexesCount[i]++;
			}
		}
	}
	MostFrequentVal_Index = TabMax_Index(IndexesCount);
	MostFrequent_Average = ((float)IndexesCount[MostFrequentVal_Index]) / ((float)DataVals.size());
}
//--------------------------------------------------------------------------------------------------------------
void mcv::SampleAround(float * pDest,float * pSrc,float Scale,int Index)
{
	if(Index==-1)
	{
		for(int i=0;i<17;i++)
		{
			int R = rand();
			float f1 =  (float)( ((float)R/RAND_MAX)-0.5 ) * Scale;
			float res = pSrc[i] + f1;
			if(res>1)
				res=1;
			else if(res<0)
				res=0;

			pDest[i] = res;
		}
	}
	else
	{
		int R = rand();
		float f1 =  (float)( ((float)R/RAND_MAX)-0.5 ) * Scale;
		float res = pSrc[Index] + f1;
		if(res>1)
			res=1;
		else if(res<0)
			res=0;

		pDest[Index] = res;
	}
}
//--------------------------------------------------------------------------------------------------------------
void mcv::ValRand(float &Dest,float minSample,float maxSample)
{
	int R = rand();
	float f1 =  (float)((float)R/RAND_MAX );
	Dest = f1 * (maxSample - minSample) + minSample;
}
//--------------------------------------------------------------------------------------------------------------
void mcv::TabRand(std::vector<float> &pDest,float minSample,float maxSample)
{
	for(int i=0;i<pDest.size();i++)
	{
		int R = rand();
		float f1 =  (float)((float)R/RAND_MAX );
		pDest[i] = f1 * (maxSample - minSample) + minSample;
	}
}
//--------------------------------------------------------------------------------------------------------------
float mcv::DistMax(float V1,float V2,float V3)
{
	float ResMax;
	if(V1>V2) ResMax = V1;else ResMax = V2;
	if(V3>ResMax) ResMax = V3;
	return ResMax;
}
//---------------------------------------------------------------------------------------------------------------
cv::Rect mcv::BoundingRect(cv::Mat Image)
{
	int minX = Image.cols,
		minY = Image.rows,
		maxX = 0,
		maxY = 0;
	for(int i=0;i<Image.cols;i++)
	for(int j=0;j<Image.rows;j++)
	{
		if(PIXEL8UC1(Image,i,j))
		{
			if(minX>i)minX = i;
			if(minY>j)minY = j;
			if(maxX<i)maxX = i;
			if(maxY<j)maxY = j;
		}
	}
	//to avoid negative width height on the case of total black
	if(minX>maxX)minX = maxX;
	if(minY>maxY)minY = maxY;
	return cv::Rect(minX,minY,maxX-minX,maxY-minY);
}

//-------------------------------------------------------------------------------------------------------
float mcv::readLineFloatLine(std::ifstream &File)
{
	float Var;
	std::string line;
	getline(File,line);
	File >> Var;	
	getline(File,line);
	return Var;
}
//---------------------------------------------------------------------------------------------------------------
float mcv::readFloatLine(std::ifstream &File)
{
	float Var;
	std::string line;
	File >> Var;	
	getline(File,line);
	return Var;
}
//--------------------------------------------------------------------------------------------------------------
std::string mcv::GetMainPath(std::string Str)
{
	return Str.substr( 0, Str.find_last_of( '\\' ) +1 );
}
//--------------------------------------------------------------------------------------------------------------
std::string mcv::GetFileName(std::string Str)
{
	return Str.substr(Str.find_last_of( '\\' ) +1,Str.length());
}
//--------------------------------------------------------------------------------------------------------------
std::string mcv::TakeParseTo(std::string &Str,char Separator)
{
	size_t FirstofSep = Str.find_first_of(Separator);
	std::string Parsed = Str.substr(0 , FirstofSep);
	Str = Str.substr(FirstofSep+1 ,Str.length());
	return Parsed;
}
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//-------------MixedFileManager_c-------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------

//	DataInfo_t		DataInfo;
//	YAML::Parser	MParser;
//	std::string FileName;
//--------------------------------------------------------------------------------------------------------------
mcv::MixedFileManager_c::MixedFileManager_c()
{
	DataInfo.DataOffset = 0;
	DataInfo.DataSizeBytes = 0;
	DataInfo.CheckSum = 0;
}
//--------------------------------------------------------------------------------------------------------------
std::stringstream mcv::MixedFileManager_c::Parse(std::string vFileName)
{
	FileName = vFileName;

	std::ifstream inFile(FileName,ios::in | ios::binary);
	//-------------------------------------------------------------------------------------------
	std::stringstream YamlHeader;
	if(inFile.is_open())
	{
		bool isTagFound = false;
		while(!isTagFound && !inFile.eof())
		{
			std::string Line;
			getline(inFile,Line);
			if(Line.compare("***") == 0)
			{
				isTagFound = true;
			}
			else
			{
				YamlHeader << Line << endl;
			}
		}
		assert(isTagFound);
		//-------------------------------------------------------------------------------------------
		std::stringstream DataFileHeader;
		isTagFound = false;
		while(!isTagFound && !inFile.eof())
		{
			std::string Line;
			getline(inFile,Line);
			if(Line.compare("***") == 0)
			{
				isTagFound = true;
			}
			else
			{
				DataFileHeader << Line << endl;
			}
		}
		assert(isTagFound);
		unsigned int begin,end;
		begin = inFile.tellg();//might be @ eof()
		inFile.seekg (0, ios::end);
		end = inFile.tellg();	//ifstream::pos_type size;
		inFile.close();
		DataInfo.DataOffset = begin;
		//ull => 18446744072091646652 in stand of 2677062332 and size_t gets negative val
		unsigned int RealDataSizeBytes = end - begin;
		//----------------------------------------------------------Parse DataFileHeader-------------
		YAML::Parser parser;
		//cout << DataFileHeader.str();
		parser.Load(DataFileHeader);
		YAML::Node doc;
		parser.GetNextDocument(doc);
		doc["DataSizeBytes"] >> DataInfo.DataSizeBytes;
		doc["CheckSumXor_32bits"] >> DataInfo.CheckSum;
		//printf("DataInfo %u ; Real %u\n",DataInfo.DataSizeBytes,RealDataSizeBytes);
		unsigned int V1 = DataInfo.DataSizeBytes;
		unsigned int V2 = RealDataSizeBytes;
		//assert(DataInfo.DataSizeBytes == RealDataSizeBytes); //Doesn't pass the comparison test even if equal
		assert(V1 == V2);
		//assert(DataInfo.DataSizeBytes == RealDataSizeBytes);//when we will cross the 2^32 we will feel it here
	}
	return YamlHeader;
}
//--------------------------------------------------------------------------------------------------------------
void mcv::MixedFileManager_c::LoadData(char*	pAllocatedDest,size_t DataSizeBytes)
{
	if(FileName.empty())
	{
		printf("Parse a File before loading Data\n");
		return;
	}
	std::ifstream inFile(FileName,ios::in | ios::binary);
	inFile.seekg(DataInfo.DataOffset,ios::beg);
	inFile.read(pAllocatedDest,DataSizeBytes);
	int DataCheckSum = CheckSum((int*)pAllocatedDest,DataSizeBytes/4);
	assert(DataCheckSum == DataInfo.CheckSum);
	inFile.close();
}
//--------------------------------------------------------------------------------------------------------------
void mcv::MixedFileManager_c::Save(std::string FileName,char*	pData,size_t DataSizeBytes,std::string UserHeader)
{
	std::string DataFileHeader;

	//assert((DataSizeBytes % sizeofOneData) == 0);// why ? Data could be chars[]
	int checksum = CheckSum((int*)pData,DataSizeBytes/4);//The last bytes not checked by this function
	//int checksum = CheckSum((int*)pData,NbData8);//Would Check Sum absolutely all the bytes

	YAML::Emitter MEmitter;
	MEmitter << YAML::BeginMap;
	MEmitter << YAML::Key << "DataSizeBytes";
	MEmitter << YAML::Value << DataSizeBytes;
	MEmitter << YAML::Key << "CheckSumXor_32bits";
	MEmitter << YAML::Value << YAML::Hex << checksum;
	MEmitter << YAML::EndMap;
	DataFileHeader = MEmitter.c_str();


	std::ofstream outFile(FileName,ios::out | ios::binary | ios::trunc);

	outFile << UserHeader << endl;
	outFile << "***" << endl;//Separate between user Header and DataFileHeader
	outFile << DataFileHeader << endl;
	outFile << "***" << endl;//The end of the YAML File
	//outFile << "This is my personal Data Not parsed by YAML" << endl;
	outFile.write(pData,DataSizeBytes);
}

//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
