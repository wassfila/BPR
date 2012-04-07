
#include "DetectorBPR.h"

#include <iostream>
#include <fstream>

#include <ppl.h>

using namespace hpr;
using namespace mcv;

#define MAX_CHAR 512

#define PI 3.1415926535897932384626433832795f
#define PI_div_2 1.5707963267948966192313216916398f
#define PI_div_12 0.26179938779914943653855361527329f
#define PIx2 6.283185307179586476925286766559f

//---------------------------------------------------------------------------------------------------------------
INLINE float GetWeight_Body(float InitialWeight,float WeightedPart)
{
	if(WeightedPart == PART_BODY)
	{
		return InitialWeight*=2;
	}
	else if(WeightedPart == PART_SHOULDERS)// || (WeightedPart == PART_UNCKNOWNBODYPART)) Uncknown Not liked goes to /2
	{
		return InitialWeight;
	}
	else//Another Part
	{
		return InitialWeight/2;
	}
}
//---------------------------------------------------------------------------------------------------------------
INLINE float GetWeight_Elbow(float InitialWeight,float WeightedPart)
{
	if((WeightedPart == PART_SHOULDERS) || (WeightedPart == PART_ELBOW))
	{
		return InitialWeight*=2;
	}
	else if((WeightedPart == PART_BODY) || (WeightedPart == PART_HANDS))// || (WeightedPart == PART_UNCKNOWNBODYPART))
	{
		return InitialWeight;
	}
	else//Another Part
	{
		return InitialWeight/2;
	}
}
//---------------------------------------------------------------------------------------------------------------
INLINE float GetWeight_Hand(float InitialWeight,float WeightedPart)
{
	if((WeightedPart == PART_ELBOW) || (WeightedPart == PART_HANDS))
	{
		return InitialWeight*=2;
	}
	else if(WeightedPart == PART_SHOULDERS)// || (WeightedPart == PART_UNCKNOWNBODYPART))
	{
		return InitialWeight;
	}
	else//Another Part
	{
		return InitialWeight/2;
	}
}
//---------------------------------------------------------------------------------------------------------------
INLINE float GetWeight_Knee(float InitialWeight,float WeightedPart)
{
	if((WeightedPart == PART_BODY) || (WeightedPart == PART_KNEE))
	{
		return InitialWeight*=2;
	}
	else if((WeightedPart == PART_FEET) || (WeightedPart == PART_UNCKNOWNBODYPART))
	{
		return InitialWeight;
	}
	else//Another Part
	{
		return InitialWeight/2;
	}
}
//---------------------------------------------------------------------------------------------------------------
INLINE float GetWeight_Feet(float InitialWeight,float WeightedPart)
{
	if((WeightedPart == PART_FEET) || (WeightedPart == PART_KNEE))
	{
		return InitialWeight*=2;
	}
	else if(WeightedPart == PART_UNCKNOWNBODYPART)
	{
		return InitialWeight;
	}
	else//Another Part
	{
		return InitialWeight/2;
	}
}
//---------------------------------------------------------------------------------------------------------------

INLINE float GetWeightOfClass(float InitialWeight,uchar CurrentPart,uchar WeightedPart)
{
	float Res = 0;
	switch(CurrentPart)
	{
		case PART_BODY:		Res = GetWeight_Body(InitialWeight,WeightedPart);		break;
		case PART_ELBOW:	Res = GetWeight_Elbow(InitialWeight,WeightedPart);		break;
		case PART_HANDS:	Res = GetWeight_Hand(InitialWeight,WeightedPart);		break;
		case PART_KNEE:		Res = GetWeight_Knee(InitialWeight,WeightedPart);		break;
		case PART_FEET:		Res = GetWeight_Feet(InitialWeight,WeightedPart);		break;
	}
	return Res;
}

//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
Sgn_s::Sgn_s()
{
	NbDiv = 1;
	Distcm = 0;
	AngleOffset = 0.0f;	//Teta - [0,2Pi] - Goes on (x,y)
	AngleLatitude = 0.0f;//Beta - [-Pi/2,Pi/2]
	NbDivLat = 1;
}
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//								BasicDescriptor_c
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------
void BasicDescriptor_c::clear()
{
	SignsList.clear();
	Vects.clear();
	Size_cm = 0.0f;
	MinVectsDist_cm = 0.0f;
}
//-------------------------------------------------------------------------------------------------------------
void BasicDescriptor_c::SignsListToVects()
{
	Vects.clear();//Why not, it should
	for(int i=0;i<SignsList.size();i++)
	{
		float dTeta = 2*PI / SignsList[i].NbDiv;
		for(int j=0;j<SignsList[i].NbDiv;j++)
		{
			float Teta = SignsList[i].AngleOffset + j*dTeta;
			float dPhi = PI / SignsList[i].NbDivLat;
			for(int k=0;k<=SignsList[i].NbDivLat;k++)
			{
				float Phi = SignsList[i].AngleLatitude + k*dPhi;
				Vector3f V;
				V.x() = SignsList[i].Distcm * cos(Phi) * sin(Teta);
				V.y() = SignsList[i].Distcm * cos(Phi) * cos(Teta);
				V.z() = SignsList[i].Distcm * sin(Phi);
				//Check proximity to others to see if it's needed to be added
				float dist = FLT_MAX;//for the case when the vect is empty
				g3d::GetNearestDist(Vects,V,dist);
				if(dist > MinVectsDist_cm)//less than usual vox & pix size, then not necessary
				{
					Vects.push_back(V);
				}
			}
		}
	}
	assert(!Vects.empty());
	Size_cm = Vects[0].norm();
	for(int i=1;i<Vects.size();i++)
	{
		float Norm = Vects[i].norm();
		if(Size_cm < Norm)Size_cm = Norm;
	}
	assert(Size_cm > 0.0f);//cause the user needs Scale_pxlpercm, and there's no sense on a one point descriptor
}
//-------------------------------------------------------------------------------------------------------------
float BasicDescriptor_c::GetSize_cm()
{
	if(Size_cm == 0.0f)
	{
		SignsListToVects();
	}
	assert(Size_cm>0.0f);
	return Size_cm;
}
//--------------------------------------------------------------------------------------------------------------
//YAML::Emitter& operator << (YAML::Emitter& out, const Sgn_s Sign) : DO NOT WORK when using emit << std::vector<Sgn_s>
//if relying on the std::vector templatisation, the following error is returned
//error C2678: binary '<<' : no operator found which takes a left-hand operand of type 'YAML::Emitter'
//while trying to match the argument list '(YAML::Emitter, const hpr::Sgn_s)'
YAML::Emitter& operator << (YAML::Emitter& out, const std::vector<Sgn_s> SignsList)
{
	out << YAML::Flow;
	out << YAML::BeginSeq;
	for(int i=0;i<SignsList.size();i++)
	{
		out << YAML::Flow;
		out << YAML::BeginMap;
		if(SignsList[i].AngleOffset != 0.0f)
		{
			out << YAML::Key << "Ang";
			out << YAML::Value << SignsList[i].AngleOffset;
		}
		out << YAML::Key << "cm";
		out << YAML::Value << SignsList[i].Distcm;
		if(SignsList[i].NbDiv > 1)
		{
			out << YAML::Key << "div";
			out << YAML::Value << SignsList[i].NbDiv;
		}
		if(SignsList[i].AngleLatitude != 0.0f)
		{
			out << YAML::Key << "ALat";
			out << YAML::Value << SignsList[i].AngleLatitude;
		}
		if(SignsList[i].NbDivLat > 1)
		{
			out << YAML::Key << "dLat";
			out << YAML::Value << SignsList[i].NbDivLat;
		}
		out << YAML::EndMap;
	}
	out << YAML::EndSeq;
	return out;

	return out;
}
//--------------------------------------------------------------------------------------------------------------
YAML::Emitter& operator<<(YAML::Emitter& out, const BasicDescriptor_c& PxS)
{
	//out << YAML::BeginMap;------------------------------No duplicate Begin End Map to avoid duplicate docs
	out << YAML::Key	<< "GeneralParams";
	out << YAML::Value;//The Map
		out << YAML::BeginMap;
		if(PxS.Vects.size() != 0)
		{
			out << YAML::Key << "NbVects";
			out << YAML::Value << PxS.Vects.size();
		}
		if(PxS.SignsList.size() != 0)
		{
			out << YAML::Key << "SignsList";
			out << YAML::Value << PxS.SignsList;
		}
		else if(PxS.Vects.size() != 0)
		{
			out << YAML::Key << "VectorsList";
			out << YAML::Value << PxS.Vects;
		}
		if(PxS.MinVectsDist_cm!=0.0f)
		{
			out << YAML::Key << "MinVectsDist_cm";
			out << YAML::Value << PxS.MinVectsDist_cm;
		}
		out << YAML::EndMap;
	//out << YAML::EndMap;
	return out;
}
//--------------------------------------------------------------------------------------------------------------
ostream& operator<<(ostream &out, const BasicDescriptor_c& PxS)
{
	YAML::Emitter Emit;
	Emit << PxS;
	out << Emit.c_str() << endl;
	return out;
}
//--------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------Parse
void operator >> (const YAML::Node& node, Sgn_s& Sign)
{
	Sign.NbDiv = 1;
	if(const YAML::Node *pNbDivNode = node.FindValue("div"))
	{
		*pNbDivNode >> Sign.NbDiv;
	}
	node["cm"] >> Sign.Distcm;
	Sign.AngleOffset = 0.0f;
	if(const YAML::Node *pAngNode = node.FindValue("Ang"))
	{
		*pAngNode >> Sign.AngleOffset;
	}
	Sign.AngleLatitude = 0.0f;
	if(const YAML::Node *pAngNode = node.FindValue("ALat"))
	{
		*pAngNode >> Sign.AngleLatitude;
	}
	Sign.NbDivLat = 1;
	if(const YAML::Node *pAngNode = node.FindValue("dLat"))
	{
		*pAngNode >> Sign.NbDivLat;
	}
}
//---------------------------------------------------------------------------------------------Parse
void operator >> (const YAML::Node& node, BasicDescriptor_c& PxS)
{
	PxS.clear();
	const YAML::Node& GPNode = node["GeneralParams"];
	//First Look if we have the SignsList :
	//std::cout << GPNode;
	if(GPNode.FindValue("MinVectsDist_cm"))
	{
		GPNode["MinVectsDist_cm"] >> PxS.MinVectsDist_cm;
		//std::string VS = GPNode["MinVectsDist_cm"].to<std::string>();		PxS.MinVectsDist_cm = atof(VS.c_str());
	}
	else
	{
		PxS.MinVectsDist_cm = 0.0f;
	}

	if(const YAML::Node *pSgNode = GPNode.FindValue("SignsList"))
	{
		for(int i=0;i<(*pSgNode).size();i++)
		{
			Sgn_s Sign;
			(*pSgNode)[i] >> Sign;
			PxS.SignsList.push_back(Sign);
		}
	}
	else
	{
		const YAML::Node *pVectsNode = GPNode.FindValue("VectorsList");
		for(int i=0;i<(*pVectsNode).size();i++)
		{
			Vector3f V;
			(*pVectsNode)[i] >> V;
			PxS.Vects.push_back(V);
		}
	}

}
//--------------------------------------------------------------------------------------------------------------
void BasicDescriptor_c::load(const std::string &FileName)//Act on signs only
{
	std::ifstream inFile(FileName);
    YAML::Parser parser(inFile);
	YAML::Node doc;
	parser.GetNextDocument(doc);
	doc >> (*this);
	if(SignsList.empty() && Vects.empty())
	{
		printf("Couldn't load PixelSign from file (%s)\n",FileName.c_str());
	}
	else
	{
		printf("PixelSign loaded from file (%s)\n",FileName.c_str());
	}
}
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//								PixSign_c
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
PixSign_c::PixSign_c()
{
	clear();
}
//-------------------------------------------------------------------------------------------------------------
void PixSign_c::clear()
{
	BasicDescriptor_c::clear();
	PixOffsets.clear();
	ImgW = 0;
	ImgH = 0;
	SizeR_px = 0.0f;
	Pxlpercm = 0;
	BKG_Default = 1000;
}
//--------------------------------------------------------------------------------------------------------------
void PixSign_c::ScaleToImage(float NewScale_pxlpercm,const cv::Mat &Image)//Adds the correct offsets
{
	Pxlpercm = NewScale_pxlpercm;//just for record purpose
	ImgW = Image.cols;
	ImgH = Image.rows;
	PixOffsets.clear();
	if(Vects.empty())
	{
		assert(!SignsList.empty());
		SignsListToVects();
	}
	SizeR_px = Pxlpercm * GetSize_cm();//compute it from the vectors List
	for(int i=0;i<Vects.size();i++)
	{
		int Ix = (int)ceil(Vects[i].x() * Pxlpercm - 0.5f);
		int Iy = (int)ceil(Vects[i].y() * Pxlpercm - 0.5f);
		PixOffsets.push_back(Iy * Image.cols + Ix );//Bug of the Year !!!!!!!!!!!   (int)(floatY * ImgW + floatX);
	}
}
//----------------------------------------------------------------------------------------------------------Emit
YAML::Emitter& operator<<(YAML::Emitter& out, const PixSign_c& PxS)
{
	out << YAML::BeginMap;
	out << (BasicDescriptor_c&)PxS;//doesn't matter if the next is on a different doc, as we won't parse it anyway, just user info
	if(PxS.SizeR_px != 0.0f)//It has been scaled to an Image
	{
		//out << YAML::BeginMap;//----------------The continuation of the same Map
		out << YAML::Key << "ScaleToImageParams";
		out << YAML::Value;
			out << YAML::BeginMap;
			out << YAML::Key << "ImgW";
			out << YAML::Value << PxS.ImgW;
			out << YAML::Key << "ImgH";
			out << YAML::Value << PxS.ImgH;
			out << YAML::Key << "ScalePxlpercm";
			out << YAML::Value << PxS.Pxlpercm;
			out << YAML::Key << "SizePx";
			out << YAML::Value << PxS.SizeR_px;
			out << YAML::Key << "PixOffsets";
			out << YAML::Value << YAML::Flow << PxS.PixOffsets;
			out << YAML::EndMap;
		//out << YAML::EndMap;
	}
	out << YAML::EndMap;
	return out;
}
//---------------------------------------------------------------------------------------------------------Parse
void operator >> (const YAML::Node& node, PixSign_c& PxS)
{
	//No Need to parse the other variables (usecaseParams), they're just for debug and change with scale
	PxS.clear();//So that we add signs
	const YAML::Node& SNode = node["GeneralParams"];
	//SNode >> (BasicDescriptor_c)PxS;//This way the PxS is Kept empty
	//PxS  = (PixSign_c)PS;//cannot cast of course
	//SNode >> PxS;//Exec Error
	BasicDescriptor_c PS;
	SNode >> PS;
	PxS.SignsList = PS.SignsList;
	PxS.Vects = PS.Vects;
}
//--------------------------------------------------------------------------------------------------------------
void PixSign_c::save(const std::string &FileName)
{
	std::ofstream outFile(FileName,ios::out | ios::trunc);
	YAML::Emitter Emit;
	Emit << (*this)	;
	outFile << Emit.c_str();
	printf("File (%s) saved\n",FileName.c_str());
}
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
void PixSign_c::PixSigns2D(const cv::Mat &Img,int CentralPixelIdx,float*pVals)//inline won nothing less than 1%
{
	int DataSize = (int)(Img.dataend - Img.datastart);//possible loss of data
	for(int i=0;i<PixOffsets.size();i++)
	{
		int PixToPick = CentralPixelIdx + PixOffsets[i];
		if((PixToPick<0) || (PixToPick>=DataSize))
		{
			pVals[i] = 0;
		}
		else
		{
			uchar Val = *(Img.datastart + PixToPick);
			if(Val > 5)pVals[i] = 1.0f;
			else pVals[i] = 0;
		}
	}
}
//--------------------------------------------------------------------------------------------------------------
void PixSign_c::PixSignsDepth(const cv::Mat &ImgSilhouette,const cv::Mat &DepthImg24,int CentralPixelIdx,float*pVals)
{
	unsigned short RefVal;
	DATA24U16Low(DepthImg24,CentralPixelIdx,RefVal);
	int ImageNbPix = DepthImg24.cols * DepthImg24.rows;
	for(int i=0;i<PixOffsets.size();i++)
	{
		//if( (Ix < 0) || (Ix >= Image.cols) || (Iy < 0) || (Iy >= Image.rows) )
		int PxIdx = CentralPixelIdx + PixOffsets[i];//to check if with pointer we win something
		//not perfect, just approximation but we assume that a silhouette image is always here
		//and the pixels out of scope should fall out of the silhouette as it covers rarely half of the image or more
		//nor the descriptor does
		if( (PxIdx < 0) || (PxIdx >= ImageNbPix) )//pseudo outside the image
		{
			(*pVals) = BKG_Default;//1000;//2 meters is a value supposed maximal for diff so not to break the variance of the other variables
		}
		else if(DATA8UC1(ImgSilhouette,PxIdx) == 0)
		{
			(*pVals) = BKG_Default;//1000;//2 meters is a value supposed maximal for diff so not to break the variance of the other variables
		}
		else
		{
			unsigned short Val;
			DATA24U16Low(DepthImg24,PxIdx,Val);
			(*pVals) = (float)(RefVal - Val);//difference convex positive
		}
		pVals++;
	}
}
//--------------------------------------------------------------------------------------------------------------
void PixSign_c::PixSignsDepthDraw(const cv::Mat &DepthImg24,int CentralPixelIdx)
{
	cv::Mat DebugDepthImg;
	DepthImg24.copyTo(DebugDepthImg);
	//DebugDepthImg.setTo(cv::Scalar(0,0,0));

	int ImageNbPix = (int)(DepthImg24.cols * DepthImg24.rows);//possible loss of data

	uchar* pRefVal = DebugDepthImg.datastart + CentralPixelIdx*3;
	(*pRefVal++) = 128;
	(*pRefVal++) = 255;
	(*pRefVal) = 128;

	int x = CentralPixelIdx % DebugDepthImg.cols;
	int y = CentralPixelIdx / DebugDepthImg.cols;
	cv::circle(DebugDepthImg,cv::Point(x,y),(int)SizeR_px-1,cv::Scalar(0,0,0),2);
	cv::circle(DebugDepthImg,cv::Point(x,y),(int)SizeR_px+1,cv::Scalar(255,255,255),2);
	for(int i=0;i<PixOffsets.size();i++)
	{
		//-----------------------------------------------------------------------------------------------------
		int PxIdx = CentralPixelIdx + PixOffsets[i];//to check if with pointer we win something
		uchar* pValR = DepthImg24.datastart + PxIdx*3;
		uchar* pVal = DebugDepthImg.datastart + PxIdx*3;
		if( (PxIdx >= 0) && (PxIdx < ImageNbPix) )
		{
			(*pVal++) = 255-(*pValR++);
			(*pVal++) = 255-(*pValR++);
			(*pVal) = 255-(*pValR);
		}
		//-----------------------------------------------------------------------------------------------------
		PxIdx = CentralPixelIdx + PixOffsets[i]+1;//to check if with pointer we win something
		pValR = DepthImg24.datastart + PxIdx*3;
		pVal = DebugDepthImg.datastart + PxIdx*3;
		if( (PxIdx >= 0) && (PxIdx < ImageNbPix) )
		{
			(*pVal++) = 255-(*pValR++);
			(*pVal++) = 255-(*pValR++);
			(*pVal) = 255-(*pValR);
		}
		//-----------------------------------------------------------------------------------------------------
		PxIdx = CentralPixelIdx + PixOffsets[i]-1;//to check if with pointer we win something
		pValR = DepthImg24.datastart + PxIdx*3;
		pVal = DebugDepthImg.datastart + PxIdx*3;
		if( (PxIdx >= 0) && (PxIdx < ImageNbPix) )
		{
			(*pVal++) = 255-(*pValR++);
			(*pVal++) = 255-(*pValR++);
			(*pVal) = 255-(*pValR);
		}
		//-----------------------------------------------------------------------------------------------------
		PxIdx = CentralPixelIdx + PixOffsets[i] + DebugDepthImg.cols;//to check if with pointer we win something
		pValR = DepthImg24.datastart + PxIdx*3;
		pVal = DebugDepthImg.datastart + PxIdx*3;
		if( (PxIdx >= 0) && (PxIdx < ImageNbPix) )
		{
			(*pVal++) = 255-(*pValR++);
			(*pVal++) = 255-(*pValR++);
			(*pVal) = 255-(*pValR);
		}
		//-----------------------------------------------------------------------------------------------------
		PxIdx = CentralPixelIdx + PixOffsets[i] - DebugDepthImg.cols;//to check if with pointer we win something
		pValR = DepthImg24.datastart + PxIdx*3;
		pVal = DebugDepthImg.datastart + PxIdx*3;
		if( (PxIdx >= 0) && (PxIdx < ImageNbPix) )
		{
			(*pVal++) = 255-(*pValR++);
			(*pVal++) = 255-(*pValR++);
			(*pVal) = 255-(*pValR);
		}
	}

	//cv::imshow("DepthImg24",DepthImg24);
	cv::imshow("DebugDepthImg",DebugDepthImg);cv::waitKey(20);
}
//-------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//							VoxSign_3D_c
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
VoxSign_3D_c::VoxSign_3D_c()
{
	clear();
}
//----------------------------------------------------------------------------------------------------------Emit
void VoxSign_3D_c::clear()
{
	Vx = 0;
	Vy = 0;
	Vz = 0;
	pVox = NULL;
}
//----------------------------------------------------------------------------------------------------------Emit
YAML::Emitter& operator<<(YAML::Emitter& out, const VoxSign_3D_c& VxS)
{
	out << YAML::BeginMap;
	out << (BasicDescriptor_c&)VxS;//doesn't matter if the next is on a different doc, as we won't parse it anyway, just user info
	if(VxS.Size_cm != 0.0f)//It has been scaled to an Image
	{
		//out << YAML::BeginMap;//----------------The continuation of the same Map
		out << YAML::Key << "ScaleToVoxParams";
		out << YAML::Value;
			out << YAML::BeginMap;
			out << YAML::Key << "Vx";
			out << YAML::Value << VxS.Vx;
			out << YAML::Key << "Vy";
			out << YAML::Value << VxS.Vy;
			out << YAML::Key << "Vz";
			out << YAML::Value << VxS.Vz;
			out << YAML::Key << "VoxSize_cm";
			out << YAML::Value << VxS.Size_cm;
			out << YAML::Key << "VoxOffsets";
			out << YAML::Value << YAML::Flow << VxS.VoxOffsets;
			out << YAML::EndMap;
		//out << YAML::EndMap;
	}
	out << YAML::EndMap;
	return out;
}
//-------------------------------------------------------------------------------------------------------------
ostream& operator<<(ostream &out, const hpr::VoxSign_3D_c& VxS)
{
	YAML::Emitter Yout;
	Yout << VxS;
	out << Yout.c_str();
	return out;
}
//-------------------------------------------------------------------------------------------------------------
void VoxSign_3D_c::FitOffsetsToVox(g3d::VoxelSpace_c* pvVox)//equivalent to the 2D Scale and fit
{
	pVox = pvVox;
	if(Size_cm == 0.0f)//first time only after load or clear and set
	{
		SignsListToVects();
	}
	Vx = pVox->Xv;
	Vy = pVox->Yv;
	Vz = pVox->Zv;
	//LastVoxOffsetJustForRender = ;
	VoxOffsets.resize(Vects.size());
	for(int i=0;i<Vects.size();i++)
	{
		VoxOffsets[i] = pVox->VectToOffset(Vects[i]);//Relative Vect to Relative Offset
	}
}
//-------------------------------------------------------------------------------------------------------------
void VoxSign_3D_c::VoxSigns3D(int VoxOffset,float*pVals)//on the Vox Taken at FitOffsetsToVox
{
	size_t NbPoints = VoxOffsets.size();
	size_t DataSize = pVox->Data.size();
	int*pVOffsets = &VoxOffsets[0];
	for(size_t i=0;i<NbPoints;i++)
	{
		int CurrentVox = VoxOffset + (*pVOffsets++);
		if((CurrentVox < 0)||(CurrentVox>=DataSize))
		{
			(*pVals++) = 0.0f;
		}
		else
		{
			(*pVals++) = (float)(pVox->Data[CurrentVox]);
		}
	}
}
//-------------------------------------------------------------------------------------------------------------
void VoxSign_3D_c::VoxSigns3D(int VoxOffset,unsigned char*pBitVals)
{
}
//-------------------------------------------------------------------------------------------------------------
void VoxSign_3D_c::save(const std::string &FileName)//saves specific info but loads only General ones
{
}
//-------------------------------------------------------------------------------------------------------------
void VoxSign_3D_c::Render(Soft3DCamera_c &Cam)//Renderable_c overload
{
	Vector3f BoxCenter = pVox->Box.GetCenter();
	for(size_t i=0;i<Vects.size();i++)
	{
		Vector3f PToRender = BoxCenter + Vects[i];
		if(pVox->Box.isInside(PToRender))
		{
			if((*pVox)(PToRender))
			{
				Cam.DrawPoint3D(V3To4(PToRender),cv::Scalar(0,0,255));
			}
			else
			{
				Cam.DrawPoint3D(V3To4(PToRender),cv::Scalar(255,0,0));
			}
		}
		else
		{
			Cam.DrawPoint3D(V3To4(PToRender),cv::Scalar(0,0,0));
		}
	}
}
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//								PartsPixels_c
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
int PartsPixels2D_c::ProcessTrain2D(const cv::Mat &ImgSilhouette,VectorsTable_c &TrainTable)//Here we append into Array
{
	assert(ImgSilhouette.type() == CV_8UC1);

	size_t VectSize = PxSign.Vects.size();//PxSign loaded on constructor Default.pxSign
	if(TrainTable.Data.size() == 0)
	{
		TrainTable.Reset((int)VectSize);
	}
	else
	{
		assert(TrainTable.VectSize == VectSize);
	}
	
	int NbPixThisImage = cv::countNonZero(ImgSilhouette);
	size_t OldDataSize = TrainTable.Data.size();
	TrainTable.Data.resize(OldDataSize + NbPixThisImage * VectSize);
	float *pNewData = &TrainTable.Data[OldDataSize];//The New Data starts aftar the end of the old one (0 supported)

	int ImgSize = ImgSilhouette.cols*ImgSilhouette.rows;
	for(int i=0;i<ImgSize;i++)	{
		if(DATA8UC1(ImgSilhouette,i))
		{
			//std::vector<float> PxSignDesc(VectSize);// == PxSign.NbVect
			//PxSign.PixSigns2D(ImgSilhouette,i,&PxSignDesc[0]);
			PxSign.PixSigns2D(ImgSilhouette,i,pNewData);
			pNewData+=VectSize;
			//TrainTable.push(PxSignDesc);
		}
	}
	TrainTable.TypeName = "Desc2DTrain";
	
	return NbPixThisImage;
}
//-------------------------------------------------------------------------------------------------------------
void PartsPixels2D_c::ScaleSignDepth(const cv::Mat &ImgSilhouette,const cv::Mat &ImgDepthmm)
{
	bool isAverageAll = true;
	int MinNbPoints = 20;
	float AvgRatio = 0.001f;//Homogeneous Sampling - too much, better just take only MinNbPoints

	std::vector<int> Indexes;
	int ImgSize = ImgSilhouette.cols*ImgSilhouette.rows;
	Indexes.reserve(ImgSize);
	for(int i=0;i<ImgSize;i++)	
	{
		if(DATA8UC1(ImgSilhouette,i))
		{
			Indexes.push_back(i);
		}
	}
	int NbPoints = (int)Indexes.size();
	int NbSamplePoints = (int) (NbPoints * AvgRatio);
	if (NbSamplePoints < MinNbPoints)NbSamplePoints = MinNbPoints;
	size_t SampleStep = NbPoints / NbSamplePoints;
	int SumDist = 0;
	cv::Point Center;
	int ActualNbSamples = 0;
	for(size_t i=0;i<NbPoints;i+=SampleStep)
	{
		int Index = Indexes[i];
		unsigned short Val;
		DATA24U16Low(ImgDepthmm,Index,Val);

		SumDist += Val;

		Center.x += Index % ImgDepthmm.cols;
		Center.y += Index / ImgDepthmm.cols;
		ActualNbSamples++;
	}
	assert(ActualNbSamples > 0);
	double AvgDist = SumDist / 10;// CONVERSION FROM mm to cm HERE !!!!!!!!!!!!!!
	AvgDist /= ActualNbSamples;
	Center.x /= ActualNbSamples;
	Center.y /= ActualNbSamples;
	
	//printf("Center (%d,%d)\n",Center.x,Center.y);
	EigenRay Ray = pCam->GetRay(Center);//	Ray.printf("Ray");
	Vector3f Point = Ray.GetPoint((float)AvgDist);//	g3d::printfVector3f(Point,"Point");
	//Extract Samples Points
	//if(PxSign.Size_cm == 0.0f)		PxSign.SignsListToVects();//in GetSize_cm();
	float Size_px = pCam->ProjectSize(Point,PxSign.GetSize_cm());//	printf("PxlSize: %1.3f;  PxSign.SizeR_cm: %1.3f\n",PxlSize,PxSign.SizeR_cm);
	float Scale_pxlpercm = Size_px / PxSign.GetSize_cm();//	printf("Scale_pxlpercm: %1.3f\n",Scale_pxlpercm);
	PxSign.ScaleToImage(Scale_pxlpercm,ImgDepthmm);


	int CenterIndex = Center.x + Center.y * ImgDepthmm.cols;
	//printf("Center : %d %d (%d)\n",Center.x,Center.y,CetnerIndex);

	//PxSign.PixSignsDepthDraw(ImgDepthmm,CenterIndex);



}
//-------------------------------------------------------------------------------------------------------------
int PartsPixels2D_c::ProcessTrainDepth(const cv::Mat &ImgSilhouette,const cv::Mat &ImgDepthmm,VectorsTable_c &TrainDepthTable)
{
	assert(ImgSilhouette.type() == CV_8UC1);
	assert(ImgDepthmm.type() == CV_8UC3);
	assert(ImgSilhouette.rows == ImgDepthmm.rows);
	assert(ImgSilhouette.cols == ImgDepthmm.cols);

	ScaleSignDepth(ImgSilhouette,ImgDepthmm);

	int VectSize = (int)PxSign.Vects.size();//PxSign loaded on constructor Default.pxSign
	if(TrainDepthTable.Data.size() == 0)
	{
		TrainDepthTable.Reset(VectSize);
	}
	else
	{
		assert(TrainDepthTable.VectSize == VectSize);
	}

	//-------------------------------------------------------------------------------Process the Desc on the Whole Image
	int NbPixThisImage = cv::countNonZero(ImgSilhouette);
	size_t OldDataSize = TrainDepthTable.Data.size();
	TrainDepthTable.Data.resize(OldDataSize + NbPixThisImage * VectSize);
	float *pNewData = &TrainDepthTable.Data[OldDataSize];//The New Data starts aftar the end of the old one (0 supported)

	int ImgSize = ImgSilhouette.cols*ImgSilhouette.rows;
	for(int i=0;i<ImgSize;i++)	{
		if(DATA8UC1(ImgSilhouette,i))
		{
			//std::vector<float> PxSignDesc(PxSign.NbVect);
			//PxSign.PixSignsDepth(ImgSilhouette,ImgDepthmm,i,&PxSignDesc[0]);
			PxSign.PixSignsDepth(ImgSilhouette,ImgDepthmm,i,pNewData);
			pNewData +=VectSize;
			//TrainDepthTable.push(PxSignDesc);
		}
	}
	TrainDepthTable.TypeName = "DepthTrain";
	
	return NbPixThisImage;
}
//-------------------------------------------------------------------------------------------------------------
int PartsPixels2D_c::ProcessLabels(const cv::Mat &ImgSilhouette,const cv::Mat &ImgParts,VectorsTable_c &LabelsTable)
{
	assert(ImgSilhouette.type() == CV_8UC1);
	assert(ImgParts.type() == CV_8UC3);
	assert(ImgParts.cols == ImgSilhouette.cols);
	assert(ImgParts.rows == ImgSilhouette.rows);

	if(LabelsTable.size() == 0)
	{
		LabelsTable.Reset(1);//Just the body part classe => 1 float
	}
	else
	{
		assert(LabelsTable.VectSize == 1);
	}
	//-------------------------------------------------------------------------------Process the Desc on the Whole Image
	int NbPixThisImage = cv::countNonZero(ImgSilhouette);

	//LabelsTable.Array.reserve(LabelsTable.Array.size() + NbPixThisImage);
	size_t OldDataSize = LabelsTable.Data.size();
	LabelsTable.Data.resize(OldDataSize + NbPixThisImage * 1);//VectSize == 1
	float *pNewData = &LabelsTable.Data[OldDataSize];//the new starts after the old ends

	int ImgSize = ImgSilhouette.cols*ImgSilhouette.rows;
	for(int i=0;i<ImgSize;i++)	
	{
		if(DATA8UC1(ImgSilhouette,i))
		{
			uchar v1,v2,v3;
			uchar *pPix = ImgParts.datastart + i*3;
			v1 = (*pPix++);
			v2 = (*pPix++);
			v3 = (*pPix);
			(*pNewData++) = (float)pModel->GetClass(ScalarC(v1,v2,v3));
			//LabelsTable.push(&ResClass);
		}
	}
	LabelsTable.TypeName = "LabelResp";

	return NbPixThisImage;
}
//-------------------------------------------------------------------------------------------------------------
int PartsPixels2D_c::MatchLabelsToImg(const cv::Mat &ImgSilhouette,cv::Mat &ImgParts,VectorsTable_c &LabelsTable)
{
	assert(ImgSilhouette.type() == CV_8UC1);
	if((ImgParts.type() != CV_8UC3) || (ImgParts.cols != ImgSilhouette.cols) || (ImgParts.rows != ImgSilhouette.rows))
	{
		ImgParts = cv::Mat(ImgSilhouette.rows,ImgSilhouette.cols,CV_8UC3);
	}
	ImgParts.setTo(cv::Scalar(255,255,255));//Clear Image to write new data !!!!
	int scount = cv::countNonZero(ImgSilhouette);
	assert(scount == LabelsTable.size());
	float *pData = &LabelsTable.Data[0];
	int ImgSize = ImgSilhouette.cols*ImgSilhouette.rows;
	for(int i=0;i<ImgSize;i++)	
	{
		if(DATA8UC1(ImgSilhouette,i))
		{
			ScalarC Color = pModel->GetColor((int)(*pData));
			uchar *pPix = ImgParts.datastart + i*3;
			(*pPix++) = Color.val[0];
			(*pPix++) = Color.val[1];
			(*pPix) = Color.val[2];
			pData++;
		}
	}
	return scount;
}
//-------------------------------------------------------------------------------------------------------------overloads
int PartsPixels2D_c::ProcessTrain2D(const cv::Mat &ImgSilhouette,std::string &FileName)
{
	bool isDTime = true;
	bool isExport = false;//Should be replaced with BPR_EXPORT(FileName,TrainTable,UserHeader);

	VectorsTable_c TrainTable;
	double t;
	if(isDTime)TStart(t);
		int NbPix = ProcessTrain2D(ImgSilhouette,TrainTable);
	if(isDTime)TStop(t,"Process2D()");

	//if(isDTime)TStart(t);
	YAML::Emitter Emit;
	Emit << YAML::BeginDoc << PxSign << YAML::EndDoc;
	std::string UserHeader = Emit.c_str();
	//if(isDTime)TStop(t,"YAML Emitting()");

	if(isDTime)TStart(t);
		TrainTable.save(FileName,UserHeader,false);
	if(isDTime)TStop(t,"save()");

	
	if(isExport)
	{
		if(isDTime)TStart(t);
		std::string TxtFileName = FileName;
		TxtFileName+= ".csv";
		TrainTable.Export(TxtFileName,UserHeader);
		if(isDTime)TStop(t,"Export()");
	}

	return NbPix;
}
//-------------------------------------------------------------------------------------------------------------
int PartsPixels2D_c::ProcessTrainDepth(const cv::Mat &ImgSilhouette,const cv::Mat &ImgDepthmm,std::string &FileName)
{
	bool isDTime = true;
	bool isExport = false;

	VectorsTable_c TrainDepthTable;
	double t;
	if(isDTime)TStart(t);
		int NbPix = ProcessTrainDepth(ImgSilhouette,ImgDepthmm,TrainDepthTable);
	if(isDTime)TStop(t,"ProcessDepth()");
	
	//if(isDTime)TStart(t);
	YAML::Emitter Emit;
	Emit << YAML::BeginDoc << PxSign << YAML::EndDoc;
	std::string UserHeader = Emit.c_str();
	//if(isDTime)TStop(t,"YAML Emitting()");

	if(isDTime)TStart(t);
		TrainDepthTable.save(FileName,UserHeader,true);
	if(isDTime)TStop(t,"save()");

	if(isExport)
	{
		if(isDTime)TStart(t);
		std::string TxtFileName = FileName;
		TxtFileName+= ".csv";
		TrainDepthTable.Export(TxtFileName,UserHeader);
		if(isDTime)TStop(t,"Export()");
	}
	return NbPix;
}
//-------------------------------------------------------------------------------------------------------------
int PartsPixels2D_c::ProcessLabels(const cv::Mat &ImgSilhouette,const cv::Mat &ImgParts,std::string &FileName)
{
	bool isDTime = true;
	bool isExport = false;

	VectorsTable_c LabelsTable;
	double t;

	if(isDTime)TStart(t);
		int NbPix = ProcessLabels(ImgSilhouette,ImgParts,LabelsTable);
	if(isDTime)TStop(t,"ProcessLabels()");

	//if(isDTime)TStart(t);
	YAML::Emitter Yout;
	Yout << YAML::BeginDoc;
	Yout << YAML::BeginMap;
	pModel->outputPartsColors(Yout);
	Yout << YAML::EndMap;
	Yout << YAML::EndDoc;
	//if(isDTime)TStop(t,"YAML Emitting()");

	if(isDTime)TStart(t);
		LabelsTable.save(FileName,Yout.c_str(),true);
	if(isDTime)TStop(t,"save()");

	if(isExport)
	{
		if(isDTime)TStart(t);
		std::string EFName;
		EFName = FileName + ".csv";
		LabelsTable.Export(EFName,Yout.c_str());
		if(isDTime)TStop(t,"Export()");
	}
	return NbPix;
}
//-------------------------------------------------------------------------------------------------------------
int PartsPixels2D_c::MatchLabelsToImg(const cv::Mat &ImgSilhouette,cv::Mat &ImgParts,std::string &FileName)
{
	bool isDTime = true;

	double t;
	if(isDTime)TStart(t);
		VectorsTable_c LabelsTable(FileName);
	if(isDTime)TStop(t,"LoadTable()");
	
	if(isDTime)TStart(t);
		int NbPix = MatchLabelsToImg(ImgSilhouette,ImgParts,LabelsTable);
	if(isDTime)TStop(t,"MatchLabelsToImg()");
	return NbPix;
}
//-------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//									2D MultiView PartsPixels2DMultiView_c
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
PartsPixels2DMultiView_c::PartsPixels2DMultiView_c(S3DEnv_c	*pvS3DEnv,std::string vCfgFileName)
{
	pS3D = pvS3DEnv;
	CfgFileName = vCfgFileName;

	SetUpDescSign("Desc2DSign");
	
}
//--------------------------------------------------------------------------------------------------------------
void PartsPixels2DMultiView_c::SetUpDescSign(const char* TagName)
{
	cv::FileStorage Fs(CfgFileName,cv::FileStorage::READ);
	assert(!Fs[TagName].empty());
	std::string DescFile = (std::string)Fs[TagName];
	std::string MainPath = mcv::GetMainPath(CfgFileName);
	DescFile = MainPath + DescFile;
	PixSign_c PxDescSign;
	PxDescSign.load(DescFile);
	int NbViews = (int)pS3D->Cams.size();
	Parts2D.resize(NbViews);
	for(int i=0;i<NbViews;i++)
	{
		Parts2D[i].pCam = &pS3D->Cams[i];
		Parts2D[i].PxSign = PxDescSign;//The Image width is independent upadted on scale
	}
}
//--------------------------------------------------------------------------------------------------------------
void PartsPixels2DMultiView_c::ScaleSigns(std::vector<cv::Mat> &ImgsSilh)
{
	int NbCams = (int)pS3D->Cams.size();
	assert((int)ImgsSilh.size() == NbCams);
	bool isDebug = false;
	Vector3f P3D_t = pS3D->BlobCenter(ImgsSilh);
	if(isDebug)g3d::printfVector3f(P3D_t,false,"P3D_t");
	for(int i=0;i<NbCams;i++)
	{
		assert((int)ImgsSilh[i].cols == pS3D->Cams[i].SurfWidth);
		if(isDebug)printf("Camera %d\n",i);
		float DescSignSize_cm = Parts2D[i].PxSign.GetSize_cm();
		float Size_px = pS3D->Cams[i].ProjectSize(P3D_t,DescSignSize_cm);
		float Scale_pxlpercm = Size_px / DescSignSize_cm;
		Parts2D[i].PxSign.ScaleToImage(Scale_pxlpercm,ImgsSilh[i]);
		if(isDebug)printf("Scale pxl/cm = %1.2f , ",Scale_pxlpercm);
	}
}
//--------------------------------------------------------------------------------------------------------------
void PartsPixels2DMultiView_c::DebugDisplayPxSign(std::vector<cv::Mat> &ImgsSilh)
{
	int NbCams = (int)pS3D->Cams.size();
	assert((int)ImgsSilh.size() == NbCams);
	
	bool Debug = true;

	Vector3f P3D_t = pS3D->BlobCenter(ImgsSilh);
	g3d::printfVector3f(P3D_t,"P3D_t");

	for(int i=0;i<NbCams;i++)
	{
		cv::Point proj = pS3D->Cams[i].Project(P3D_t);
		//Parts2D[i].PxSign.PixSigns2DDraw();
	}
	pS3D->FillBuffers(ImgsSilh);
	pS3D->DrawReference(g3d::MatrixTranslation(P3D_t.x(),P3D_t.y(),P3D_t.z()),20);
	/*for(int i=0;i<NbCams;i++)
	{
		cv::circle(pS3D->Cams[i].ImgRender,proj,5,mcv::clGreen,2);
	}*/
	pS3D->Display("Debug Predict");
	printf("Debug Predict Display, press key...\n");
	cv::waitKey();
}
//-------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//							VoxDetector_c
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
int VoxDetector_c::ProcessResp3DParts(BodyJointsPose_c& Pose,VoxelSpace_c& Vox,VectorsTable_c &Resp3D)//Set Colors on Vox
{
	UpdateVoxColorTable(Vox);
	int NbVox = ProcessVoxParts(Pose,Vox);
	Vox.ToVectTab(Resp3D);
	Resp3D.TypeName = "Parts3DResp";
	return NbVox;
}
//--------------------------------------------------------------------------------------------------------------
int VoxDetector_c::ProcessResp3DJVects(BodyJointsPose_c& Pose,VoxelSpace_c& Vox,VectorsTable_c &Resp3D)//Set Colors on Vox
{
	bool ThisfunctionisNotReadyYet = true;	assert(ThisfunctionisNotReadyYet);
	Resp3D.TypeName = "JVects3DResp";
	return 0;
}
//--------------------------------------------------------------------------------------------------------------
int VoxDetector_c::ProcessTrain3D(VoxelSpace_c& Vox,VectorsTable_c &Train3D)
{
	assert(!Vox.Data.empty());
	VoxSign.FitOffsetsToVox(&Vox);
#ifdef DEBUG_VOX_SIGN_DISPLAY
	std::cout << Vox.Box;
	S3DFlyCamViewer_c Viewer(pS3D);
	Vox.RenderStep = 5;
	Viewer.AddToRender(&Vox);
	Viewer.AddToRender(&VoxSign);
	std::cout << "NbVects " << VoxSign.Vects.size() << std::endl;
	char Key = cv::waitKey();
	while(Key!='q')
	{
		Viewer.Render();
		Viewer.Cam.DrawBox(Vox.Box);
		Viewer.Display("Anim");
		Viewer.Cam.RotateY(0.02f);
		Key = cv::waitKey(30);
	}
#endif
	size_t VectSize = VoxSign.Vects.size();
	assert(VectSize != 0);//Good !! Catched a real bug of FitOffsetsToVox() call missing !!!!
	if(Train3D.Data.size() == 0)
	{
		Train3D.Reset((int)VectSize);
	}
	else
	{
		assert(Train3D.VectSize == VectSize);
	}
	size_t OldDataSize = Train3D.Data.size();
	Train3D.Data.resize(OldDataSize + Vox.NbFill * VectSize);
	float *pNewData = &Train3D.Data[OldDataSize];//The New Data starts aftar the end of the old one (0 supported)

	unsigned char *pData = &Vox.Data[0];//It's this step that needs the Vox not to be const !!!!
	unsigned char *pDataEnd = (&Vox.Data[Vox.Data.size()-1]) + 1;
	int CurrentVoxIndex = 0;
	while(pData < pDataEnd)
	{
		if(*pData++)
		{
			VoxSign.VoxSigns3D(CurrentVoxIndex,pNewData);
			pNewData+=VectSize;
		}
		CurrentVoxIndex++;
	}
	
	Train3D.TypeName = "Desc3DTrain";

	
	return Vox.NbFill;
}
//--------------------------------------------------------------------------------------------------------------
void VoxDetector_c::SetUpDescSign(const std::string& TagName)
{
	cv::FileStorage Fs(CfgFileName,cv::FileStorage::READ);
	if(!Fs[TagName].empty())
	{
		std::string DescFile = (std::string)Fs[TagName];
		DescFile = mcv::GetMainPath(CfgFileName) + DescFile;
		VoxSign.load(DescFile);
	}
	else
	{
		printf("(%s) Tag is not available on cfg File (%s)\nNo Vox Training capability\n",TagName.c_str(),CfgFileName.c_str());
	}
}
//--------------------------------------------------------------------------------------------------------------
void VoxDetector_c::UpdateVoxColorTable(VoxelSpace_c& Vox)
{
	Vox.ColorTable = pModel->BPartsColors;
}
//--------------------------------------------------------------------------------------------------------------
int VoxDetector_c::ProcessVoxParts(BodyJointsPose_c& Pose,VoxelSpace_c& Vox)
{
#define PARTS_PROPAGATION
#ifdef PARTS_PROPAGATION
	return ProcessVoxPartsPropagation(Pose,Vox);
#else
	return ProcessVoxPartsClosest(Pose,Vox);
#endif
}
//--------------------------------------------------------------------------------------------------------------
int	VoxDetector_c::ProcessVoxPartsClosest(BodyJointsPose_c& Pose,VoxelSpace_c& Vox)
{
	unsigned char *pData = &Vox.Data[0];
	int VCount = 0;
	for(int k=0;k<Vox.Zv;k++)
		for(int j=0;j<Vox.Yv;j++)
			for(int i=0;i<Vox.Xv;i++)
			{
				if(*pData)
				{
					Vector4f VoxPos = Vox.Pos(i,j,k);
					//#define CLOSE_TO_CENTER_10
					#ifdef CLOSE_TO_CENTER_10
					int ClosestPartIndex = pModel->JointToPart(Pose.GetClosestJointIndex_10(VoxPos));
					#else
					int ClosestPartIndex = pModel->JointToPart(Pose.GetClosestJointIndex(VoxPos));
					#endif
					(*pData) = (unsigned char)ClosestPartIndex;//Part index to Table Shift
					VCount++;
				}
				pData++;
				//VIndex++;//not needed
			}
	assert(VCount == Vox.NbFill);
	return VCount;
}
//--------------------------------------------------------------------------------------------------------------
int VoxDetector_c::ProcessVoxPartsPropagation(BodyJointsPose_c& Pose,VoxelSpace_c& Vox)//Set Colors on Vox
{
	bool isDispTime = false;
//#define DEBUG_PROCESSToVOX_Propagation
#ifdef	DEBUG_PROCESSToVOX_Propagation
	S3DFlyCamViewer_c Viewer(pS3D,"VoxTest");
	//Vox.RenderStep = 2;
	Viewer.AddToRender(&Vox);
	Viewer.EnableExport("G:\\PosturesDB\\VirtualDB\\SoftMotion\\Poses\\Filling Voxes\\Imgr%03d.png");
#endif
	//Yes the assumption is made that the vox space is connected
	//Incemination has to be done with the joints indexes cause we allow one id to propagate one step per loop
	//and if many parts have the same Id they won't propagate equally
	//this incemination uses the atClosestNZ function and not simply the position of the body part
	//because the data precision is not good enough and the Joint Center might be outside of the filled vox
	//----------------------------------------------------------incemination
	double t;
	if(isDispTime)TStart(t);
	for(size_t i=0;i<Pose.BodyParts.size();i++)
	{
		Vox.SetAtClosestOne(Pose.BodyParts[i],(unsigned char)i+2);//0 = empty, 1 = Uncknown, 2= First Class, ...
	}
	if(isDispTime)TStop(t,"Incemination");
	if(isDispTime)TStart(t);
#ifdef	DEBUG_PROCESSToVOX_Propagation
	int NHigh = Vox.countHigherThan(1);
	printf("%d Voxes higher than 1\n",NHigh);
#endif
	//----------------------------------------------------------propagation
	int NbChanged = 0;//used in the while test
	int NbTotalChanged = Pose.BodyParts.size();
	int Propagationloops = 0;
	do
	{
		NbChanged = Vox.Propagate();//All the non 1 voxes propagates on the 1 (undefined) voxes
		NbTotalChanged += NbChanged;
		Propagationloops++;
#ifdef	DEBUG_PROCESSToVOX_Propagation
	int prevSize = Vox.ColorTable.size();
	Vox.ColorTable.resize(Pose.BodyParts.size()+2);//0,1 not used
	Viewer.Render("VoxTest");
	Vox.ColorTable.resize(prevSize);
	printf("TotalVoxels: %d\nNbFill: %d changed this loop: %d\n",Vox.Data.size(),Vox.NbFill,NbChanged);
	cv::waitKey(10);
	int NHigh2 = Vox.countHigherThan(1);
	printf("%d Voxes higher than 1\n",NHigh2);
#endif
	}while(NbChanged>0);
	if(isDispTime)TStop(t,"Propagation");

#ifdef	DEBUG_PROCESSToVOX_Propagation
	cv::waitKey();
#endif
	//----------------------------------------------------------conversion to PartsId
	if(isDispTime)TStart(t);
	for(size_t i=0;i<Pose.BodyParts.size();i++)
	{
		int PartId = pModel->JointToPart(i);//inceminate with parts not with Joints Ids
		//Doesn't touch (0 and 1) 0 = empty, 1 = Uncknown, changes from : 2= First Class, ...
		Vox.Replace(i+2,(unsigned char)PartId);
	}
	//Vox.Replace(1,2);//replace the uncknown by the first one (e.g Body)
	if(isDispTime)TStop(t,"Replacement");
#ifdef DEBUG_PROCESSToVOX_Propagation
	Viewer.Render("VoxTest");	printf("Vox After replacement\n");
	cv::waitKey();
#endif

	if(isDispTime)printf("Propagation took %d loops\n",Propagationloops);
	if(NbTotalChanged != Vox.NbFill)
	{
		if(isDispTime)printf("Warning!! Total Vox(%d) from which %d are left uncknown\n",Vox.NbFill,Vox.NbFill - NbTotalChanged);
	}// we did well set classes of all voxels !!! - we'll see if we have open Voxes
	//or else you'll just have some uncknown left

	return NbTotalChanged;
}
//--------------------------------------------------------------------------------------------------------------
void VoxDetector_c::ProcessVoxParts2Centers(VoxelSpace_c& Vox,std::vector<Vector3f>& BodyParts)//Starts from 2
{
//#define DEBUG_ProcessVoxParts2Centers
	float MaxRatio = 0.8f;
	//float minRatio = 0.01f;//no min Ratio but minNb
	int minNbVoxes = 15;//less than that it could simply have been a mislabeling
	float BoxDataScatterBet = 1.2f;//we assume the data ratio found would take two sizes of it's count
	int minBoxSide = 15;
#ifdef DEBUG_ProcessVoxParts2Centers
		printf("MaxRatio(%1.2f) minNbVoxes(%d) BoxDataScatterBet(%1.2f)\n",MaxRatio,minNbVoxes,BoxDataScatterBet);
#endif
	assert(Vox.NbFill!=0);
	unsigned char *pData = &Vox.Data[0];
	std::vector<int> NbPartId;
	std::vector<int> Mean_i;
	std::vector<int> Mean_j;
	std::vector<int> Mean_k;
	//First Scan to define all classes and the first iteration means and volume
	//--------------------------------------------------------------------------
	for(int k=0;k<Vox.Zv;k++)
		for(int j=0;j<Vox.Yv;j++)
			for(int i=0;i<Vox.Xv;i++)
			{
				if((*pData)>1)
				{
					if((*pData)>=NbPartId.size())
					{
						NbPartId.resize((*pData)+1);
						Mean_i.resize((*pData)+1);
						Mean_j.resize((*pData)+1);
						Mean_k.resize((*pData)+1);
					}
					NbPartId[(*pData)]++;
					Mean_i[(*pData)]+=i;
					Mean_j[(*pData)]+=j;
					Mean_k[(*pData)]+=k;
				}
				pData++;
			}
	//---------------------------------------------------------------------------------
	//loop on every Part
	BodyParts.resize(NbPartId.size());
	#ifdef DEBUG_ProcessVoxParts2Centers
	mcv::TabPrintf(NbPartId,"NbPartId");
	mcv::TabPrintf(Mean_i,"Mean_i");
	mcv::TabPrintf(Mean_j,"Mean_j");
	mcv::TabPrintf(Mean_k,"Mean_k");
	std::cout << "Vox.Box:"<< std::endl << Vox.Box;
	#endif
	for(size_t Pid=2;Pid<NbPartId.size();Pid++)//misses 0, 1
	{
		#ifdef DEBUG_ProcessVoxParts2Centers
		printf("Pid(%d)-----------------------------------------\n",Pid);
		#endif
		int Center_i,Center_j,Center_k;
		int NbTotal = Vox.Xv*Vox.Yv*Vox.Zv;//(ei-si) * (ej-sj) * (ek-sk);
		float Ratio = ((float)NbPartId[Pid]) / ((float)(NbTotal));
		float prevRatio = 0.0f;//hard to do worse
		float BoxCubicVol = Ratio * NbTotal * BoxDataScatterBet;//unit voxsize^3
		int BoxSide = (int)ceil(pow(( double )BoxCubicVol,( double )1/3)-0.5);
		if(BoxSide<minBoxSide)BoxSide=minBoxSide;
		int Nbloop = 0;
		#ifdef DEBUG_ProcessVoxParts2Centers
		printf("Before loop; NbPartId(%d) / NbTotal(%d) = Ratio(%1.4f) ; NextBoxVol(%1.2f) NextSide(%d)\n",NbPartId[Pid],NbTotal,Ratio,BoxCubicVol,BoxSide);
		#endif
		if(NbPartId[Pid]>0)
		{
			Center_i = (int)ceil(((float)Mean_i[Pid])/NbPartId[Pid]-0.5);
			Center_j = (int)ceil(((float)Mean_j[Pid])/NbPartId[Pid]-0.5);
			Center_k = (int)ceil(((float)Mean_k[Pid])/NbPartId[Pid]-0.5);
		}
		else
		{
			Center_i = Vox.Xv / 2;
			Center_j = Vox.Yv / 2;
			Center_k = Vox.Zv / 2;
		}
		#ifdef DEBUG_ProcessVoxParts2Centers
		printf("Before loop;Center_i(%d) Center_j(%d) Center_k(%d) - BoxSide(%d) BoxCubicVol(%1.2f)\n",Center_i,Center_j,Center_k,BoxSide,BoxCubicVol);
		#endif
		while( (Ratio > prevRatio) && (Ratio < MaxRatio) && (NbPartId[Pid] > minNbVoxes) )//&& (BoxSide > minBoxSide) )
		{
			Center_i = (int)ceil(((float)Mean_i[Pid])/NbPartId[Pid]-0.5);
			Center_j = (int)ceil(((float)Mean_j[Pid])/NbPartId[Pid]-0.5);
			Center_k = (int)ceil(((float)Mean_k[Pid])/NbPartId[Pid]-0.5);
			//#ifdef DEBUG_ProcessVoxParts2Centers
			//printf("to loop(%d) : Center_i(%d) Center_j(%d) Center_k(%d) - BoxSide(%d) BoxCubicVol(%1.2f)\n",Nbloop,Center_i,Center_j,Center_k,BoxSide,BoxCubicVol);
			//#endif

			int si = Center_i-BoxSide/2;	if(si<0)si=0;
			int sj = Center_j-BoxSide/2;	if(sj<0)sj=0;
			int sk = Center_k-BoxSide/2;	if(sk<0)sk=0;
			int ei = Center_i+BoxSide/2;	if(ei>Vox.Xv)ei=Vox.Xv;
			int ej = Center_j+BoxSide/2;	if(ej>Vox.Yv)ej=Vox.Yv;
			int ek = Center_k+BoxSide/2;	if(ek>Vox.Zv)ek=Vox.Zv;
			#ifdef DEBUG_ProcessVoxParts2Centers
			EigenBox3D DispBox;
			DispBox.Low = Vector3f(Center_i-BoxSide,Center_j-BoxSide,Center_k-BoxSide);
			DispBox.High = Vector3f(Center_i+BoxSide,Center_j+BoxSide,Center_k+BoxSide);
			DispBox.HasPointAdded = true;
			#endif
			//process : * NbPartId[i]
			//--------------------------------------------------------------------------------------
			NbPartId[Pid] = 0;//clear to Sum again
			Mean_i[Pid] = 0;
			Mean_j[Pid] = 0;
			Mean_k[Pid] = 0;
			for(int k=sk;k<ek;k++)
				for(int j=sj;j<ej;j++)
					for(int i=si;i<ei;i++)
					{
						unsigned char cData = Vox(i,j,k);
						if(cData == Pid)
						{
							NbPartId[Pid]++;
							Mean_i[Pid]+=i;
							Mean_j[Pid]+=j;
							Mean_k[Pid]+=k;
						}
					}
			//--------------------------------------------------------------------------------------
			NbTotal = (ei-si) * (ej-sj) * (ek-sk);
			prevRatio = Ratio;
			Ratio = ((float)NbPartId[Pid]) / ((float)(NbTotal));
			BoxCubicVol = Ratio * NbTotal * BoxDataScatterBet;//unit voxsize^3
			BoxSide = (int)ceil(pow(( double )BoxCubicVol,( double )1/3)-0.5);
			if(BoxSide<minBoxSide)BoxSide=minBoxSide;
			#ifdef DEBUG_ProcessVoxParts2Centers
			printf(		"after loop:(%d) [%d,%d,%d]; NbPartId(%d) / NbTotal(%d) = Ratio(%1.4f) ; NextBoxVol(%1.2f) NextSide(%d)\n",
						Nbloop,Center_i,Center_j,Center_k,NbPartId[Pid],NbTotal,Ratio,BoxCubicVol,BoxSide);
			#endif
			Nbloop++;
		}



		BodyParts[Pid] = V4To3(Vox.Pos(Center_i,Center_j,Center_k));
	}
	//--------------------------------------------------------------------------
	//Compute RatioParts
	//check if stop condition is met (box size, NbParts, Ratio)
	//Compute si,ei j,k for Parts
	//loop i,j,k to compute New NbParts then / NbTotalin s,e
			
			
}

//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//							DetectorBPR_c
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
void DetectorBPR_c::SetUpJoints(const std::string& TagName)
{
	cv::FileStorage Fs(ConfigFileName,cv::FileStorage::READ);
	if(!Fs["Poses"][TagName].empty())
	{
		std::string JointsFile = (std::string)Fs["Poses"][TagName];
		JointsFile = mcv::GetMainPath(ConfigFileName) + JointsFile;
		Model.load(JointsFile);
	}
	else
	{
		printf("(%s) Tag is not available on 'Poses' of cfg File (%s)\nNo Joints Poses Availble\n",TagName.c_str(),ConfigFileName.c_str());
	}

}
//--------------------------------------------------------------------------------------------------------------
void DetectorBPR_c::ProcessTrain2D(int Start,int Last,const char*MStreamsBKG_In,const char*MStreamsD2DTrain_Out)
{
	//printf("Start Processing Train Data (%d,%d) from Path :\n%s\n",Start,Last,pS3DEnv->SMainPath.c_str());
	//---------------------------------------------------------------------------------BKGs
	MultiCamStream BKGStreams;
	int NbBKGViews = BKGStreams.SetStreams(ConfigFileName,"Views",MStreamsBKG_In);
	assert(NbBKGViews == pS3DEnv->NbCams);
	std::vector<cv::Mat> ImgsBKG(pS3DEnv->NbCams);
	//---------------------------------------------------------------------------------2D.Train
	MultiCamStream Train2DStreams;
	int NbTrain2DStreams = Train2DStreams.SetStreams(ConfigFileName,"Views",MStreamsD2DTrain_Out);
	assert(NbTrain2DStreams == pS3DEnv->NbCams);
	if(Start<0)
	{
		assert(BKGStreams.FirstIndex == Train2DStreams.FirstIndex);
		Start = BKGStreams.FirstIndex;
	}
	if(Last<0)
	{
		assert(BKGStreams.LastIndex == Train2DStreams.LastIndex);
		Last = BKGStreams.LastIndex;
	}
	//---------------------------------------------------------------------------------Loop on Poses
	int NbAllPoses = 0;
	for(int PoseIndex=Start;PoseIndex<=Last;PoseIndex++)
	{
		BKGStreams.GetFramesByIndex(ImgsBKG,PoseIndex);
		int NbDataAllViews = 0;
		//---------------------------------------------------------------Loop on Views
		BPR_2DMV.ScaleSigns(ImgsBKG);
		for(int i=0;i<NbBKGViews;i++)
		{
			std::string FileName = Train2DStreams.ImgsStream[i].GetName(PoseIndex);
			NbDataAllViews += BPR_2DMV.Parts2D[i].ProcessTrain2D(ImgsBKG[i],FileName);
		}
		//------------------------------------------------------------------------
		printf("Train2D(%d) total: %d\n",PoseIndex,NbDataAllViews);
		NbAllPoses += NbDataAllViews;
	}
	printf(">>>> Train In Poses (%d,%d) : %d\n",Start,Last,NbAllPoses);
}
//--------------------------------------------------------------------------------------------------------------
void DetectorBPR_c::ProcessTrain2D(int Start,int Last,			const char*MStreamsBKG_In,VectorsTable_c &TrainTable)
{
	//printf("Start Processing Train Data (%d,%d) from Path :\n%s\n",Start,Last,pS3DEnv->SMainPath.c_str());
	//---------------------------------------------------------------------------------BKGs
	MultiCamStream BKGStreams;
	int NbBKGViews = BKGStreams.SetStreams(ConfigFileName,"Views",MStreamsBKG_In);
	assert(NbBKGViews == pS3DEnv->NbCams);
	std::vector<cv::Mat> ImgsBKG(pS3DEnv->NbCams);
	//---------------------------------------------------------------------------------------
	if(Start<0)	Start = BKGStreams.FirstIndex;
	if(Last<0)	Last = BKGStreams.LastIndex;
	//---------------------------------------------------------------------------------Loop on Poses
	int NbAllPoses = 0;
	for(int PoseIndex=Start;PoseIndex<=Last;PoseIndex++)
	{
		BKGStreams.GetFramesByIndex(ImgsBKG,PoseIndex);
		int NbDataAllViews = 0;
		//---------------------------------------------------------------Loop on Views
		BPR_2DMV.ScaleSigns(ImgsBKG);
		for(int i=0;i<NbBKGViews;i++)
		{
			NbDataAllViews += BPR_2DMV.Parts2D[i].ProcessTrain2D(ImgsBKG[i],TrainTable);
		}
		//------------------------------------------------------------------------
		printf("Train2D(%d) total: %d\n",PoseIndex,NbDataAllViews);
		NbAllPoses += NbDataAllViews;
	}
	printf(">>>> Train In Poses (%d,%d) : %d\n",Start,Last,NbAllPoses);
}
//--------------------------------------------------------------------------------------------------------------
void DetectorBPR_c::ProcessTrainDepth(int Start,int Last,const char*MStreamsBKG_In,const char*MStreamsDepth_In,const char*MStreamsDepthTrain_Out)
{
	//printf("Start Processing Train Data (%d,%d) from Path :\n%s\n",Start,Last,pS3DEnv->SMainPath.c_str());
	//---------------------------------------------------------------------------------BKGs in
	MultiCamStream BKGStreams;
	int NbBKGViews = BKGStreams.SetStreams(ConfigFileName,"Views",MStreamsBKG_In);
	assert(NbBKGViews == pS3DEnv->NbCams);
	std::vector<cv::Mat> ImgsBKG(pS3DEnv->NbCams);
	//---------------------------------------------------------------------------------Depth in
	MultiCamStream DepthStreams;
	int NbDepthViews = DepthStreams.SetStreams(ConfigFileName,"Views",MStreamsDepth_In);
	assert(NbDepthViews == pS3DEnv->NbCams);
	std::vector<cv::Mat> ImgsDepthmm(pS3DEnv->NbCams);
	//---------------------------------------------------------------------------------Depth.Train out
	MultiCamStream TrainDepthStreams;
	int NbTrainDepthStreams = TrainDepthStreams.SetStreams(ConfigFileName,"Views",MStreamsDepthTrain_Out);
	assert(NbTrainDepthStreams == pS3DEnv->NbCams);
	//---------------------------------------------------------------------------------Loop on Poses
	if(Start<0)
	{
		assert(BKGStreams.FirstIndex == DepthStreams.FirstIndex);
		assert(TrainDepthStreams.FirstIndex == DepthStreams.FirstIndex);
		Start = BKGStreams.FirstIndex;
	}
	if(Last<0)
	{
		assert(BKGStreams.LastIndex == DepthStreams.LastIndex);
		assert(TrainDepthStreams.LastIndex == DepthStreams.LastIndex);
		Last = BKGStreams.LastIndex;
	}
	int NbAllPoses = 0;
	for(int PoseIndex=Start;PoseIndex<=Last;PoseIndex++)
	{
		BKGStreams.GetFramesByIndex(ImgsBKG,PoseIndex);
		DepthStreams.GetFramesByIndex(ImgsDepthmm,PoseIndex);
		int NbDataAllViews = 0;
		//---------------------------------------------------------------Loop on Views
		//BPR_2DMV.ScaleSigns(ImgsBKG);// no not for depth, use avg depth or every pixel's depth
		for(int i=0;i<NbBKGViews;i++)
		{
			std::string FileName = TrainDepthStreams.ImgsStream[i].GetName(PoseIndex);
			NbDataAllViews += BPR_2DMV.Parts2D[i].ProcessTrainDepth(ImgsBKG[i],ImgsDepthmm[i],FileName);
		}
		//------------------------------------------------------------------------
		printf("Depth(%d) total: %d\n",PoseIndex,NbDataAllViews);
		NbAllPoses += NbDataAllViews;
	}
	printf(">>>> Depth in Poses (%d,%d) in all Views : %d\n",Start,Last,NbAllPoses);
}
//--------------------------------------------------------------------------------------------------------------
void DetectorBPR_c::ProcessTrainDepth(int Start,int Last,const char*MStreamsBKG_In,const char*MStreamsDepth_In,VectorsTable_c &TrainTable)
{
	//printf("Start Processing Train Data (%d,%d) from Path :\n%s\n",Start,Last,pS3DEnv->SMainPath.c_str());
	//---------------------------------------------------------------------------------BKGs in
	MultiCamStream BKGStreams;
	int NbBKGViews = BKGStreams.SetStreams(ConfigFileName,"Views",MStreamsBKG_In);
	assert(NbBKGViews == pS3DEnv->NbCams);
	std::vector<cv::Mat> ImgsBKG(pS3DEnv->NbCams);
	//---------------------------------------------------------------------------------Depth in
	MultiCamStream DepthStreams;
	int NbDepthViews = DepthStreams.SetStreams(ConfigFileName,"Views",MStreamsDepth_In);
	assert(NbDepthViews == pS3DEnv->NbCams);
	std::vector<cv::Mat> ImgsDepthmm(pS3DEnv->NbCams);
	//---------------------------------------------------------------------------------Loop on Poses
	if(Start<0)
	{
		assert(BKGStreams.FirstIndex == DepthStreams.FirstIndex);
		Start = BKGStreams.FirstIndex;
	}
	if(Last<0)
	{
		assert(BKGStreams.LastIndex == DepthStreams.LastIndex);
		Last = BKGStreams.LastIndex;
	}
	int NbAllPoses = 0;
	for(int PoseIndex=Start;PoseIndex<=Last;PoseIndex++)
	{
		BKGStreams.GetFramesByIndex(ImgsBKG,PoseIndex);
		DepthStreams.GetFramesByIndex(ImgsDepthmm,PoseIndex);
		int NbDataAllViews = 0;
		//---------------------------------------------------------------Loop on Views
		//BPR_2DMV.ScaleSigns(ImgsBKG);// no not for depth, use avg depth or every pixel's depth
		for(int i=0;i<NbBKGViews;i++)
		{
			NbDataAllViews += BPR_2DMV.Parts2D[i].ProcessTrainDepth(ImgsBKG[i],ImgsDepthmm[i],TrainTable);
		}
		//------------------------------------------------------------------------
		printf("Depth(%d) total: %d\n",PoseIndex,NbDataAllViews);
		NbAllPoses += NbDataAllViews;
	}
	printf(">>>> Depth in Poses (%d,%d) in all Views : %d\n",Start,Last,NbAllPoses);
}
//--------------------------------------------------------------------------------------------------------------
void DetectorBPR_c::Process2DToVox(int Start,int Last,float VoxSize,		const char*Streams_In,const char*StreamVox_Out)
{
	//---------------------------------------------------------------------------------BKGs in
	MultiCamStream BKGStreams;
	int NbBKGViews = BKGStreams.SetStreams(ConfigFileName,"Views",Streams_In);
	assert(NbBKGViews == pS3DEnv->NbCams);
	std::vector<cv::Mat> ImgsBKG(pS3DEnv->NbCams);
	//---------------------------------------------------------------------------------Depth in
	IMGFileStream FVoxStream(ConfigFileName,"Poses",StreamVox_Out);//already asserted
	std::vector<cv::Mat> ImgsDepthmm(pS3DEnv->NbCams);
	int NbDataAllPoses = 0;
	//---------------------------------------------------------------------------------Loop on Poses
	for(int PoseIndex=Start;PoseIndex<=Last;PoseIndex++)
	{
		BKGStreams.GetFramesByIndex(ImgsBKG,PoseIndex);
		std::string VoxFile = FVoxStream.GetName(PoseIndex);
		int NbData = Voxelliser.Process2DToVox(ImgsBKG,VoxFile,VoxSize);
		//------------------------------------------------------------------------
		printf("Vox(%d) NbData: %d\n",PoseIndex,NbData);
		NbDataAllPoses += NbData;
	}
	printf(">>>> Depth in Poses (%d,%d) in all Views : %d\n",Start,Last,NbDataAllPoses);

}
//--------------------------------------------------------------------------------------------------------------
void DetectorBPR_c::ProcessTrain3D(int Start,int Last,const char*MStreamsVox_In,const char*MStreams3DTrain_Out)
{
	double t;
	bool isTime = true;
	//---------------------------------------------------------------------------------Vox in
	IMGFileStream VoxStream(ConfigFileName,"Poses",MStreamsVox_In);
	//---------------------------------------------------------------------------------Depth in
	IMGFileStream Train3DStream(ConfigFileName,"Poses",MStreams3DTrain_Out);//already asserted
	int NbVoxAllPoses = 0;
	//---------------------------------------------------------------------------------Loop on Poses
	for(int PoseIndex=Start;PoseIndex<=Last;PoseIndex++)
	{
		std::string VoxFile = VoxStream.GetName(PoseIndex);
		g3d::VoxelSpace_c Vox;
		Vox.load(VoxFile);
		mrg::VectorsTable_c Train3D;
		if(isTime)TStart(t);
		int NbVox = Voxelliser.ProcessTrain3D(Vox,Train3D);
		if(isTime)TStop(t,"ProcessTrain3D()");
		std::string Train3DFile = Train3DStream.GetName(PoseIndex);
		YAML::Emitter Yout;
		Yout << YAML::BeginDoc << Voxelliser.VoxSign << YAML::EndDoc;//Add the Vox Fit details
		Train3D.save(Train3DFile,Yout.c_str(),true);
//#define EXPORT_CSV
#ifdef EXPORT_CSV
		char RespCSVFileName[100];
		sprintf(RespCSVFileName,"G:\\PosturesDB\\temp\\Train%04d.csv",PoseIndex);
		Train3D.Export(RespCSVFileName);
#endif
		//------------------------------------------------------------------------
		printf("Train3D(%d) NbData: %d - ",PoseIndex,NbVox);
		NbVoxAllPoses += NbVox;
	}
	printf(">>>> Depth in Poses (%d,%d) in all Views : %d\n",Start,Last,NbVoxAllPoses);
}
//--------------------------------------------------------------------------------------------------------------
void DetectorBPR_c::ProcessTrain3D(int Start,int Last,	const char*StreamVox_In,VectorsTable_c &Train3DTable)//append on Train3D
{
	//---------------------------------------------------------------------------------Vox in
	IMGFileStream VoxStream(ConfigFileName,"Poses",StreamVox_In);
	int NbVoxAllPoses = 0;
	//---------------------------------------------------------------------------------Loop on Poses
	for(int PoseIndex=Start;PoseIndex<=Last;PoseIndex++)
	{
		std::string VoxFile = VoxStream.GetName(PoseIndex);
		g3d::VoxelSpace_c Vox;
		Vox.load(VoxFile);
		int NbVox = Voxelliser.ProcessTrain3D(Vox,Train3DTable);
		//------------------------------------------------------------------------
		printf("Train3D(%d) NbData: %d\n",PoseIndex,NbVox);
		NbVoxAllPoses += NbVox;
	}
	printf(">>>> Depth in Poses (%d,%d) in all Views : %d\n",Start,Last,NbVoxAllPoses);
}
//--------------------------------------------------------------------------------------------------------------
//do not precise 2D or Depth cause it works for both, the 3DLabels do precise it
void DetectorBPR_c::ProcessRespLabels(int Start,int Last,const char*MStreamsBKG_In,const char*MStreamsBPR_In,const char*MStreamsLabelsResp_Out)
{
	//printf("Start Processing Resp Labels (%d,%d) from Path :\n%s\n",Start,Last,pS3DEnv->SMainPath.c_str());
	//---------------------------------------------------------------------------------BKGs
	MultiCamStream BKGStreams;
	int NbBKGViews = BKGStreams.SetStreams(ConfigFileName,"Views",MStreamsBKG_In);
	assert(NbBKGViews == pS3DEnv->NbCams);
	std::vector<cv::Mat> ImgsBKG(pS3DEnv->NbCams);
	//---------------------------------------------------------------------------------BPRs
	MultiCamStream BPRStreams;
	int NbBPRViews = BPRStreams.SetStreams(ConfigFileName,"Views",MStreamsBPR_In);
	assert(NbBPRViews == pS3DEnv->NbCams);
	std::vector<cv::Mat> ImgsBPR(pS3DEnv->NbCams);
	//---------------------------------------------------------------------------------Labels.Resp
	MultiCamStream StreamsLabelsResp;
	int NbLabelRespStreams = StreamsLabelsResp.SetStreams(ConfigFileName,"Views",MStreamsLabelsResp_Out);
	assert(NbLabelRespStreams == pS3DEnv->NbCams);
	//---------------------------------------------------------------------------------Loop on Poses
	if(Start<0)
	{
		assert(BKGStreams.FirstIndex == BPRStreams.FirstIndex);
		assert(StreamsLabelsResp.FirstIndex == BPRStreams.FirstIndex);
		Start = BKGStreams.FirstIndex;
	}
	if(Last<0)
	{
		assert(BKGStreams.LastIndex == BPRStreams.LastIndex);
		assert(StreamsLabelsResp.LastIndex == BPRStreams.LastIndex);
		Last = BKGStreams.LastIndex;
	}
	int NbAllPoses = 0;
	for(int PoseIndex=Start;PoseIndex<=Last;PoseIndex++)
	{
		BKGStreams.GetFramesByIndex(ImgsBKG,PoseIndex);
		BPRStreams.GetFramesByIndex(ImgsBPR,PoseIndex);
		int NbDataAllViews = 0;
		//---------------------------------------------------------------Loop on Views
		BPR_2DMV.ScaleSigns(ImgsBKG);
		for(int i=0;i<NbBKGViews;i++)
		{
			std::string FileName = StreamsLabelsResp.ImgsStream[i].GetName(PoseIndex);
			NbDataAllViews += BPR_2DMV.Parts2D[i].ProcessLabels(ImgsBKG[i],ImgsBPR[i],FileName);
		}
		//------------------------------------------------------------------------
		printf("Labels(%d) total: %d\n",PoseIndex,NbDataAllViews);
		NbAllPoses += NbDataAllViews;
	}
	printf(">>>> Resp In Poses (%d,%d) : %d\n",Start,Last,NbAllPoses);
}
//--------------------------------------------------------------------------------------------------------------
void DetectorBPR_c::ProcessRespLabels(int Start,int Last,		const char*MStreamsBKG_In,const char*MStreamsBPR_In,VectorsTable_c &LabelsTable)
{
	//printf("Start Processing Resp Labels (%d,%d) from Path :\n%s\n",Start,Last,pS3DEnv->SMainPath.c_str());
	//---------------------------------------------------------------------------------BKGs
	MultiCamStream BKGStreams;
	int NbBKGViews = BKGStreams.SetStreams(ConfigFileName,"Views",MStreamsBKG_In);
	assert(NbBKGViews == pS3DEnv->NbCams);
	std::vector<cv::Mat> ImgsBKG(pS3DEnv->NbCams);
	//---------------------------------------------------------------------------------BPRs
	MultiCamStream BPRStreams;
	int NbBPRViews = BPRStreams.SetStreams(ConfigFileName,"Views",MStreamsBPR_In);
	assert(NbBPRViews == pS3DEnv->NbCams);
	std::vector<cv::Mat> ImgsBPR(pS3DEnv->NbCams);
	//---------------------------------------------------------------------------------Loop on Poses
	if(Start<0)
	{
		assert(BKGStreams.FirstIndex == BPRStreams.FirstIndex);
		Start = BKGStreams.FirstIndex;
	}
	if(Last<0)
	{
		assert(BKGStreams.LastIndex == BPRStreams.LastIndex);
		Last = BKGStreams.LastIndex;
	}
	int NbAllPoses = 0;
	for(int PoseIndex=Start;PoseIndex<=Last;PoseIndex++)
	{
		BKGStreams.GetFramesByIndex(ImgsBKG,PoseIndex);
		BPRStreams.GetFramesByIndex(ImgsBPR,PoseIndex);
		int NbDataAllViews = 0;
		//---------------------------------------------------------------Loop on Views
		BPR_2DMV.ScaleSigns(ImgsBKG);
		for(int i=0;i<NbBKGViews;i++)
		{
			NbDataAllViews += BPR_2DMV.Parts2D[i].ProcessLabels(ImgsBKG[i],ImgsBPR[i],LabelsTable);
		}
		//------------------------------------------------------------------------
		printf("Labels(%d) total: %d\n",PoseIndex,NbDataAllViews);
		NbAllPoses += NbDataAllViews;
	}
	printf(">>>> Resp In Poses (%d,%d) : %d\n",Start,Last,NbAllPoses);
}

//--------------------------------------------------------------------------------------------------------------
// .vtx "Voxels" + .yjt "Joints" => .vt "Parts3DResp" + [.vtx "VoxelsParts"]
void DetectorBPR_c::ProcessResp3D(int Start,int Last,const char*StreamVox_In,const char*StreamJCenters_In,const char*StreamResp3D_Out,const char*ExportVoxParts)
{
	double t;
	bool isTime = true;
	//---------------------------------------------------------------------------------Vox In
	IMGFileStream VoxStream(ConfigFileName,"Poses",StreamVox_In);
	int NbVoxAllPoses = 0;
	//---------------------------------------------------------------------------------JointsCenters In
	stringmap Config;
	Config["CfgFile"] = ConfigFileName;
	Config["Node"] = "Poses";
	Config["Tag"] = StreamJCenters_In;
	BodyJoints_c JointsPoses(Config);
	//---------------------------------------------------------------------------------Stream Resp 3D out - VectorsTable
	IMGFileStream Resp3DStream(ConfigFileName,"Poses",StreamResp3D_Out);
	//---------------------------------------------------------------------------------If To Export VoxParts
	IMGFileStream VoxPartsStream;
	if(ExportVoxParts!=NULL)
	{
		VoxPartsStream.Init(ConfigFileName,"Poses",ExportVoxParts);
	}
	//---------------------------------------------------------------------------------Loop on Poses
	int NbVoxinAllPoses = 0;
	for(int PoseIndex=Start;PoseIndex<=Last;PoseIndex++)
	{
		g3d::VoxelSpace_c Vox;
		Vox.load(VoxStream.GetName(PoseIndex));
		VectorsTable_c TableResp3D;
		if(isTime)TStart(t);
		int NbVox = Voxelliser.ProcessResp3DParts(JointsPoses.Poses[PoseIndex-JointsPoses.FirstIndex],Vox,TableResp3D);//The Vox here became a VoxPart
		if(isTime)TStop(t,"ProcessResp3DParts()");
		//------------------------------------------------------------------------TableResp3D user Header
		/*YAML::Emitter Yout;
		Yout << YAML::BeginDoc;
		Yout << YAML::BeginMap;
		Model.outputPartsColors(Yout);
		Yout << YAML::EndMap;
		Yout << YAML::EndDoc;*/
		//TableResp3D.save(Resp3DStream.GetName(PoseIndex),Yout.c_str(),true);
		
		if(!JointsPoses.BPartsNames.empty())
		{
			Vox.ClassesNames = JointsPoses.BPartsNames;
		}
		else
		{
			Vox.ClassesNames = JointsPoses.BJointsNames;
		}
		TableResp3D.ClassesNames = Vox.ClassesNames;

		TableResp3D.TypeName = "Parts3DResp";
		TableResp3D.save(Resp3DStream.GetName(PoseIndex));
//#define EXPORT_CSV
#ifdef EXPORT_CSV
		char RespCSVFileName[30];
		sprintf(RespCSVFileName,"G:\\PosturesDB\\temp\\Resp%04d.csv",PoseIndex);
		TableResp3D.Export(RespCSVFileName);
#endif
		//------------------------------------------------------------------------
		if(ExportVoxParts!=NULL)
		{
			Vox.TypeName = "VoxelsParts";
			Vox.save(VoxPartsStream.GetName(PoseIndex),"---\nVoxType: Parts\n...\n");
		}
		printf("Resp3D(%d) NbVox: %d\n",PoseIndex,NbVox);
		NbVoxinAllPoses += NbVox;
	}
	printf(">>>> Resp3D Parts in Poses (%d,%d) in all Views : %d\n",Start,Last,NbVoxinAllPoses);

}

//--------------------------------------------------------------------------------------------------------------
void DetectorBPR_c::ProcessRespLabels2Imgs(int Start,int Last,const char*MStreamsBKG_In,const char*MStreamsLabelsResp_In,const char*MStreamsBPR_Out)
{
	//printf("Start ProcessRespLabels2Imgs (%d,%d) from Path :\n%s\n",Start,Last,MainPath.c_str());
	MultiCamStream BKGStreams;
	int NbBKGViews = BKGStreams.SetStreams(ConfigFileName,"Views",MStreamsBKG_In);
	assert(NbBKGViews == pS3DEnv->NbCams);
	std::vector<cv::Mat> ImgsBKG(pS3DEnv->NbCams);
	//---------------------------------------------------------------------------------Label Resp
	MultiCamStream LabelRespStreams;
	int NbLBRViews = LabelRespStreams.SetStreams(ConfigFileName,"Views",MStreamsLabelsResp_In);
	assert(NbLBRViews == pS3DEnv->NbCams);
	//---------------------------------------------------------------------------------BPR Res
	MultiCamStream StreamsBPRres;
	int NbLabelRespStreams = StreamsBPRres.SetStreams(ConfigFileName,"Views",MStreamsBPR_Out);
	assert(NbLabelRespStreams == pS3DEnv->NbCams);
	std::vector<cv::Mat> ImgsBPR(pS3DEnv->NbCams);
	//---------------------------------------------------------------------------------Loop on Poses
	if(Start<0)
	{
		assert(BKGStreams.FirstIndex == LabelRespStreams.FirstIndex);
		assert(StreamsBPRres.FirstIndex == LabelRespStreams.FirstIndex);
		Start = BKGStreams.FirstIndex;
	}
	if(Last<0)
	{
		assert(BKGStreams.LastIndex == LabelRespStreams.LastIndex);
		assert(StreamsBPRres.LastIndex == LabelRespStreams.LastIndex);
		Last = BKGStreams.LastIndex;
	}
	int NbAllPoses = 0;
	for(int PoseIndex=Start;PoseIndex<=Last;PoseIndex++)
	{
		BKGStreams.GetFramesByIndex(ImgsBKG,PoseIndex);
		int NbDataAllViews = 0;
		//---------------------------------------------------------------Loop on Views
		BPR_2DMV.ScaleSigns(ImgsBKG);
		for(int i=0;i<NbBKGViews;i++)
		{
			std::string FileName = LabelRespStreams.ImgsStream[i].GetName(PoseIndex);
			NbDataAllViews += BPR_2DMV.Parts2D[i].MatchLabelsToImg(ImgsBKG[i],ImgsBPR[i],FileName);
		}
		StreamsBPRres.WriteFramesByIndex(ImgsBPR,PoseIndex);
		//------------------------------------------------------------------------
		printf("Resp2Img(%d) total: %d\n",PoseIndex,NbDataAllViews);
		NbAllPoses += NbDataAllViews;
	}
	printf(">>>> Resp2Imgs in Poses (%d,%d) in all Views : %d\n",Start,Last,NbAllPoses);
}

//--------------------------------------------------------------------------------------------------------------
// .vtx "VoxelsParts" + .vt "Parts3DResp" => .vtx "VoxelsParts"
void DetectorBPR_c::ProcessResp3DToVox(int Start,int Last,const char*RefVox_In,const char*RespResVt_In,const char*RespResVox_Out)
{
	double t;
	bool isTime = true;
	IMGFileStream RefVoxes(ConfigFileName,"Poses",RefVox_In);
	IMGFileStream RespVT(ConfigFileName,"Poses",RespResVt_In);
	IMGFileStream ResVoxes(ConfigFileName,"Poses",RespResVox_Out);

	for(int PoseIndex=Start;PoseIndex<=Last;PoseIndex++)
	{
		g3d::VoxelSpace_c Vox(RefVoxes.GetName(PoseIndex));
		VectorsTable_c TableResp3D(RespVT.GetName(PoseIndex));
		if(isTime)TStart(t);
		Vox.FromVectTab(TableResp3D);
		if(isTime)TStop(t,"ProcessResp3DParts()");
		//nothing changes : TypeName ClassesNames, ColorsTable
		Vox.save(ResVoxes.GetName(PoseIndex));
		//------------------------------------------------------------------------
		printf("ProcessResp3DToVox(%d)\n",PoseIndex);
	}
	printf("ProcessResp3DToVox (%d,%d)\n",Start,Last);

}
//--------------------------------------------------------------------------------------------------------------
void DetectorBPR_c::VoxMedianFilter(int Start,int Last,const char*RefVox_InOut,const char*RespResVt_Out)
{
	double t;
	bool isTime = true;
	IMGFileStream Voxes(ConfigFileName,"Poses",RefVox_InOut);
	IMGFileStream RespVT(ConfigFileName,"Poses",RespResVt_Out);

	for(int PoseIndex=Start;PoseIndex<=Last;PoseIndex++)
	{
		g3d::VoxelSpace_c Vox(Voxes.GetName(PoseIndex));
		if(isTime)TStart(t);
		Vox.MedianFilter(Model.BPartsNames.size()+1);
		if(isTime)TStop(t,"Vox.MedianFilter()");
		Vox.save(Voxes.GetName(PoseIndex));
		if(RespResVt_Out != NULL)
		{
			VectorsTable_c VT;
			Vox.ToVectTab(VT);
			//Because this is a Type checked before comparison so we can't use "Parts3DRespres"
			VT.TypeName = "Parts3DResp";
			VT.save(RespVT.GetName(PoseIndex));
		}
		//------------------------------------------------------------------------
		printf("VoxMedianFilter(%d)\n",PoseIndex);
	}
	printf("Process VoxMedianFilter (%d,%d)\n",Start,Last);
}

//--------------------------------------------------------------------------------------------------------------
// .vtx "VoxelsParts" => .yjt "Joints"
void DetectorBPR_c::ProcessVoxParts2Centers(int Start,int Last,const char*PartsResp_in,const char*JointsRes_Out)
{
	double t;
	bool isTime = true;
	IMGFileStream Voxes(ConfigFileName,"Poses",PartsResp_in);
	//---------------------------------------------------------------------------------JointsCenters Out
	BodyJoints_c JointsPoses;
	BodyJointsPose_c CurrentPose;
	YAML::Node user_doc;
	g3d::VoxelSpace_c Vox;
	for(int PoseIndex=Start;PoseIndex<=Last;PoseIndex++)
	{
		Vox.load(Voxes.GetName(PoseIndex),user_doc);
		if(isTime)TStart(t);
		Voxelliser.ProcessVoxParts2Centers(Vox,CurrentPose.BodyParts);//starting from 2
		if(isTime)TStop(t,"Voxelliser.ProcessVoxParts2Centers()");
		JointsPoses.push(CurrentPose);
		//------------------------------------------------------------------------
		printf("ProcessVoxParts2Centers(%d)\n",PoseIndex);
	}
	//Configure the Joints Names and Colors
	if(!Vox.ClassesNames.empty())
	{
		JointsPoses.BJointsNames = Vox.ClassesNames;//send Names from "Voxels" to "Joints"
	}
	
	JointsPoses.BPartsColors = Vox.ColorTable;
	JointsPoses.FirstIndex = Start;

	cv::FileStorage Fs(ConfigFileName,cv::FileStorage::READ);
	assert(!Fs["Poses"][JointsRes_Out].empty());
	std::string JFileName = mcv::GetMainPath(ConfigFileName) + (std::string)Fs["Poses"][JointsRes_Out];
	JointsPoses.save(JFileName);

	printf("Process VoxMedianFilter (%d,%d)\n",Start,Last);
}
//--------------------------------------------------------------------------------------------------------------
// .yjt "Joints" + .yjt "Joints" => Error cm
void DetectorBPR_c::CompareCenters(int Start,int Last,			const char*JointsRef_in,const char*JointsRes_in)
{
	double t;
	bool isTime = true;

	BodyJoints_c JPRes;
	BodyJoints_c JPRef;

	cv::FileStorage Fs(ConfigFileName,cv::FileStorage::READ);
	assert(!Fs["Poses"][JointsRef_in].empty());
	assert(!Fs["Poses"][JointsRes_in].empty());
	std::string JFileName = mcv::GetMainPath(ConfigFileName) + (std::string)Fs["Poses"][JointsRef_in];
	JPRef.load(JFileName);
	JFileName = mcv::GetMainPath(ConfigFileName) + (std::string)Fs["Poses"][JointsRes_in];
	JPRes.load(JFileName);

	if(isTime)TStart(t);
	float AvgAll = JPRef.Compare(JPRes,Start,Last);
	if(isTime)TStop(t,"BodyJoints_c.Compare()");

	cout << ">>>>>>>>>>>>>>>>>>Compare(" << JointsRef_in << " to " << JointsRes_in << ") => Avg All: " << AvgAll << endl;
}
//--------------------------------------------------------------------------------------------------------------
void DetectorBPR_c::SetROCParam(float RCP)//custom parameter change method
{
	for(int i=0;i<BPR_2DMV.Parts2D.size();i++)
	{
		BPR_2DMV.Parts2D[i].PxSign.BKG_Default = RCP;
	}
}


//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//									S3 Voxel Body Parts Fitting ModelNodeFitParts_s
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//					MeanShift3DPart_c
//--------------------------------------------------------------------------------------------------------------
MeanShift3DPart_c::MeanShift3DPart_c()
{
	isPosFound = false;
}
//--------------------------------------------------------------------------------------------------------------
bool MeanShift3DPart_c::AddPos(Vector3f NewVect,float delta,float fill)
{
	bool isItAdded = false;
	bool isFar = true;
	for(int i=0;(i<MeanShift3DRes.size()) && isFar;i++)
	{
		Vector3f Diff = NewVect - MeanShift3DRes[i];
		if(	 (Diff.x() < delta)  &&  (Diff.y() < delta)  &&  (Diff.z() < delta)  )
			isFar = false;
	}
	if(isFar)
	{
		MeanShift3DRes.push_back(NewVect);
		FillRate.push_back(fill);
		isItAdded = true;
		isPosFound = true;
	}
	return isItAdded;
}
//--------------------------------------------------------------------------------------------------------------
Vector3f MeanShift3DPart_c::GetPos()
{
	if(isPosFound)
	{
		int MaxIndex = TabMax_Index(FillRate);
		return MeanShift3DRes[MaxIndex];
	}
	else
	{
		return Vector3f();
	}
}
//--------------------------------------------------------------------------------------------------------------
bool MeanShift3DPart_c::GetPos(Vector3f &P1,Vector3f &P2)
{
	if(isPosFound)
	{
		int MaxIndex = TabMax_Index(FillRate);
		P1 = MeanShift3DRes[MaxIndex];
		Pos[0] = P1;
		if(Pos.size()==2)
		{
			if(FillRate.size()>=2)
			{
				float Safe = FillRate[MaxIndex];
				FillRate[MaxIndex] = FLT_MIN;
				int SecIndex = TabMax_Index(FillRate);
				FillRate[MaxIndex] = Safe;
				P2 = MeanShift3DRes[SecIndex];
				Pos[1] = P2;
			}
			else
			{
				Pos[1] = P1;
				P2 = P1;
			}
		}

		return true;
	}
	else
	{
		return false;
	}
}
//-------------------------------------------------------------------------------------------------------------
void MeanShift3DPart_c::ClearPos()
{
	MeanShift3DRes.clear();
	FillRate.clear();
	isPosFound = false;
}
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
