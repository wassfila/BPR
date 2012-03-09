#include "S3DModel.h"

#include "S3DGeom.h"
#include <iostream>
#include <fstream>
#include "mcvGeneral.h"

using namespace std;
using namespace mcv;
using namespace g3d;

//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//							BodyJointsPose_c
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
std::vector<float> BodyJointsPose_c::Get()
{
	std::vector<float> Res;
	Res.reserve(BodyParts.size() * 3);
	for(int i=0;i<BodyParts.size();i++)
	{
		Res.push_back(BodyParts[i].x());
		Res.push_back(BodyParts[i].y());
		Res.push_back(BodyParts[i].z());
	}
	return Res;
}
//--------------------------------------------------------------------------------------------------------------
void BodyJointsPose_c::Set(std::vector<float> &Vect)
{
	assert((Vect.size() % 3) == 0);
	int NbJoints = (int)Vect.size() / 3;
	BodyParts.resize(NbJoints);
	for(int i=0;i<NbJoints;i++)
	{
		Vector3f V;
		V.x() = Vect[3*i + 0];
		V.y() = Vect[3*i + 1];
		V.z() = Vect[3*i + 2];
		BodyParts[i] = V;
	}
}
//--------------------------------------------------------------------------------------------------------------
int BodyJointsPose_c::GetClosestJointIndex(const Vector4f& Pos)
{
//#define DEBUG_CLOSEST
#ifdef DEBUG_CLOSEST
	std::cout << "Pos: " << Pos << std::endl << "BParts:" << std::endl;
	for(int i=0;i<BodyParts.size();i++)
	{
		std::cout << BodyParts[i] << std::endl;
	}
#endif
	std::vector<float> Dists(BodyParts.size());
	for(int i=0;i<BodyParts.size();i++)
	{
		Dists[i] = (BodyParts[i] - V4To3(Pos)).norm();
	}
#ifdef DEBUG_CLOSEST
	mcv::TabPrintf(Dists,"Dists",true,' ',2);
#endif
	return mcv::TabMin_Index(Dists);
}
//--------------------------------------------------------------------------------------------------------------
int BodyJointsPose_c::GetClosestJointIndex_10(const Vector4f& Pos)
{
	std::vector<float> Dists(BodyParts.size());
	for(int i=0;i<BodyParts.size();i++)
	{
		Dists[i] = (BodyParts[i] - V4To3(Pos)).norm();
	}
	int minIndex = mcv::TabMin_Index(Dists);
	if(Dists[minIndex] < 10)//not close to a center
	{
		minIndex = 1;//Classified as Unckown Body Part
	}
	return minIndex;
}
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//					BodyJoints_c
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
void BodyJoints_c::clear()
{
	Poses.clear();
	BJointsNames.clear();
	BJointsParts.clear();
	BPartsNames.clear();
	BPartsColors.clear();
	CurrentPoseIndex = 0;
	FirstIndex = 0;
}
//--------------------------------------------------------------------------------------------------------------
BodyJoints_c::BodyJoints_c(stringmap &Config)
{
	clear();//not needed on construction
	SetupGrabber(Config);
}
//--------------------------------------------------------------------------------------------------------------
void BodyJoints_c::SetupGrabber(stringmap &Config)//Loads "Joints" node from "Poses" Tag of "CfgFile"
{
	//assert(!Config["CfgFile"].empty());
	std::string FileName = (std::string) Config["CfgFile"];
	cv::FileStorage Fs(FileName,cv::FileStorage::READ);
	std::string TagName = Config["Tag"];
	std::string NodeName = Config["Node"];
	assert(!Fs[NodeName][TagName].empty());
	std::string JFileName = (std::string)Fs["Poses"][TagName];
	JFileName = mcv::GetMainPath(FileName) + JFileName;
	load(JFileName);
	//Sequence.loadData("G:\\PosturesDB\\VirtualDB\\SoftMotion\\All10Seq.JPos.csv");
	//Sequence.save(JFileName);
}
//--------------------------------------------------------------------------------Renderable_c
void BodyJoints_c::GrabFrameIndex(int Index)
{
	CurrentPoseIndex = Index-FirstIndex;//nasty nasty should all seq start from 0 or how to synchronize ??!!
}
//--------------------------------------------------------------------------------Renderable_c
int BodyJoints_c::JointToPart(int i)
{
	int Res;
	if(BJointsParts.empty())
	{
		Res = i;
	}
	else
	{
		Res = BJointsParts[i];
	}
	return Res;
}
//--------------------------------------------------------------------------------Renderable_c
void BodyJoints_c::Render(Soft3DCamera_c &Cam)//Takes the Cam Id and fill it with the corresponding Channel
{
	for(int i=0;i<BJointsNames.size();i++)
	{
		int PartId = JointToPart(i);
		ScalarC Color = BPartsColors[PartId];
		//if(Cam.Id == 0)		{			Color = ScalarC(0,0,255,0);		}
		Cam.DrawPoint3D(	V3To4(Poses[CurrentPoseIndex].BodyParts[i]),Color);
	}
}
//--------------------------------------------------------------------------------------------------------------
void BodyJoints_c::ToVectTab(mrg::VectorsTable_c &VectTab)
{
	VectTab.Reset((int)BJointsNames.size() * 3);//x,y,z
	for(size_t i=0;i<Poses.size();i++)
	{
		VectTab.push(Poses[i].Get());
	}
}
//--------------------------------------------------------------------------------------------------------------
void BodyJoints_c::FromVectTab(mrg::VectorsTable_c &VectTab)
{
	Poses.clear();//ResetAll
	assert((VectTab.VectSize % 3 ) == 0);
	int NbJoints = VectTab.VectSize / 3;
	//assert(NbJoints == BJointsNames.size());//x,y,z for every joint
	Poses.reserve(VectTab.size());
	for(int i=0;i<VectTab.size();i++)
	{
		BodyJointsPose_c Pose;
		Pose.Set(VectTab.at(i));
		push(Pose);
	}
}
//--------------------------------------------------------------------------------------------------------------
void BodyJoints_c::load(const std::string &FileName) //loads a JointsPosDesc Channel File with FileStorage
{
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! why not just use the user header of VactorsTable load!!!!???!!
	mcv::MixedFileManager_c FManager;
	std::stringstream YamlHeader = FManager.Parse(FileName);
	if(YamlHeader.str().empty())
	{
		std::cout << "BodyJointsPosSeq_c::load() Couldn't load File: " << FileName << std::endl;
		return;
	}
	YAML::Parser parser;
	parser.Load(YamlHeader);
	//YAML::Node ThrowAwayThisNode;
	//parser.GetNextDocument(ThrowAwayThisNode);
	YAML::Node doc;
	parser.GetNextDocument(doc);
	
	int NbPoses;
	doc["NbPoses"] >> NbPoses;
	doc["BJointsNames"] >> BJointsNames;
	doc["BPartsColors"] >> BPartsColors;
	//optionnal BJointsParts
	if(doc.FindValue("FirstIndex"))//in the case where the motion doesn't start at Frame 0
	{
		doc["FirstIndex"] >> FirstIndex;
	}
	else
	{
		FirstIndex = 0;
	}
	if(doc.FindValue("BJointsParts"))
	{
		doc["BJointsParts"] >> BJointsParts;
		doc["BPartsNames"] >> BPartsNames;//If you have Joints Parts then you must have Joints Names
		assert(BJointsParts.size() == BJointsNames.size());//A Part for every Joint
	}
	else
	{
		assert(BPartsColors.size() == BJointsNames.size());//A Color for Every Part that is Every Joint
	}

	int NbJoints = (int)BJointsNames.size();
	mrg::VectorsTable_c VectTab;
	VectTab.resize(NbJoints*3,NbPoses);

	if(!VectTab.Data.empty())//So that we can load even a header without data
	{
		FManager.LoadData((char*)&VectTab.Data[0],VectTab.Data.size()*4);
	}

	FromVectTab(VectTab);
}
//--------------------------------------------------------------------------------------------------------------
void BodyJoints_c::outputPartsColors(YAML::Emitter& Yout)
{
	if(BJointsParts.empty())//Parts indexing not used
	{
		assert(BJointsNames.size() == BPartsColors.size());//A Color for Every Part that is Every Joint
		Yout << YAML::Key << "BJointsNames";
		Yout << YAML::Value	<< BJointsNames;//Vector<std::string>
	}
	else
	{
		assert(BPartsNames.size() == BPartsColors.size());//Every Part has a Color and a Name
		Yout << YAML::Key << "BPartsNames";
		Yout << YAML::Value	<< BPartsNames;//Vector<std::string>
	}
	Yout << YAML::Key << "BPartsColors";
	Yout << YAML::Value	<< BPartsColors;//Vector<cv::Scalar>
}
//--------------------------------------------------------------------------------------------------------------
void BodyJoints_c::save(const std::string &FileName)// list [- - -]  struct{r:val, g:val}
{
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! why not just use the user header of VactorsTable Save !!!!???!!
	assert(!Poses.empty());//Yeah, no empty file, by design

	YAML::Emitter Yout;
	Yout << YAML::BeginDoc;
	Yout << YAML::BeginMap;
	Yout << YAML::Key << "NbPoses";
	Yout << YAML::Value	<< Poses.size();//Vector<std::string>
	Yout << YAML::Key << "FirstIndex";
	Yout << YAML::Value	<< FirstIndex;
	Yout << YAML::Key << "NbJoints";
	Yout << YAML::Value	<< BJointsNames.size();


	Yout << YAML::Key << "BJointsNames";
	Yout << YAML::Value	<< BJointsNames;//Vector<std::string>
	if(!BJointsParts.empty())
	{
		Yout << YAML::Key << "NbParts";
		Yout << YAML::Value	<< BPartsNames.size();
		assert(BJointsParts.size() == BJointsNames.size());//A Part for every Joint
		assert(BPartsColors.size() == BPartsNames.size());//Every Part has a Color and a Name
		Yout << YAML::Key << "BJointsParts";
		Yout << YAML::Value	<< BJointsParts;
		Yout << YAML::Key << "BPartsNames";
		Yout << YAML::Value	<< BPartsNames;
	}
	else
	{
		assert(BJointsNames.size() == BPartsColors.size());//A Color for Every Part that is Every Joint
	}
	Yout << YAML::Key << "BPartsColors";
	Yout << YAML::Value	<< BPartsColors;//Vector<cv::Scalar>

	mrg::VectorsTable_c VectTab;
	ToVectTab(VectTab);
	Yout << YAML::EndDoc;

	mcv::MixedFileManager_c FManager;
	FManager.Save(FileName,(char*)&VectTab.Data[0],VectTab.Data.size()*4,  Yout.c_str()  );
}
//--------------------------------------------------------------------------------------------------------------
void BodyJoints_c::loadData(const std::string &FileName) //loads the Data Joints Pos through a .csv file
{
	//Just for retrocompatibility with .csv files without header
	Poses.clear();//ResetAll
	mrg::VectorsTable_c RegData;
	RegData.Import(FileName,false);
	int NbJoints = RegData.VectSize / 3;
	//assert(NbJoints == BJointsNames.size());//x,y,z for every joint
	Poses.reserve(RegData.size());

	for(int i=0;i<RegData.size();i++)
	{
		BodyJointsPose_c Pose;
		Pose.Set(RegData.at(i));
		push(Pose);
	}
	
}
//--------------------------------------------------------------------------------------------------------------
void BodyJoints_c::push(BodyJointsPose_c Pose)
{
	Poses.push_back(Pose);
}
//--------------------------------------------------------------------------------------------------------------
int BodyJoints_c::GetClass(ScalarC Color)
{
	bool isFound = false;
	int Index = -1;
	for(int i=0;(i<BPartsColors.size())&&!isFound;i++)
	{
		if(Color == BPartsColors[i])
		{
			isFound = true;
			Index = i;
		}
	}
	if(!isFound)
	{
		std::vector<float> Dists(BPartsColors.size());
		for(int i=0;i<BPartsColors.size();i++)
		{
			Dists[i] = (float)cv::norm(Color - BPartsColors[i]);
		}
		Index = mcv::TabMin_Index(Dists);
	}
	return Index;
}
//--------------------------------------------------------------------------------------------------------------
ScalarC BodyJoints_c::GetColor(int Index)
{
	return BPartsColors[Index];
}
//--------------------------------------------------------------------------------------------------------------
float BodyJoints_c::Compare(BodyJoints_c& OtherBCenters,int StartIndex,int LastIndex)
{
	bool isVerbose = true;
	//------------------------------------------------------- Resolving the BPartsNames BJointsNames duality --
	std::vector<std::string> Names = BJointsNames;
	std::vector<std::string> OtherNames = OtherBCenters.BJointsNames;
	//-----------------------------------------------------------------------------------------------------

	//------------------------------------------------------------------- Finding Common Names ------------
	std::vector<std::string> CommonNames;
	std::vector<int> NIndexes;
	std::vector<int> OtherNIndexes;
	for(int i=0;i<Names.size();i++)
	{
		for(int j=0;j<OtherNames.size();j++)
		{
			if(Names[i].compare(OtherNames[j]) == 0)
			{
				CommonNames.push_back(Names[i]);
				NIndexes.push_back(i);
				OtherNIndexes.push_back(j);
			}
		}
	}
	if(isVerbose)mcv::TabPrintf(CommonNames,"CommonNames");
	//-----------------------------------------------------------------------------------------------------
	assert(StartIndex<=LastIndex);
	assert(StartIndex>=FirstIndex);
	assert(StartIndex>=OtherBCenters.FirstIndex);
	std::vector<float>CurrentErrors(CommonNames.size());
	double SumErr = 0;
	int NbSum = 0;
	for(int i=StartIndex;i<=LastIndex;i++)
	{
		int Id = i-FirstIndex;
		int OtherId = i-OtherBCenters.FirstIndex;
		for(int j=0;j<CommonNames.size();j++)
		{
			int BPId = NIndexes[j];
			int OtherBPId = OtherNIndexes[j];
			Vector3f PosDiff = (Poses[Id].BodyParts[BPId] - OtherBCenters.Poses[OtherId].BodyParts[OtherBPId]);
			float err = PosDiff.norm();
			CurrentErrors[j] = err;
			SumErr+=err;
			NbSum++;
		}
		if(isVerbose)
		{
			mcv::TabPrintf(CurrentErrors,"CurrentErrors",false,',',4);
			cout << "Avg," << mcv::TabAverage(CurrentErrors) << endl;
		}
	}
	return((float) (SumErr/NbSum));
}
//--------------------------------------------------------------------------------------------------------------
//				S3DJTConfigRev
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
Matrix4f S3DJTConfigRev::GetLocalFromConf()
{
	return Matrix3fTo4f(	Eigen::Matrix3f(Eigen::AngleAxisf(	Angle,RotAxis)));
}
//-------------------------------------------------------------------------------------------------------
void S3DJTConfigRev::GetParam(float &param)
{
	param = (Angle-MinAngle) / (MaxAngle - MinAngle);
	Pos = param;
}
//-------------------------------------------------------------------------------------------------------
void S3DJTConfigRev::SetParam(float param)
{
	Pos = param;
	Angle = param * (MaxAngle - MinAngle) + MinAngle;
}
//-------------------------------------------------------------------------------------------------------
//				CBallCahche_s
void CBallCahche_c::PosToIndex(const Vector2f vPos,int &i,int &j)
{
	printf("Not Implemented\n");
}
//-------------------------------------------------------------------------------------------------------
Vector2f CBallCahche_c::IndexToPos(int i,int j)
{
	Vector2f vPos;
	vPos.x() = ((float)i) / ((float)(NbSteps-1));
	vPos.y() = ((float)j) / ((float)(NbSteps-1));
	return vPos;
}
//-------------------------------------------------------------------------------------------------------
Vector2f CBallCahche_c::Find(const Vector3f &vVect)
{
	bool isPrintf = true;
	float minDist = FLT_MAX;
	int minI,minJ;
	for(int i=0;i<NbSteps;i++)
	for(int j=0;j<NbSteps;j++)
	{
		float dist = (Table[i][j].Vect3D - vVect).norm();
		if(dist < minDist)
		{
			minDist = dist;
			minI = i;
			minJ = j;
		}
	}
	if(isPrintf)
	{
		printf("Cache.Find()\n");
		printf("i,j (%d,%d) dist(%1.3f) Pos(%1.2f,%1.2f)",minI,minJ,minDist,Table[minI][minJ].Pos.x(),Table[minI][minJ].Pos.y());
		Quaternionf Quat = Table[minI][minJ].Quat;
		printf("Quat(%1.2f %1.2f %1.2f %1.2f)\n",Quat.x(),Quat.y(),Quat.z(),Quat.w());
		Vector3f Vect = Table[minI][minJ].Vect3D;
		printf("Vect(%1.2f %1.2f %1.2f)\n",Vect.x(),Vect.y(),Vect.z());
	}
	return Table[minI][minJ].Pos;
}
//-------------------------------------------------------------------------------------------------------
void CBallCahche_c::Printf()
{
	printf("Pos:_________________________________________________\n");
	for(int i=0;i<NbSteps;i++)
	for(int j=0;j<NbSteps;j++)
	{
		Eigen::Vector2f Pos = Table[i][j].Pos;
		printfVector2f(Pos);
		printf("(%1.2f,%1.2f)",Pos.x(),Pos.y());
		if(j==NbSteps-1)printf("---\n");
	}
	printf("Vect:_________________________________________________\n");
	for(int i=0;i<NbSteps;i++)
	for(int j=0;j<NbSteps;j++)
	{
		Eigen::Vector3f Vect3D = Table[i][j].Vect3D;
		printf("(%1.2f %1.2f %1.2f)",Vect3D.x(),Vect3D.y(),Vect3D.z());
		if(j==NbSteps-1)printf("---\n");
	}
	printf("Quat:_________________________________________________\n");
	for(int i=0;i<NbSteps;i++)
	for(int j=0;j<NbSteps;j++)
	{
		Eigen::Quaternionf Quat = Table[i][j].Quat;
		printf("(%1.2f %1.2f %1.2f %1.2f)",Quat.x(),Quat.y(),Quat.z(),Quat.w());
		if(j==NbSteps-1)printf("---\n");
	}
}
//-------------------------------------------------------------------------------------------------------
//				S3DJTConfigBall
//-------------------------------------------------------------------------------------------------------
S3DJTConfigBall::S3DJTConfigBall()
{
}
//-------------------------------------------------------------------------------------------------------
void S3DJTConfigBall::FillCache()
{
	bool isPrintf = false;
	CBallCahche_s VCache;
	Vector2f vPos(0,0);
	Quaternionf Qres;
	Vector3f Pos3D;
	Cache.NbSteps = 101;
	Cache.Table.resize(Cache.NbSteps);
	for(int i=0;i<Cache.NbSteps;i++)
	{
		Cache.Table[i].resize(Cache.NbSteps);
	}
	for(int i=0;i<Cache.NbSteps;i++)
	for(int j=0;j<Cache.NbSteps;j++)
	{
		VCache.Pos = Cache.IndexToPos(i,j);
		GeomProject_P2Q(VCache.Pos,VCache.Quat);
		VCache.Vect3D = VCache.Quat * DefaultRotAxis;
		Cache.Table[i][j] = VCache;
		if(isPrintf)
		{
			printf("i,j (%d,%d) Pos(%1.2f,%1.2f)\n",i,j,VCache.Pos.x(),VCache.Pos.y());
			g3d::printfVector3f(VCache.Vect3D,"VCache.Vect3D");
			g3d::printfVector3f(Cache.Table[i][j].Vect3D,"Cache.Table[i][j].Vect3D");
		}
	}
}
//-------------------------------------------------------------------------------------------------------
char S3DJTConfigBall::Optim_OutOfRange(Vector2f &TrackP)
{
	char OutOR = 0;
	if(TrackP.y()>PTop.y())
	{
		TrackP.y()=PTop.y();
		OutOR = 1;
	}
	if(TrackP.y()<PBottom.y())
	{
		TrackP.y()=PBottom.y();
		OutOR = 2;
	}
	if(TrackP.x()>PRight.x())
	{
		TrackP.x()=PRight.x();
		OutOR = 3;
	}
	if(TrackP.x()<PLeft.x())
	{
		TrackP.x()=PLeft.x();
		OutOR = 4;
	}
	return OutOR;
}
//-------------------------------------------------------------------------------------------------------
float S3DJTConfigBall::Optim_PQDist(const Vector2f &RefP,const Quaternionf &QTrack)
{
	//Here we will use the DefaultRotAxis;
	bool isPrintf = false;
	float d1 = (RefP-PTop).norm();
	float d2 = (RefP-PBottom).norm();
	float d3 = (RefP-PRight).norm();
	float d4 = (RefP-PLeft).norm();
#ifdef BADDISTANCE
	float a1 = QTrack.angularDistance(QTop);//To check first
	float a2 = QTrack.angularDistance(QBottom);
	float a3 = QTrack.angularDistance(QRight);
	float a4 = QTrack.angularDistance(QLeft);
#else
	Vector3f VTrack = QTrack*DefaultRotAxis;
	Vector3f VTop = QTop*DefaultRotAxis;
	Vector3f VBottom = QBottom*DefaultRotAxis;
	Vector3f VRight = QRight*DefaultRotAxis;
	Vector3f VLeft = QLeft*DefaultRotAxis;
	//could compute angular distance
	//float a1 = VTrack.dot(VTop); Cos(Teta);
	float a1 = (VTrack - VTop).norm();
	float a2 = (VTrack - VBottom).norm();
	float a3 = (VTrack - VRight).norm();
	float a4 = (VTrack - VLeft).norm();
#endif


	float C12 = abs(a1*d2-a2*d1);
	float C34 = abs(a3*d4-a4*d3);
	float C14 = abs(a1*d4-a4*d1);
	float C23 = abs(a2*d3-a3*d2);
	float C13 = abs(a1*d3-a3*d1);
	float C24 = abs(a2*d4-a4*d2);
	float TotalDist = (C12 + C34 + C14 + C23 + C13 + C24);
	if(isPrintf)printf("d(%1.3f , %1.3f , %1.3f , %1.3f) a(%1.3f , %1.3f , %1.3f , %1.3f)\n",d1,d2,d3,d4,a1,a2,a3,a4);
	if(isPrintf)printf("C12 34 14 23 13 24(%1.3f , %1.3f , %1.3f , %1.3f , %1.3f , %1.3f)\nTotalDist(%1.3f)\n",C12,C34,C14,C23,C13,C24,TotalDist);
	return TotalDist;
}
//-------------------------------------------------------------------------------------------------------
char S3DJTConfigBall::Optim_CalcGrad(const Vector2f &RefP,const Vector2f &TrackP,const float &DeltaGrad,Vector2f &Grad)
{
	bool isPrintf = false;
	char GradDir = 0;
	Vector2f GradVal(0,0);
	Vector2f Vdxp = TrackP;	Vdxp.x()+=DeltaGrad;
	Vector2f Vdxm = TrackP;	Vdxm.x()-=DeltaGrad;
	Vector2f Vdyp = TrackP;	Vdyp.y()+=DeltaGrad;
	Vector2f Vdym = TrackP;	Vdym.y()-=DeltaGrad;
	Quaternionf QTrack,Qdxp,Qdxm,Qdyp,Qdym;
	GeomProject_P2Q(TrackP,QTrack);
	GeomProject_P2Q(Vdxm,Qdxp);
	GeomProject_P2Q(Vdxp,Qdxm);
	GeomProject_P2Q(Vdyp,Qdyp);
	GeomProject_P2Q(Vdym,Qdym);
	float d = Optim_PQDist(RefP,QTrack);
	float dxp = Optim_PQDist(RefP,Qdxp);
	float dxm = Optim_PQDist(RefP,Qdxm);
	float dyp = Optim_PQDist(RefP,Qdyp);
	float dym = Optim_PQDist(RefP,Qdym);
	if(isPrintf)printf("d xpm ypm %f (%1.3f , %1.3f , %1.3f , %1.3f)\n",d,dxp,dxm,dyp,dym);
	//printf("d %f\n",d);
	if(dxp < d)
	{
		if(dxm < dxp)//This case should not happen, means our function has multiple mins from both sides !!!!!
		{
			Grad.x()=-DeltaGrad;
			GradDir = 2;
			GradVal.x() = dxm - d;
		}
		else
		{
			Grad.x()=DeltaGrad;
			GradDir = 1;
			GradVal.x() = d - dxp;
		}
	}
	else if(dxm < dxp)
	{
		Grad.x()=-DeltaGrad;
		GradDir = 2;
		GradVal.x() = dxm - d;
	}

	if(dyp < d)
	{
		if(dym < dyp)
		{
			Grad.y()=-DeltaGrad;
			GradDir = 4;
			GradVal.y() = dym - d;
		}
		else
		{
			Grad.y()=DeltaGrad;
			GradDir = 3;
			GradVal.y() = d - dyp;
		}
	}
	else if(dym < d)
	{
		Grad.y()=-DeltaGrad;
		GradDir = 4;
		GradVal.y() = dym - d;
	}
	Grad = GradVal;
	if(isPrintf)printf("GradVal %1.3f (%1.3f,%1.3f)",GradVal.norm(),GradVal.x(),GradVal.y());
	if(Grad.norm() < 0.008f)
	{
		GradDir = 0;
	}
	return GradDir;
}
//-------------------------------------------------------------------------------------------------------
void S3DJTConfigBall::GeomProject_P2Q(const Vector2f &vPos, Quaternionf &Qresult)
{
	float ratioP1,ratioP2;
	ratioP1 = Point2DProject(PLeft,PRight,vPos);
	Vector2f ProjectedPoint1 = Point2DInterpolate(PLeft,PRight,ratioP1);
	Quaternionf QInter1;
	QInter1 = QLeft.slerp(ratioP1,QRight);
	if(Point2DHalfPlane(vPos,ProjectedPoint1,PTop))
	{
		ratioP2 = Point2DProject(ProjectedPoint1,PTop,vPos);
		Qresult = QInter1.slerp(ratioP2,QTop);
	}
	else
	{
		ratioP2 = Point2DProject(ProjectedPoint1,PBottom,vPos);
		Qresult = QInter1.slerp(ratioP2,QBottom);
	}
}
//-------------------------------------------------------------------------------------------------------
void S3DJTConfigBall::DoubleSlerp_P2Q(const Vector2f &vPos, Quaternionf &Qresult)
{
	float ratioP1,ratioP2;
	ratioP1 = Point2DProject(PLeft,PRight,vPos);
	ratioP2 = Point2DProject(PTop,PBottom,vPos);
	Vector2f ProjectedPoint1 = Point2DInterpolate(PLeft,PRight,ratioP1);
	Quaternionf QInter1,QInter2;
	QInter1 = QLeft.slerp(ratioP1,QRight);
	QInter2 = QTop.slerp(ratioP2,QBottom);

	Qresult = QInter1 * QInter2;
	//Qresult = QInter2 * QInter1;
}
//-------------------------------------------------------------------------------------------------------
void S3DJTConfigBall::GeomProject_Q2P(const Quaternionf &QLocal,Vector2f &PRes)
{
	Vector3f VLocal = QLocal * DefaultRotAxis;
	PRes = Cache.Find(VLocal);
}
//-------------------------------------------------------------------------------------------------------
void S3DJTConfigBall::Optimise_Q2P(const Quaternionf &Qref,Vector2f &PosRes)
{
	
}
//-------------------------------------------------------------------------------------------------------
void S3DJTConfigBall::Optimise_P2Q(const Vector2f &vPos, Quaternionf &Qresult)
{/*
	Vector2f RefP(0,0);
	//vPos = RefP;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	cv::Mat MatRes(200,200,CV_8UC3);
	float MaxVal = 0;
	for(int j=0;j<200;j++)
	for(int i=0;i<200;i++)
	{
		Vector2f vPos(((float)i)/200.0,((float)j)/200.0);
		Quaternionf QTrack;
		GeomProject_P2Q(vPos,QTrack);
		float Res = Optim_PQDist(RefP,QTrack);
		if(Res>MaxVal)MaxVal = Res;
		PIXEL8UC3_1(MatRes,i,(200-j)) = (uchar)(Res*40.0);
		PIXEL8UC3_2(MatRes,i,(200-j)) = (uchar)(Res*40.0);
		PIXEL8UC3_3(MatRes,i,(200-j)) = (uchar)(Res*40.0);
	}
	printf("MaxVal %1.3f\n",MaxVal);
	*/
	bool isPrintf = false;
	Vector2f TrackP(0.5,0.5);
	char Step;
	Vector2f Grad(0,0);
	float DeltaStep = 0.02f;
	float DeltaGrad = 0.005f;//if deltaGrad < DeltaStep then it never ends !!!!!
	int IterLeft = 30;
	do
	{
		TrackP-=Grad;//20 = DeltaStep / DeltaGrad;
		if(Optim_OutOfRange(TrackP))
		{
			Step = 0;
		}
		else
		{
			Step = Optim_CalcGrad(vPos,TrackP,DeltaGrad,Grad);
		}
		IterLeft--;
		if(isPrintf)printf(" IterLeft:%d P(%1.3f,%1.3f) Grad(%1.3f,%1.3f)\n",IterLeft,TrackP.x(),TrackP.y(),Grad.x(),Grad.y());
		//cv::Point PDraw(TrackP.x()*200,TrackP.y()*200);
		//cv::circle(MatRes,PDraw,1,mcv::clGreen);
		//printf("PDraw(%d,%d)\n",PDraw.x,PDraw.y);
	}while((Step!=0)&&(IterLeft!=0));
	GeomProject_P2Q(TrackP,Qresult);
	if(isPrintf)printf("P(%1.3f,%1.3f)",vPos.x(),vPos.y());
	if(isPrintf)printf("TrackP(%1.3f,%1.3f), Q(%1.3f,%1.3f,%1.3f,%1.3f)\n",TrackP.x(),TrackP.y(),TrackP,Qresult.x(),Qresult.y(),Qresult.z(),Qresult.w());

	/*cv::imshow("IMGTestDist",MatRes);
	cv::waitKey();*/
}
void S3DJTConfigBall::TestDrawPQ()
{
	Vector2f RefP(0,0);
	cv::Mat MatRes(200,200,CV_8UC1);
	float MaxVal = 0;
	for(int j=0;j<200;j++)
	for(int i=0;i<200;i++)
	{
		Vector2f vPos(((float)i)/200.0,((float)j)/200.0);
		Quaternionf QTrack;
		GeomProject_P2Q(vPos,QTrack);
		float Res = Optim_PQDist(RefP,QTrack);
		if(Res>MaxVal)MaxVal = Res;
		PIXEL8UC1(MatRes,i,j) = (uchar)(Res*40.0);
	}
	printf("MaxVal %1.3f\n",MaxVal);
	cv::imshow("IMGTestDist",MatRes);
	cv::waitKey();
}
//-------------------------------------------------------------------------------------------------------
Matrix4f S3DJTConfigBall::GetLocalFromConf()
{
	Matrix4f MRotAxis;
	Quaternionf QInterRes;
#ifdef OPTIMISE_QUAT
	Optimise_P2Q(Pos,QInterRes);
#endif
#ifdef GEOMPROJECT_P2Q
	GeomProject_P2Q(Pos,QInterRes);
#else
	DoubleSlerp_P2Q(Pos,QInterRes);
#endif
	Eigen::Matrix3f M3 = Eigen::Matrix3f(QInterRes);
	Vector3f NewRotAxis = M3 * DefaultRotAxis;//RotAxis is the Default Rotation Axis
	Quaternionf QRotAxis(Eigen::AngleAxisf(Angle,NewRotAxis));
	return Matrix3fTo4f(Eigen::Matrix3f(QRotAxis * QInterRes));// Had to be inverted
}
//-------------------------------------------------------------------------------------------------------
void S3DJTConfigBall::SetConfFromLocal(Matrix4f &mLocal)
{
	Quaternionf QLocal(Matrix4fTo3f(mLocal));
	GeomProject_Q2P(QLocal,Pos);
	
	Quaternionf QLocalRef;
	GeomProject_P2Q(Pos,QLocalRef);

	Angle = QLocalRef.angularDistance(QLocal);
	if(Angle<MinAngle)Angle = MinAngle;
	else if(Angle>MaxAngle)Angle = MaxAngle;
}

//-------------------------------------------------------------------------------------------------------
void S3DJTConfigBall::SetParam_Angle(float paramFrom0To1)
{
	Angle = paramFrom0To1 * (MaxAngle - MinAngle) + MinAngle;
}
//-------------------------------------------------------------------------------------------------------
void S3DJTConfigBall::GetParam_Angle(float &paramFrom0To1)
{
	paramFrom0To1 = (Angle - MinAngle) /  (MaxAngle - MinAngle);
}
//--------------------------------------------------------------------------------------------------------------
//										ModelNode_s
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
ModelNode_s::ModelNode_s()
{
}
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
void ModelNode_s::UpdateCurrent()// !!! Hierarchy has to be respected when calling this function
{
	if(pParent)
	{
		mCurrent = pParent->mCurrent * mDefault * mLocal;
	}
	else//Root has no parent
	{
		mCurrent = mDefault * mLocal;
	}
	Pos = TranslationVector(mCurrent);//Absolute Pos
}
//--------------------------------------------------------------------------------------------------------------
void ModelNode_s::UpdateCurrentHierarchy()
{
	UpdateCurrent();
	if(pFirstChild)
	{
		pFirstChild->UpdateCurrentHierarchy();
	}
}
//--------------------------------------------------------------------------------------------------------------
void ModelNode_s::GetParams(float &vfx,float &vfy)
{
	if (Joint.JT == S3DJT_Revolute)
	{
		pParent->Joint.ConfBall.GetParam_Angle(fx);
		Joint.ConfRev.GetParam(fy);
	}
	else if (Joint.JT == S3DJT_Ball)
	{
		fx = Joint.ConfBall.Pos.x();
		fy = Joint.ConfBall.Pos.y();
	}
	vfx = fx;
	vfy = fy;
}
//--------------------------------------------------------------------------------------------------------------
void ModelNode_s::SetParams(float vfx,float vfy)
{
	fx = vfx;
	fy = vfy;
	if (Joint.JT == S3DJT_Revolute)
	{
		pParent->Joint.ConfBall.SetParam_Angle(fx);
		Joint.ConfRev.SetParam(fy);
	}
	else if (Joint.JT == S3DJT_Ball)
	{
		Joint.ConfBall.Pos.x() = fx;
		Joint.ConfBall.Pos.y() = fy;
	}
}
//--------------------------------------------------------------------------------------------------------------
void ModelNode_s::MoveTo(float vfx,float vfy)
{
	SetParams(vfx,vfy);
	if (Joint.JT == S3DJT_Revolute)//Then The parent has been affected
	{
		pParent->UpdateLocalFromConfig();
		UpdateLocalFromConfig();
		pParent->UpdateCurrentHierarchy();
	}
	else if (Joint.JT == S3DJT_Ball)
	{
		UpdateLocalFromConfig();
		UpdateCurrentHierarchy();
	}
}
//--------------------------------------------------------------------------------------------------------------
void ModelNode_s::MoveToAbsolute(const Vector4f &AbsolutePos)
{
	if((Joint.JT == S3DJT_Ball) || (Joint.JT == S3DJT_Revolute))
	{
		float minLength = FLT_MAX;
		float minFx,minFy;
		float StartLimit = 0.0f;
		float StopLimit = 1.05f;
		for(float vfy=StartLimit;vfy<StopLimit;vfy+=0.1f)
		for(float vfx=StartLimit;vfx<StopLimit;vfx+=0.1f)
		{
			Vector4f CPos = GetChildPosFromParams(vfx,vfy);
			Vector3f VectLength = V4To3(AbsolutePos - CPos);
			float VLength = VectLength.norm();
			if (VLength < minLength) 
			{
				minLength = VLength;
				minFx = vfx;
				minFy = vfy;
			}
		}
		MoveTo(minFx,minFy);
	}
	else
	{
		printf("don't know how to move this type of Node, ask for (Child's list) feature to handle this\n");
	}
}
//--------------------------------------------------------------------------------------------------------------
void ModelNode_s::MoveToAbsoluteCheckAll(const Vector4f &AbsolutePos,float precision)
{
	float minLength = FLT_MAX;
	float minFx,minFy;
	float StartLimit = 0.0f;
	float StopLimit = 1.05f;
	for(float vfy=StartLimit;vfy<StopLimit;vfy+=precision)
	for(float vfx=StartLimit;vfx<StopLimit;vfx+=precision)
	{
		Vector4f CPos = GetChildPosFromParams(vfx,vfy);
		Vector3f VectLength = V4To3(AbsolutePos - CPos);
		float VLength = VectLength.norm();
		if (VLength < minLength) 
		{
			minLength = VLength;
			minFx = vfx;
			minFy = vfy;
		}
	}
	MoveTo(minFx,minFy);
}
//--------------------------------------------------------------------------------------------------------------
void ModelNode_s::MoveToAbsoluteCheckAllDouble(const Vector4f &AbsolutePos,float precision1,float precision2)
{
	float minLength = FLT_MAX;
	float minFx,minFy;
	float StartLimit = 0.0f;
	float StopLimit = 1.05f;
	for(float vfy=StartLimit;vfy<StopLimit;vfy+=precision1)
	for(float vfx=StartLimit;vfx<StopLimit;vfx+=precision1)
	{
		Vector4f CPos = GetChildPosFromParams(vfx,vfy);
		Vector3f VectLength = V4To3(AbsolutePos - CPos);
		float VLength = VectLength.norm();
		if (VLength < minLength) 
		{
			minLength = VLength;
			minFx = vfx;
			minFy = vfy;
		}
	}
	float StartFx = minFx - precision1;
	float StopFx = minFx + precision1;
	float StartFy = minFy - precision1;
	float StopFy = minFy + precision1;

	for(float vfy=StartFx;vfy<StopFx;vfy+=precision2)
	for(float vfx=StartFy;vfx<StopFy;vfx+=precision2)
	{
		Vector4f CPos = GetChildPosFromParams(vfx,vfy);
		Vector3f VectLength = V4To3(AbsolutePos - CPos);
		float VLength = VectLength.norm();
		if (VLength < minLength) 
		{
			minLength = VLength;
			minFx = vfx;
			minFy = vfy;
		}
	}


	MoveTo(minFx,minFy);
}
//--------------------------------------------------------------------------------------------------------------
Vector4f ModelNode_s::GetRelativePosToDefault(const Vector4f &WorldPos)
{
	if(pParent)
	{
		Matrix4f RevTransform;
		Vector4f ResVect;
		GetReverseTransform(RevTransform, pParent->mCurrent * mDefault);
		ResVect = RevTransform * WorldPos;
		return ResVect;
	}
	else
	{
		return WorldPos;
	}
	
}
//--------------------------------------------------------------------------------------------------------------
Vector4f ModelNode_s::GetRelativePos(const Vector4f &WorldPos)//returns the Node relative Pos
{
	// Caaarefull !!!! Child Pos always return the same as in the same reference of mCurrent, rather use ToDefault
	Matrix4f RevTransform;
	GetReverseTransform(RevTransform,mCurrent);
	Vector4f ResVect = RevTransform * WorldPos;
	return ResVect;
}
//--------------------------------------------------------------------------------------------------------------
void ModelNode_s::Translate(const Vector4f &VTr)
{
	mLocal = mLocal * MatrixTranslation(VTr.x(),VTr.y(),VTr.z());
}
//--------------------------------------------------------------------------------------------------------------
void ModelNode_s::Rotate(float alpha)
{
	mLocal = mLocal * MatrixRotationY(alpha);
}
//-------------------------------------------------------------------------------------------------------
void ModelNode_s::UpdateLocalFromConfig()
{
	
	if(Joint.JT == S3DJT_Revolute)
	{
		mLocal = Joint.ConfRev.GetLocalFromConf();
	}
	else if(Joint.JT == S3DJT_Ball)//
	{
		mLocal = Joint.ConfBall.GetLocalFromConf();
	}
	else
	{
		mLocal = Matrix4f::Identity();
	}
}
//-------------------------------------------------------------------------------------------------------
void ModelNode_s::UpdateConfigFromLocalMat(const Matrix4f &vMat)
{
	mLocal = vMat;
	if(Joint.JT == S3DJT_Revolute)
	{
//		Joint.ConfRev.GetConfFromLocal(mLocal);//Not Yet
	}
	else if(Joint.JT == S3DJT_Ball)//
	{
		Joint.ConfBall.SetConfFromLocal(mLocal);
		fx = Joint.ConfBall.Pos.x();
		fy = Joint.ConfBall.Pos.y();
		Joint.ConfBall.GetParam_Angle(pFirstChild->fx);//child Fx updated
	}
}
//--------------------------------------------------------------------------------------------------------------
Vector4f ModelNode_s::GetChildPosFromParams(float vfx,float vfy)// /!\ Process it !!!!!!!!!!! Do not take it from params
{
	Vector4f PosRes;
#ifdef DEPRECATEDCACHE_USE
	bool IsInterpolate = true;
	if((fx<0) || (fx>1) || (fy<0) || (fy>0) )
	{
		return Vector4f(0,0,0,1);
	}
	if(IsInterpolate)
	{
		int Xindex = (int)floor(fx/0.1f);
		int Yindex = (int)floor(fy/0.1f);
		Vector4f Ph1 = LocalsCache[Yindex * 11 + Xindex].V;
		Vector4f Ph2 = LocalsCache[Yindex * 11 + (Xindex+1)].V;
		Vector4f Pb1 = LocalsCache[(Yindex+1) * 11 + Xindex].V;
		Vector4f Pb2 = LocalsCache[(Yindex+1) * 11 + (Xindex+1)].V;
		float Xcoeff = (fx/0.1f) - floor(fx/0.1f);
		float Ycoeff = (fy/0.1f) - floor(fy/0.1f);
		Vector4f PhAvg = Xcoeff * Ph1 + (1-Xcoeff) * Ph2;
		Vector4f PbAvg = Xcoeff * Pb1 + (1-Xcoeff) * Pb2;
		PosRes = Ycoeff * PhAvg + (1-Ycoeff) * PbAvg;
	}
	else
	{
		int Xindex = (int)floor((fx/0.1f)+0.5f);
		int Yindex = (int)floor((fy/0.1f)+0.5f);
		PosRes = LocalsCache[Yindex * 11 + Xindex].V;
	}
#else
	float tempFx = fx;
	float tempFy = fy;
	MoveTo(vfx,vfy);
	PosRes = pFirstChild->Pos;
	MoveTo(tempFx,tempFy);
#endif
	return PosRes;
}
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//										WireModel_c
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
WireModel_c::WireModel_c():Motion(NbConfigParams)
{
	pNodes.resize(NbNodes);
	for(int i=0;i<NbNodes;i++)pNodes[i] = NULL;

	InitNodes();
	Init();
	InitSkel();
	//UpdateLocals
	ResetLocals();
	//Root.mLocal = S3DMatrixRotationX((float)-PI/2);
	UpdateDefault();
	UpdateCurrent();
}
//--------------------------------------------------------------------------------------------------------------
WireModel_c::~WireModel_c()
{
	for(int i=0;i<NbNodes;i++)
	{
		if(pNodes[i])
		{
			//delete pNodes[i];
			pNodes[i] = NULL;
		}
	}
}
//--------------------------------------------------------------------------------------------------------------
void WireModel_c::InitSkel()
{
	float ArmsAxisLimitAngle = PI/2;
	float LegsAxisLimitAngle = PI/2;
	float ForeLimbsAngle = 3*PI/4;
	float RevolveAngle = PI/2;
	S3DJTConfigBall	*pConfigBall;
	S3DJTConfigRev	*pConfigRev;
	//------------------------------------------------------------
	for(int i=0;i<NbNodes;i++)
	{
		pNodes[i]->Joint.ConfBall.PLeft		= Vector2f(0.0f,0.5f);
		pNodes[i]->Joint.ConfBall.PRight	= Vector2f(1.0f,0.5f);
		pNodes[i]->Joint.ConfBall.PTop		= Vector2f(0.5f,1.0f);
		pNodes[i]->Joint.ConfBall.PBottom	= Vector2f(0.5f,0.0f);
		pNodes[i]->Joint.JT = S3DJT_Fixed;
	}
	pRoot->Joint.JT = S3DJT_Free;
	//By default, specific ones will be updated by load
	//Pelvis------------------------------------------------------------
	pPelvis->Joint.JT = S3DJT_Free;//No more Revolute
	/*pConfigRev = &Pelvis->Joint.ConfRev;
	pConfigRev->RotAxis = Vector3f::UnitY();
	pConfigRev->Angle = 0;
	pConfigRev->MaxAngle = PI/2;
	pConfigRev->MinAngle = -PI/2;*/
	//Right Shoulder--------------------------------------------------------
	pRShoulder->Joint.JT = S3DJT_Ball;
	pConfigBall = &pRShoulder->Joint.ConfBall;
	//pConfigBall->QRight		= Quaternionf::Identity();
	pConfigBall->QRight		= Quaternionf(Eigen::AngleAxisf(-ArmsAxisLimitAngle,Vector3f::UnitY()));
	pConfigBall->QLeft		= Quaternionf(Eigen::AngleAxisf(ArmsAxisLimitAngle,Vector3f::UnitY()));
	pConfigBall->QTop		= Quaternionf(Eigen::AngleAxisf(-ArmsAxisLimitAngle,Vector3f::UnitZ()));
	pConfigBall->QBottom	= Quaternionf(Eigen::AngleAxisf(ArmsAxisLimitAngle,Vector3f::UnitZ()));
	pConfigBall->Pos = Vector2f(1.0f,0.5f);
	pConfigBall->DefaultRotAxis = -Vector3f::UnitX();// /!\ ! NO AXIS NEEDED HERE - use as default
	pConfigBall->Angle = 0;
	pConfigBall->MaxAngle = RevolveAngle;
	pConfigBall->MinAngle = -RevolveAngle;
	//---------------------------------------------------------------------
	//Right Elbow----------------------------------------------------------
	pRElbow->Joint.JT = S3DJT_Revolute;
	pConfigRev = &pRElbow->Joint.ConfRev;
	pConfigRev->RotAxis = Vector3f::UnitY();
	pConfigRev->Angle = 0;
	pConfigRev->MaxAngle = ForeLimbsAngle;
	pConfigRev->MinAngle = 0;
	
	//Left Shoulder--------------------------------------------------------
	pLShoulder->Joint.JT = S3DJT_Ball;
	pConfigBall = &pLShoulder->Joint.ConfBall;
	//pConfigBall->QLeft		= Quaternionf::Identity();
	pConfigBall->QLeft		= Quaternionf(Eigen::AngleAxisf(ArmsAxisLimitAngle,Vector3f::UnitY()));
	pConfigBall->QRight		= Quaternionf(Eigen::AngleAxisf(-ArmsAxisLimitAngle,Vector3f::UnitY()));
	pConfigBall->QTop		= Quaternionf(Eigen::AngleAxisf(ArmsAxisLimitAngle,Vector3f::UnitZ()));
	pConfigBall->QBottom	= Quaternionf(Eigen::AngleAxisf(-ArmsAxisLimitAngle,Vector3f::UnitZ()));
	pConfigBall->Pos = Vector2f(0.0f,0.5f);
	pConfigBall->DefaultRotAxis = Vector3f::UnitX();// /!\ ! NO AXIS NEEDED HERE - use as default
	pConfigBall->Angle = 0;
	pConfigBall->MaxAngle = RevolveAngle;
	pConfigBall->MinAngle = -RevolveAngle;
	
	//Left Elbow--------------------------------------------------------
	pLElbow->Joint.JT = S3DJT_Revolute;
	pConfigRev = &pLElbow->Joint.ConfRev;
	pConfigRev->RotAxis = -Vector3f::UnitY();
	pConfigRev->Angle = 0;
	pConfigRev->MaxAngle = ForeLimbsAngle;
	pConfigRev->MinAngle = 0;
	
	//Right Thigh--------------------------------------------------------
	pRThigh->Joint.JT = S3DJT_Ball;
	pConfigBall = &pRThigh->Joint.ConfBall;
	//pConfigBall->QLeft		= Quaternionf::Identity();
	pConfigBall->QLeft		= Quaternionf(Eigen::AngleAxisf(LegsAxisLimitAngle,Vector3f::UnitZ()));
	pConfigBall->QRight		= Quaternionf(Eigen::AngleAxisf(-LegsAxisLimitAngle,Vector3f::UnitZ()));
	pConfigBall->QTop		= Quaternionf(Eigen::AngleAxisf(LegsAxisLimitAngle,Vector3f::UnitX()));
	pConfigBall->QBottom	= Quaternionf(Eigen::AngleAxisf(-LegsAxisLimitAngle,Vector3f::UnitX()));
	pConfigBall->Pos = Vector2f(0.0f,0.5f);
	pConfigBall->DefaultRotAxis = -Vector3f::UnitY();// /!\ ! NO AXIS NEEDED HERE - use as default
	pConfigBall->Angle = 0;
	pConfigBall->MaxAngle = RevolveAngle;
	pConfigBall->MinAngle = -RevolveAngle;

	//Right Knee--------------------------------------------------------
	pRKnee->Joint.JT = S3DJT_Revolute;
	pConfigRev = &pRKnee->Joint.ConfRev;
	pConfigRev->RotAxis = Vector3f::UnitX();
	pConfigRev->Angle = 0;
	pConfigRev->MaxAngle = ForeLimbsAngle;
	pConfigRev->MinAngle = 0;
	
	//Left Thigh--------------------------------------------------------
	pLThigh->Joint.JT = S3DJT_Ball;
	pConfigBall = &pLThigh->Joint.ConfBall;
	//pConfigBall->QRight		= Quaternionf::Identity();
	pConfigBall->QRight		= Quaternionf(Eigen::AngleAxisf(-LegsAxisLimitAngle,Vector3f::UnitZ()));
	pConfigBall->QLeft		= Quaternionf(Eigen::AngleAxisf(LegsAxisLimitAngle,Vector3f::UnitZ()));
	pConfigBall->QTop		= Quaternionf(Eigen::AngleAxisf(LegsAxisLimitAngle,Vector3f::UnitX()));
	pConfigBall->QBottom	= Quaternionf(Eigen::AngleAxisf(-LegsAxisLimitAngle,Vector3f::UnitX()));
	//pConfigBall->PTop		= Vector2f(1.0f,1.0f);
	//pConfigBall->PBottom	= Vector2f(1.0f,0.0f);
	pConfigBall->Pos = Vector2f(1.0f,0.5f);
	pConfigBall->DefaultRotAxis = -Vector3f::UnitY();// /!\ ! NO AXIS NEEDED HERE - use as default
	pConfigBall->Angle = 0;
	pConfigBall->MaxAngle = RevolveAngle;
	pConfigBall->MinAngle = -RevolveAngle;

	//Left Knee--------------------------------------------------------
	pLKnee->Joint.JT = S3DJT_Revolute;
	pConfigRev = &pLKnee->Joint.ConfRev;
	pConfigRev->RotAxis = Vector3f::UnitX();
	pConfigRev->Angle = 0;
	pConfigRev->MaxAngle = ForeLimbsAngle;
	pConfigRev->MinAngle = 0;

	//Initialise the cache after affecting the default Axis
	for(int i=0;i<NbNodes;i++)
	{
		if(pNodes[i]->Joint.JT == S3DJT_Ball)
			pNodes[i]->Joint.ConfBall.FillCache();
	}
}
//--------------------------------------------------------------------------------------------------------------
void WireModel_c::InitNodes()//will be replaced by load
{
	Nodes.resize(NbNodes);
	for(int i=0;i<NbNodes;i++)
	{
		//pNodes[i] = new (std::nothrow) ModelNode_s;
		pNodes[i] = &Nodes[i];
	}

	pRoot	= pNodes[0];		//Free
	pPelvis = pNodes[1];		//Free
	pLThigh = pNodes[2];		//Ball
	pRThigh = pNodes[3];		//Ball
	pLKnee = pNodes[4];			//Revolute
	pRKnee = pNodes[5];			//Revolute
	pLFoot = pNodes[6];			//--
	pRFoot = pNodes[7];			//--
	pChest = pNodes[8];			//--
	pLShoulder = pNodes[9];		//Ball
	pRShoulder = pNodes[10];	//Ball
	pLElbow = pNodes[11];		//Revolute
	pRElbow = pNodes[12];		//Revolute
	pLHand = pNodes[13];		//--
	pRHand = pNodes[14];		//--
	pHead = pNodes[15];			//--
	pNeck = pNodes[16];			//--
}
//--------------------------------------------------------------------------------------------------------------
void WireModel_c::Init()//will be replaced by load
{

	pDimentions[0] = &hBody;
	pDimentions[1] = &wShoulders,
	pDimentions[2] = &wWaist;
	pDimentions[3] = &Arm;
	pDimentions[4] = &Forearm;
	pDimentions[5] = &Leg;
	pDimentions[6] = &Calf;
	pDimentions[7] = &Neck;

	//units are in cm
	Scale = 100;

	hBody = 0.48f * Scale;
	wShoulders = 0.30f * Scale;
	wWaist = 0.25f * Scale;
	Arm = 0.25f * Scale;
	Forearm = 0.25f * Scale;
	Leg = 0.40f * Scale;
	Calf = 0.43f * Scale;
	Neck = 0.15f * Scale;


	sprintf_s(pRoot->Name,32,"Root\0");
	sprintf_s(pPelvis->Name,32,"Pelvis\0");
	sprintf_s(pLThigh->Name,32,"LThigh\0");
	sprintf_s(pRThigh->Name,32,"RThigh\0");
	sprintf_s(pLKnee->Name,32,"LKnee\0");
	sprintf_s(pRKnee->Name,32,"RKnee\0");
	sprintf_s(pLFoot->Name,32,"LFoot\0");
	sprintf_s(pRFoot->Name,32,"RFoot\0");
	sprintf_s(pChest->Name,32,"Chest\0");
	sprintf_s(pLShoulder->Name,32,"LShoulder\0");
	sprintf_s(pRShoulder->Name,32,"RShoulder\0");
	sprintf_s(pLElbow->Name,32,"LElbow\0");
	sprintf_s(pRElbow->Name,32,"RElbow\0");
	sprintf_s(pLHand->Name,32,"LHand\0");
	sprintf_s(pRHand->Name,32,"RHand\0");
	sprintf_s(pHead->Name,32,"Head\0");
	sprintf_s(pNeck->Name,32,"Neck\0");

	//Parents relations
	pRoot->pParent = NULL;
	pPelvis->pParent = pRoot;
	pLThigh->pParent = pPelvis;
	pRThigh->pParent = pPelvis;
	pLKnee->pParent = pLThigh;
	pRKnee->pParent = pRThigh;
	pLFoot->pParent = pLKnee;
	pRFoot->pParent = pRKnee;
	pChest->pParent = pPelvis;
	pLShoulder->pParent = pChest;
	pRShoulder->pParent = pChest;
	pLElbow->pParent = pLShoulder;
	pRElbow->pParent = pRShoulder;
	pLHand->pParent = pLElbow;
	pRHand->pParent = pRElbow;
	pHead->pParent = pNeck;
	pNeck->pParent = pChest;

	//children Relations
	pRoot->pFirstChild = pPelvis;
	pPelvis->pFirstChild = pChest;
	pLThigh->pFirstChild = pLKnee;
	pRThigh->pFirstChild = pRKnee;
	pLKnee->pFirstChild = pLFoot;
	pRKnee->pFirstChild = pRFoot;
	pLFoot->pFirstChild = NULL;
	pRFoot->pFirstChild = NULL;
	pChest->pFirstChild = pNeck;
	pLShoulder->pFirstChild = pLElbow;
	pRShoulder->pFirstChild = pRElbow;
	pLElbow->pFirstChild = pLHand;
	pRElbow->pFirstChild = pRHand;
	pLHand->pFirstChild = NULL;
	pRHand->pFirstChild = NULL;
	pHead->pFirstChild = NULL;
	pNeck->pFirstChild = pHead;

	pRoot->Color = cv::Scalar(0,0,0);
	pPelvis->Color = cv::Scalar(0,0,255);
	pLThigh->Color = cv::Scalar(255,0,100);
	pRThigh->Color = cv::Scalar(0,255,100);
	pLKnee->Color = cv::Scalar(255,0,50);
	pRKnee->Color = cv::Scalar(0,255,50);
	pLFoot->Color = cv::Scalar(255,0,0);
	pRFoot->Color = cv::Scalar(0,255,0);
	pChest->Color = cv::Scalar(0,0,255);
	pLShoulder->Color = cv::Scalar(255,0,150);
	pRShoulder->Color = cv::Scalar(0,255,150);
	pLElbow->Color = cv::Scalar(255,0,200);
	pRElbow->Color = cv::Scalar(0,255,200);
	pLHand->Color = cv::Scalar(255,0,255);
	pRHand->Color = cv::Scalar(0,255,255);
	pHead->Color = cv::Scalar(0,0,255);
	pNeck->Color = cv::Scalar(0,0,150);

}

//--------------------------------------------------------------------------------------------------------------
void WireModel_c::PrintNodes()
{
	int PmInd = 0;
	for(int i=0;i<NbNodes;i++)
			{
				printf("Node[%d] is %s with JT:",i,pNodes[i]->Name);
				switch(pNodes[i]->Joint.JT)
				{
					case S3DJT_Ball		:printf(" JT_Ball params(%d,%d %d)\n",PmInd,PmInd+1,PmInd+2);PmInd+=3;break;
					case S3DJT_Revolute	:printf(" JT_Revolute params(%d)\n",PmInd++);break;
					case S3DJT_Fixed	:printf(" JT_Fixed np()\n");break;
					case S3DJT_Free		:printf(" JT_Free np()\n");break;
				}
			}
}
//--------------------------------------------------------------------------------------------------------------
void WireModel_c::load(const char* FileName)
{
	S3DJTConfigBall	*pConfigBall;
	S3DJTConfigRev	*pConfigRev;
	char JointTypeName[20];
	std::string line;
	std::ifstream myfile(FileName);
	if (myfile.is_open())
	{
		getline (myfile,line);//	"//Simple Line index parsing - cudaD3DEnv Skel_c::Save3()\n";
		getline (myfile,line);//	"Skeleton Dimentions:\n";
		Scale		= readLineFloatLine(myfile);
		hBody		= readLineFloatLine(myfile) * Scale;
		wShoulders	= readLineFloatLine(myfile) * Scale;
		wWaist		= readLineFloatLine(myfile) * Scale;
		Arm			= readLineFloatLine(myfile) * Scale;
		Forearm		= readLineFloatLine(myfile) * Scale;
		Leg			= readLineFloatLine(myfile) * Scale;
		Calf		= readLineFloatLine(myfile) * Scale;
		Neck		= readLineFloatLine(myfile) * Scale;
		//------------------------------Set Dimentions to nodes bone Length
		getline (myfile,line);//NbNodes
		int vNbNodes;
		myfile >> vNbNodes;		getline(myfile,line);
		assert(vNbNodes == NbNodes);
		for(UINT i=0;i<NbNodes;i++)
		{
			getline (myfile,line);getline(myfile,line);sprintf_s(pNodes[i]->Name,32,"%s\0",line.c_str());
			getline (myfile,line);sprintf_s(JointTypeName,20,"%s\0",line.c_str());

			if (strcmp(JointTypeName,"S3DJT_Revolute") == 0)
			{
				pNodes[i]->Joint.JT = S3DJT_Revolute;
				pConfigRev = &pNodes[i]->Joint.ConfRev;
				getline (myfile,line);//"Rotation Axis : Axis(x,y,z) - Angle - Max - Min\n"
				pConfigRev->RotAxis		= readVector3f(myfile);
				pConfigRev->Angle		= readFloatLine(myfile);
				pConfigRev->MaxAngle	= readFloatLine(myfile);
				pConfigRev->MinAngle	= readFloatLine(myfile);
			}
			else if (strcmp(JointTypeName,"S3DJT_Ball") == 0)
			{
				pNodes[i]->Joint.JT = S3DJT_Ball;
				pConfigBall = &pNodes[i]->Joint.ConfBall;

				getline (myfile,line);//"Reference Points : Pos(x,y) - PRight(x,y) - PLeft(x,y) - PTop(x,y) - PBottom(x,y)\n";
				pConfigBall->Pos		= readVector2f(myfile);
				pConfigBall->PRight		= readVector2f(myfile);
				pConfigBall->PLeft		= readVector2f(myfile);
				pConfigBall->PTop		= readVector2f(myfile);
				pConfigBall->PBottom	= readVector2f(myfile);

				getline (myfile,line);//"Reference Quaternions : QRight(x,y,z,w) - QLeft(x,y,z,w) - QTop(x,y,z,w) - QBottom(x,y,z,w)\n";
				pConfigBall->QRight		= readQuaternionf(myfile);
				pConfigBall->QLeft		= readQuaternionf(myfile);
				pConfigBall->QTop		= readQuaternionf(myfile);
				pConfigBall->QBottom	= readQuaternionf(myfile);

				getline (myfile,line);//"Rotation Axis : Axis(x,y,z) - Angle - Max - Min\n"
				pConfigBall->DefaultRotAxis	= readVector3f(myfile);
				pConfigBall->Angle		= readFloatLine(myfile);
				pConfigBall->MaxAngle	= readFloatLine(myfile);
				pConfigBall->MinAngle	= readFloatLine(myfile);
			}//else nothing more to do for Free and Fixed as name already set
		}
		//Initialise the cache after affecting the default Axis
		for(int i=0;i<NbNodes;i++)
		{
			if(pNodes[i]->Joint.JT == S3DJT_Ball)
				pNodes[i]->Joint.ConfBall.FillCache();
		}


		printf("Model loaded from file (%s)\n",FileName);
		myfile.close();

		UpdateDefault();
		//UpdateConfig(); - Later with the specific Params[]
		//UpdateLocals(); // Locals initialised with Identity
		UpdateCurrent();
		PrintNodes();
	}
	else
	{
		printf("File (%s) Not available\n",FileName);
		printf("Creating File (%s)\n",FileName);
		save(FileName);
	}
}
//--------------------------------------------------------------------------------------------------------------
void WireModel_c::save(const char* FileName)
{
	S3DJTConfigBall	*pConfigBall;
	S3DJTConfigRev	*pConfigRev;
	char text[255];
	std::ofstream myfile;
	myfile.open(FileName);
	myfile << "//Simple Line index parsing - cudaD3DEnv Skel_c::Save3()\n";
	myfile << "Skeleton Dimentions:\n";
	myfile << "Scale:\n";		myfile << Scale				<< std::endl;
	myfile << "hBody:\n";		myfile << (hBody/Scale)		<< std::endl;
	myfile << "wShoulders:\n";	myfile << (wShoulders/Scale)<< std::endl;
	myfile << "wWaist:\n";		myfile << (wWaist/Scale)	<< std::endl;
	myfile << "Arm:\n";			myfile << (Arm/Scale)		<< std::endl;
	myfile << "Forearm:\n";		myfile << (Forearm/Scale)	<< std::endl;
	myfile << "Leg:\n";			myfile << (Leg/Scale)		<< std::endl;
	myfile << "Calf:\n";		myfile << (Calf/Scale)		<< std::endl;
	myfile << "Neck:\n";		myfile << (Neck/Scale)		<< std::endl;

	myfile << "NbNodes:\n";sprintf(text,"%d\n",NbNodes);
	myfile << text;
	for(UINT i=0;i<NbNodes;i++)
	{
		myfile << "Name:\n";			myfile << pNodes[i]->Name			<< std::endl;
		if(pNodes[i]->Joint.JT == S3DJT_Revolute)
		{
			myfile << "S3DJT_Revolute\n";
			myfile << "Rotation Axis : Axis(x,y,z) - Angle - Max - Min\n";
			pConfigRev = &pNodes[i]->Joint.ConfRev;
			writeVector3f(myfile,pConfigRev->RotAxis);
			myfile << pConfigRev->Angle		<< std::endl;
			myfile << pConfigRev->MaxAngle	<< std::endl;
			myfile << pConfigRev->MinAngle	<< std::endl;
		}
		else if(pNodes[i]->Joint.JT == S3DJT_Ball)//
		{
			pConfigBall = &pNodes[i]->Joint.ConfBall;
			myfile << "S3DJT_Ball\n";
			myfile << "Reference Points : Pos(x,y) - PRight(x,y) - PLeft(x,y) - PTop(x,y) - PBottom(x,y)\n";
			writeVector2f(myfile,pConfigBall->Pos);
			writeVector2f(myfile,pConfigBall->PRight);
			writeVector2f(myfile,pConfigBall->PLeft);
			writeVector2f(myfile,pConfigBall->PTop);
			writeVector2f(myfile,pConfigBall->PBottom);
			myfile << "Reference Quaternions : QRight(x,y,z,w) - QLeft(x,y,z,w) - QTop(x,y,z,w) - QBottom(x,y,z,w)\n";
			writeQuaternionf(myfile,pConfigBall->QRight);
			writeQuaternionf(myfile,pConfigBall->QLeft);
			writeQuaternionf(myfile,pConfigBall->QTop);
			writeQuaternionf(myfile,pConfigBall->QBottom);
			myfile << "Rotation Axis : Axis(x,y,z) - Angle - Max - Min\n";
			writeVector3f(myfile,pConfigBall->DefaultRotAxis);
			myfile << pConfigBall->Angle		<< std::endl;
			myfile << pConfigBall->MaxAngle		<< std::endl;
			myfile << pConfigBall->MinAngle		<< std::endl;
		}
		else if(pNodes[i]->Joint.JT == S3DJT_Fixed)
		{
			myfile << "S3DJT_Fixed\n";
		}
		else if(pNodes[i]->Joint.JT == S3DJT_Free)
		{
			myfile << "S3DJT_Free\n";
		}
	}
	myfile.close();
	printf("Model File(%s) saved\n",FileName);
}
//--------------------------------------------------------------------------------------------------------------
void WireModel_c::UpdateDefault()
{
	pRoot->mDefault = MatrixTranslation(0,0,0);
	//pPelvis->mDefault = MatrixTranslation(0,Leg+Calf,0);
	pPelvis->mDefault = MatrixTranslation(0,0,0);//The Root is on the Pelvis
	pLThigh->mDefault = MatrixTranslation(wWaist/2,0,0);
	pRThigh->mDefault = MatrixTranslation(-wWaist/2,0,0);
	pLKnee->mDefault = MatrixTranslation(0,-Leg,0);
	pRKnee->mDefault = MatrixTranslation(0,-Leg,0);
	pLFoot->mDefault = MatrixTranslation(0,-Calf,0);
	pRFoot->mDefault = MatrixTranslation(0,-Calf,0);
	pChest->mDefault = MatrixTranslation(0,hBody,0);
	pLShoulder->mDefault = MatrixTranslation(wShoulders/2,0,0);
	pRShoulder->mDefault = MatrixTranslation(-wShoulders/2,0,0);
	pLElbow->mDefault = MatrixTranslation(Arm,0,0);
	pRElbow->mDefault = MatrixTranslation(-Arm,0,0);
	pLHand->mDefault = MatrixTranslation(Forearm,0,0);
	pRHand->mDefault = MatrixTranslation(-Forearm,0,0);
	pHead->mDefault = MatrixTranslation(0,0,0);
	pNeck->mDefault = MatrixTranslation(0,Neck,0);
}
//--------------------------------------------------------------------------------------------------------------
void WireModel_c::ResetLocals()
{
	for(int i=0;i<NbNodes;i++)pNodes[i]->mLocal = MatrixIdentity();
}
//--------------------------------------------------------------------------------------------------------------
void WireModel_c::UpdateConfigDeprecated(std::vector<float> &Params)//Deprecated - Better Update every Nodes.Config
{
	UINT Index=0;
	for(int i=0;i<NbNodes;i++)
	{
		if (pNodes[i]->Joint.JT == S3DJT_Revolute)
		{
			Index++;
			pNodes[i]->Joint.ConfRev.Angle = Params[Index-1] 
											* (pNodes[i]->Joint.ConfRev.MaxAngle - pNodes[i]->Joint.ConfRev.MinAngle)
											+ pNodes[i]->Joint.ConfRev.MinAngle;
		}
		else if (pNodes[i]->Joint.JT == S3DJT_Ball)
		{
			Index+=3;
			
			pNodes[i]->Joint.ConfBall.Pos.x() = Params[Index-3];
			pNodes[i]->Joint.ConfBall.Pos.y() = Params[Index-2];
			pNodes[i]->Joint.ConfBall.Angle = Params[Index-1]
											* (pNodes[i]->Joint.ConfBall.MaxAngle - pNodes[i]->Joint.ConfBall.MinAngle)
											+ pNodes[i]->Joint.ConfBall.MinAngle;
		}
	}
}
//--------------------------------------------------------------------------------------------------------------
void WireModel_c::UpdateLocals(bool isNoPelvisMove)//Rotation with Joints - Other functions should support the deprecated Rot_XYZ and Rot_YawPitchRoll
{
	int StartFrom = 0;
	if(isNoPelvisMove)
		StartFrom = 2;//No PRoot, No Pelvis
	for(int i=StartFrom;i<NbNodes;i++)
	{
		pNodes[i]->UpdateLocalFromConfig();
	}
}
//--------------------------------------------------------------------------------------------------------------
void WireModel_c::UpdateCurrent()
{
	pRoot->mCurrent = pRoot->mDefault * pRoot->mLocal;
	for(int i=1;i<NbNodes;i++)pNodes[i]->mCurrent = pNodes[i]->pParent->mCurrent * pNodes[i]->mDefault * pNodes[i]->mLocal;
	for(int i=0;i<NbNodes;i++)pNodes[i]->Pos = TranslationVector(pNodes[i]->mCurrent);
}
//--------------------------------------------------------------------------------------------------------------
void WireModel_c::GetPosture(std::vector<float> &Params)//should Just return the fx,fy that are updated
{
	float tfx,tfy;
	pRShoulder->GetParams(Params[11],Params[12]);
	pRElbow->GetParams(tfx,tfy);					Params[13]=1-tfx;		Params[15] = 1-tfy;
	pLShoulder->GetParams(Params[8],Params[9]);
	pLElbow->GetParams(tfx,tfy);					Params[10]=1-tfx;	Params[14]= 1-tfy;
	pRThigh->GetParams(tfx,tfy);					Params[3]=1-tfx;	Params[4] = tfy;
	pRKnee->GetParams(tfx,tfy);						Params[5]=1-tfx;	Params[7] = 1-tfy;
	pLThigh->GetParams(tfx,tfy);					Params[0]=1-tfx;	Params[1] = tfy;
	pLKnee->GetParams(tfx,tfy);						Params[2]=1-tfx;	Params[6] = 1-tfy;
}
//--------------------------------------------------------------------------------------------------------------
void WireModel_c::UpdateConfig(std::vector<float> &Params)
{
	pRShoulder->SetParams(Params[11],Params[12]);
	pRElbow->SetParams(1-Params[13],1-Params[15]);
	pLShoulder->SetParams(Params[8],Params[9]);
	pLElbow->SetParams(1-Params[10],1-Params[14]);
	pRThigh->SetParams(1-Params[3],Params[4]);
	pRKnee->SetParams(1-Params[5],1-Params[7]);
	pLThigh->SetParams(1-Params[0],Params[1]);
	pLKnee->SetParams(1-Params[2],1-Params[6]);
}
//--------------------------------------------------------------------------------------------------------------
void WireModel_c::SetPosture(std::vector<float> &vParams)
{
	UpdateConfig(vParams);
	UpdateLocals();
	//pPelvis->mLocal = S3DMatrixTranslation(0,Leg+Calf,0);
	UpdateCurrent();
}
//--------------------------------------------------------------------------------------------------------------
void WireModel_c::SetPosture_Position(const std::vector<float> &Params3)
{
	Vector3f Vect3(Params3[0],Params3[1],Params3[2]);
	MoveTo(Vect3);
}
//--------------------------------------------------------------------------------------------
void WireModel_c::SetPosture_Orientation(const std::vector<float> &Params6)
{
	Vector3f VectY,VectZ;
	VectY.x() = Params6[0];
	VectY.y() = Params6[1];
	VectY.z() = Params6[2];
	VectZ.x() = Params6[3];
	VectZ.y() = Params6[4];
	VectZ.z() = Params6[5];
	Vector3f VectX = VectY.cross(VectZ);
	Eigen::Matrix3f MOr3 = g3d::MatrixAxis(VectX,VectY,VectZ);
	SetOrientation(MOr3);
}
//--------------------------------------------------------------------------------------------------------------
void WireModel_c::Draw(std::vector<cv::Mat> &Imgs,Soft3DCamera_c *pCams,float Light)
{
	for(int i=0;i<3;i++)
	{
		Draw(Imgs[i],pCams[i],Light);
	}
}
//--------------------------------------------------------------------------------------------------------------
void WireModel_c::Draw(Soft3DCamera_c *pCamArray,float Light)
{
	for(int i=0;i<3;i++)
	{
		Draw(pCamArray[i],Light);
	}
}
//--------------------------------------------------------------------------------------------------------------
void WireModel_c::Draw(cv::Mat &ImgDrawOn,Soft3DCamera_c &Cam,float Light)//deprecated
{
	if(Light<0)Light=0;else if(Light>1)Light=1;
//	Cam.DrawRefPoint(ImgDrawOn,Vector3f(0,0,0));
	std::vector<cv::Point> P,PP;
	for(int i=2;i<NbNodes;i++)
	{
		P.push_back(Cam.Project(pNodes[i]->Pos));
		PP.push_back(Cam.Project(pNodes[i]->pParent->Pos));
	}
	for(UINT i=0;i<P.size();i++)
	{
		cv::line(ImgDrawOn,P[i],PP[i],cv::Scalar(0,0,0),5);
		cv::line(ImgDrawOn,P[i],PP[i],cv::Scalar(255,255,255),2);
	}
	for(UINT i=0;i<P.size();i++)
	{
		cv::Scalar Color;
		Color = cv::Scalar(0,Light*255,255-Light*255);
		//cv::circle(ImgDrawOn,P[i],3,pNodes[i]->Color,3);
		cv::circle(ImgDrawOn,P[i],3,Color,3);
	}
}
//--------------------------------------------------------------------------------------------------------------
void WireModel_c::Draw(Soft3DCamera_c &Cam,float Light)
{
	if(Light<0)Light=0;else if(Light>1)Light=1;
	//Cam.DrawRefPoint(ImgDrawOn,Vector3f(0,0,0));
	cv::Scalar Color;
	Color = cv::Scalar(0,Light*255,255-Light*255);
	for(int i=2;i<NbNodes;i++)
	{
		if((pNodes[i]->pParent!=NULL)&&(i!=15))//Head Draw Bug
			Cam.DrawLine3D(pNodes[i]->Pos,pNodes[i]->pParent->Pos,Color);
	}
	
	/*for(int i=0;i<P.size();i++)
	{
		Cam.DrawPoint3D(pNodes[i]->Pos,Color);
		//pNodes[i]->Color
	}*/
}
//--------------------------------------------------------------------------------------------------------------
void WireModel_c::DrawReferences(Soft3DCamera_c &Cam)
{
	float CharSize = -1;//0.2f
	Cam.DrawReference(pRShoulder->mCurrent,10,CharSize);
	Cam.DrawReference(pLShoulder->mCurrent,10,CharSize);
	Cam.DrawReference(pRElbow->mCurrent,10,CharSize);
	Cam.DrawReference(pLElbow->mCurrent,10,CharSize);
	Cam.DrawReference(pRThigh->mCurrent,10,CharSize);
	Cam.DrawReference(pLThigh->mCurrent,10,CharSize);
	Cam.DrawReference(pRKnee->mCurrent,10,CharSize);
	Cam.DrawReference(pLKnee->mCurrent,10,CharSize);
}
//--------------------------------------------------------------------------------------------------------------
int WireModel_c::EvalFit(cv::Mat &ImgBkg,Soft3DCamera_c &Cam)
{
	int ValRes = 0;
	std::vector<cv::Point> P;
	for(int i=2;i<NbNodes;i++)
	{
		P.push_back(Cam.Project(pNodes[i]->Pos));
	}
	for(UINT i=0;i<P.size();i++)
	{
		if(PIXEL8UC1(ImgBkg,P[i].x,P[i].y))
			ValRes++;
	}
	return ValRes;
}
//--------------------------------------------------------------------------------------------------------------
void WireModel_c::MoveTo(const Vector4f &VPos)
{
#ifdef MOVE_ANDPLACE_UP
	pRoot->mLocal = S3DMatrixTranslation(VPos.x(),0,VPos.z());
	pPelvis->mLocal = S3DMatrixTranslation(0,VPos.y(),0);
#else
	pRoot->mLocal = MatrixIdentity();
	pPelvis->mLocal(0,3) = VPos.x();
	pPelvis->mLocal(1,3) = VPos.y();
	pPelvis->mLocal(2,3) = VPos.z();
#endif

	UpdateCurrent();//Yeah for acceleration purpose this function is independant, a bit tricky
}
//--------------------------------------------------------------------------------------------------------------
void WireModel_c::MoveTo(const Vector3f &Vect)
{
	Vector4f Vect4 = V3To4(Vect);
	MoveTo(Vect4);
}
//--------------------------------------------------------------------------------------------------------------
void WireModel_c::MoveTo(const Matrix4f &VMat)
{
	pRoot->mLocal = MatrixIdentity();
	pPelvis->mLocal = VMat;
	UpdateCurrent();//Yeah for acceleration purpose this function is independant, a bit tricky
}
//--------------------------------------------------------------------------------------------------------------
void WireModel_c::SetOrientation(const Eigen::Matrix3f &VMat)
{
	pRoot->mLocal = MatrixIdentity();
	pPelvis->mLocal(0,0) = VMat(0,0);	pPelvis->mLocal(0,1) = VMat(0,1);	pPelvis->mLocal(0,2) = VMat(0,2);
	pPelvis->mLocal(1,0) = VMat(1,0);	pPelvis->mLocal(1,1) = VMat(1,1);	pPelvis->mLocal(1,2) = VMat(1,2);
	pPelvis->mLocal(2,0) = VMat(2,0);	pPelvis->mLocal(2,1) = VMat(2,1);	pPelvis->mLocal(2,2) = VMat(2,2);
	UpdateCurrent();//Yeah for acceleration purpose this function is independant, a bit tricky
}
//--------------------------------------------------------------------------------------------------------------
void WireModel_c::GetPosition(Vector3f &vPos)
{
	vPos = V4To3(TranslationVector(pPelvis->mLocal));
}
//--------------------------------------------------------------------------------------------------------------
void WireModel_c::GetOrientation(Eigen::Matrix3f &MatOriantation)
{
	MatOriantation(0,0) = pPelvis->mLocal(0,0);	MatOriantation(0,1) = pPelvis->mLocal(0,1);	MatOriantation(0,2) = pPelvis->mLocal(0,2);
	MatOriantation(1,0) = pPelvis->mLocal(1,0);	MatOriantation(1,1) = pPelvis->mLocal(1,1);	MatOriantation(1,2) = pPelvis->mLocal(1,2);
	MatOriantation(2,0) = pPelvis->mLocal(2,0);	MatOriantation(2,1) = pPelvis->mLocal(2,1);	MatOriantation(2,2) = pPelvis->mLocal(2,2);
}
//--------------------------------------------------------------------------------------------------------------
float WireModel_c::GetDistance_Config(WireModel_c &WModel)
{
	
	std::vector<float> Params(NbConfigParams);
	std::vector<float> WParams(NbConfigParams);
	GetPosture(Params);
	WModel.GetPosture(WParams);

	return TabDist(Params,WParams);
}
//--------------------------------------------------------------------------------------------------------------
float WireModel_c::GetDistance_Degree(WireModel_c &WModel)
{
	return 0;
}
//--------------------------------------------------------------------------------------------------------------
float WireModel_c::GetDistance_cm(WireModel_c &WModel)
{
	double SumDist = 0;
	assert(NbNodes == WModel.NbNodes);
	for(int i=0;i<NbNodes;i++)
	{
		SumDist += V4To3(Nodes[i].Pos-WModel.Nodes[i].Pos).norm();
	}

	return (float)(SumDist/NbNodes);
}
//--------------------------------------------------------------------------------------------------------------
void WireModel_c::MoveToMat(const Matrix4f &PelvisMat,const Vector3f &LShoulder,const Vector3f &RShoulder)
{
	bool isPrinf = false;
	pRoot->mLocal = MatrixIdentity();
	Vector3f PelvisPos = V4To3(TranslationVector(PelvisMat));
	Vector3f CenterShoulders = (LShoulder + RShoulder)/2;
	Vector3f ZAxis = g3d::MatrixGetZVector(PelvisMat);
	Vector3f YAxis = CenterShoulders - PelvisPos;
	YAxis.normalize();
	Vector3f XAxis = YAxis.cross(ZAxis);
	XAxis.normalize();
	ZAxis = XAxis.cross(YAxis);
	ZAxis.normalize();
	if(isPrinf)
	{
		g3d::printfVector3f(XAxis,"XAxis");
		g3d::printfVector3f(YAxis,"YAxis");
		g3d::printfVector3f(ZAxis,"ZAxis");
	}
	pPelvis->mLocal = g3d::MatrixAxisTr(XAxis,YAxis,ZAxis,PelvisPos);
	UpdateCurrent();//Yeah for acceleration purpose this function is independant, a bit tricky
}
//--------------------------------------------------------------------------------------------------------------
void WireModel_c::MoveTo(const Matrix4f &PelvisMat,const Vector3f &LShoulder,const Vector3f &RShoulder,const Vector3f &LThigh,const Vector3f &RThigh)
{
	Vector3f PelvisPos = V4To3(TranslationVector(PelvisMat));
	MoveTo(PelvisPos,LShoulder,RShoulder,LThigh,RThigh);
}
//--------------------------------------------------------------------------------------------------------------
void WireModel_c::MoveTo(const Vector3f &PelvisPos,const Vector3f &LShoulder,const Vector3f &RShoulder,const Vector3f &LThigh,const Vector3f &RThigh)
{
	bool isPrinf = false;
	pRoot->mLocal = MatrixIdentity();
	Vector3f CenterShoulders = (LShoulder + RShoulder)/2;
	Vector3f XAxis = (LThigh - RThigh);
	XAxis.normalize();
	Vector3f YAxis = CenterShoulders - PelvisPos;
	YAxis.normalize();
	Vector3f ZAxis = XAxis.cross(YAxis);
	ZAxis.normalize();
	if(isPrinf)
	{
		g3d::printfVector3f(XAxis,"XAxis");
		g3d::printfVector3f(YAxis,"YAxis");
		g3d::printfVector3f(ZAxis,"ZAxis");
	}
	pPelvis->mLocal = g3d::MatrixAxisTr(XAxis,YAxis,ZAxis,PelvisPos);
	UpdateCurrent();//Yeah for acceleration purpose this function is independant, a bit tricky
}
//--------------------------------------------------------------------------------------------------------------
void WireModel_c::SetOrientation(const Matrix4f &PelvisMat,const Vector3f &LShoulder,const Vector3f &RShoulder)
{
	bool isPrinf = false;
	pRoot->mLocal = MatrixIdentity();
	Vector3f PelvisPos(0,0,0);//V4To3(S3DTranslationVector(PelvisMat));//---------will be Stick To Base
	Vector3f CenterShoulders = (LShoulder + RShoulder)/2;
	Vector3f ZAxis = g3d::MatrixGetZVector(PelvisMat);
	Vector3f YAxis = CenterShoulders - PelvisPos;
	YAxis.normalize();
	Vector3f XAxis = YAxis.cross(ZAxis);
	XAxis.normalize();
	ZAxis = XAxis.cross(YAxis);
	ZAxis.normalize();
	if(isPrinf)
	{
		g3d::printfVector3f(XAxis,"XAxis");
		g3d::printfVector3f(YAxis,"YAxis");
		g3d::printfVector3f(ZAxis,"ZAxis");
	}
	pPelvis->mLocal = g3d::MatrixAxisTr(XAxis,YAxis,ZAxis,PelvisPos);
	UpdateCurrent();//Yeah for acceleration purpose this function is independant, a bit tricky
}
//--------------------------------------------------------------------------------------------------------------
void WireModel_c::UpdatePos(float x, float y,float Teta)//Not good function
{
	pRoot->mLocal = MatrixTranslation(x,0,y) * MatrixRotationY(Teta);
}
//--------------------------------------------------------------------------------------------------------------
void WireModel_c::SetAnimByIndex(int AnimIndex,bool isPosition)
{
	std::vector<float> Params(Motion.NbParams);
	std::vector<float> Params6(6);
	std::vector<float> Params3(3);

	Params = Motion.Postures.at(AnimIndex);
	Params3 = Motion.Position.at(AnimIndex);
	Params6 = Motion.Orientation.at(AnimIndex);
	SetPosture(Params);
	SetPosture_Orientation(Params6);
	if(isPosition)
	{
		SetPosture_Position(Params3);
	}
}
//--------------------------------------------------------------------------------------------------------------
//						BodyMoCap_c
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
void BodyMoCap_c::load(const char*FileName)
{
	std::string line;
	std::ifstream myfile(FileName);
	if (myfile.is_open())
	{
		getline (myfile,line);//*Nb Body Parts
		int NbBodyParts	= (int)readFloatLine(myfile);
		assert(NbBodyParts<MAX_BODY_PARTS);
		BodyParts.resize(NbBodyParts);
		getline (myfile,line);//*Body Parts Names
		for(int i=0;i<NbBodyParts;i++)
		{
			getline (myfile,line);
			sprintf(BodyParts[i].Name,"%s",line.c_str());
		}
		getline (myfile,line);//*NbFrames
		int NbFrames = (int)readFloatLine(myfile);
		for(int iBdParts=0;iBdParts<NbBodyParts;iBdParts++)
		{
			BodyParts[iBdParts].PartAnim.resize(NbFrames);
		}
		for(int iFrames=0;iFrames<NbFrames;iFrames++)
		{
			getline (myfile,line);//- Frame %iFrames
			//assert(line.(last chars to int) == iFrames)
			for(int iBdParts=0;iBdParts<NbBodyParts;iBdParts++)
			{
				BodyParts[iBdParts].PartAnim[iFrames] = readVector3f(myfile);
				BodyParts[iBdParts].PartAnim[iFrames]/=10;//to go from the mm scale to cm scale
			}
		}

		printf("Body MoCap loaded from file (%s)\n",FileName);
		myfile.close();
	}
	else
	{
		printf("File (%s) not available\n",FileName);
	}
}
//--------------------------------------------------------------------------------------------------------------
float BodyMoCap_c::GetLengthBody(int iFrame)
{
	Vector3f VLArm = BodyParts[MC_LeftShoulder].PartAnim[iFrame];
	Vector3f VRArm = BodyParts[MC_RightShoulder].PartAnim[iFrame];
	Vector3f VArmsCenter = (VLArm + VRArm) / 2;
	Vector3f VPelvis = BodyParts[MC_Pelvis].PartAnim[iFrame];
	return ((VPelvis - VArmsCenter).norm());
}
//--------------------------------------------------------------------------------------------------------------
float BodyMoCap_c::GetLength(int iPart1,int iPart2,int iFrame)
{
	Vector3f VPart1 = BodyParts[iPart1].PartAnim[iFrame];
	Vector3f VPart2 = BodyParts[iPart2].PartAnim[iFrame];
	return ((VPart1 - VPart2).norm());
}
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//						SkelMotion_c
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
void SkelMotion_c::load(const char* BaseFileName)
{
	/*
	char FileName[512];sprintf(FileName,"%s.posture.db.rg",BaseFileName);
	Postures.load(FileName,false);
	char FileNamePosition[512];sprintf(FileNamePosition,"%s.position.db.rg",BaseFileName);
	Position.load(FileNamePosition,false);
	char FileNameOrientation[512];sprintf(FileNameOrientation,"%s.orientation.db.rg",BaseFileName);
	Orientation.load(FileNameOrientation,false);
	*/
	printf("couldn't load Motion File (%s) as function is deprecated\n",BaseFileName);
}
