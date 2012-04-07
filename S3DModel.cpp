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
