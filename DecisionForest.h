/*
	File started on 29/03/2012 by wassfila
	Decision Forest for posture recognition
	inspired by :

	http://www.iis.ee.ic.ac.uk/~tkkim/iccv09_tutorial
	&
	http://stat-www.berkeley.edu/users/breiman/RandomForests/cc_home.htm

	The main objective of repimplementing Decision Forests from scratch
	is the performance optimisation for a single Feature per node as
	the vector is converted to an offset. So every test is just memory access 
	of Data[RefOffset + DescOffset]
	The tree structure is also kept simple, as is modification during its
	construction consist only on adding nodes, so a vector is used with
	indexed access to link nodes.
	Trying to avoid dynamic memory allocation, no new, only through vectors resize
	which allocated size might be fixed and known.

*/

#pragma once
#ifndef __MCVDECISIONFOREST__
#define __MCVDECISIONFOREST__


#include "cv.h"
#include "highgui.h"

#include "S3DGeom.h"
#include "mcvGeneral.h"

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#define MAX_CHAR 512

namespace bpr
{

//--------------------------------------------------------------------------------------------------------------
class SmartDescriptor
{
public:
	EigenBox3D Range3DSpace;//The we need a density function if we want a specific sampling
	//Range Density function
	std::vector<Vector3f> RangeVectors;//Given a density function when filling this table, result is already well sampled
	
public:
	void FillRangeVectors();
	SmartDescriptor()
	{
		FillRangeVectors();
		Seed(0);
	}
	void Seed(unsigned int seed);
	Vector3f GetFeatureVector();
};

//--------------------------------------------------------------------------------------------------------------
class Histogram_c
{
public:
	unsigned int				Sum;
	float						Entropy;
	std::vector<unsigned int>	Columns;
	std::vector<float>			ColumnsF;
public:
	Histogram_c()
	{
		Sum = 0;
		Entropy = 0;
	}
	Histogram_c(int NbClasses)
	{
		Sum = 0;
		Entropy = 0;
		Columns.resize(NbClasses);
	}
	void Evaluate();
	void clear();

};
//--------------------------------------------------------------------------------------------------------------
void SwitchVals(int &a,int &b);
//--------------------------------------------------------------------------------------------------------------
class Case_c
{
public:
	float ratio;
	cv::Point	TL;
	cv::Point	BR;
	cv::Size	GetSize(){return cv::Size(BR.x - TL.x + 1, BR.y - TL.y + 1);}
	cv::Rect	GetRect(){cv::Size s = GetSize();return cv::Rect(TL.x,TL.y,s.width,s.height);}
	float GetRatio()
	{
		cv::Size S = GetSize();
		if(S.height == 0)return 0;
		return ((float)S.width)/((float)S.height);
	}
	void Set(cv::Point &TopLeft,cv::Point &BottomRight)
	{
		if(TopLeft.x > BottomRight.x)SwitchVals(TopLeft.x,BottomRight.x);
		if(TopLeft.y > BottomRight.y)SwitchVals(TopLeft.y,BottomRight.y);
		TL = TopLeft;
		BR = BottomRight;
	}
	void Set(cv::Point &TopLeft,int width,int height)
	{
		TL = TopLeft;
		BR = cv::Point(TL.x + width - 1,TL.y + height - 1);
	}
	void Draw(cv::Mat &Img)
	{
		cv::rectangle(Img,TL,BR,mcv::clBlack);
	}
};
//--------------------------------------------------------------------------------------------------------------
class TreesDrawParams_c
{
public:
	int minDepth,maxDepth;
	int minSum,maxSum;
	float minIG,maxIG;
	ScalarC Colormin,Colormax;
public:
	ScalarC	GetIGColor(float IG)
	{
		float IGRatio = (IG - minIG) / (maxIG - minIG);
		//ScalarC RColor = Colormin + IGRatio * (Colormax - Colormin);
		ScalarC RColor = ScalarC(0,IGRatio*255,255-IGRatio*255);
		return RColor;
	}
};
//--------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------------------
class Node_c
{
public:
	enum Node_State_e{JustCreated,HasCandidates,FixedFeature,FixedState};
	std::vector<Node_c>	CandidateNodes;
public:
	SmartDescriptor*	pDesc;
	Vector3f			FeatureVector;
	int					Offset;
	Node_State_e		State;
	bool				isLeafNode;
	float				InfoGain;
	int					Depth;
	//These parameters exist only after training when the node is fixed
	size_t				LeftNodeId;
	Node_c*				pLeftNode;
	size_t				RightNodeId;
	Node_c*				pRightNode;
	size_t				HistogramId;
	bpr::Histogram_c*	pHistogram;
	//Params used on Training step
	bpr::Histogram_c	HistogramTrain;
	//Params used as Candidate only
	bpr::Histogram_c	HistogramRight;
	bpr::Histogram_c	HistogramLeft;

public:
	Node_c()
	{
		Offset = 0;//We do not expect a Null vector Descriptor so this is the uninitialized state
		LeftNodeId = (size_t)-1;
		RightNodeId = (size_t)-1;
		isLeafNode = false;
		InfoGain = 0;
		State = Node_c::JustCreated;
		Depth = 0;//Valid for Head Nodes Only
		pHistogram = NULL;
	}
	void		ComputeOffsets(int Xv,int Yv,int Zv);
	void		ReadPoint(g3d::VoxelSpace_c &VoxData,unsigned char *pData);
	int			TestFeature(g3d::VoxelSpace_c &VoxData,unsigned char *pData);
	void		ReadDataBlock(g3d::VoxelSpace_c &VoxData);
	void		FillCandidates(int NbCandidates,int NbClasses);
	void		UpdateCandidatesHist(g3d::VoxelSpace_c &Vox,unsigned char *pData);
	void		TakeBestCandidate();

	void		Draw(cv::Mat &Img,Case_c Case,TreesDrawParams_c &Params);
};




class Tree_c
{
public:
	unsigned char*				pData;//Yes The Data To learn
	std::vector<Node_c>			Nodes;
	std::vector<Histogram_c>	Histograms;
public:
	void Draw(cv::Mat &Img,Case_c &Case,TreesDrawParams_c &Params);
};

//--------------------------------------------------------------------------------------------------------------
class Forest_c
{
public:
	SmartDescriptor		Desc;
	std::vector<Tree_c> Trees;
public:
	void AppendForest(Forest_c AnotherForest);
	//--------------------------------------------------------
	void Export(const std::string &FileName);
	void save(const std::string &FileName);
	void load(const std::string &FileName);
	//--------------------------------------------------------

	cv::Mat Render(const char*WinName = NULL);

	void Grow_inMem(int Start,int Last,const char*ConfigFileName,const char* VoxelsChannel = "VoxelsParts");//repeat ReadPass() NbFeaturesToTry x Depth

	//From a "Train" Channel to a "Resp" Channel
	void Classify(int Start,int Last,const char*ConfigFileName,const char* VoxelsChannel = "Voxels");

	//Forest Growing utilities
private:
	std::vector<g3d::VoxelSpace_c> VoxDataVect;
private:
	void ClearOffsets();
	void ReadDataBlock(g3d::VoxelSpace_c &VoxData);
	void SetCandidatesForJustCreatedNodes(int NbCandidatesPerNode,int NbClasses);
	void PassMemDataOnCandidates();
	void SelectBestCandidates();
	void SetToLeafOrCreateDescendants(float infoGainThreshold,int MaxDepth,bool isKeepHistoForAllNodes,int minSamplesPerNode);
	bool AllFixed();
public:
	void Printf();
};
//------------------------------------------------------------------------------------------------------------------
}/*end of namespace mrg*/


//------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------
//						Boost Serialization
//------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------
namespace boost {
namespace serialization {
//------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------
template<class Archive>
void serialize(Archive & ar,Vector3f &Vect,const unsigned int version)
{
	ar & Vect.x();
	ar & Vect.y();
	ar & Vect.z();
}

//------------------------------------------------------------------------------------------------------------------
template<class Archive>
void serialize(Archive & ar,bpr::Histogram_c &Histogram,const unsigned int version)
{
	ar & Histogram.Sum;
	ar & Histogram.Entropy;
	ar & Histogram.Columns;
}

//------------------------------------------------------------------------------------------------------------------
template<class Archive>
void serialize(Archive & ar,bpr::Node_c &Node,const unsigned int version)
{
	ar & Node.FeatureVector;
	ar & Node.LeftNodeId;
	ar & Node.RightNodeId;
	ar & Node.isLeafNode;
	ar & Node.HistogramId;
	ar & Node.InfoGain;
}

//------------------------------------------------------------------------------------------------------------------
template<class Archive>
void serialize(Archive & ar,bpr::Tree_c &Tree,const unsigned int version)
{
	ar & Tree.Nodes;
}
//------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------
} // namespace serialization
} // namespace boost


#endif /*__MCVDECISIONFOREST__*/
