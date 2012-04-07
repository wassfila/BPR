
#include "S3DVox.h"
#include "DecisionForest.h"
#include "MultiCamStream.h"

using namespace std;
using namespace bpr;
using namespace mcv;


//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//								SmartDescriptor
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
void SmartDescriptor::Seed(unsigned int seed)
{
	srand ( seed );
}
//--------------------------------------------------------------------------------------------------------------
void SmartDescriptor::FillRangeVectors()
{
	boost::mt19937 rng; // I don't seed it on purpouse (it's not relevant)

	boost::normal_distribution<> nd(0.0, 30.0);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<>> Generator(rng, nd);

	for(int i=0;i<10000;i++)
	{
		Vector3f v;
		do
		{
			double x = Generator();
			double y = Generator();
			double z = Generator();
			v << x,y,z;
		}while(v.norm()<5);
		RangeVectors.push_back(v);
	}

}
//--------------------------------------------------------------------------------------------------------------
Vector3f SmartDescriptor::GetFeatureVector()
{
	int r = rand()%10000;
	return RangeVectors[r];
}

//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//								Histogram_c
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
void Histogram_c::clear()
{
	Sum = 0;
	Entropy = 0;
	Columns.clear();
	ColumnsF.clear();
}
//--------------------------------------------------------------------------------------------------------------
void Histogram_c::Evaluate()//Normalize, Compute Entropy
{
	if(Sum == 0)
	{
		Entropy = 0;
		return;
	}
	//Normalize the Histogram, the sum of its colums will equal to 1
	int NbColumns = (int)Columns.size();
	ColumnsF.resize(NbColumns);
	std::vector<double> ColumnsD(NbColumns);
	for(int i=0;i<NbColumns;i++)
	{
		double DVal = (double)Columns[i];//double: 1.7E +/- 308 (15 digits)
		double NormalizedDVal = DVal / Sum;
		ColumnsD[i] = NormalizedDVal;
		ColumnsF[i] = (float)NormalizedDVal;
	}
	double EntropyD = 0;
	for(int i=0;i<NbColumns;i++)
	{
		double DVal = ColumnsD[i];
		if(DVal != 0)//cause E(0) => 0
		{
			EntropyD -= DVal * log(DVal) / log(2.0);
		}
	}
	Entropy = (float)EntropyD;
}
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//								Node_c
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
void Node_c::UpdateCandidatesHist(g3d::VoxelSpace_c &Vox,unsigned char *pData)
{
	//we're on a node with Candidates
	for(int k=0;k<CandidateNodes.size();k++)
	{
		Node_c *pCNode = &CandidateNodes[k];
		//----------------------------------------------------------------------------------
		unsigned char Val = pCNode->TestFeature(Vox,pData);
		if(Val == 0)//Empty side
		{
			pCNode->HistogramLeft.Columns[(*pData)]++;
			pCNode->HistogramLeft.Sum++;
		}
		//else//could possibly learn depending on negihboring parts ??!!? if(Val == c)
		else//Full Side
		{
			pCNode->HistogramRight.Columns[(*pData)]++;
			pCNode->HistogramRight.Sum++;
		}
	}
}
//--------------------------------------------------------------------------------------------------------------
int Node_c::TestFeature(g3d::VoxelSpace_c &Vox,unsigned char *pData)
{
	unsigned char *pDataStart = &Vox.Data[0];
	unsigned char *pDataEnd = &Vox.Data[Vox.Data.size()-1]+1;
	if(Offset == 0)
	{
		Offset = Vox.VectToOffset(FeatureVector);
	}
	unsigned char *pFeatureData = pData + Offset;

	if((pFeatureData < pDataStart)||(pFeatureData>=pDataEnd))//outside data range
	{
		return 0;
	}
	else
	{
		return (*pFeatureData);
	}
}
//--------------------------------------------------------------------------------------------------------------
void Node_c::ReadPoint(g3d::VoxelSpace_c &Vox,unsigned char *pData)
{
	Node_c *pNode = this;
	//Crossing the Tree till we find The unique Node that has candidates and concerned with This Point
	while((pNode->State == Node_c::FixedState) && (!pNode->isLeafNode))
	{
		if(TestFeature(Vox,pData))
		{
			pNode = pNode->pRightNode;
		}
		else
		{
			pNode = pNode->pLeftNode;
		}
	}
	//-----------------------------------------------------------------Updating Node's Hist and Candidates Hist
	if(!pNode->isLeafNode)
	{
		assert(pNode->State == Node_c::HasCandidates);
		//Update Node's Hist
		pNode->HistogramTrain.Columns[(*pData)]++;
		pNode->HistogramTrain.Sum++;
		pNode->UpdateCandidatesHist(Vox,pData);
	}
}
//--------------------------------------------------------------------------------------------------------------
void Node_c::ReadDataBlock(g3d::VoxelSpace_c &Vox)
{
	//if(Offset == 0)Offset = Vox.VectToOffset(FeatureVector);//Not here caus the first might Has Candidates and no Feature

	unsigned char *pData = &Vox.Data[0];
	unsigned char *pDataStart = &Vox.Data[0];
	unsigned char *pDataEnd = &Vox.Data[Vox.Data.size()-1]+1;
	int DataSize = Vox.Data.size();
	int VCount = 0;
	for(int k=0;k<Vox.Zv;k++)
		for(int j=0;j<Vox.Yv;j++)
			for(int i=0;i<Vox.Xv;i++)
			{
				if(*pData)
				{
					ReadPoint(Vox,pData);//Cross The Tree till candidates and update their Hist
					VCount++;
				}
				pData++;
			}
	assert(VCount == Vox.NbFill);
}
//--------------------------------------------------------------------------------------------------------------
void Node_c::FillCandidates(int NbCandidates,int NbClasses)
{
	CandidateNodes.clear();
	CandidateNodes.resize(NbCandidates);
	Histogram_c EmptyHist(NbClasses);
	for(int j=0;j<CandidateNodes.size();j++)
	{
		Vector3f Vt = pDesc->GetFeatureVector();
		CandidateNodes[j].FeatureVector = Vt;
		CandidateNodes[j].HistogramRight = EmptyHist;
		CandidateNodes[j].HistogramLeft = EmptyHist;
	}
}
//--------------------------------------------------------------------------------------------------------------
//HasCandidates -> FixedFeatures
//Compute InfoGain for all Candidates, Copy Feature, Clear Candidates
void Node_c::TakeBestCandidate()
{
	//-----------------------------------------------------------------------------Finding Best Candidate
	int NbCandidates = CandidateNodes.size();
	std::vector<double> CInfoGains(NbCandidates);
	for(int i=0;i<NbCandidates;i++)
	{
		double wL = ((double)CandidateNodes[i].HistogramLeft.Sum) / HistogramTrain.Sum;
		double wR = ((double)CandidateNodes[i].HistogramRight.Sum) / HistogramTrain.Sum;

		CInfoGains[i] =		HistogramTrain.Entropy 
							- wL * CandidateNodes[i].HistogramLeft.Entropy
							- wR * CandidateNodes[i].HistogramRight.Entropy;
	}
	int BestCandidate = mcv::TabMax_Index(CInfoGains);
	//-----------------------------------------------------------------------------Copy Feature
	FeatureVector = CandidateNodes[BestCandidate].FeatureVector;
	InfoGain = CInfoGains[BestCandidate];//to be used to judge if InfoGain is enough to split again
	//-----------------------------------------------------------------------------Clear Candidates
	CandidateNodes.clear();
	State = Node_c::FixedFeature;
}
//--------------------------------------------------------------------------------------------------------------
void AdjustRatio(float &x)
{
	x = (0.5+(10*(x-0.5)/(1+abs(10*(x-0.5))))/2);
}
//--------------------------------------------------------------------------------------------------------------
void Node_c::Draw(cv::Mat &Img,Case_c Case,TreesDrawParams_c &Params)
{
	//printf("Depth %d\n",Depth);
	if(State == Node_c::FixedState)
	{
		cv::Scalar Color = Params.GetIGColor(InfoGain);
		cv::rectangle(Img,Case.GetRect(),Color,CV_FILLED);
		//cv::rectangle(Img,Case.GetRect(),mcv::clBlack);
		//printf("(%d,%d,%d,%d)\n",Case.TL.x,Case.TL.y,Case.BR.x,Case.BR.y);
		if(!isLeafNode)
		{
			Case_c CLeft,CRight;
			if((pLeftNode->pHistogram != NULL) && (pRightNode->pHistogram != NULL))
			{
				float rL = ((float)pLeftNode->pHistogram->Sum) / ((float)Params.maxSum);
				if(rL<0.4)rL = 0.4;
				if(rL>0.6)rL = 0.6;
				if((pLeftNode->isLeafNode) && (!pRightNode->isLeafNode))
				{
					rL = 0.1;
				}
				else if((!pLeftNode->isLeafNode) && (pRightNode->isLeafNode))
				{
					rL = 0.9;
				}
				//AdjustRatio(rL);
				cv::Size s = Case.GetSize();
				int wL = rL * s.width;
				int wR = s.width - wL;
				Case.TL.y += s.height;
				CLeft.Set(Case.TL,wL,s.height);//CLeft.Set(Case.TL,wL,Case.GetSize().height); doesn't work ???!!!
				Case.TL.x+=wL;
				CRight.Set(Case.TL,wR,s.height);
				pLeftNode->Draw(Img,CLeft,Params);
				pRightNode->Draw(Img,CRight,Params);
			}
		}
	}


}
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//								Tree_c
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
void Tree_c::Draw(cv::Mat &Img,Case_c &Case,TreesDrawParams_c &Params)
{
	//---------------------------------------------------------------------Drawing first node
	int ch = Case.GetSize().height / Params.maxDepth;
	Case_c NodeCase;
	NodeCase.Set(Case.TL,Case.GetSize().width,ch);
	Nodes[0].Draw(Img,NodeCase,Params);
}
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//								Forest_c
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
void Forest_c::Export(const std::string &FileName)
{
	std::ofstream outFile(FileName);

	boost::archive::text_oarchive oa(outFile);

	oa << Trees;
	
}
//--------------------------------------------------------------------------------------------------------------
void Forest_c::save(const std::string &FileName)
{
	std::ofstream outFile(FileName);

	boost::archive::binary_oarchive oa(outFile);

	oa << Trees;
	
}
//--------------------------------------------------------------------------------------------------------------
void Forest_c::load(const std::string &FileName)
{
	std::ifstream inFile(FileName);

	boost::archive::binary_iarchive ia(inFile);

	ia >> Trees;

}
//--------------------------------------------------------------------------------------------------------------
void Forest_c::ClearOffsets()
{
	for(int i=0;i<Trees.size();i++)
		for(int j=0;j<Trees[i].Nodes.size();j++)
		{
			Trees[i].Nodes[j].Offset = 0;//So that we process it just one time
		}
}
//--------------------------------------------------------------------------------------------------------------
void Forest_c::ReadDataBlock(g3d::VoxelSpace_c &VoxData)
{
	//New DataBlock, ClearOffsets
	ClearOffsets();
	for(int i=0;i<Trees.size();i++)
	{
		//Feed the data to the Head Nodes and they'll propagate it to the Their Descendants
		Trees[i].Nodes[0].ReadDataBlock(VoxData);
	}
}
//--------------------------------------------------------------------------------------------------------------
void Forest_c::SetCandidatesForJustCreatedNodes(int NbCandidatesPerNode,int NbClasses)
{
	for(int i=0;i<Trees.size();i++)
		for(int j=0;j<Trees[i].Nodes.size();j++)
			if(Trees[i].Nodes[j].State == Node_c::JustCreated)
			{
				Trees[i].Nodes[j].FillCandidates(NbCandidatesPerNode,NbClasses);
				Trees[i].Nodes[j].State = Node_c::HasCandidates;
				Histogram_c EmptyHist(NbClasses);
				Trees[i].Nodes[j].HistogramTrain = EmptyHist;
			}
}
//--------------------------------------------------------------------------------------------------------------
void Forest_c::SelectBestCandidates()//(HasCandidates -> FixedFeatures)
{
	for(int i=0;i<Trees.size();i++)
		for(int j=0;j<Trees[i].Nodes.size();j++)
			if(Trees[i].Nodes[j].State == Node_c::HasCandidates)
			{
				Trees[i].Nodes[j].HistogramTrain.Evaluate();//Normalize, compute Entropy
				for(int k=0;k<Trees[i].Nodes[j].CandidateNodes.size();k++)
				{
					//Normalize, Compute Entropy
					Trees[i].Nodes[j].CandidateNodes[k].HistogramRight.Evaluate();
					Trees[i].Nodes[j].CandidateNodes[k].HistogramLeft.Evaluate();
				}
				//HasCandidates -> FixedFeatures
				//Compute InfoGain for all Candidates, Copy Feature, Clear Candidates
				Trees[i].Nodes[j].TakeBestCandidate();
			}
}
//-------------------------------------------------------------------------------------------------
void Forest_c::SetToLeafOrCreateDescendants(float infoGainThreshold,int MaxDepth,bool isKeepHistoForAllNodes,int minSamplesPerNode)
{
	for(int i=0;i<Trees.size();i++)
	{
		int NbNodes = Trees[i].Nodes.size();//Might change, but the new ones are not needed in this loop
		for(int j=0;j<NbNodes;j++)
		{
			Node_c &Node = Trees[i].Nodes[j];
			if(Node.State == Node_c::FixedFeature)
			{
				if( (Node.InfoGain < infoGainThreshold) ||
					(Node.Depth >= MaxDepth) || 
					(Node.HistogramTrain.Sum < minSamplesPerNode) )//No Not Good, stop growing the Tree
				{
					Node.isLeafNode = true;
				}
				else//Nice Continue Growing
				{
					//Create New Left and Right Nodes
					Node.LeftNodeId = Trees[i].Nodes.size();				//Index
					Node.RightNodeId = Node.LeftNodeId + 1;
					Node_c NewNode;//JustCreated
					NewNode.pDesc = Node.pDesc;								//Transmit the pDesc
					Trees[i].Nodes.push_back(NewNode);						//push_back
					Trees[i].Nodes.push_back(NewNode);						
					Node.pLeftNode = &Trees[i].Nodes[Node.LeftNodeId];		//Point
					Node.pRightNode = &Trees[i].Nodes[Node.RightNodeId];
					Node.pLeftNode->Depth = Node.Depth + 1;					//Set Depth
					Node.pRightNode->Depth = Node.Depth + 1;
				}
				if(isKeepHistoForAllNodes || (!isKeepHistoForAllNodes && Node.isLeafNode))
				{
					//------------------------------------------------------------SetUp the Histogram
					Node.HistogramId = Trees[i].Histograms.size();				//Index
					Trees[i].Histograms.push_back(Node.HistogramTrain);			//push_back
					Node.pHistogram = &Trees[i].Histograms[Node.HistogramId];	//Point
					Node.HistogramTrain.clear();								//Free Mem
					Node.State = Node_c::FixedState;
				}
			}
		}
	}
}
//-------------------------------------------------------------------------------------------------Breadth first
void Forest_c::PassMemDataOnCandidates()
{
	double t;
	TStart(t);
	for(int i=0;i<VoxDataVect.size();i++)
	{
		ReadDataBlock(VoxDataVect[i]);
	}
	TStop(t,"PassMemDataOnCandidates()");
}
//--------------------------------------------------------------------------------------------------------------
bool Forest_c::AllFixed()
{
	bool AreAllFixed = true;
	for(int i=0;(i<Trees.size()) && AreAllFixed;i++)
		for(int j=0;(j<Trees[i].Nodes.size()) && AreAllFixed;j++)
			if(Trees[i].Nodes[j].State != Node_c::FixedState)
			{
				AreAllFixed = false;
			}
	return AreAllFixed;
}
//--------------------------------------------------------------------------------------------------------------
void Forest_c::Grow_inMem(int Start,int Last,const char*ConfigFileName,const char* VoxelsChannel)
{
	int NbTrees = 9;
	int NbCandidatesPerNode = 5;
	int NbClasses = 14;//Empty,Uncknown,Body,...RightWrist,Head
	float infoGainThreshold = 0.1;
	int MaxDepth = 20;
	bool isKeepHistoForAllNodes = true;
	int minSamplesPerNode = 50;

	int MaxExpectedNodesPerTrees = 1000;

	double t;
	bool isTime = true;

	//----------------------------------------------------------Load VoxelData into Mem
	//---------------------------------------------------------------------------------
	IMGFileStream VoxelFiles(ConfigFileName,"Poses",VoxelsChannel);
	if(isTime)TStart(t);
	VoxDataVect.clear();
	for(int PoseIndex=Start;PoseIndex<=Last;PoseIndex++)
	{
		g3d::VoxelSpace_c Vox(VoxelFiles.GetName(PoseIndex));
		VoxDataVect.push_back(Vox);
	}
	if(isTime)TStop(t,"Loading Voxel Data");
	//--------------------------------------------------------Set The Number of Trees in the forest
	//---------------------------------------------------------------------------------------------
	Trees.clear();
	Trees.resize(NbTrees);//Empty Trees

	//--------------------------------------------------------Create the Head Node of every Tree
	//------------------------------------------------------------------------------------------
	for(int i=0;i<Trees.size();i++)
	{
		Node_c EmptyNode;
		EmptyNode.pDesc = &Desc;
		Trees[i].Nodes.reserve(MaxExpectedNodesPerTrees);
		Trees[i].Nodes.push_back(EmptyNode);
	}

	//---------------------------------------------------------------Growing the Forest
	//---------------------------------------------------------------------------------
	do
	{
		SetCandidatesForJustCreatedNodes(NbCandidatesPerNode,NbClasses);//(JustCreated -> HasCandidates)
		PassMemDataOnCandidates();//increment Histograms of all candidates
		SelectBestCandidates();//(HasCandidates -> FixedFeatures)
		SetToLeafOrCreateDescendants(infoGainThreshold,MaxDepth,isKeepHistoForAllNodes,minSamplesPerNode);//(FixedFeatures -> Fixed) while adding new descendants nodes to the vector
		//Here we draw what we got on this Pass
		Render("Forest");
		cv::waitKey(100);

	}while(!AllFixed());//or no we have more new nodes

	cv::waitKey();

}
//--------------------------------------------------------------------------------------------------------------
void Forest_c::Printf()
{
	for(int i=0;i<Trees.size();i++)
	{
		cout << "Tree " << i << "-----------------------" << endl;
		for(int j=0;j<Trees[i].Nodes.size();j++)
		{
			cout << "N[" << j << "] ";
			cout << "IGain( " << Trees[i].Nodes[j].InfoGain << " ) ";
			cout << "Depth( " << Trees[i].Nodes[j].Depth << " ) ";
			cout << "HistId( " << Trees[i].Nodes[j].HistogramId << " ) ";
			if(Trees[i].Nodes[j].isLeafNode)
				cout << "isLeaf";
			cout << endl;
		}
		for(int j=0;j<Trees[i].Histograms.size();j++)
		{
			cout << "Hist " << j << " :";
			cout << "Sum(" << Trees[i].Histograms[j].Sum << ") ";
			cout << "Ent(" << Trees[i].Histograms[j].Entropy << ") : ";
			for(int k=0;k<Trees[i].Histograms[j].Columns.size();k++)
			{
				cout << Trees[i].Histograms[j].Columns[k] << ",";
			}
			cout << endl;
		}
	}
}
//--------------------------------------------------------------------------------------------------------------
void Forest_c::Classify(int Start,int Last,const char*ConfigFileName,const char* VoxelsChannel)
{

}
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
void SwitchVals(int &a,int &b){	int c = a;	a = b;	b = c; }

class Grid_c
{
public:
	int X,Y;
	int cw,ch;
	int Tw,Th;
	std::vector<Case_c>	Cases;
public:
	void SetGrid_Optim(int vWidth,int vHeight,int vNbCases,float caseRatio)
	{
		Tw = vWidth;
		Th = vHeight;
		float r = caseRatio;
		int Nb = vNbCases;
		std::vector<float> Surfs(Nb);
		for(int i=0;i<Nb;i++)
		{
			X = i + 1;
			Y = (int)(Nb / X);
			if(Nb > NbCases())Y++;
			ch = Th / Y;
			cw = Tw / X;
			if(cw > ch*r)
			{
				cw = ch*r;
				ch = cw/r;
			}
			if(ch > cw/r)
			{
				ch = cw/r;
				cw = ch*r;
			}
			Surfs[i] = Nb * cw * ch;
		}
		int Best_i = mcv::TabMax_Index(Surfs);
		X = Best_i + 1;
		Y = (int)(Nb / X);
		if(Nb > NbCases())Y++;
		ch = Th / Y;
		cw = Tw / X;
		if(cw > ch*r)
		{
			cw = ch*r;
			ch = cw/r;
		}
		if(ch > cw/r)
		{
			ch = cw/r;
			cw = ch*r;
		}
		Cases.resize(Nb);
		int iY=0;
		int iX=0;
		for(int i=0;i<Nb;i++)
		{
			Cases[i].Set(cv::Point(iX*cw,iY*ch),cw,ch);
			iX++;
			if(iX==X)
			{
				iX=0;
				iY++;
			}
		}
	}
	void SetGrid_SQR(int vWidth,int vHeight,int vNbCases,float caseRatio)
	{
		Tw = vWidth;
		Th = vHeight;
		float R = ((float)vWidth)/((float)vHeight);
		float r = caseRatio;
		int Nb = vNbCases;
		float RXY = R / r;
		X = (int)(sqrt((double)(Nb))*RXY );
		Y = (int)(sqrt((double)(Nb))/RXY );
		if(Nb > NbCases())Y++;
		ch = Th / Y;
		cw = Tw / X;
		if(cw > ch*r)
		{
			cw = ch*r;
			ch = cw/r;
		}
		if(ch > cw/r)
		{
			ch = cw/r;
			cw = ch*r;
		}
		Cases.resize(Nb);
		int iY=0;
		int iX=0;
		for(int i=0;i<Nb;i++)
		{
			Cases[i].Set(cv::Point(iX*cw,iY*ch),cw,ch);
			iX++;
			if(iX==X)
			{
				iX=0;
				iY++;
			}
		}
	}
	Grid_c(int vWidth,int vHeight,int vNbCases,float caseRatio)
	{
		SetGrid_Optim(vWidth,vHeight,vNbCases,caseRatio);
	}
	int NbCases()
	{
		return X * Y;
	}
	int GetCaseId(int vx,int vy)
	{
		int XIndex = vx % cw;
		int YIndex = vy % ch;
		return (XIndex + YIndex * X);
	}
	void Draw(cv::Mat &Img)
	{
		for(int i=0;i<Cases.size();i++)
		{
			Cases[i].Draw(Img);
		}
	}
};
//--------------------------------------------------------------------------------------------------------------
cv::Mat Forest_c::Render(const char*WinName)
{
#ifdef WIN32
	int x_res = GetSystemMetrics(SM_CXSCREEN)-100;
	int y_res = GetSystemMetrics(SM_CYSCREEN)-200;
#else
	int x_res = 1280-10;
	int y_res = 1024-40;
#endif
	cv::Mat Img(y_res,x_res,CV_8UC3);
	Img.setTo(mcv::clWhite);
	cv::imshow(WinName,Img);
	//cvMoveWindow(WinName, 0, 0);
	//--------------------------------------------------------------------Define The Grid number
	int NbTrees = Trees.size();
	Grid_c Grid(Img.cols,Img.rows,NbTrees,1);
	Grid.Draw(Img);
	//--------------------------------------------------------------------Process the drawing Params
	TreesDrawParams_c P;
	P.minSum	= Trees[0].Histograms[0].Sum;
	P.maxSum	= Trees[0].Histograms[0].Sum;
	P.minDepth	= Trees[0].Nodes[0].Depth;
	P.maxDepth	= Trees[0].Nodes[0].Depth;
	P.minIG		= Trees[0].Nodes[0].InfoGain;
	P.maxIG		= Trees[0].Nodes[0].InfoGain;
	for(int i=0;i<NbTrees;i++)
	{
		for(int j=0;j<Trees[i].Histograms.size();j++)
		{
			if(P.minSum > Trees[i].Histograms[j].Sum)
				P.minSum = Trees[i].Histograms[j].Sum;
			if(P.maxSum < Trees[i].Histograms[j].Sum)
				P.maxSum = Trees[i].Histograms[j].Sum;
		}
		for(int j=0;j<Trees[i].Nodes.size();j++)
		{
			if(P.minDepth > Trees[i].Nodes[j].Depth)
				P.minDepth = Trees[i].Nodes[j].Depth;
			if(P.maxDepth < Trees[i].Nodes[j].Depth)
				P.maxDepth = Trees[i].Nodes[j].Depth;
			if(P.maxIG < Trees[i].Nodes[j].InfoGain)
				P.maxIG = Trees[i].Nodes[j].InfoGain;
			if(P.minIG > Trees[i].Nodes[j].InfoGain)
				P.minIG = Trees[i].Nodes[j].InfoGain;
		}
	}
	P.Colormin = mcv::ClRed;
	P.Colormax = mcv::ClGreen;
	printf("Max Tree Depth(%d)  MaxSum(%d) InfoGain(min:%1.2f max:%1.2f)\n",P.maxDepth,P.maxSum,P.minIG,P.maxIG);
	//--------------------------------------------------------------------Daw The Trees
	for(int i=0;i<NbTrees;i++)
	{
		Trees[i].Draw(Img,Grid.Cases[i],P);
	}
	//--------------------------------------------------------------------Display
	if(WinName)
	{
		cv::imshow(WinName,Img);
	}
	return Img;
}
//--------------------------------------------------------------------------------------------------------------
