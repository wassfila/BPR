/*
	File started on 14/02/2011 by wassfila
	Regression for posture recognition
	can be used with "mcvShapeContext.h"

	totally changed on January 2012 by wassfila
	 - Regression_c class deprecated
	 simplified into a table : VectorsTable_c
	 MLData_c joins the Train and Resp tables
	 MLModel_c is the generic Machine Learning interface
	 ANN_Model_c is an Approximate Nearest Neighbor implementation out of cvflann

*/


#ifndef __MCVREGRESSION__
#define __MCVREGRESSION__


//#include <flann/flann.h>


#include "cv.h"
#include "highgui.h"
#include "opencv2/ml/ml.hpp"

#include "mcvGeneral.h"

//#include <unordered_map>

//#include <boost/dynamic_bitset.hpp>



#define MAX_CHAR 512

namespace mrg
{


	typedef std::map<std::vector<float>,int> MLMap_t;
	typedef std::vector<std::vector<int>> MLCMat_t;


	typedef enum { Model_Not_Specified = 0, Model_ANN = 1, Model_SVM = 2, Model_RF = 3, Model_Adaboost = 4,
					Model_NeuralNet = 5 } MLModel_t;

	void PrintSVMParams(cv::SVMParams Pr);
	bool train(cv::SVM &Learn,cv::Mat Mtrain,cv::Mat MResp,bool isTrainAuto=true,bool verbose = false);

	void ParseFloatLine(std::string &line,std::vector<float> &Data);

	float CompareClasses_Printf(		MLMap_t &RefClassMap,
										MLMap_t &ResClassMap,
										MLCMat_t &ConfMat,
										bool isPrintfTable = true,
										bool isPrintfAverageResult = true);

#define REG_DATAOFFSET 40

struct HeaderInfo_t
{
	int HeaderOffset;
	int DataOffset;
	int VectSize;
	int NbVects;
	int CheckSum;
};
struct FileInfo_t
{
	HeaderInfo_t	HeaderInfo;
	cv::FileStorage Fs;
};
//------------------------------------------------------------------------------------------------------------------
class VectorsTable_c
{
public:
	std::vector< float >	Data;//Array;
public:
	int VectSize;
	size_t NbVects;
	//int NbVectsInFile;
	std::string TypeName;
	//std::string UserHeader;
	std::vector<std::string>	ClassesNames;//Optionnal Use

	//FileInfo_t Info;

	cv::Mat GetMat();
private:
	void Init();
	std::vector<float> RefVector;
public:
	VectorsTable_c();
	VectorsTable_c(int vVectSize);
	VectorsTable_c(std::string FileName,int verb = 0);
	void Reset(int vVectSize);//nasty should be cleaned with multiple constructors

	//--------------------------------------------------------------------------------------Container utilities
	std::vector<float>& VectorsTable_c::at(size_t i);
	void copyto(size_t i,std::vector<float> &Vect);
	void push(float *pVal);
	void push(std::vector<float>&vVect);
	size_t size();
	void resize(int vVectSize,int vNbVects);
	void clear();

	//----------------------------------------------------------------------------------------
	void save(const std::string &FileName);
	void save(const std::string &FileName,const std::string &UserHeader,bool verbose);
	bool load(const std::string &FileName,bool isAppend = false	,int verb=1);
	bool load(const std::string &FileName,bool isAppend			,int verb	,YAML::Node &user_doc);//so that we're shure append is well treated

	void CopyAppend(const VectorsTable_c &Table);

	void Export(std::string TextFileName,const std::string &UserHeader = "");
	void Import(std::string TextFileName,bool isAppend = true);

	//---------------------------------------------------------------------------------------filtering
	void FilterRemoveDuplicates();
	//---------------------------------------------------------------------------------------Testing
	//big issue, the classes are issued at random of their coming in the map construction
	float CompareClasses(VectorsTable_c &RefTable,int MaxClasses = 20,bool isPrintTab = false,bool isPrintAvg = true);
	//simple and Fine
	//std::vector<float> VectorsTable_c::CompareClass9(const VectorsTable_c &Table);
	
	void CompareClasses_ComputeMap(		VectorsTable_c &RefTable,int MaxClasses,
										MLMap_t &RefClassMap,
										MLMap_t &ResClassMap);
	void CompareClasses_ComputeConf(	VectorsTable_c &RefTable,
										MLMap_t &RefClassMap,
										MLMap_t &ResClassMap,
										MLCMat_t &ConfMat			);
};

/*
//------------------------------------------------------------------------------------------------------------------
class BitVectorsTable_c
{
public:
	std::vector<unsigned long>	Blocks;//Data; Array;

	int VectSizeBits;
	int VectSizeBlocks;
	int NbVects;

public:
	//int NbVectsInFile;
	std::string TypeName;
	std::string UserHeader;

	//FileInfo_t Info;
public:

	cv::Mat GetMat();
private:
	void Init();
	std::vector<float> RefVector;
public:
	BitVectorsTable_c();
	BitVectorsTable_c(int vVectSizeBits);
	BitVectorsTable_c(std::string FileName,int verb = 0);
	void Reset(int vVectSizeBits);//nasty should be cleaned with multiple constructors

	//--------------------------------------------------------------------------------------Container utilities
	void resize(int vVectSize,int vNbVects);
	void clear();
	//--------------------------------------------------------------------------------------Access Members
	boost::dynamic_bitset<unsigned long> BitVectorsTable_c::GetBitSet(int i);
	unsigned long& GetBitBlocksAt(int i);//at(j*stride + i);//hold it with (*pData)
	//--------------------------------------------------------------------------------------Serial Processing
	void SetWCounter(int i);//at the beginning of line i
	void SetRCounter(int i);//at the beginning of line i
	void BitSet(bool BitVal = true);
	void BitClear();
	bool BitGet();
	//----------------------------------------------------------------------------------------
	void save(const std::string &FileName,const std::string &UserHeader);
	bool load(const std::string &FileName,bool isAppend,YAML::Node &user_doc);
	void CopyAppend(const VectorsTable_c &Table);
	void Export(std::string TextFileName,const std::string &UserHeader = "");
	void Import(std::string TextFileName,bool isAppend = false);

};
//------------------------------------------------------------------------------------------------------------------
*/

class MLData_c
{
private:
	std::vector<std::string> TypeNames;
public:
	VectorsTable_c Train;
	VectorsTable_c Resp;
	//Optionnal Ids for channel loading
	cv::FileStorage Fs;
	std::string		ConfigFileName;
	std::string		MainPath;//Taken from ConfigFileName
	std::string		NodeName;
public:
	MLData_c();
	MLData_c(std::string vCfgFileName,std::string MLDataNodeName);//Fs for MultiStreams, and Node for monoStream
	void Init(std::string vCfgFileName,std::string MLDataNodeName);

	void save(const char*TrainTag = "Train", const char*RespTag = "Resp");//else use Train.save and Resp.save
	void load(const char*TrainTag = "Train", const char*RespTag = "Resp");

	void Import(const char*TrainTag = "Train", const char*RespTag = "Resp");//else use Train.save and Resp.save
	void Export(const char*TrainTag = "Train", const char*RespTag = "Resp");

	void CopyAppend(MLData_c &Data);

	//-------------------------------------------------------------------------Filtering
	void FilterData();// Remove duplicates, cluster....
	void Filter_RemoveDuplicates();
	void Filter_RemoveSimilar(float eps);
	void Filter_MergeTovect(VectorsTable_c &AllVect);
	void Filter_SplitFromVect(VectorsTable_c &AllVect);

	//---------------------------------------------------------------------------------------overloads
	void loadTrainChannel(int Start,int Last,const char* NodeTag,const char*TrainTag);
	void loadRespChannel(int Start,int Last,const char* NodeTag,const char*RespTag);

};
//------------------------------------------------------------------------------------------------------------------
class MLSmartData_c:public MLData_c
{
	//void SmartFunction();//should not be here to benefit of the heritage
	void GetFeatureValue(int ValId, float Val);
};
//------------------------------------------------------------------------------------------------------------------
class ANN_Model_c
{
private:
	cv::flann::Index *pFlannIndex;
	cv::flann::Index FlannIndex;
	bool	ANNisReady;
	bool isLearnedDataTobeFreed;
	//std::vector<float> ANNSearchK(int K, float *pDesc, std::vector<int> &Indices,bool ComputeDist=false);
public:
	MLData_c	*pLearnedData;
	ANN_Model_c();
	~ANN_Model_c();
	void Train(MLData_c	&Data,float Param = 0.0f);//Keep a pointer on pLearnedData and BuildANN
	void Predict(MLData_c	&Data,int K=1);//From the Data.Train to the Data.Resp using the LearnedData

	void saveModel(cv::FileNode &ANNModelNode,std::string MainPath);
	void loadModel(cv::FileNode &ANNModelNode,std::string MainPath);
	void PrintModel(cv::flann::IndexParams &ANNIndexParams);

};
//------------------------------------------------------------------------------------------------------------------
class BinFLANN_Model_c
{
protected:
	//typedef flann::Hamming<unsigned char> Distance;
private:
	//flann::LshIndex<Distance> *pIndex;
	bool	isReady;
	MLData_c	*pLearnedData;
	bool isLearnedDataTobeFreed;

public:
	BinFLANN_Model_c();
	~BinFLANN_Model_c();
	void Train(MLData_c	&Data,float Param = 0.0f);//Keep a pointer on pLearnedData and BuildANN
	void Predict(MLData_c	&Data,int K=1);//From the Data.Train to the Data.Resp using the LearnedData

	void saveModel(std::string CfgFile,std::string Tag);
	void loadModel(cv::FileNode &ANNModelNode,std::string MainPath);
	void PrintModel(cv::flann::IndexParams &ANNIndexParams);

};
//------------------------------------------------------------------------------------------------------------------
class ANN_SmartModel_c:public ANN_Model_c
{

	template <class MLData_t> void Predict(MLData_t	&Data,int K=1);//From the Data.Train to the Data.Resp using the LearnedData

};
//------------------------------------------------------------------------------------------------------------------
class MLModel_c//KNN only for now
{
public:
	MLModel_t	ModelType;

	//these are just for channels read write but not for load save
	cv::FileStorage Fs;
	std::string		ConfigFileName;
	std::string		MainPath;//Taken from ConfigFileName
	std::string		NodeName;

	float ROC_Param;

	//------------------------Different Machine Learning Models
	ANN_Model_c ANN;
private:
	//------------------------Common data for all models
	MLData_c	*pLearnedData;
	bool isLearnedDataTobeFreed;

public:
	void Learn(MLData_c	&LearnData);// Learn = Load(Train,Resp) + Train(KNN => BuildANN)
	void LearnCopy(MLData_c	&LearnData);//Learn or copy into ANNData - can we keep a pointer, yes but need a pointer
	//void Learn(const char* TrainChan_In, const char* RespChan_In);
	void Train(MLModel_t mdl_type = mrg::Model_ANN,float Param = 0.0f);//if ANN then Train ANN with LearnedData

	void saveModel();//The model Files as defined in cfg, not the model node in cfg file
	void loadModel();//The model Files as defined in cfg, not the model node in cfg file

	void Predict(MLData_c	&Data);//From the Data.Train to the Data.Resp
	void Predict(int Start,int Last,const char* NodeTag,const char* TrainChan_In, const char *PredictRespChan_Out);//From a "Train" Channel to a "Resp" Channel

	float CompareClass(int Start,int Last,const char* NodeTag,const char* RespChanRef_In, const char *RespChanRes_In,bool ShowEveryConfTable = false);
	//------------utilities
	void PrintModel();
public:
	MLModel_c();
	MLModel_c(std::string vCfgFName,std::string MLDataNodeName);
	~MLModel_c();
	void Init(std::string vCfgFName,std::string MLDataNodeName);


public:

};




//------------------------------------------------------------------------------------------------------------------
}/*end of namespace mrg*/

#endif /*__MCVREGRESSION__*/


