

#include "mrgRegression.h"
#include "MultiCamStream.h"

#include <time.h>
#include <iostream>
#include <fstream>

#include <ppl.h>

using namespace std;
using namespace mrg;
using namespace mcv;

//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------General Utilities Functions---------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
void mrg::PrintSVMParams(cv::SVMParams Pr)
{
	switch(Pr.svm_type)
	{
	case CvSVM::C_SVC:printf("C_SVC");break;
	case CvSVM::NU_SVC:printf("NU_SVC");break;
	case CvSVM::ONE_CLASS:printf("ONE_CLASS");break;
	case CvSVM::EPS_SVR:printf("EPS_SVR");break;
	case CvSVM::NU_SVR:printf("NU_SVR");break;
	}
	printf(" with ");
	switch(Pr.kernel_type)
	{
	case CvSVM::LINEAR:printf("LINEAR");break;
	case CvSVM::POLY:printf("POLY");break;
	case CvSVM::RBF:printf("RBF");break;
	case CvSVM::SIGMOID:printf("SIGMOID");break;
	}
	printf(" kernel\n");
	if(Pr.kernel_type == CvSVM::POLY)
		printf("Degree %f : ",Pr.degree);
	if((Pr.kernel_type == CvSVM::POLY)||(Pr.kernel_type == CvSVM::RBF)||(Pr.kernel_type == CvSVM::SIGMOID))
		printf("Gamma %g : ",Pr.gamma);
	if((Pr.kernel_type == CvSVM::POLY)||(Pr.kernel_type == CvSVM::SIGMOID))								
		printf("Coeff %f : ",Pr.coef0);
	if((Pr.svm_type == CvSVM::C_SVC)||(Pr.svm_type == CvSVM::EPS_SVR)||(Pr.svm_type == CvSVM::NU_SVR))
		printf("C %g : ",Pr.C);
	if((Pr.svm_type == CvSVM::NU_SVC)||(Pr.svm_type == CvSVM::ONE_CLASS)||(Pr.svm_type == CvSVM::NU_SVR))
		printf("Nu %f : ",Pr.nu);
	if(Pr.svm_type == CvSVM::EPS_SVR)
		printf("p %f : ",Pr.p);
	printf("\n");
}
//-------------------------------------------------------------------------------------------------------------------
bool mrg::train(cv::SVM &Learn,cv::Mat Mtrain,cv::Mat MResp, bool isTrainAuto,bool verbose)
{


	bool res;
	assert(Mtrain.rows == MResp.rows);
	if(verbose)printf("Start Training auto:%d of %d samples of %d dims\n",isTrainAuto,Mtrain.rows,Mtrain.cols);
	double t;TStart(t);

	cv::SVMParams Params;

	int KernelType = CvSVM::POLY;//RBF,SIGMOID,LINEAR,POLY
	int SVMType = CvSVM::C_SVC;
	double	C,p,Gamma,Nu,Coeff,Degree;
	switch(KernelType)
	{
	case CvSVM::LINEAR:
			C = 6e-4;//grid_C.min_val = 1e-6;		grid_C.max_val = 1e-4;		grid_C.step = 30; gets only 30% in stand of 40% !!!
		break;
	case CvSVM::POLY:
			C = 8e-12;
			Gamma = 1e-6;
			Coeff = 1e-3;
			Degree = 1e-5;
		break;
	case CvSVM::RBF:
		// /70
			//C = 1e-11;Gamma = 0.0001;
		// ==
			C = 1e-15;Gamma = 0.0005;
		break;
	}
	p = 0.01;
	Nu = 0.1;
	if(isTrainAuto)	
	{
		cv::ParamGrid grid_C,grid_Gamma,grid_P,grid_NU,grid_COEFF,grid_DEGREE;
		
		grid_C = cv::SVM::get_default_grid(cv::SVM::C);
		grid_Gamma = cv::SVM::get_default_grid(cv::SVM::GAMMA);
		grid_P = cv::SVM::get_default_grid(cv::SVM::P);
		grid_NU = cv::SVM::get_default_grid(cv::SVM::NU);
		grid_COEFF = cv::SVM::get_default_grid(cv::SVM::COEF);
		grid_DEGREE = cv::SVM::get_default_grid(cv::SVM::DEGREE);

		if(KernelType == CvSVM::POLY)
		{//Degree
			grid_DEGREE.min_val = 1e-7;		grid_DEGREE.max_val = 1e4;	grid_DEGREE.step = 5;
		}
		if((KernelType == CvSVM::POLY)||(KernelType == CvSVM::RBF)||(KernelType == CvSVM::SIGMOID))
		{//Gamma
			grid_Gamma.min_val = 1e-8;		grid_Gamma.max_val = 1e5;	grid_Gamma.step = 5;
		}
		if((KernelType == CvSVM::POLY)||(KernelType == CvSVM::SIGMOID))								
		{//Coeff
			grid_COEFF.min_val = 1e-4;		grid_COEFF.max_val = 1e4;	grid_COEFF.step = 5;
		}
		if((SVMType == CvSVM::C_SVC)||(SVMType == CvSVM::EPS_SVR)||(SVMType == CvSVM::NU_SVR))
		{//C
			grid_C.min_val = 1e-12;		grid_C.max_val = 10e4;		grid_C.step = 5;
		}
		if((SVMType == CvSVM::NU_SVC)||(SVMType == CvSVM::ONE_CLASS)||(SVMType == CvSVM::NU_SVR))
		{//NU
			grid_NU.min_val = 0.01;			grid_NU.max_val = 2000;		grid_NU.step = 5;
		}
		if(SVMType == CvSVM::EPS_SVR)
		{//P
			grid_P.min_val = 0.00001;		grid_P.max_val = 0.1;		grid_P.step = 5;
		}
		

		//grid_COEFF.min_val = 0.1;		grid_COEFF.max_val = 300;	grid_COEFF.step = 14;
		res = Learn.train_auto(	Mtrain,MResp,cv::Mat(),cv::Mat(),
							CvSVMParams(SVMType,
							KernelType,
							Degree,Gamma,Coeff,C,Nu,p,//Gamma = 1
							NULL,cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER,10,0.01)),
							10,//10
							grid_C,
							grid_Gamma,
							grid_P,
							grid_NU,
							grid_COEFF,
							grid_DEGREE
							);
		if(verbose)TStop(t,"SVM Auto Training");
	}
	else
	{
		res = Learn.train(	Mtrain,MResp,cv::Mat(),cv::Mat(),
							CvSVMParams(SVMType,
							KernelType,
							Degree,Gamma,Coeff,C,Nu,p,//Gamma = 1
							NULL,cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER,10,0.01))
							);
		if(verbose)TStop(t,"SVM Training");
	}

	if(verbose)PrintSVMParams(Learn.get_params());
	if(verbose)printf("%d support vectors\n",Learn.get_support_vector_count());

	return res;
}
//----------------------------------------------------------------------------------
void mrg::ParseFloatLine(std::string &line,std::vector<float> &Data)
{
	Data.clear();
	//stringstream ss (stringstream::in | stringstream::out);
	//ss << line;
	while(!line.empty())
	{
		float Val;
		std::string Vs = line.substr( 0, line.find_first_of(',' ) );
		if(line.find_first_of(',' ) != line.npos)
		{
			line = line.substr( line.find_first_of(',' )+1,line.length() );
		}
		else
		{
			line = "";
		}
		Val = (float)atof(Vs.c_str());
		Data.push_back(Val);
	}
}
//----------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------
float mrg::CompareClasses_Printf(	MLMap_t &RefClassMap,
											MLMap_t &ResClassMap,
											MLCMat_t &ConfMat,
											bool isPrintfTable,
											bool isPrintfAverageResult)
{
	size_t NbRefClasses = RefClassMap.size();
	size_t NbResClasses = ResClassMap.size();
	int idx;
	std::vector<std::vector<float>> RefD(NbRefClasses);
	std::vector<std::vector<float>> ResD(NbResClasses);

	bool isT = isPrintfTable;
	if(isT)printf("Confusion Matrix:\n");
	if(isT)printf("Res Classes horz:\n");
	MLMap_t::iterator Rit;
	idx = 0;
	for(Rit = ResClassMap.begin();Rit!=ResClassMap.end();Rit++)
	{
		ResD[idx] = (*Rit).first;
		//mcv::TabPrintf(ResD[idx],NULL,false,' ',0);printf(",");
		if(isT)
		{
			for(int i=0;i<ResD[idx].size();i++)printf("% 6.0f",ResD[idx][i]);printf(",");
		}
		//for(int i=0;i<Vect.size();i++)printf("%1.2f ",ResD[idx][i]);printf(",");
		idx++;
	}
	if(isT)printf("<-- classified as\n");
	//printf("Ref Classes vert: ");
	idx = 0;
	for(Rit = RefClassMap.begin();Rit!=RefClassMap.end();Rit++)
	{
		//std::vector<float> Vect = (*Rit).first;
		RefD[idx] = (*Rit).first;
		//mcv::TabPrintf(RefD[idx],NULL,false,' ',0);
		//for(int i=0;i<Vect.size();i++)printf("%1.2f ",Vect[i]);printf(",");
		idx++;
	}
	//printf("\n");
		
	for(size_t ref=0;ref<NbRefClasses;ref++)
	{
		for(size_t res=0;res<NbResClasses;res++)
		{
			if(isT)printf("%6d,",ConfMat[ref][res]);
		}
		if(isT)printf("| ");
		//for(int i=0;i<RefD.size();i++)printf("% 6.0f",RefD[ref][i]);printf(",");
		if(isT)mcv::TabPrintf(RefD[ref],NULL,false,' ',0);
		if(isT)printf("\n");
	}
	//Yeah pretty complex : knowning that mapped value is unique, we need reverse mapping of the Ref Res Maps
	std::map<int,std::vector<float>> revRefClassMap;
	std::map<int,std::vector<float>> revResClassMap;
	for(Rit = RefClassMap.begin();Rit!=RefClassMap.end();Rit++)
	{
		revRefClassMap[(*Rit).second] = (*Rit).first;
	}
	for(Rit = ResClassMap.begin();Rit!=ResClassMap.end();Rit++)
	{
		revResClassMap[(*Rit).second] = (*Rit).first;
	}


	std::vector<int> ClassTotal(NbRefClasses);
	std::vector<int> ClassNbGood(NbRefClasses);
	std::vector<float> ClassRatio(NbRefClasses);

	for(int ref=0;ref<NbRefClasses;ref++)
	{
		for(int res=0;res<NbResClasses;res++)
		{
			ClassTotal[ref]+=ConfMat[ref][res];
			if(revRefClassMap[ref] == revResClassMap[res])//used reverse mapping to check equality
			{
				ClassNbGood[ref]+=ConfMat[ref][res];
			}
		}
	}

	for(int i=0;i<NbRefClasses;i++)
	{
		ClassRatio[i] = (float)ClassNbGood[i];
		if(ClassTotal[i] == 0)
		{
			ClassRatio[i] = 0;
		}
		else
		{
			ClassRatio[i] /= ClassTotal[i];
		}
	}


	bool isA = isPrintfAverageResult;
	if(isA)
	{
		mcv::TabPrintf(ClassRatio,"GoodClassesRatio",false);
		//printf("Good Classes Ratios");
		//for(int j=0;j<ClassRatio.size();j++)printf(",%1.3f",ClassRatio[j]);
	}

	float TAvg = mcv::TabAverage(ClassRatio);

	if(isA)printf("Parts Avg,%1.1f%%\n",TAvg*100);
	

	return TAvg;
}
//----------------------------------------------------------------------------------
//								VectorsTable_c
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
void VectorsTable_c::Init()
{
	VectSize = 0;//Not 0 so that size() can be returned without additionnal overhead
	NbVects = 0;
	//NbVectsInFile = 0;
	TypeName = "NotInitialized";
}
//----------------------------------------------------------------------------------
VectorsTable_c::VectorsTable_c()
{
	Init();
}
//----------------------------------------------------------------------------------
VectorsTable_c::VectorsTable_c(int vVectSize)
{
	Init();
	VectSize = vVectSize;
}
//----------------------------------------------------------------------------------
VectorsTable_c::VectorsTable_c(std::string FileName,int verb)
{
	Init();
	load(FileName,false,verb);
}
//----------------------------------------------------------------------------------
void VectorsTable_c::Reset(int vVectSize)
{
	VectSize = vVectSize;
	clear();
}
//----------------------------------------------------------------------------------
size_t VectorsTable_c::size()
{
	if(VectSize == 0)
		return Data.size();
	else
		return Data.size() / VectSize;
}
//----------------------------------------------------------------------------------
void VectorsTable_c::resize(int vVectSize,int vNbVects)
{
	VectSize = vVectSize;
	Data.resize(vNbVects * VectSize);
}
//----------------------------------------------------------------------------------
std::vector<float>& VectorsTable_c::at(size_t i)
{
	if(RefVector.size()!=VectSize)
	{
		RefVector.resize(VectSize);
	}
	memcpy(&RefVector[0],&Data[i * VectSize],VectSize * sizeof(float));
	return RefVector;
}
//----------------------------------------------------------------------------------
void VectorsTable_c::push(std::vector<float>&vVect)
{
	//for(int i=0;i<VectSize;i++)Data.push_back(vVect[i]);
	assert(!vVect.empty());
	float *pData = &vVect[0];
	float *pDataAfterLast = pData + VectSize;
	while(pData<pDataAfterLast)
	{
		Data.push_back(*pData++);
	}
}
//----------------------------------------------------------------------------------
//This function still copies the data, for faster access, resize Data[] then take a pointer
void VectorsTable_c::push(float *pVal)
{
	for(int i=0;i<VectSize;i++)Data.push_back(*pVal++);
}
//----------------------------------------------------------------------------------
void VectorsTable_c::copyto(size_t i,std::vector<float> &Vect)
{
	memcpy(&Data[i * VectSize],&Vect[0],VectSize * sizeof(float));
}
//----------------------------------------------------------------------------------
void VectorsTable_c::save(const std::string &FileName,const std::string &UserHeader,bool verbose)// Vect Size and Name Should be initialized
{
	
	assert(!Data.empty());//Yeah, no empty file, by design

	YAML::Emitter MEmitter;
	if(UserHeader.empty())
	{
		MEmitter << YAML::BeginDoc;
		MEmitter << YAML::EndDoc;
	}
	MEmitter << YAML::BeginDoc;
	MEmitter << YAML::BeginMap;
	MEmitter << YAML::Key << "TypeName";
	MEmitter << YAML::Value << TypeName;//user TypeName is obligatory while additional Header is optional
	if(!ClassesNames.empty())
	{
		MEmitter << YAML::Key << "ClassesNames";
		MEmitter << YAML::Value << ClassesNames;
	}
	MEmitter << YAML::Key << "NbFloats";
	MEmitter << YAML::Value << Data.size();
	MEmitter << YAML::Key << "VectSize";
	MEmitter << YAML::Value << VectSize;
	MEmitter << YAML::Key << "NbVects";
	MEmitter << YAML::Value << size();
	MEmitter << YAML::EndMap;
	MEmitter << YAML::EndDoc;
	
	mcv::MixedFileManager_c FManager;
	FManager.Save(FileName,(char*)&Data[0],Data.size()*4,  UserHeader + MEmitter.c_str()  );
	//FManager.Save(FileName,(char*)&Data[0],Data.size()*4,  MEmitter.c_str()  );
	

}
//----------------------------------------------------------------------------------
void VectorsTable_c::save(const std::string &FileName)
{
	save(FileName,"",true);
}
//----------------------------------------------------------------------------------
void VectorsTable_c::Export(std::string TextFileName,const std::string &UserHeader)
{
	YAML::Emitter MEmitter;
	if(UserHeader.empty())
	{
		MEmitter << YAML::BeginDoc;
		MEmitter << YAML::EndDoc;
	}
	MEmitter << YAML::BeginDoc;
	MEmitter << YAML::BeginMap;
	MEmitter << YAML::Key << "TypeName";
	MEmitter << YAML::Value << TypeName;//user TypeName is obligatory while additional Header is optional
	MEmitter << YAML::Key << "NbFloats";
	MEmitter << YAML::Value << Data.size();
	MEmitter << YAML::Key << "VectSize";
	MEmitter << YAML::Value << VectSize;
	MEmitter << YAML::Key << "NbVects";
	MEmitter << YAML::Value << size();
	MEmitter << YAML::EndMap;
	MEmitter << YAML::EndDoc;

	std::ofstream myfile;
	myfile.open(TextFileName,ios::out | ios::trunc);
	if(myfile.is_open())
	{
		//cout << "UserHeader: " << UserHeader << "Emitter: " << MEmitter.c_str() << endl;
		myfile << UserHeader << MEmitter.c_str();

		float *pData = &Data[0];
		NbVects = size();
		for(UINT i=0;i<NbVects;i++)
		{
			myfile << (*pData++);
			for(int j=1;j<VectSize;j++)
			{
				myfile << "," << (*pData++);
			}
			myfile << std::endl;
		}
		myfile.close();
		printf("File (%s) Exported with %d Features\n",TextFileName.c_str(),VectSize);
	}
	else
	{
		printf("Couldn't Export File (%s)\n",TextFileName.c_str());
	}
}
//----------------------------------------------------------------------------------
void VectorsTable_c::Import(std::string TextFileName,bool isAppend)
{
	//-----------------------------Left Issue do not parse the YAML Header to set tge TypeName !!!
	std::ifstream myfile;
	std::string line;
	myfile.open(TextFileName,ios::in);
	if(Data.empty())
	{
		isAppend = false;
	}
	else
	{
		if(!isAppend)//if(isReWrite)
		{
			clear();
		}
	}
	if(myfile.is_open())
	{
		//--------------------------remove docs '...'
		int NbTagsFound = 0;
		while((NbTagsFound !=2) && !myfile.eof())
		{
			std::string Line;
			getline(myfile,Line);
			if(Line.compare("...") == 0)
			{
				NbTagsFound++;
			}
			if(Line.compare("TypeName") == 0)
			{
				TypeName = Line.substr( Line.find_last_of( ':' ) +2,Line.length() );//BAD Manually parsing YAML what laziness
				NbTagsFound++;
			}
		}
		if(NbTagsFound == 0)
		{
			//myfile.seekg(0, ios::beg);//just a simple .csv File without header - so restart
			myfile.close();//apparently seekg brakes the text format and getline() works no more after it
			myfile.open(TextFileName,ios::in);
		}
		//--------------------------remove docs

		std::vector<float> VData;
		getline(myfile,line);
		ParseFloatLine(line,VData);//Set DataSize
		int NbDataPerLine = (int)VData.size();
		if(isAppend)
		{
			assert(VectSize == NbDataPerLine);
		}
		else
		{
			VectSize = NbDataPerLine;
		}
		push(VData);
		//-----------Start Cycle with getline
		int NbLines = 1;
		getline(myfile,line);
		while(!myfile.eof())
		{
			ParseFloatLine(line,VData);
			assert(VData.size() == NbDataPerLine);
			push(VData);
			getline(myfile,line);
			NbLines++;
		}
		myfile.close();
		printf("Imported %d Samples of VectSize %d (lines %d)\n",size(),VectSize,NbLines);
	}
	else
	{
		printf("Couldn't Import File (%s)\n",TextFileName.c_str());
	}
	
}
//----------------------------------------------------------------------------------
bool VectorsTable_c::load(const std::string &FileName,bool isAppend,int verb,YAML::Node &user_doc)
{
	bool Res = true;

	if(Data.empty())
	{
		isAppend = false;//override it
	}

	mcv::MixedFileManager_c FManager;
	std::stringstream YamlHeader = FManager.Parse(FileName);
	if(YamlHeader.str().empty())
	{
		cout << "VectorsTable_c::load() Couldn't load File: " << FileName << endl;
		return false;
	}
	YAML::Parser parser;
	parser.Load(YamlHeader);
	parser.GetNextDocument(user_doc);
	YAML::Node doc;
	parser.GetNextDocument(doc);
	
	doc["TypeName"] >> TypeName;
	if(doc.FindValue("ClassesNames"))
	{
		doc["ClassesNames"] >> ClassesNames;
	}
	size_t vNbFloats,vVectSize,vNbVects;
	doc["NbFloats"] >> vNbFloats;
	doc["VectSize"] >> vVectSize;
	doc["NbVects"] >> vNbVects;
	assert(vNbVects == vNbFloats / vVectSize);
	assert((vNbFloats % vVectSize) == 0);
	assert(FManager.DataInfo.DataSizeBytes == vNbFloats*4);//Check Coherence between myinfo and FManager Info

	if(isAppend)//Data not empty
	{
		assert(vVectSize == VectSize);
	}
	size_t oldDataSize = Data.size();
	size_t FileDataSize = vNbFloats;

	Data.resize(oldDataSize + FileDataSize);
	float *pNewData = &Data[oldDataSize];
	FManager.LoadData((char*)pNewData,FileDataSize*4);

	VectSize = vVectSize;//Already asserted on Append or Affect it here
	NbVects = Data.size() / VectSize;
	
	return Res;
}
//----------------------------------------------------------------------------------
bool VectorsTable_c::load(const std::string &FileName,bool isAppend,int verb)
{
	YAML::Node ThrowAwayThisDoc;
	return load(FileName,isAppend,verb,ThrowAwayThisDoc);
}
//----------------------------------------------------------------------------------
void VectorsTable_c::CopyAppend(const VectorsTable_c &Table)
{
	if(Data.size() == 0)//just copy and Initialize VectSize and Type
	{
		VectSize = Table.VectSize;
		TypeName = Table.TypeName;
	}
	else//append case
	{
		assert(Table.VectSize == VectSize);
		assert(TypeName.compare(Table.TypeName) == 0);
	}
	size_t OldDataSize = Data.size();
	Data.resize(OldDataSize + Table.Data.size());//here the reallocation magic operates
	float * pNewData = &Data[OldDataSize];
	memcpy(&Data[OldDataSize],&Table.Data[0],Table.Data.size()*sizeof(float));
}
//----------------------------------------------------------------------------------
void VectorsTable_c::clear()
{
	Data.clear();
}
//----------------------------------------------------------------------------------
cv::Mat VectorsTable_c::GetMat()
{
	NbVects = size();
	return cv::Mat((int)NbVects,VectSize,CV_32FC1,&Data[0]);//Shallow copy the constructor in TableMat
}
//----------------------------------------------------------------------------------
void VectorsTable_c::FilterRemoveDuplicates()
{
	std::set<std::vector<float>> TableSet;
	NbVects = size();
	assert(NbVects!=0);
	std::vector<float> Vecti(VectSize);
	for(int i=0;i<NbVects;i++)
	{
		Vecti = at(i);
		TableSet.insert(Vecti);
	}
	//TStop(t,"SetInsertion");// 78 ms
	Data.resize(TableSet.size() * VectSize);
	float *pData = &Data[0];
	//TStop(t,"resize");// 7 ms  ???!!! what for in resize

	std::set<std::vector<float>>::iterator it;
	int index = 0;
	for(it=TableSet.begin(); it!=TableSet.end(); it++)
	{
		for(int i=0;i<VectSize;i++)
		{
			(*pData++) = (*it)[i];//couldn't get a pointer at it as it's a const float*
		}
	}
	//TStop(t,"copy from set to vect");// 1.3 ~ 1.8 ms
}
//---------------------------------------------------------------------------------------------------------------
//CompareClasses_ComputeMap can append the processing
void VectorsTable_c::CompareClasses_ComputeMap(	VectorsTable_c &RefTable,int MaxClasses,
												MLMap_t &RefClassMap,
												MLMap_t &ResClassMap)
{
	assert(RefTable.VectSize == VectSize);
	assert(TypeName.compare(RefTable.TypeName) == 0);
	assert(RefTable.Data.size() == Data.size());
	//------------------------------------------------------------------find the classes map in Reference
	int NbRefClasses = 0;
	size_t NbRefVects = RefTable.size();
	for(size_t i=0;i<NbRefVects;i++)
	{
		std::vector<float> Sample = RefTable.at(i);
		if(RefClassMap.find(Sample) == RefClassMap.end())//Not found
		{
			RefClassMap[Sample] = NbRefClasses++;//New Class Added
		}
	}
	assert(NbRefClasses <= MaxClasses);
	//------------------------------------------------------------------Reaffect Ordered indices
	MLMap_t::iterator Rit;
	int index = 0;
	for(Rit = RefClassMap.begin();Rit!=RefClassMap.end();Rit++)
	{
		(*Rit).second = index++;
	}
	//------------------------------------------------------------------find the classes map in Result
	int NbResClasses = 0;
	NbVects = size();
	for(size_t i=0;i<NbVects;i++)
	{
		std::vector<float> Sample = at(i);
		if(ResClassMap.find(Sample) == ResClassMap.end())//Not found
		{
			ResClassMap[Sample] = NbResClasses++;//New Class Added
		}
	}
	assert(NbResClasses <= MaxClasses);
	//------------------------------------------------------------------Reaffect Ordered indices
	index = 0;
	for(Rit = ResClassMap.begin();Rit!=ResClassMap.end();Rit++)
	{
		(*Rit).second = index++;
	}
}
//---------------------------------------------------------------------------------------------------------------
//CompareClasses_ComputeConf also appends the process
void VectorsTable_c::CompareClasses_ComputeConf(	VectorsTable_c &RefTable,
													MLMap_t &RefClassMap,
													MLMap_t &ResClassMap,
													MLCMat_t &ConfMat			)
{
	assert(RefTable.VectSize == VectSize);
	assert(TypeName.compare(RefTable.TypeName) == 0);
	assert(RefTable.Data.size() == Data.size());
	//------------------------------------------------------------------Process Confusion Matrix
	size_t NbRefClasses = RefClassMap.size();
	size_t NbResClasses = ResClassMap.size();
	if(ConfMat.size() != NbRefClasses)//if First time only
	{
		ConfMat = MLCMat_t(NbRefClasses,NbResClasses);//Initialized to 0 ??!!
	}
	NbVects = Data.size() / VectSize;
	for(int i=0;i<NbVects;i++)
	{
		std::vector<float> RefSample = RefTable.at(i);
		std::vector<float> ResSample = at(i);
		int RefClassID = RefClassMap[RefSample];
		int ResClassID = ResClassMap[ResSample];
		ConfMat[RefClassID][ResClassID]++;
	}
}
//---------------------------------------------------------------------------------------------------------------
float VectorsTable_c::CompareClasses(VectorsTable_c &RefTable,int MaxClasses,bool isPrintTab,bool isPrintAvg)
{
	MLMap_t RefClassMap;
	MLMap_t ResClassMap;
	MLCMat_t ConfMat;
	//mcv::TabPrintf(ClassesNames,"ClassesNames");
	CompareClasses_ComputeMap(RefTable,MaxClasses,RefClassMap,ResClassMap);
	CompareClasses_ComputeConf(RefTable,RefClassMap,ResClassMap,ConfMat);
	return CompareClasses_Printf(RefClassMap,ResClassMap,ConfMat,isPrintTab,isPrintAvg);
}
//---------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------
//							MLData_c
//---------------------------------------------------------------------------------------------------------------
MLData_c::MLData_c()
{
	NodeName = "NotInitialized";
}
//---------------------------------------------------------------------------------------------------------------
MLData_c::MLData_c(std::string vCfgFName,std::string MLDataNodeName)//Fs for MultiStreams, and Node for monoStream
{
	Init(vCfgFName,MLDataNodeName);
}
//---------------------------------------------------------------------------------------------------------------
void MLData_c::Init(std::string vCfgFName,std::string MLDataNodeName)
{
	ConfigFileName = vCfgFName;
	NodeName = MLDataNodeName;
	Fs = cv::FileStorage(ConfigFileName,cv::FileStorage::READ);
	MainPath = ConfigFileName.substr( 0, ConfigFileName.find_last_of( '\\' ) +1 );
}
//---------------------------------------------------------------------------------------------------------------
void MLData_c::save(const char*TrainTag, const char*RespTag)
{
	std::string TrainFileName = MainPath + (std::string)Fs[NodeName][TrainTag];
	std::string RespFileName = MainPath + (std::string)Fs[NodeName][RespTag];

	assert(Train.size() == Resp.size());

	Train.save(TrainFileName);
	Resp.save(RespFileName);
}
//---------------------------------------------------------------------------------------------------------------
void MLData_c::load(const char*TrainTag, const char*RespTag)
{
	std::string TrainFileName = MainPath + (std::string)Fs[NodeName][TrainTag];
	std::string RespFileName = MainPath + (std::string)Fs[NodeName][RespTag];

	Train.load(TrainFileName,false);//No append here
	Resp.load(RespFileName,false);

	assert(Train.size() == Resp.size());
}
//---------------------------------------------------------------------------------------------------------------
void MLData_c::Export(const char*TrainTag, const char*RespTag)
{
	std::string TrainFileName = MainPath + (std::string)Fs[NodeName][TrainTag];
	std::string RespFileName = MainPath + (std::string)Fs[NodeName][RespTag];

	assert(Train.size() == Resp.size());

	Train.Export(TrainFileName);
	Resp.Export(RespFileName);
}
//---------------------------------------------------------------------------------------------------------------
void MLData_c::Import(const char*TrainTag, const char*RespTag)
{
	std::string TrainFileName = MainPath + (std::string)Fs[NodeName][TrainTag];
	std::string RespFileName = MainPath + (std::string)Fs[NodeName][RespTag];

	Train.Import(TrainFileName);
	Resp.Import(RespFileName);

	assert(Train.size() == Resp.size());
}
//---------------------------------------------------------------------------------------------------------------
void MLData_c::CopyAppend(MLData_c &Data)
{
	bool isDispTime = false;

	double t;
	if(isDispTime)TStart(t);
	Resp.CopyAppend(Data.Resp);
	Train.CopyAppend(Data.Train);
	if(isDispTime)TStop(t,"MLDataCopyAppend()");
}
//---------------------------------------------------------------------------------------------------------------
void MLData_c::loadTrainChannel(int Start,int Last,const char* NodeTag,const char*TrainTag)//From "Views" Node
{
	MultiCamStream TrainStreams(ConfigFileName,NodeTag,TrainTag);//can be one stream if the NodeTag is a Map
	Train.clear();//---------------------------------------------NO Append, WILL only append channel Files !!!!
	if(Start<0)
	{
		Start = TrainStreams.FirstIndex;
	}
	if(Last<0)
	{
		Last = TrainStreams.LastIndex;
	}
	for(int i=Start;i<=Last;i++)
	{
		for(int ViewIndex = 0;ViewIndex<TrainStreams.ImgsStream.size();ViewIndex++)
		{
			std::string FileName = TrainStreams.ImgsStream[ViewIndex].GetName(i);
			Train.load(FileName,true,0);//load Append
		}
	}
}
//---------------------------------------------------------------------------------------------------------------
void MLData_c::loadRespChannel(int Start,int Last,const char* NodeTag,const char*RespTag)//From "Views" Node
{
	MultiCamStream RespStreams(ConfigFileName,NodeTag,RespTag);
	Resp.clear();
	if(Start<0)
	{
		Start = RespStreams.FirstIndex;
	}
	if(Last<0)
	{
		Last = RespStreams.LastIndex;
	}
	for(int i=Start;i<=Last;i++)
	{
		for(int ViewIndex = 0;ViewIndex<RespStreams.ImgsStream.size();ViewIndex++)
		{
			std::string FileName = RespStreams.ImgsStream[ViewIndex].GetName(i);
			Resp.load(FileName,true,0);//load Append
		}
	}
}
//---------------------------------------------------------------------------------------------------------------
//----- Filtering Utilities -------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------
void MLData_c::Filter_MergeTovect(VectorsTable_c &AllVect)
{
	assert(Train.size() == Resp.size());
	int VectSize = Train.VectSize + Resp.VectSize;
	int cpyTrainsize = Train.VectSize * 4;
	int cpyRespsize = Resp.VectSize * 4;
	//AllVect.Reset(VectSize);
	AllVect.VectSize = VectSize;//reset no clear fo future resize		remove clear() won nothing
	
	int NbVects = Train.size();
	AllVect.Data.resize(NbVects * VectSize);
	for(int i=0;i<NbVects;i++)
	{
		std::vector<float> OneVect(VectSize);
		memcpy(&OneVect[0],&Train.at(i)[0],cpyTrainsize);				//30% speedup over OneVect.insert(OneVect.begin(),...
		memcpy(&OneVect[Train.VectSize],&Resp.at(i)[0],cpyRespsize);
		AllVect.copyto(i,OneVect);
	}
}
//---------------------------------------------------------------------------------------------------------------
void MLData_c::Filter_SplitFromVect(VectorsTable_c &AllVect)
{
	assert(AllVect.VectSize == Train.VectSize + Resp.VectSize);
	size_t NbVects = AllVect.size();
	size_t cpyTrainsize = Train.VectSize * 4;
	size_t cpyRespsize = Resp.VectSize * 4;
	Train.Data.resize(NbVects * Train.VectSize);//							resize no clear won 15%
	Resp.Data.resize(NbVects * Resp.VectSize);
	for(size_t i=0;i<NbVects;i++)
	{
		std::vector<float> OneVectTrain(Train.VectSize);
		std::vector<float> OneVectResp(Resp.VectSize);
		memcpy(&OneVectTrain[0],&AllVect.at(i)[0],cpyTrainsize);					//won nothing over insert
		memcpy(&OneVectResp[0],&AllVect.at(i)[Train.VectSize],cpyRespsize);
		Train.copyto(i,OneVectTrain);
		Resp.copyto(i,OneVectResp);
	}
}
//---------------------------------------------------------------------------------------------------------------
void MLData_c::Filter_RemoveDuplicates()
{
	double t;
	TStart(t);
	VectorsTable_c AllVect;
	Filter_MergeTovect(AllVect);//					TStop(t,"Filter_MergeTovect");		286 ms /M
	size_t NbStart = AllVect.size();
	AllVect.FilterRemoveDuplicates();//				TStop(t,"FilterRemoveDuplicates");	1000 ms /M
	size_t NbEnd = AllVect.size();
	Filter_SplitFromVect(AllVect);//				TStop(t,"Filter_SplitFromVect");	130 ms /M
	printf("Reduce from %d Kept %d :",NbStart,NbEnd);
	TStop(t," ");
}
//---------------------------------------------------------------------------------------------------------------
void MLData_c::Filter_RemoveSimilar(float eps)
{
}
//---------------------------------------------------------------------------------------------------------------
void MLData_c::FilterData()//Rather Filter Adding data
{
}
//---------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------
//						MLModel_c
//---------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------
MLModel_c::MLModel_c()
{
	pLearnedData = NULL;
	isLearnedDataTobeFreed = false;
	ModelType = mrg::Model_Not_Specified;
}
//---------------------------------------------------------------------------------------------------------------
MLModel_c::MLModel_c(std::string vCfgFName,std::string MLModelNodeName)
{
	pLearnedData = NULL;
	isLearnedDataTobeFreed = false;
	ModelType = mrg::Model_Not_Specified;

	Init(vCfgFName,MLModelNodeName);

}
//---------------------------------------------------------------------------------------------------------------
void MLModel_c::Init(std::string vCfgFileName,std::string MLDataNodeName)
{
	NodeName = MLDataNodeName;
	ConfigFileName = vCfgFileName;
	Fs = cv::FileStorage(ConfigFileName,cv::FileStorage::READ);
	MainPath = ConfigFileName.substr( 0, ConfigFileName.find_last_of( '\\' ) +1 );
}
//---------------------------------------------------------------------------------------------------------------
MLModel_c::~MLModel_c()
{
	if(isLearnedDataTobeFreed)
	{
		free(pLearnedData);
	}
}
//---------------------------------------------------------------------------------------------------------------
void MLModel_c::Learn(MLData_c	&LearnData)//the ANN Implementation, to be separated with switch Type
{
	if(isLearnedDataTobeFreed)
	{
		free(pLearnedData);
	}
	pLearnedData = &LearnData;
	isLearnedDataTobeFreed = false;
}
//---------------------------------------------------------------------------------------------------------------
void MLModel_c::LearnCopy(MLData_c	&LearnData)//the ANN Implementation, to be separated with switch Type
{
	if(isLearnedDataTobeFreed)
	{
		free(pLearnedData);
	}
	pLearnedData = new MLData_c(LearnData);//Check that everything is copied
	isLearnedDataTobeFreed = true;
	//then copy LearnData into pANNData
}
//---------------------------------------------------------------------------------------------------------------
void MLModel_c::saveModel()
{
	if(ModelType == mrg::Model_ANN)
	{
		ANN.saveModel(Fs[NodeName],MainPath);
	}
}
//---------------------------------------------------------------------------------------------------------------
void MLModel_c::loadModel()
{
	//"NotSpecified", "ANN", "SVM", "RndForest", "Adaboost", "NeuralNet", 
	assert(!Fs[NodeName].empty());// Voila !
	std::string ModelTypeName = (std::string)Fs[NodeName]["Type"];
	if(ModelTypeName.compare("ANN") == 0)
	{
		ModelType = mrg::Model_ANN;
		ANN.loadModel(Fs[NodeName],MainPath);
	}
}
//---------------------------------------------------------------------------------------------------------------
void MLModel_c::Train(MLModel_t mdl_type,float Param)//if ANN then Train ANN with LearnedData
{
	if(mdl_type == mrg::Model_ANN)
	{
		ModelType = mrg::Model_ANN;
		ANN.Train(*pLearnedData,Param);
	}
}
//---------------------------------------------------------------------------------------------------------------
void MLModel_c::Predict(MLData_c	&Data)
{
	assert(ModelType != mrg::Model_Not_Specified);
	double t;
	TStart(t);
	if(ModelType == mrg::Model_ANN)
	{
		ANN.Predict(Data,13);// K-Votes with 13
	}
	TStop(t,"Predict(Data)");
}
//---------------------------------------------------------------------------------------------------------------
void MLModel_c::Predict(int Start,int Last,const char* NodeTag,const char* TrainChan_In, const char *PredictRespChan_Out)
{
	//---------------------------------------------------------------------------------Train In
	MultiCamStream TrainStreams;
	int NbTrainStreams = TrainStreams.SetStreams(ConfigFileName,NodeTag,TrainChan_In);
	printf("Start MLModel_c::Predict (%d,%d) %d Channels from:\n%s\n",Start,Last,NbTrainStreams,MainPath.c_str());
	//---------------------------------------------------------------------------------Resp out
	MultiCamStream RespStreams;
	int NbRespStreams = RespStreams.SetStreams(ConfigFileName,NodeTag,PredictRespChan_Out);
	assert(NbRespStreams == NbTrainStreams);
	//---------------------------------------------------------------------------------Loop on Poses
	double t;
	TStart(t);
	if(Start<0)
	{
		assert(TrainStreams.FirstIndex == RespStreams.FirstIndex);
		Start = TrainStreams.FirstIndex;
	}
	if(Last<0)
	{
		assert(TrainStreams.LastIndex == RespStreams.LastIndex);
		Last = TrainStreams.LastIndex;
	}
	for(int PoseIndex=Start;PoseIndex<=Last;PoseIndex++)
	{
		for(int i=0;i<NbTrainStreams;i++)
		{
			printf("p(%d)v(%d):",PoseIndex,i);
			std::string TrainFileName = TrainStreams.ImgsStream[i].GetName(PoseIndex);
			std::string RespFileName = RespStreams.ImgsStream[i].GetName(PoseIndex);
			MLData_c Data;
			Data.Train.load(TrainFileName,false,0);//No need for YAML Header
			Predict(Data);
			Data.Resp.save(RespFileName);//Should forward the MLData.Resp Header
			bool isExport = false;
			if(isExport)
			{
				std::string EFName;
				EFName = RespFileName + ".csv";
				Data.Resp.Export(EFName);
			}
		}
	}
	TStop(t,"Predict(From Files)");
	printf("Prediction from Poses (%d,%d) in %d Streams over\n",Start,Last,NbTrainStreams);
}
//---------------------------------------------------------------------------------------------------------------
float MLModel_c::CompareClass(int Start,int Last,const char* NodeTag,const char* RespChanRef_In, const char *RespChanRes_In,bool ShowEveryConfTable)
{
	//---------------------------------------------------------------------------------Train In
	MultiCamStream RespRefStreams;
	int NbRespRefStreams = RespRefStreams.SetStreams(ConfigFileName,NodeTag,RespChanRef_In);
	//---------------------------------------------------------------------------------Resp out
	MultiCamStream RespResStreams;
	int NbRespResStreams = RespResStreams.SetStreams(ConfigFileName,NodeTag,RespChanRes_In);
	assert(NbRespRefStreams == NbRespResStreams);
	printf("\nStart MLModel_c::CompareClass (%d,%d) %d Streams:\n",Start,Last,NbRespRefStreams);
	printf("(%s) and (%s) from (%s)\n",RespChanRef_In,RespChanRes_In,MainPath.c_str());
	double t;
	TStart(t);
	//---------------------------------------------------------------------------------Loop on Poses
	if(Start<0)
	{
		assert(RespRefStreams.FirstIndex == RespResStreams.FirstIndex);
		Start = RespRefStreams.FirstIndex;
	}
	if(Last<0)
	{
		assert(RespRefStreams.LastIndex == RespResStreams.LastIndex);
		Last = RespRefStreams.LastIndex;
	}
	std::vector<float> AllClassesStats;
	int NbUnits = 0;

	MLMap_t RefClassMap;
	MLMap_t ResClassMap;
	MLCMat_t ConfMat;
	printf("Loading and processing all Data...\n");
	for(int PoseIndex=Start;PoseIndex<=Last;PoseIndex++)
	{
		for(int i=0;i<NbRespRefStreams;i++)
		{
			std::string RespRefFileName = RespRefStreams.ImgsStream[i].GetName(PoseIndex);
			std::string RespResFileName = RespResStreams.ImgsStream[i].GetName(PoseIndex);
			VectorsTable_c RefTab(RespRefFileName);
			VectorsTable_c ResTab(RespResFileName);
			ResTab.CompareClasses_ComputeMap(RefTab,20,RefClassMap,ResClassMap);
		}
	}
	printf("Processing average and Display:\n");
	bool onetime = true;
	for(int PoseIndex=Start;PoseIndex<=Last;PoseIndex++)
	{
		for(int i=0;i<NbRespRefStreams;i++)
		{
			std::string RespRefFileName = RespRefStreams.ImgsStream[i].GetName(PoseIndex);
			std::string RespResFileName = RespResStreams.ImgsStream[i].GetName(PoseIndex);
			VectorsTable_c RefTab(RespRefFileName);
			VectorsTable_c ResTab(RespResFileName);
			if(onetime)
			{
				if(!RefTab.ClassesNames.empty())
					mcv::TabPrintf(RefTab.ClassesNames,"ClassesNames");
				onetime = false;
			}
			if(ShowEveryConfTable)
			{
				printf("pose,%d,view,%d,",PoseIndex,i);ResTab.CompareClasses(RefTab,20);//so that we process one Unit
			}
			ResTab.CompareClasses_ComputeConf(RefTab,RefClassMap,ResClassMap,ConfMat);//so that we append all computation
			NbUnits++;
		}
	}
			
	TStop(t,"CompareClass(From Files)");
	printf("Total:%d\n",NbUnits);

	return CompareClasses_Printf(RefClassMap,ResClassMap,ConfMat);//display concatenated result
}
//---------------------------------------------------------------------------------------------------------------
//--------------------------  ANN_Model_c  ----------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------
ANN_Model_c::ANN_Model_c()
{
	pFlannIndex = NULL;
	ANNisReady = false;
	pLearnedData = NULL;
	isLearnedDataTobeFreed = false;
}
//---------------------------------------------------------------------------------------------------------------
ANN_Model_c::~ANN_Model_c()
{
	if(isLearnedDataTobeFreed)//is it our new or others @
	{
		free(pLearnedData);
		pLearnedData = NULL;//Who cares
	}
	if(pFlannIndex != NULL)//used only by me
	{
		free(pFlannIndex);
	}
}
//---------------------------------------------------------------------------------------------------------------
void ANN_Model_c::Train(MLData_c	&Data,float Param)
{
	int verb = 1;

	if(isLearnedDataTobeFreed)
	{
		free(pLearnedData);
		pLearnedData = NULL;
	}
	pLearnedData = &Data;
	isLearnedDataTobeFreed = false;

	assert(pLearnedData->Train.size() == pLearnedData->Resp.size());
	
	if(pLearnedData->Train.Data.empty())
	{
		printf("No Training, Learn Data First...\n");
		return;
	}

	if(verb>0)printf("Building flann Index of %d samples of %d dims...\n",
		pLearnedData->Train.size(),pLearnedData->Train.VectSize);
	double t;
	if(pFlannIndex != NULL)
	{
		free(pFlannIndex);
	}
	cv::flann::flann_algorithm_t ANN_Index = cv::flann::FLANN_INDEX_KMEANS;printf("Algo: FLANN_INDEX_KMEANS\n");
	// FLANN_INDEX_KMEANS - FLANN_INDEX_LINEAR - FLANN_INDEX_KDTREE - FLANN_INDEX_HIERARCHICAL - FLANN_INDEX_LSH - FLANN_INDEX_AUTOTUNED

	cv::flann::flann_distance_t distType = cv::flann::FLANN_DIST_L2;printf("distance: FLANN_DIST_L2\n");
    //FLANN_DIST_L2 = 1,FLANN_DIST_L1,FLANN_DIST_MINKOWSKI,FLANN_DIST_MAX,FLANN_DIST_HIST_INTERSECT,
    //FLANN_DIST_HELLINGER,FLANN_DIST_CHI_SQUARE,FLANN_DIST_KULLBACK_LEIBLER,
	//cv::flann::Index FlannIndex;

	TStart(t);
	switch(ANN_Index)
	{
		case cv::flann::FLANN_INDEX_LINEAR :		
			{
				pFlannIndex = new	cv::flann::Index(pLearnedData->Train.GetMat(),
									cv::flann::LinearIndexParams());
			}
			break;
		case cv::flann::FLANN_INDEX_KDTREE:		
			{
				int trees = 1;//KDTrees 4 Trees
				printf("trees: %d\n",trees);
				pFlannIndex = new	cv::flann::Index(pLearnedData->Train.GetMat(),
									cv::flann::KDTreeIndexParams(trees));
			}
			break;
		case cv::flann::FLANN_INDEX_KMEANS:
			{
				int branching = 8;				printf("branching == %d\n",branching);
				int iterations = 1;				printf("iterations == %d\n",iterations);
				cvflann::flann_centers_init_t CInit = cvflann::CENTERS_RANDOM;// - CENTERS_KMEANSPP - CENTERS_GONZALES
				printf("CENTERS_RANDOM\n");
				float cb_index = 0.2f;//0.1 0.2 0.4 0.8 1.0 no noticable changes
				printf("cb_index == %1.2f\n",cb_index);
#ifdef DOESNT_THIS_WORK
				cv::flann::Index ANNIndexParams;
				ANNIndexParams = cv::flann::KMeansIndexParams(branching,iterations,CInit,cb_index);
				pFlannIndex = new cv::flann::Index(pLearnedData->Train.TableMat,ANNIndexParams);//DO NOT WORK
#else
				pFlannIndex = new	cv::flann::Index(pLearnedData->Train.GetMat(),
									cv::flann::KMeansIndexParams(branching,iterations,CInit,cb_index));// WORK
#endif
			}
			break;
		case cv::flann::FLANN_INDEX_AUTOTUNED://		spent one day and a half and still no sign of life with 3.5 M Points
			{
				float target_precision = 1.0f;//float target_precision = Param;//0.8f
				float build_weight = 0.0f;//float build_weight = 0.01f;
				float memory_weight = 0;
				float sample_fraction = 0.1f;
				printf("target_precision == %1.2f\n",target_precision);
				printf("build_weight == %1.2f\n",build_weight);
				printf("memory_weight == %1.2f\n",memory_weight);
				printf("sample_fraction == %1.2f\n",sample_fraction);
				pFlannIndex = new	cv::flann::Index(pLearnedData->Train.GetMat(),
									cv::flann::AutotunedIndexParams(target_precision,build_weight,memory_weight,sample_fraction)	);
			}
			break;
	}
	TStop(t,"ANN Index Construction time");
	ANNisReady = true;
}
//---------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------
void ANN_Model_c::PrintModel(cv::flann::IndexParams &ANNIndexParams)
{

	cv::flann::flann_distance_t Dist = pFlannIndex->getDistance();
	switch(Dist)
	{
		case cvflann::FLANN_DIST_L2 :printf("FLANN_DIST_L2\n");break;
		case cvflann::FLANN_DIST_L1 :printf("FLANN_DIST_L1\n");break;
		case cvflann::FLANN_DIST_MINKOWSKI :printf("FLANN_DIST_MINKOWSKI\n");break;
		case cvflann::FLANN_DIST_MAX :printf("FLANN_DIST_MAX\n");break;
		case cvflann::FLANN_DIST_HIST_INTERSECT :printf("FLANN_DIST_HIST_INTERSECT\n");break;
		case cvflann::FLANN_DIST_HELLINGER :printf("FLANN_DIST_HELLINGER\n");break;
		case cvflann::FLANN_DIST_CHI_SQUARE :printf("FLANN_DIST_CHI_SQUARE\n");break;
		case cvflann::FLANN_DIST_KULLBACK_LEIBLER :printf("FLANN_DIST_KULLBACK_LEIBLER\n");break;
		default:printf("Another distance with no printf support\n");break;
	}
	cv::flann::flann_algorithm_t Algo = pFlannIndex->getAlgorithm();
	switch(Algo)
	{
	case cv::flann::FLANN_INDEX_KDTREE :
		printf("FLANN_INDEX_KDTREE:\n");

		//printf("KDTreeIndexParams(trees = %d)\n",trees);
		break;
	case cv::flann::FLANN_INDEX_KMEANS :
		//printf("CENTERS_RANDOM\n");
		//CInit = cv::flann::CENTERS_KMEANSPP;printf("CENTERS_KMEANSPP\n");
		//CInit = cv::flann::CENTERS_GONZALES;printf("CENTERS_GONZALES\n");
		//float cb_index = 0.2f;//0.1 0.2 0.4 0.8 1.0 no noticable changes
		//ANNIndexParams = cv::flann::KMeansIndexParams(branching,iterations,CInit,cb_index);
		//printf("KMeansIndexParams(branching = %d, iterations = %d, , cb_index = %1.2f)\n",branching,iterations,cb_index);
		printf("FLANN_INDEX_KMEANS:\n");
		break;
	case cv::flann::FLANN_INDEX_HIERARCHICAL :
		printf("FLANN_INDEX_HIERARCHICAL:\n");
		break;
	case cv::flann::FLANN_INDEX_LSH :
		printf("FLANN_INDEX_LSH:\n");
		break;
	case cv::flann::FLANN_INDEX_AUTOTUNED :
		printf("FLANN_INDEX_AUTOTUNED:\n");
		/*printf("AutotunedIndexParams(target_precision = %f\n",target_precision);
		printf("build_weight = %f, \n",build_weight);
		printf("memory_weight = %f,\n",memory_weight);
		printf("sample_fraction = %f)\n\n",sample_fraction);*/
		break;
	default:
		printf("Another Index with no printf support\n");
		break;
	}

/*	std::vector<std::string> names;
	std::vector<int> types;
	std::vector<std::string> strValues;
	std::vector<double> numValues;
	ANNIndexParams.getAll(names,types,strValues,numValues);
	*/
        //(*this)["algorithm"] = FLANN_INDEX_KMEANS;
       // (*this)["branching"] = branching;
        //(*this)["iterations"] = iterations;
        //(*this)["centers_init"] = centers_init;
        //] = cb_index;

	/*for(int i=0;i<names.size();i++)
	{
		printf("%s : %s\n",names[i].c_str(),strValues[i].c_str());
	}
	*/
}
//---------------------------------------------------------------------------------------------------------------

void ANN_Model_c::Predict(MLData_c	&Data,int K)//no concatenation here
{
	assert(Data.Train.TypeName.compare(pLearnedData->Train.TypeName) == 0);
	assert(Data.Train.VectSize == pLearnedData->Train.VectSize);

	//Data.Resp takes info from Learned Resp
	Data.Resp.TypeName = pLearnedData->Resp.TypeName;
	Data.Resp.resize(pLearnedData->Resp.VectSize,Data.Train.size());//sets VectSize

	if(!ANNisReady)
	{
		printf("Train ANN Before precdicting\n");
		return;
	}
	//int checks = 32;
	int checks = 16;//(8 Vs 16) 0.75 per 1000 different while (16==32==64)
	float eps = 0.0f;
	cv::flann::SearchParams SP(checks,eps);
	std::vector<int> Indices(K);
	std::vector<float> Dists(K);

	size_t NbVectsToPredict = Data.Train.size();
	for(size_t i=0;i<NbVectsToPredict;i++)
	{
		pFlannIndex->knnSearch(Data.Train.at(i),Indices,Dists,K,SP);
		if(K == 1)
		{
			Data.Resp.copyto(i,pLearnedData->Resp.at(Indices[0]));
		}
		else
		{
			MLMap_t Map;
			for(int j=0;j<K;j++)
			{
				Map[pLearnedData->Resp.at(Indices[j])] = j;//dummy Val will be replaced by index; how to do better ?
			}
			//------------------------------------------------------------------Reaffect Ordered indices
			int NbLabels = Map.size();//max of K or NbLabels in All dataset
			std::vector<float>Votes(NbLabels);
			std::vector<std::vector<float>> VotingLabels(NbLabels);
			MLMap_t::iterator Rit;
			int index = 0;
			for(Rit = Map.begin();Rit!=Map.end();Rit++)
			{
				VotingLabels[index] = (*Rit).first;	//so that we have a direct inxed table
				(*Rit).second = index++;			//Here is the reverse indexed Table	
			}
			float refdist = TabMin(Dists);
			for(int j=0;j<K;j++)
			{
				float Vote;
				if(Dists[j] == refdist)// the case of 0 is treated here
				{
					Vote = 1;
				}
				else
				{
					Vote = refdist / Dists[j];//if Distances[i] == 0 then it's equal to minDist and treated in previous case
				}

				//Rather Read decomposed form below
				Votes[Map[pLearnedData->Resp.at(Indices[j])]] += Vote;//Equivalent to
			}
			int MaxVotesIndex = TabMax_Index(Votes);
			Data.Resp.copyto(i,VotingLabels[MaxVotesIndex]);//direct indexing to give back label

		}
	}
}
//---------------------------------------------------------------------------------------------------------------
void ANN_Model_c::saveModel(cv::FileNode &ANNModelNode,std::string MainPath)
{
	std::string ModelType = (std::string)ANNModelNode["Type"];
	assert(ModelType.compare("ANN") == 0);

	std::string ANNFile = MainPath + (std::string)ANNModelNode["ANNFile"];
	std::string ANNTrainFile = MainPath + (std::string)ANNModelNode["ANN_MLData"]["Train"];
	std::string ANNRespFile = MainPath + (std::string)ANNModelNode["ANN_MLData"]["Resp"];

	pLearnedData->Train.save(ANNTrainFile);
	pLearnedData->Resp.save(ANNRespFile);
	if(ANNisReady)
	{
		pFlannIndex->save(ANNFile);
	}
}
//---------------------------------------------------------------------------------------------------------------
void ANN_Model_c::loadModel(cv::FileNode &ANNModelNode,std::string MainPath)
{
	std::string ANNFile = MainPath + (std::string)ANNModelNode["ANNFile"];
	std::string ANNTrainFile = MainPath + (std::string)ANNModelNode["ANN_MLData"]["Train"];
	std::string ANNRespFile = MainPath + (std::string)ANNModelNode["ANN_MLData"]["Resp"];

	if(isLearnedDataTobeFreed)
	{
		free(pLearnedData);
		pLearnedData = NULL;
	}
	pLearnedData = new MLData_c();
	isLearnedDataTobeFreed = true;
	pLearnedData->Train.load(ANNTrainFile);
	pLearnedData->Resp.load(ANNRespFile);

	std::ifstream myfile = std::ifstream(ANNFile,ios::in | ios::binary);

	if (myfile.is_open())
	{
		myfile.close();
		cv::waitKey(10);//Maybe the File Handel is not released while it's already asked - ????!!

		printf("Loading ANN from File (%s)...\n",ANNFile.c_str());
		if(pFlannIndex != NULL)
		{
			free(pFlannIndex);
		}

		cv::flann::SavedIndexParams SIndex = cv::flann::SavedIndexParams(ANNFile);
		pFlannIndex = new cv::flann::Index(pLearnedData->Train.GetMat(),SIndex);
		PrintModel(SIndex);
		ANNisReady = true;
	}
	else
	{
		printf("No ANN from File (%s), Train the model before use...\n",ANNFile.c_str());
		if(pFlannIndex != NULL)
		{
			free(pFlannIndex);
			pFlannIndex = NULL;
		}
		ANNisReady = false;
	}
}
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//						BinFLANN_Model_c
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
BinFLANN_Model_c::BinFLANN_Model_c()
{
}
//-------------------------------------------------------------------------------------------------------------
BinFLANN_Model_c::~BinFLANN_Model_c()
{
}
//-------------------------------------------------------------------------------------------------------------
void BinFLANN_Model_c::Train(MLData_c	&Data,float Param)
{
}
//-------------------------------------------------------------------------------------------------------------
void BinFLANN_Model_c::Predict(MLData_c	&Data,int K)
{
}
//-------------------------------------------------------------------------------------------------------------
void BinFLANN_Model_c::saveModel(std::string CfgFile,std::string Tag)
{
	//FILE *stream;
	//flann::save_header<Distance>(stream,*pIndex);
}
//-------------------------------------------------------------------------------------------------------------
void BinFLANN_Model_c::loadModel(cv::FileNode &ANNModelNode,std::string MainPath)
{
}
//-------------------------------------------------------------------------------------------------------------
void BinFLANN_Model_c::PrintModel(cv::flann::IndexParams &ANNIndexParams)
{
}
//-------------------------------------------------------------------------------------------------------------
