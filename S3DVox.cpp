
#include "S3DVox.h"


#include <iostream>
#include <fstream>
#include "mcvGeneral.h"

using namespace g3d;
using namespace mcv;
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//---------------------------------VoxelSpace_c----------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
g3d::VoxelSpace_c::VoxelSpace_c(const EigenBox3D	&vBox,float vVoxelSize)
{
	Reset(vBox,vVoxelSize);
	isVisible=true;
}
//-------------------------------------------------------------------------------------------------------
g3d::VoxelSpace_c::VoxelSpace_c(const std::string& FileToloadFrom)
{
	load(FileToloadFrom);
	isVisible=true;
}
//-------------------------------------------------------------------------------------------------------
void g3d::VoxelSpace_c::Reset(Vector3f RefBoxLow, int vXv,int vYv,int vZv, float vVoxelSize)
{
	VoxelSize = vVoxelSize;
	Xv = vXv;//		ceil(BSize.x() / VoxelSize);
	Yv = vYv;//		ceil(BSize.y() / VoxelSize);
	Zv = vZv;//		ceil(BSize.z() / VoxelSize);
	strideY = Xv;
	strideZ = Xv * Yv;
	Data.resize(Zv * Yv * Xv);
	Box.Low = RefBoxLow;
	Box.High = RefBoxLow + Vector3f(Xv*VoxelSize,Yv*VoxelSize,Zv*VoxelSize);
	Box.HasPointAdded = true;//Should use Box.AddPoint, but left for flexibility
	if(Data.size()!=0)
	{
		memset(&Data[0],0,Data.size());
	}
	NbFill = 0;
	RenderStep = 1;
	ColorTable.clear();
	ColorTable.push_back(ScalarC(0,0,0,0));//Empty
	ColorTable.push_back(ScalarC(0,0,0,0));//Full - Uncknown
}
//-------------------------------------------------------------------------------------------------------
void g3d::VoxelSpace_c::Reset(const EigenBox3D	&vBox,float vVoxelSize)
{
	Box = vBox;
	Reset(vVoxelSize);
}
//-------------------------------------------------------------------------------------------------------
void g3d::VoxelSpace_c::Reset(float vVoxelSize)
{
	VoxelSize = vVoxelSize;
	Vector3f BSize = Box.size();
	//std::cout << Box.size();
	Xv = (int)ceil(BSize.x() / VoxelSize);
	Yv = (int)ceil(BSize.y() / VoxelSize);
	Zv = (int)ceil(BSize.z() / VoxelSize);
	strideY = Xv;
	strideZ = Xv * Yv;
	Data.resize(Zv * Yv * Xv);
	if(Data.size()!=0)
	{
		memset(&Data[0],0,Data.size());
	}
	NbFill = 0;
	RenderStep = 1;
	ColorTable.clear();
	ColorTable.push_back(ScalarC(0,0,0,0));
}
//--------------------------------------------------------------------------------------------------------------
unsigned char& g3d::VoxelSpace_c::operator()(int x,int y,int z)// Vox indexes to content
{
	return Data[z*strideZ + y*strideY + x];
}
//--------------------------------------------------------------------------------------------------------------
Vector4f g3d::VoxelSpace_c::Pos(int x,int y,int z)// Vox indexes to World Pos
{
	Vector4f Pos;
	if(VoxelSize == 1.0f)
	{
		Pos <<	x + Box.Low.x() + 0.5f,
				y + Box.Low.y() + 0.5f,
				z + Box.Low.z() + 0.5f,
				1;
	}
	else
	{
		Pos <<	VoxelSize * (x+0.5f) + Box.Low.x(),
				VoxelSize * (y+0.5f) + Box.Low.y(),
				VoxelSize * (z+0.5f) + Box.Low.z(),
				1;
	}
	return Pos;
}
//--------------------------------------------------------------------------------------------------------------
Vector4f g3d::VoxelSpace_c::Pos(int VoxOffset)						// Vox indexe to World Pos
{
	int vx = VoxOffset % strideY;
	int vy = (VoxOffset % strideZ)/strideY;
	int vz = VoxOffset / strideZ;
	return Pos(vx,vy,vz);
}
//--------------------------------------------------------------------------------------------------------------
unsigned char& g3d::VoxelSpace_c::operator()(const Vector3f &Pos)// World Pos to Vox content
{
	int lx,ly,lz;
	if(VoxelSize == 1.0f)
	{
		lx = (int)ceil(Pos.x() - Box.Low.x() - 0.5f);
		ly = (int)ceil(Pos.y() - Box.Low.y() - 0.5f);
		lz = (int)ceil(Pos.z() - Box.Low.z() - 0.5f);
	}
	else
	{
		lx = (int)ceil((Pos.x() - Box.Low.x())/VoxelSize - 0.5f);
		ly = (int)ceil((Pos.y() - Box.Low.y())/VoxelSize - 0.5f);
		lz = (int)ceil((Pos.z() - Box.Low.z())/VoxelSize - 0.5f);
	}
	return Data[lz*strideZ + ly*strideY + lx];
}
//--------------------------------------------------------------------------------------------------------------
unsigned char& g3d::VoxelSpace_c::operator()(const Vector4f &Pos)// World Pos to Vox content
{
	int lx,ly,lz;
	if(VoxelSize == 1.0f)
	{
		lx = (int)ceil(Pos.x() - Box.Low.x() - 0.5f);
		ly = (int)ceil(Pos.y() - Box.Low.y() - 0.5f);
		lz = (int)ceil(Pos.z() - Box.Low.z() - 0.5f);
	}
	else
	{
		lx = (int)ceil((Pos.x() - Box.Low.x())/VoxelSize - 0.5f);
		ly = (int)ceil((Pos.y() - Box.Low.y())/VoxelSize - 0.5f);
		lz = (int)ceil((Pos.z() - Box.Low.z())/VoxelSize - 0.5f);
	}
	return Data[lz*strideZ + ly*strideY + lx];
}
//--------------------------------------------------------------------------------------------------------------
int g3d::VoxelSpace_c::VectToOffset(const Vector3f &RelativeVect)		// RelativeVect to Relative Offset
{
	int lx,ly,lz;
	lx = (int)ceil(RelativeVect.x()/VoxelSize - 0.5f);
	ly = (int)ceil(RelativeVect.y()/VoxelSize - 0.5f);
	lz = (int)ceil(RelativeVect.z()/VoxelSize - 0.5f);
	return (lz*strideZ + ly*strideY + lx);
}
//--------------------------------------------------------------------------------------------------------------
void g3d::VoxelSpace_c::save(const std::string &FileName,const std::string &UserHeader)// Vect Size and Name Should be initialized
{
	assert(!Data.empty());//Yeah, no empty file, by design
	assert(Data.size() == Xv*Yv*Zv);
	YAML::Emitter MEmitter;
	if(UserHeader.empty())
	{
		MEmitter << YAML::BeginDoc;
		MEmitter << YAML::EndDoc;
	}
	MEmitter << YAML::BeginDoc;
	MEmitter << YAML::BeginMap;
	MEmitter << YAML::Key << "TypeName";
	MEmitter << YAML::Value << TypeName;
	MEmitter << YAML::Key << "VoxBox";
	MEmitter << YAML::Value << Box;
	MEmitter << YAML::Key << "VoxelSize";
	MEmitter << YAML::Value << VoxelSize;
	MEmitter << YAML::Key << "Xv";
	MEmitter << YAML::Value << Xv;
	MEmitter << YAML::Key << "Yv";
	MEmitter << YAML::Value << Yv;
	MEmitter << YAML::Key << "Zv";
	MEmitter << YAML::Value << Zv;
	MEmitter << YAML::Key << "TotalVoxels";
	MEmitter << YAML::Value << Data.size();
	MEmitter << YAML::Key << "NbFill";
	MEmitter << YAML::Value << NbFill;
	MEmitter << YAML::Key << "ColorTable";
	MEmitter << YAML::Value << ColorTable;
	if(!ClassesNames.empty())
	{
		MEmitter << YAML::Key << "ClassesNames";
		MEmitter << YAML::Value << ClassesNames;
	}
	MEmitter << YAML::EndMap;
	MEmitter << YAML::EndDoc;
	
	mcv::MixedFileManager_c FManager;
	FManager.Save(FileName,(char*)&Data[0],Data.size(),  UserHeader + MEmitter.c_str()  );

}
//----------------------------------------------------------------------------------
bool g3d::VoxelSpace_c::load(const std::string &FileName)
{
	YAML::Node ThrowAwayThisDoc;
	return load(FileName,ThrowAwayThisDoc);
}
//----------------------------------------------------------------------------------
bool g3d::VoxelSpace_c::load(const std::string &FileName,YAML::Node &user_doc)
{
	bool Res = true;
	
	mcv::MixedFileManager_c FManager;
	std::stringstream YamlHeader = FManager.Parse(FileName);
	if(YamlHeader.str().empty())
	{
		std::cout << "VoxelSpace_c::load() Couldn't load File: " << FileName << std::endl;
		return false;
	}
	YAML::Parser parser;
	parser.Load(YamlHeader);
	parser.GetNextDocument(user_doc);
	YAML::Node doc;
	parser.GetNextDocument(doc);
	
	doc["VoxelSize"] >> VoxelSize;
	int TotalVoxels,vXv,vYv,vZv;
	doc["Xv"] >> vXv;
	doc["Yv"] >> vYv;
	doc["Zv"] >> vZv;
	doc["TotalVoxels"] >> TotalVoxels;
	int vNbFill;
	doc["NbFill"] >> vNbFill;
	EigenBox3D vBox;
	doc["VoxBox"] >> vBox;

	//This is the load SetUp as the data is the master with its Xv,Yv,Zv
	//and the <float> is not guarantied through text save and load
	//whether use double if more precision is needed but won't correct the text conversion issue
	//or save the critical information on data, but it becomes no more editable by the user
	//the remaining issue is the float -> text -> float conversion !!! ??? is this safe ?
	Reset(vBox.Low,vXv,vYv,vZv,VoxelSize);//will get Xv = vXv ....

	doc["TypeName"] >> TypeName;
	doc["ColorTable"] >> ColorTable;

	if(doc.FindValue("ClassesNames"))
	{
		doc["ClassesNames"] >> ClassesNames;
	}

	//assert(Xv == vXv);
	//assert(Yv == vYv);
	//assert(Zv == vZv);
	assert(TotalVoxels == Xv*Yv*Zv);
	assert(Data.size() == TotalVoxels);
	assert(FManager.DataInfo.DataSizeBytes == TotalVoxels);//Check Coherence between myinfo and FManager Info

	Data.resize(TotalVoxels);
	FManager.LoadData((char*)&Data[0],TotalVoxels);

	NbFill = countNonZero();//So that the data is what we expect
	assert(NbFill == vNbFill);

	return Res;
}
//-------------------------------------------------------------------------------------------------------------
void g3d::VoxelSpace_c::Render(Soft3DCamera_c &Cam)//overriding the Renderable_c::Render
{
	assert(!ColorTable.empty());//only if cleared by user
	if(!isVisible)return;
	if(Data.empty()){return;}
	unsigned char *pData = &Data[0];
	if(RenderStep == 1)//won 0.2 ms from 2.47 to 2.25 unsignificant for a 1M empty Vox
	{
		for(int k=0;k<Zv;k++)
			for(int j=0;j<Yv;j++)
				for(int i=0;i<Xv;i++)
				{
					unsigned char Val = (*pData++);
					if(Val)
					{
						Vector4f pos = Pos(i,j,k);
						//Cam.DrawPoint3D(pos,cv::Scalar(0,0,Val));
					if(Val>1)
						Cam.DrawPoint3D(pos,ColorTable[Val]);//Vox(0-Empty, 1-FirstPart Uncknown...)
					else
						Cam.DrawPoint3D(pos,ColorTable[Val],false);//Vox(0-Empty, 1-FirstPart Uncknown...)
					}
				}
	}
	else
	{
		for(int k=0;k<Zv;k+=RenderStep)
		{
			for(int j=0;j<Yv;j+=RenderStep)
			{
				pData= &Data[0] + k*strideZ + j*strideY;
				for(int i=0;i<Xv;i+=RenderStep)
				{
					//unsigned char Val = (*this)(i,j,k);
					unsigned char Val = (*pData);
					if(Val)
					{
						Vector4f pos = Pos(i,j,k);
						Cam.DrawPoint3D(pos,ColorTable[Val]);//Shift index from Vox to ColoTabIndex
					}
					pData+=RenderStep;
				}
				//pData+=((RenderStep-1) * strideY);
			}
			//pData+=((RenderStep-1) * strideZ);
		}
	}
	Cam.DrawBox(Box);
}
//-------------------------------------------------------------------------------------------------------------
int g3d::VoxelSpace_c::countNonZero()
{
	assert(!Data.empty());
	unsigned char *pData = &Data[0];
	unsigned char *pDataEnd = &Data[Data.size()-1] + 1;
	int Sum = 0;
	while(pData < pDataEnd)
	{
		if(*pData++)
			Sum++;
	}
	return Sum;
}
//-------------------------------------------------------------------------------------------------------------
int g3d::VoxelSpace_c::countHigherThan(unsigned char Val)
{
	assert(!Data.empty());
	unsigned char *pData = &Data[0];
	unsigned char *pDataEnd = &Data[Data.size()-1] + 1;
	int Sum = 0;
	while(pData < pDataEnd)
	{
		if((*pData++)>Val)
		{
			Sum++;
		}
	}
	return Sum;
}
//-------------------------------------------------------------------------------------------------------------
bool g3d::VoxelSpace_c::SetAtClosestOne(Vector3f& Ref,unsigned char Val)
{
	float tolerance = 15;
	assert(!Data.empty());
	unsigned char *pData = &Data[0];
	float minDist = FLT_MAX;
	int ClosestIndex = 0;
	int NbTotal = Xv * Yv * Zv;
	//int CheckNbFill = 0;
	for(int k=0;k<Zv;k++)
		for(int j=0;j<Yv;j++)
			for(int i=0;i<Xv;i++)
			{
				int Index = k*strideZ + j*strideY + i;
				if(Data[Index])
				{
					Vector3f VPos = V4To3(Pos(i,j,k));
					float Norm = (VPos-Ref).norm();
					if(Norm < minDist)
					{
						minDist = Norm;
						ClosestIndex = Index;
					}
				}
			}
	//Yeah to be sure that the Vox is '1'=Free
	//and because we might kill another incemination
	//or there's no sense in inceminating a non Free Vox
	//assertion is too rough, some images are not complete, so maybe they shouldn't inceminate missing parts ??
	//added a tolerence for far inceminations, if it's too far, then maybe there's no voxes and we're in occlusions
	Vector3f InceminPos = V4To3(Pos(ClosestIndex));
	if((InceminPos-Ref).norm() < tolerance)
	{
		//What ??!! Two Close Centers, that are outside the Vox and having the same closest Vox Point???
		assert(Data[ClosestIndex] == 1);
		Data[ClosestIndex] = Val;
		return true;
	}
	else
	{
		printf("Incemination can't go that far(%1.2f cm) tolerance is (%1.2f)\n",(InceminPos-Ref).norm(),tolerance);
		return false;
	}
}
//-------------------------------------------------------------------------------------------------------------
int g3d::VoxelSpace_c::Propagate()
{
	//The propagation needs a source and dest Voxes to avoir overrunning with the current data being modified:
	//Neighbor of pData modified OK, then pData++ we are in the neighbor that should not modify again !!!
	//we should propagate one and only step in all directions for every color
	assert(!Data.empty());
	int NbChanged = 0;
	std::vector<unsigned char> DestData(Data.size());
	unsigned char *pDestData = &DestData[0];
	unsigned char *pData = &Data[0];
	memcpy(pDestData,pData,Data.size());//start with an identical copy to fill it more and not fill itself
	unsigned char *pDestDataStart = &DestData[0];
	unsigned char *pDataEnd = &Data[Data.size()-1] + 1;
	unsigned char *pDestDataEnd = &DestData[DestData.size()-1] + 1;
	std::vector<bool> Changed(50);//max 50 Parts - nasty use var if needed - initialised at false ?
	//for(size_t i=0;i<Changed.size();i++)	{		Changed[i] = false;	}
	while(pData < pDataEnd)
	{
		//int LastNbChanged = NbChanged;
		if((*pData)>1)//Full and Not Free (as 1 is Free), then it's a candidate for propagation
		{
			unsigned char *pTop = pDestData + strideY;//+Y
			unsigned char *pBot = pDestData - strideY;//-Y
			unsigned char *pLeft = pDestData - 1;//-X
			unsigned char *pRight = pDestData + 1;//+X
			unsigned char *pFront = pDestData - strideZ;//-Z
			unsigned char *pBack = pDestData + strideZ;//+Z
			if((pTop >= pDestDataStart) && (pTop<pDestDataEnd))//access OK
			{	
				if( ((*pTop)==1) && (!Changed[(*pData)]) )
				{
					(*pTop) = (*pData);
					NbChanged++;
				}	
			}
			
			if((pBot >= pDestDataStart) && (pBot<pDestDataEnd))			{
				if(((*pBot)==1) &&(!Changed[(*pData)]))	
				{	(*pBot) = (*pData);	NbChanged++;}			}
			if((pLeft >= pDestDataStart) && (pLeft<pDestDataEnd))		{	
				if(((*pLeft)==1) &&(!Changed[(*pData)]))	
				{	(*pLeft) = (*pData);	NbChanged++;}		}
			if((pRight >= pDestDataStart) && (pRight<pDestDataEnd))		{	
				if(((*pRight)==1) &&(!Changed[(*pData)]))		
				{	(*pRight) = (*pData);	NbChanged++;}		}
			if((pFront >= pDestDataStart) && (pFront<pDestDataEnd))		{	
				if(((*pFront)==1) &&(!Changed[(*pData)]))
				{	(*pFront) = (*pData);	NbChanged++;}		}
			if((pBack >= pDestDataStart) && (pBack<pDestDataEnd))		{	
				if(((*pBack)==1) &&(!Changed[(*pData)]))		{
					(*pBack) = (*pData);	NbChanged++;}		}

			//no more needed as we use src and dest Vox, this just blocks the processing of many voxes
			//and limit the movement in one direction
			//if(NbChanged > LastNbChanged)//Changed in this loop !!!!!! or else will be stuck by the first (*pData) that blocks all
			//{
				//Changed[(*pData)] = true;//necessarily changed or else can no more change
			//}
		}
		pData++;
		pDestData++;
	}
	//update the result of the step propagation (all direction from all color voxes) in the original Vox
	memcpy(&Data[0],&DestData[0],Data.size());//start with an identical copy to fill it more and not fill itself

	return NbChanged;
}
//-------------------------------------------------------------------------------------------------------------
int g3d::VoxelSpace_c::Replace(unsigned char oldVal,unsigned char NewVal)
{
	int NbReplaced = 0;
	assert(!Data.empty());
	unsigned char *pData = &Data[0];
	unsigned char *pDataEnd = &Data[Data.size()-1] + 1;
	while(pData < pDataEnd)
	{
		if((*pData) == oldVal)
		{
			(*pData) = NewVal;
			NbReplaced++;
		}
		pData++;
	}
	return NbReplaced;
}
//-------------------------------------------------------------------------------------------------------------
int g3d::VoxelSpace_c::MedianFilter(int MaxClasses)
{
	assert(!Data.empty());
	int NbChanged = 0;
	std::vector<unsigned char> DestData(Data.size());
	unsigned char *pDestData = &DestData[0];
	unsigned char *pData = &Data[0];
	memcpy(pDestData,pData,Data.size());//start with an identical copy to fill it more and not fill itself
	unsigned char *pDestDataStart = &DestData[0];
	unsigned char *pDataEnd = &Data[Data.size()-1] + 1;
	unsigned char *pDestDataEnd = &DestData[DestData.size()-1] + 1;
	std::vector<bool> Changed(50);//max 50 Parts - nasty use var if needed - initialised at false ?
	while(pData < pDataEnd)
	{
		if(*pData)
		{
			std::vector<unsigned char> Classes(MaxClasses);
			unsigned char* pTab[9];
			pTab[0] = pData - strideY - 1 - strideZ;//(0,0)
			pTab[1] = pData - strideY     - strideZ;
			pTab[2] = pData - strideY + 1 - strideZ;
			pTab[3] = pData           - 1 - strideZ;
			pTab[4] = pData               - strideZ;
			pTab[5] = pData           + 1 - strideZ;
			pTab[6] = pData + strideY - 1 - strideZ;
			pTab[7] = pData + strideY     - strideZ;
			pTab[8] = pData + strideY + 1 - strideZ;

			for(int i=0;i<9;i++)
			{	
				if((pTab[i] >= &Data[0]) && (pTab[i]<pDataEnd))			
					{Classes[*(pTab[i])]++;}
				pTab[i]+=strideZ;	
			}
			for(int i=0;i<9;i++)
			{	
				if((pTab[i] >= &Data[0]) && (pTab[i]<pDataEnd))			
					{Classes[*(pTab[i])]++;}
				pTab[i]+=strideZ;	
			}
			for(int i=0;i<9;i++)
			{	
				if((pTab[i] >= &Data[0]) && (pTab[i]<pDataEnd))			
					{Classes[*(pTab[i])]++;}
				pTab[i]+=strideZ;	
			}

			unsigned char MaxVotedClass = TabMax_Index(Classes);

			//we do not change the number of filled voxes
			if(MaxVotedClass==0)
			{
				Classes[0] = 0;//Neutralise it, so that we get the second
				MaxVotedClass = TabMax_Index(Classes);
			}
			if((*pData)!=MaxVotedClass)
			{
				(*pDestData) = MaxVotedClass;
				NbChanged++;
			}
		}
		pData++;
		pDestData++;
	}
	memcpy(&Data[0],&DestData[0],Data.size());//start with an identical copy to fill it more and not fill itself

	return NbChanged;
}
//-------------------------------------------------------------------------------------------------------------
void g3d::VoxelSpace_c::ToVectTab(mrg::VectorsTable_c &VectTab)
{
	//supposedly empty VectTab to rewrite on
	VectTab.resize(1,NbFill);
	
	unsigned char *pData = &Data[0];
	unsigned char *pDataEnd = (&Data[Data.size()-1]) + 1;
	float *pVectData = &VectTab.Data[0];
	int VCount = 0;
	while(pData < pDataEnd)
	{
		if(*pData)
		{
			(*pVectData) = (float)(*pData);
			pVectData+=VectTab.VectSize;
			VCount++;
		}
		pData++;
	}
	assert(VCount == NbFill);
}
//-------------------------------------------------------------------------------------------------------------
void g3d::VoxelSpace_c::FromVectTab(mrg::VectorsTable_c &VectTab)
{
	//The Vox has to be already initialized with 0,1 Fill Data,
	//This function only updates the parts Classification and repartition
	//currently only the classification Table format is supported
	assert(VectTab.VectSize == 1);//the Class Id
	assert(VectTab.size() == NbFill);
	unsigned char *pData = &Data[0];
	unsigned char *pDataEnd = (&Data[Data.size()-1]) + 1;
	float *pVectData = &VectTab.Data[0];
	int VCount = 0;
	while(pData < pDataEnd)
	{
		if(*pData)
		{
			(*pData) = (unsigned char)(*pVectData);
			pVectData+=VectTab.VectSize;
			VCount++;
		}
		pData++;
	}
	assert(VCount == NbFill);
}
//-------------------------------------------------------------------------------------------------------------
//IMGFileStream	VoxFileStream
void g3d::VoxelSpace_c::SetupGrabber(stringmap &Config)
{
	VoxFileStream.Init(Config["CfgFile"],Config["Node"],Config["Tag"]);
}
//-------------------------------------------------------------------------------------------------------------
void g3d::VoxelSpace_c::GrabFrameIndex(int Index)
{
	std::string FileName = VoxFileStream.GetName(Index);
	load(FileName);
}
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//--------------Voxelliser_c-----------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
int g3d::Voxelliser_c::Process2DToVox(std::vector<cv::Mat> BKGViews,VoxelSpace_c &Vox,float VoxSize)
{
	//printf("VoxSize: %f\n",VoxSize);
	int NbCams = (int)pS3D->Cams.size();
	assert(BKGViews.size() == NbCams);


#ifdef	DEBUG_PROCESSToVOX
	EigenBox3D Box = pS3D->BlobToBox(BKGViews);
	pS3D->ClearBuffers();
	pS3D->FillBuffers(BKGViews);
	pS3D->DrawBox(Box);
	pS3D->Display("BKGViews");
	//mcv::imNShow(BKGViews,"BKGViews");
	cv::waitKey();
#endif



	EigenBox3D Box = pS3D->BlobToBox(BKGViews);
	Vox.Reset(Box,VoxSize);//one voxel per cm - clear @ SetUp()
	int NbSum = Vox.Xv * Vox.Yv * Vox.Zv;
	//printf("NbVox: %d\n",NbSum);
	int NbFill = 0;
	for(int k=0;k<Vox.Zv;k++)
		for(int j=0;j<Vox.Yv;j++)
			for(int i=0;i<Vox.Xv;i++)
			{
				Vector4f Pos = Vox.Pos(i,j,k);
				bool isAlwaysOn = true;
				for(int c=0;((c<NbCams) && isAlwaysOn);c++)
				{
					cv::Point Pos2D = pS3D->Cams[c].Project(Pos);
					//Should we check it or do we trust BlobToBox() ?
					if(	(Pos2D.x < 0)				||	(Pos2D.y < 0)					||
						(Pos2D.x > BKGViews[c].cols)||	(Pos2D.y > BKGViews[c].rows))
					{
						isAlwaysOn = false;//Voxel is outside the image !!! What to do ? Ignore / no voxel added
					}
					else
					{
						isAlwaysOn = (PIXEL8UC1(BKGViews[c],Pos2D.x,Pos2D.y)!=0);
					}
				}
				if(isAlwaysOn)
				{
					Vox(i,j,k) = 1;//Very important, this is an index of the color Table
					//(*pData) = 1
					NbFill++;
				}
				//pData++;//how much do we win over Vox(i,j,k) it depends on Project Time
			}
	Vox.NbFill = NbFill;
	return Vox.NbFill;
}
//-------------------------------------------------------------------------------------------------------------
int g3d::Voxelliser_c::Process2DToVox(std::vector<cv::Mat> BKGViews,std::string &FileName,float VoxSize)
{
	bool isDTime = true;
	double t;
	VoxelSpace_c Vox;

	if(isDTime)TStart(t);
	int NbFull = Process2DToVox(BKGViews,Vox,VoxSize);//using pS3DEnv

	if(isDTime)TStop(t,"Process2DToVox(Vox)");
	std::string user_header = "";//Add more info - Might include the Cams ?
	Vox.TypeName = "Voxels";//"BKGViews2Vox";
	Vox.save(FileName,user_header);

#ifdef	DEBUG_PROCESSToVOX
	S3DFlyCamViewer_c Viewer(pS3D,"Voxellisation");
	//Vox.RenderStep = 2;
	Viewer.AddToRender(&Vox);
	TStart(t);
	Viewer.Render();
	TStop(t,"Viewer.Render()");
	Viewer.Display("Voxellisation");
	printf("TotalVoxels: %d\nNbFill: %d\n",Vox.Data.size(),Vox.NbFill);
	cv::waitKey();
#endif
	return NbFull;
}
//-------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------
int ProcessVox(S3DGrabber_c* pGrabber,VoxelSpace_c &Vox,float VoxSize)//lol Yeah the grabber isn't ready yet
{
	bool isThisFunctionReady = false;assert(isThisFunctionReady);
	return 0;
}
//-------------------------------------------------------------------------------------------------------------
