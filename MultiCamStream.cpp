
#include "MultiCamStream.h"
#include <math.h>
#include <stdio.h>

#include <iostream>
#include <fstream>
#include "mcvGeneral.h"

using namespace mcv;
//_____________________________________________________________________________________________________
//****************************************  Internal Functions  ***************************************
//_____________________________________________________________________________________________________

//_____________________________________________________________________________________________________
//******************************************  IMGFileStream  ******************************************
//_____________________________________________________________________________________________________

IMGFileStream::IMGFileStream()
{
	Init("","",0,0);
}
//----------------------------------------------------------------------------------------------------
IMGFileStream::IMGFileStream(std::string CfgFile,const std::string &NodeName,const std::string &Tag)
{
	Init(CfgFile,NodeName,Tag);
}
//----------------------------------------------------------------------------------------------------
IMGFileStream::IMGFileStream(const char* fFileName, int fFirstImage, int fLastImage)
{
	Init(fFileName,fFirstImage,fLastImage);
}
/*
	For a File List Starting with "C:\samples\per00001.ppm"
	We use : Init("C:\samples\per",1,100,5,".ppm");
	"C:\Samples\per"	: FileNmae Start with
	1					: the First File number is
	100					: We'll use 100 files
	5					: "per00001.ppm" contains a number with 5 digits 00001
	".ppm"				: The file Extention is .ppm
*/
void IMGFileStream::Init(const char* fPath, const char* fFileName, int fFirstImage, int fLastImage)
{	
	strcpy(pPath,fPath);
	Init(fFileName,fFirstImage,fLastImage);
}
//----------------------------------------------------------------------------------------------------
void IMGFileStream::Init(const char* fFileName, int fFirstImage, int fLastImage)
{	
	strcpy(pFileName,fFileName);
	FirstImage = fFirstImage;
	FPS = 30;
	NextFrameToGrab = FirstImage;
	if(fLastImage == 0)
	{
		LastImage = FindLastImageIndex();
	}
	else
	{
		LastImage = fLastImage;
	}
	
	ChannelTypeName = "Any";
}
//----------------------------------------------------------------------------------------------------
void IMGFileStream::Init(const std::string &CfgFile,const std::string &NodeName, const std::string &Tag,int SeqIndex)
{
	cv::FileStorage Fs(CfgFile,cv::FileStorage::READ);
	std::string MainPath = CfgFile.substr( 0, CfgFile.find_last_of( '\\' ) +1 );
	int FirstIndex = (int)Fs["FirstStreamsIndex"];
	int LastIndex = (int)Fs["LastStreamsIndex"];
	cv::FileNode FNode = Fs[NodeName];
	
	std::string FileName;
	if(SeqIndex!=-1)
	{
		assert(!FNode[SeqIndex][Tag].empty());
		FileName = (std::string)FNode[SeqIndex][Tag];
	}
	else
	{
		assert(!FNode[Tag].empty());
		FileName = (std::string)FNode[Tag];
	}
	Init(MainPath.c_str(),FileName.c_str(),FirstIndex,LastIndex);
	ChannelTypeName = Tag;
}
//----------------------------------------------------------------------------------------------------
int IMGFileStream::FindLastImageIndex()
{
	double t;
	TStart(t);
	bool FileExist = true;
	LastImage = FirstImage - 1;//for the fist inc before checking
	do
	{
		LastImage++;
		char pImageFileName[512];
		char pImageFileToLoad[512];
		sprintf(pImageFileName,pFileName,LastImage);
		strcpy(pImageFileToLoad,pPath);
		strcat(pImageFileToLoad,pImageFileName);
		if(pImageFileToLoad)
		{
			std::ifstream FileTocheck(pImageFileToLoad);
			if(!FileTocheck.is_open())
			{
				FileExist = false;
			}
			FileTocheck.close();
		}
		else
		{
			FileExist = false;
		}
	}while(FileExist);
	//printf("LastImage %d : ",LastImage);TStop(t,"FindLastImageIndex()");
	
	return LastImage;
}
//----------------------------------------------------------------------------------------------------
void IMGFileStream::ChangeFileName(const char* fFileName)
{
	strcpy(pFileName,fFileName);
}
//----------------------------------------------------------------------------------------------------
void IMGFileStream::ChangeFilePathName(const char* fFileName,const char*fPathName)
{
	strcpy(pPath,fPathName);
	strcpy(pFileName,fFileName);
}
//----------------------------------------------------------------------------------------------------
IMGFileStream::~IMGFileStream() {
	//free(pFileName);
	//free(pFileIndex);
	//free(pDigitFormat);
	//free(pExtention);
	
}
//----------------------------------------------------------------------------------------------------
void IMGFileStream::GrabFrame()
{
	char pImageFileName[512];
	char pImageFileToLoad[512];

	//GetNextFrameIndex();//rolls the NextFrameToGrab with the limits
	sprintf(pImageFileName,pFileName,NextFrameToGrab++);
	strcpy(pImageFileToLoad,pPath);
	strcat(pImageFileToLoad,pImageFileName);
	CurrentFrame = cv::imread(pImageFileToLoad);
	if((ChannelTypeName.compare("BKG") == 0)&&(CurrentFrame.type() == CV_8UC3))
	{
		cv::cvtColor(CurrentFrame,CurrentFrame,cv::COLOR_BGR2GRAY);
	}
}
//----------------------------------------------------------------------------------------------------
cv::Mat IMGFileStream::GetLastFrame()
{
	return	CurrentFrame;
}
//----------------------------------------------------------------------------------------------------
cv::Mat IMGFileStream::GetFrame()
{
	GrabFrame();
	return	GetLastFrame();
}
//----------------------------------------------------------------------------------------------------
cv::Mat IMGFileStream::GetFrameByIndex(int Index)
{
	SetNextFrameIndex(Index);
	return GetFrame();
}
//----------------------------------------------------------------------------------------------------
/*cv::Mat IMGFileStream::GetFramesByIndexNocheck(int Index)
{	
	char pImageFileToLoad[512];
	sprintf(pImageFileToLoad,pFileName,Index);
	return cv::imread(pImageFileToLoad);
}*/
//----------------------------------------------------------------------------------------------------
cv::Mat IMGFileStream::GetSyncFrameIndex(int MoCapIndex)
{
	int ImageStreamIndex = SyncData.MocapIndexToStreamIndex(MoCapIndex);
	return GetFrameByIndex(ImageStreamIndex);
}
//----------------------------------------------------------------------------------------------------
std::string IMGFileStream::GetName(int Index)
{
	std::string FileWithNumber;
	char pImageFileWithFormat[512],pImageFileWithNumber[512];
	strcpy(pImageFileWithFormat,pPath);
	strcat(pImageFileWithFormat,pFileName);
	sprintf(pImageFileWithNumber,pImageFileWithFormat,Index);
	FileWithNumber = pImageFileWithNumber;
	return FileWithNumber;
}
//----------------------------------------------------------------------------------------------------
void IMGFileStream::WriteFrameByIndex(const cv::Mat &Img,int Index)
{
	char pImageFileWithFormat[512],pImageFileWithNumber[512];
	strcpy(pImageFileWithFormat,pPath);
	strcat(pImageFileWithFormat,pFileName);
	sprintf(pImageFileWithNumber,pImageFileWithFormat,Index);
	cv::imwrite(pImageFileWithNumber,Img);
}
//----------------------------------------------------------------------------------------------------
void IMGFileStream::WriteFrame(const cv::Mat &Img)
{
	WriteFrameByIndex(Img,NextFrameToGrab++);
}
//----------------------------------------------------------------------------------------------------
//					StreamSync_c
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
void StreamSync_c::load(const char*FileName)
{
	std::string line;
	std::ifstream myfile(FileName);
	if (myfile.is_open())
	{
		StreamStartingImage = (int)readFloatLine(myfile);
		MoCapStartingImage = (int)readFloatLine(myfile);
		scale = readFloatLine(myfile);
		printf("Stream to MoCap data loaded from file (%s)\n",FileName);
		myfile.close();
	}
	else
	{
		printf("File %s coudn't be found\n",FileName);
	}
	isLoadedBeingused = true;
}
//----------------------------------------------------------------------------------------------------
int StreamSync_c::MocapIndexToStreamIndex(int MoCapIndex)
{
	return (int)((double)StreamStartingImage + (((double)(MoCapIndex - MoCapStartingImage)) / scale));
}
//----------------------------------------------------------------------------------------------------
int StreamSync_c::StreamIndextoMocapIndex(int StreamIndex)
{
	return (int)((double)MoCapStartingImage + (((double)(StreamIndex - StreamStartingImage)) * scale));
}
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
//					MultiCamStream
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
MultiCamStream::MultiCamStream()
{
}
//----------------------------------------------------------------------------------------------------
MultiCamStream::MultiCamStream(std::string ConfigFileName,const char* NodeTag,const char* StreamTag)
{
	SetStreams(ConfigFileName,NodeTag,StreamTag);
}
//----------------------------------------------------------------------------------------------------
MultiCamStream::MultiCamStream(int NbStreams,const char* fPath, const char* fFileName, int fFirstImage, int fLastImage)
{
	ImgsStream.resize(NbStreams);
	for(int i=0;i<NbStreams;i++)
	{
		ImgsStream[i].Init(fPath,fFileName,fFirstImage,fLastImage);
	}
}
//----------------------------------------------------------------------------------------------------
//"Views" "FirstStreamsIndex" "LastStreamsIndex"
int MultiCamStream::SetStreams(std::string ConfigFileName,const char* NodeTag,const char* StreamTag)
{
	cv::FileStorage Fs(ConfigFileName,cv::FileStorage::READ);
	std::string MainPath = ConfigFileName.substr( 0, ConfigFileName.find_last_of( '\\' ) +1 );
	int FirstIndex = (int)Fs["FirstStreamsIndex"];
	int LastIndex = (int)Fs["LastStreamsIndex"];
	cv::FileNode FNode = Fs[NodeTag];
	int NbStreamsFound;
	if(FNode.type() == cv::FileNode::MAP)
	{
		ImgsStream.resize(1);
		NbStreamsFound = 0;
		if(!FNode[StreamTag].empty())
		{
			std::string FileName = (std::string)FNode[StreamTag];
			ImgsStream[NbStreamsFound].Init(MainPath.c_str(),FileName.c_str(),FirstIndex,LastIndex);
			ImgsStream[NbStreamsFound].ChannelTypeName = StreamTag;
			NbStreamsFound++;
		}
		else
		{
			printf("Tag (%s) not available in Node (%s) of Cfg File (%s)\n",StreamTag,NodeTag,ConfigFileName.c_str());
		}
	}
	else if(FNode.type() == cv::FileNode::SEQ)
	{
		int NbViews = (int)FNode.size();
		ImgsStream.resize(NbViews);
		NbStreamsFound = 0;
		for(int i=0;i<NbViews;i++)
		{
			if(!FNode[i][StreamTag].empty())
			{
				std::string FileName = (std::string)FNode[i][StreamTag];
				ImgsStream[NbStreamsFound].Init(MainPath.c_str(),FileName.c_str(),FirstIndex,LastIndex);
				ImgsStream[NbStreamsFound].ChannelTypeName = StreamTag;
				NbStreamsFound++;
			}
			else
			{
				printf("Tag (%s) not available in %s[%d] of Cfg File (%s)\n",StreamTag,NodeTag,i,ConfigFileName.c_str());
			}
		}
	}
	ImgsStream.resize(NbStreamsFound);

	return NbStreamsFound;
}
//----------------------------------------------------------------------------------------------------
void MultiCamStream::SetStreamFileName(int StreamID,const char*fFileName)
{
	ImgsStream[StreamID].ChangeFileName(fFileName);
}
//----------------------------------------------------------------------------------------------------
void MultiCamStream::SetStreamPathFileName(int StreamID,const char*fFileName,const char*fPathName)
{
	ImgsStream[StreamID].ChangeFilePathName(fFileName,fPathName);
}
//----------------------------------------------------------------------------------------------------
void MultiCamStream::SkipFrames(int NbFramesToskip)
{
	for(int i=0;i<ImgsStream.size();i++)
	{
		ImgsStream[i].SkipFrames(NbFramesToskip);
	}
}
//----------------------------------------------------------------------------------------------------
void MultiCamStream::GetFrames(std::vector<cv::Mat> &Imgs)
{
	for(int i=0;i<(int)ImgsStream.size();i++)
	{
		Imgs[i] = ImgsStream[i].GetFrame();
	}
}
//----------------------------------------------------------------------------------------------------
void MultiCamStream::GetFramesByIndex(std::vector<cv::Mat> &Imgs,int Index)
{
	for(int i=0;i<(int)ImgsStream.size();i++)
	{
		Imgs[i] = ImgsStream[i].GetFrameByIndex(Index);//doesn't check
	}
}
//----------------------------------------------------------------------------------------------------
void MultiCamStream::WriteFramesByIndex(const std::vector<cv::Mat> Imgs,int Index)
{
	for(int i=0;i<(int)ImgsStream.size();i++)
	{
		ImgsStream[i].WriteFrameByIndex(Imgs[i],Index);
	}
}

//----------------------------------------------------------------------------------------------------
int MultiCamStream::GetMocapStartAllStreamsValid()
{
	int LatestMocapStart = 0;
	for(int i=0;i<(int)ImgsStream.size();i++)
	{
		int iMoCapStart = ImgsStream[i].SyncData.MoCapStartingImage;
		if(iMoCapStart > LatestMocapStart)LatestMocapStart = iMoCapStart;
	}
	return LatestMocapStart;
}
//----------------------------------------------------------------------------------------------------
int MultiCamStream::GetMocapLastAllStreamsValid()
{
	int EarliestMocapEnd = INT_MAX;
	for(int i=0;i<(int)ImgsStream.size();i++)
	{
		int MocaPLastImage = ImgsStream[i].SyncData.StreamIndextoMocapIndex(ImgsStream[i].LastImage);
		if(MocaPLastImage < EarliestMocapEnd)EarliestMocapEnd = MocaPLastImage;
	}
	return EarliestMocapEnd;
}
//----------------------------------------------------------------------------------------------------
void MultiCamStream::GetSyncFramesIndex(std::vector<cv::Mat> &Imgs,int MoCapIndex)
{
	for(int i=0;i<(int)ImgsStream.size();i++)
	{
		Imgs[i] = ImgsStream[i].GetSyncFrameIndex(MoCapIndex);
	}
}
//----------------------------------------------------------------------------------------------------
/*void MultiCamStream::GetFramesByIndexNocheck(std::vector<cv::Mat> &Imgs,int Index)
{
	for(int i=0;i<(int)ImgsStream.size();i++)
	{
		Imgs[i] = ImgsStream[i].GetFramesByIndexNocheck(Index);
	}
}*/
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
void mcvShredImgFile(const char *pImgDir,const char *pImgFile,const char*pImgFileExt)
{	cv::Ptr<IplImage> BigImg,SmallImg;
	//cv::Mat BigImg,SmallImg;
	char	*pImageFileToLoad,*pImageFileToSave;
	int i,j,k=1;

	pImageFileToLoad = (char*) malloc(600);
	pImageFileToSave = (char*) malloc(600);

	strcpy(pImageFileToLoad, pImgDir);
	strcat(pImageFileToLoad, pImgFile);
	strcat(pImageFileToLoad, pImgFileExt);
	BigImg = cvLoadImage(pImageFileToLoad);
	//BigImg = cv::imread(pImageFileToLoad);
	SmallImg = cvCreateImage(cvSize(64,128),8,3);
	//SmallImg.create(128,64,BigImg.type());
	i=0;j=0;
	CvSize BigImgSize = cvGetSize(BigImg);
	for(i=0;i<BigImgSize.width-64;i+=64)
	for(j=0;j<BigImgSize.height-128;j+=128)
	{
		cvSetImageROI(BigImg,cvRect(i,j,64,128));
		//BigImg.adjustROI(j,j+128,i,i+64);
		cvCopy(BigImg,SmallImg);
		//BigImg.copyTo(SmallImg);
		sprintf(pImageFileToSave,"%s%s%03d%s",pImgDir,pImgFile,k++,pImgFileExt);
		cvSaveImage(pImageFileToSave,SmallImg);
		//cv::imwrite(pImageFileToSave,SmallImg);
	}

	free(pImageFileToLoad);
	free(pImageFileToSave);
}

