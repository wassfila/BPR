/*
 *  MultiCamStream.h
 *  
 *
 *  Created by WassFila Started on 2009-09-22
 *  No Copyright No rights reserved
 *
 */
#pragma once
#ifndef MULTICAMSTREAM_H_
#define MULTICAMSTREAM_H_

#include "cv.h"
#include "highgui.h"

class StreamSync_c
{
public:
	int StreamStartingImage;//starting image
	int MoCapStartingImage;
	double	scale;
	bool	isLoadedBeingused;
public:
	StreamSync_c(){isLoadedBeingused = false;}
	void load(const char*FileName);
	int MocapIndexToStreamIndex(int MoCapIndex);
	int StreamIndextoMocapIndex(int StreamIndex);
};

namespace mcv
{
	class ChanType
	{
	public:
		enum { NotSpecified=0, COLOR=1, BKG=2, BPR=3, Gray=4, Depth=5, Any=6, DescBPRTrain=7, DescBPRResp=8, BPRres=9  };
	};



}

class IMGFileStream {

private:
	//char 		pFileIndex[64];
	float 		FPS;//Frames per Seconds
	int 		NextFrameToGrab;//Retrive what you Grab
	cv::Mat		CurrentFrame;
public:
	int 		FirstImage;//default to 1
	int 		LastImage;//NbImages;
	char 		pFileName[512];
	char 		pPath[1024];
	//int			ChannelType;
	std::string	ChannelTypeName;
	StreamSync_c	SyncData;

	IMGFileStream();
	IMGFileStream(std::string CfgFile,const std::string &NodeName,const std::string &Tag);
	IMGFileStream(const char* fFileName, int fFirstImage, int fLastImage);
	~IMGFileStream();
	//This Init() is now optionnal to the Mapped Tag "NodeName" that also can be a Sequence Tag
	void Init(const std::string &CfgFile,const std::string &NodeName, const std::string &Tag,int SeqIndex = -1);
	void Init(const char* fPath, const char* fFileName, int fFirstImage, int fLastImage);
	void Init(const char* fFileName, int fFirstImage, int fLastImage);
	void ChangeFileName(const char* fFileName);
	void ChangeFilePathName(const char* fFileName,const char*fPathName);
	void GrabFrame();
	cv::Mat GetLastFrame();
	cv::Mat GetFrame();
	int SkipFrames(int NbFramesToskip) {NextFrameToGrab+=NbFramesToskip;return NextFrameToGrab;}
	int GetNextFrameIndex()	
		{
			if ( (NextFrameToGrab>(LastImage)) || (NextFrameToGrab<FirstImage) )
				NextFrameToGrab = FirstImage;
			return NextFrameToGrab;
		}
	void SetNextFrameIndex(int index){NextFrameToGrab = index;}

	cv::Mat GetFrameByIndex(int Index);//it doesn't check
	cv::Mat GetSyncFrameIndex(int Index);
	//cv::Mat GetFramesByIndexNocheck(int Index);

	void WriteFrame(const cv::Mat &Img);
	void WriteFrameByIndex(const cv::Mat &Img,int Index);
	std::string GetName(int Index);
	int FindLastImageIndex();
};


void mcvShredImgFile(const char *pImgDir,const char *pImgFile,const char*pImgFileExt);


class MultiCamStream
{
public:
	std::vector<IMGFileStream> ImgsStream;
	int FirstIndex;
	int LastIndex;
public:
	MultiCamStream();
	MultiCamStream(int NbStreams,const char* fPath, const char* fFileName, int fFirstImage, int fLastImage);
	MultiCamStream(std::string ConfigFileName,const char* NodeTag,const char* StreamTag);
	//"FirstStreamsIndex" "LastStreamsIndex"
	int SetStreams(std::string ConfigFileName,const char* NodeTag,const char* StreamTag);

	void SetStreamFileName(int StreamID,const char*fFileName);
	void SetStreamPathFileName(int StreamID,const char*fFileName,const char*fPathName);
	void SkipFrames(int NbFramesToskip);

	void GetFrames(std::vector<cv::Mat> &Imgs);
	void GetFramesByIndex(std::vector<cv::Mat> &Imgs,int Index);
	void WriteFramesByIndex(const std::vector<cv::Mat> Imgs,int Index);

	//--------------MoCap Synchronous functions for the nastiest thing Ever : HumaEva database
	int GetMocapStartAllStreamsValid();
	int GetMocapLastAllStreamsValid();

	void GetSyncFramesIndex(std::vector<cv::Mat> &Imgs,int MoCapIndex);
	//void GetFramesByIndexNocheck(std::vector<cv::Mat> &Imgs,int Index);


};






#endif /*MULTICAMSTREAM_H_*/
