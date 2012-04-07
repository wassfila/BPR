
#include "HPRcmd.h"

using namespace std;
//-------------------------------------------------------------
void PrintArgHelp()
{
	cout << "The arguments that can be used are :"			<< endl;
	cout													<< endl;

	cout << "    ----- Viewing -----"						<< endl;
	cout << "multiview_i    : MultiViewer_Chan_Index"			<< endl;
	cout << "multiviewJVC_i : MultiViewer_Chan_Grab"			<< endl;
	cout << "flyview_i      : FlyViewer_JointsVoxChan_Index"	<< endl;
	cout << "multiview      : MultiViewer_Chan_Grab"			<< endl;

	cout << "    ----- Channels Processing -----"				<< endl;
	cout << "voxels         : Voxellize"						<< endl;
	cout << "resp3D         : GenerateTrainResp3D(1,0)"			<< endl;
	cout << "train3D        : GenerateTrainResp3D(0,1)"			<< endl;
	cout												<< endl;
	cout << "    ----- Machine Learning -----"			<< endl;

	cout << "Example:"									<< endl;
	cout << "appname voxels"							<< endl;
	cout << "Further help will be given for every entry"<< endl;
}
//-------------------------------------------------------------
void Cmd_MultiViewer_Chan_Index(int argc, char** argv)
{//multiview_i
	if(argc == 2)//help
	{
		cout << "CfgFile ViewChannel Start Last" << endl;
		cout << "Example:" << endl;
		cout << "appname multiview_i c:\\Data\\File.env Depth 0 99" << endl;
	}
	else
	{
		stringmap Config;
		Config["CfgFile"] = argv[2];//"c:\Data\File.env"
		Config["Node"] = "Views";
		Config["Channels"] = argv[3];//"Depth";
		int Start = atoi(argv[4]);
		int Last = atoi(argv[5]);
		MultiViewer_Chan_Index(Config,Start,Last);
	}
}
//-------------------------------------------------------------
void Cmd_MultiViewer_JointsVoxChan_Index(int argc, char** argv)
{//multiviewJVC_i
	if(argc == 2)//help
	{
		cout << "CfgFile Node Start Last Joints Voxels Channel" << endl;
		cout << "Note: all arguments are needed, replace optional ones by '-'" << endl;
		cout << "Examples:" << endl;
		cout << "appname multiviewJVC_i c:\\Data\\File.env 0 99 Joints - -" << endl;
		cout << "appname multiviewJVC_i c:\\Data\\File.env 0 99 - VoxelsParts -" << endl;
	}
	else
	{
		stringmap Config;
		Config["CfgFile"] = argv[2];//"c:\Data\File.env"
		Config["Node"] = "Poses";
		int Start = atoi(argv[3]);
		int Last = atoi(argv[4]);
		if(strcmp(argv[5],"-") != 0)
		{
			Config["ViewerJoints"] = argv[5];//"Joints";
		}
		if(strcmp(argv[6],"-") != 0)
		{
			Config["ViewerVox"] = argv[6];//"VoxelsParts";
		}
		if(strcmp(argv[7],"-") != 0)
		{
			Config["ViewerChannels"] = argv[7];//"VoxelsParts";
		}
		MultiViewer_JointsVoxChan_Index(Config,Start,Last);
	}
}
//-------------------------------------------------------------
void Cmd_FlyViewer_JointsVoxChan_Index(int argc, char** argv)
{
//flyview_i "CfgFile" "Node"
//optional Flags : "ViewerJoints" "ViewerJoints2" "ViewerVox"
	if(argc == 2)//help
	{
		cout << "CfgFile Start Last Joints Joints2 Voxels" << endl;
		cout << "Note: all arguments are needed, replace optional ones by '-'" << endl;
		cout << "Examples:" << endl;
		cout << "appname flyview_i c:\\Data\\File.env 0 99 - - Voxels" << endl;
		cout << "appname flyview_i c:\\Data\\File.env 0 99 Joints JointsResp -" << endl;
	}
	else
	{
		stringmap Config;
		Config["CfgFile"] = argv[2];//"c:\Data\File.env"
		Config["Node"] = "Poses";
		int Start = atoi(argv[3]);
		int Last = atoi(argv[4]);
		if(strcmp(argv[5],"-") != 0)
		{
			Config["ViewerJoints"] = argv[5];//"Joints";
		}
		if(strcmp(argv[6],"-") != 0)
		{
			Config["ViewerJoints2"] = argv[6];//"JointsRes";
		}
		if(strcmp(argv[7],"-") != 0)
		{
			Config["ViewerVox"] = argv[7];//"VoxelsParts";
		}
		FlyViewer_JointsVoxChan_Index(Config,Start,Last);
	}
}
//-------------------------------------------------------------
void Cmd_MultiViewer_Chan_Grab(int argc, char** argv)
{//multiview
	if(argc == 2)//help
	{
		cout << "not sure ~ CfgFile AnimTag GrabTag" << endl;
		cout << "Example:" << endl;
		cout << "appname multiview c:\\Data\\File.env Joints VoxelsParts" << endl;
	}
	else
	{
		stringmap Config;
		Config["CfgFile"] = argv[2];//"c:\Data\File.env"
		Config["Channels"] = argv[3];//"Depth";
		MultiViewer_Chan_Grab(Config);
	}
}
//-------------------------------------------------------------
void Cmd_Voxellize(int argc, char** argv)
{//voxels
	if(argc == 2)//help
	{
		cout << "Arguments list for voxels :" << endl;
		cout << "CfgFile floatVoxelSize Start Last" << endl;
		cout << "Example:" << endl;
		cout << "appname voxels c:\\Data\\File.env 1.0 0 99" << endl;
	}
	else
	{
		std::string Config;
		std::string CfgFile = argv[2];//"c:\Data\File.env"
		float VoxelSize = atof(argv[3]);
		int Start = atoi(argv[4]);
		int Last = atoi(argv[5]);
		Voxellize(CfgFile,VoxelSize,Start,Last);
	}
}
//-------------------------------------------------------------
void Cmd_GenerateVoxelPartsResp3D(int argc, char** argv)
{//resp3D
	if(argc == 2)//help
	{
		cout << "Arguments list for resp3D :" << endl;
		cout << "CfgFile Start Last" << endl;
		cout << "Example:" << endl;
		cout << "appname resp3D c:\\Data\\File.env 0 99" << endl;
	}
	else
	{
		std::string Config;
		std::string CfgFile = argv[2];//"c:\Data\File.env"
		int Start = atoi(argv[3]);
		int Last = atoi(argv[4]);
		GenerateTrainResp3D(CfgFile,Start,Last,false,true);
	}
}
//-------------------------------------------------------------
void Cmd_GenerateTrain3D(int argc, char** argv)
{//train3D
	if(argc == 2)//help
	{
		cout << "Arguments list for train3D :" << endl;
		cout << "CfgFile Start Last" << endl;
		cout << "Example:" << endl;
		cout << "appname train3D c:\\Data\\File.env 0 99" << endl;
	}
	else
	{
		std::string Config;
		std::string CfgFile = argv[2];//"c:\Data\File.env"
		int Start = atoi(argv[3]);
		int Last = atoi(argv[4]);
		GenerateTrainResp3D(CfgFile,Start,Last,true,false);
	}
}
//-------------------------------------------------------------
int HPR_HandleArguments( int argc, char** argv )
{

	if(argc == 1)
	{
		PrintArgHelp();
	}
	else
	{
		std::string Command = argv[1];
		//---------------------------------------------------------------------------------------------
		if(Command.compare("multiview_i")==0)
		{
			Cmd_MultiViewer_Chan_Index(argc,argv);
		}
		//---------------------------------------------------------------------------------------------
		else if(Command.compare("multiviewJVC_i")==0)
		{
			Cmd_MultiViewer_JointsVoxChan_Index(argc,argv);
		}
		//---------------------------------------------------------------------------------------------
		else if(Command.compare("flyview_i")==0)
		{
			Cmd_FlyViewer_JointsVoxChan_Index(argc,argv);
		}
		//---------------------------------------------------------------------------------------------
		else if(Command.compare("multiview")==0)
		{
			Cmd_MultiViewer_Chan_Grab(argc,argv);
		}
		//---------------------------------------------------------------------------------------------
		else if(Command.compare("voxels")==0)
		{
			Cmd_Voxellize(argc,argv);
		}
		//---------------------------------------------------------------------------------------------
		else if(Command.compare("resp3D")==0)
		{
			Cmd_GenerateVoxelPartsResp3D(argc,argv);
		}
		//---------------------------------------------------------------------------------------------
		else if(Command.compare("train3D")==0)
		{
			Cmd_GenerateTrain3D(argc,argv);
		}
	}
	return 0;
}
