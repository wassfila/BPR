/*
	NetStream.cpp see NetStream.h


*/


#include "NetStream.h"


using namespace sck;
using namespace std;
//--------------------------------------------------------------------------------------------------------------
void StrFixedSize(std::string &Str,int MaxLength)
{
	int ln = Str.size();
	Str.resize(MaxLength);
	if(ln < MaxLength)
	{
		Str.at(ln) = '\0';
	}
	else if(ln > MaxLength)
	{
		printf("Warning : ChannelId Max Length is : %d\n",MaxLength);
	}
}
//--------------------------------------------------------------------------------------------------------------
std::string TakeParseTo(std::string &Str,char Separator)
{
	size_t FirstofSep = Str.find_first_of(Separator);
	std::string Parsed = Str.substr(0 , FirstofSep);
	Str = Str.substr(FirstofSep+1 ,Str.length());
	return Parsed;
}
//--------------------------------------------------------------------------------------------------------------
void SocketCheck(SOCKET TheSocket,int SocketErrorVal,const char *Msg)
{
	if (TheSocket == SocketErrorVal)
	{
		printf("%s: %d\n",Msg, WSAGetLastError());
		closesocket(TheSocket);
		WSACleanup();
	}
}
//--------------------------------------------------------------------------------------------------------------
std::string ChannelSize(std::string RequestChannelId,int BlobSize)
{//ChannelId<15>:size:
	std::string Command = RequestChannelId;
	StrFixedSize(Command,CHANNEL_ID_MAX_LENGTH);
	Command = Command + ":";
	Command.resize(CHANNEL_ID_MAX_LENGTH+5);
	memcpy(&Command[CHANNEL_ID_MAX_LENGTH+1],&BlobSize,4);
	Command.push_back(':');
	return Command;
}

//--------------------------------------------------------------------------------------------------------------
//										Client
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
Client::Client()
{
	ConnectSocket = INVALID_SOCKET;
	NbEmptyAnswers = 0;
}
//--------------------------------------------------------------------------------------------------------------
Client::Client(stringmap &config)
{
	Init(config);
}
//--------------------------------------------------------------------------------------------------------------
//"IPADD" "PORT" "ChannelId"
int Client::Init(stringmap &config)
{
	bool isDebug = true;
	if(config.find("ChannelId")!=config.end())
	{
		ChannelId = config["ChannelId"];
		StrFixedSize(ChannelId,CHANNEL_ID_MAX_LENGTH);
	}

	WSADATA wsaData;
	int iResult;
	//--------------------------------------------------------------------------------------- Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
	if (iResult != 0) {
		printf("WSAStartup failed: %d\n", iResult);
		return 0;//My Error is 0 - No Handle
	}
	if(isDebug)printf("WSAStartup OK\n");
	struct addrinfo *result = NULL,
					*ptr = NULL,
					hints;

	ZeroMemory( &hints, sizeof(hints) );
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;//----------------- !! STREAM !!
	hints.ai_protocol = IPPROTO_TCP;//----------------- !! TCP !!

	PCSTR IPADD = config["IPADD"].c_str();//----------------- !! IP !!
	PCSTR PORT = config["PORT"].c_str();//----------------- !! PORT !!

	iResult = getaddrinfo(IPADD, PORT, &hints, &result);
	if (iResult != 0) {
		printf("getaddrinfo failed: %d\n", iResult);
		WSACleanup();
		return 0;//My Error is 0 - No Handle
	}
	// -------------------------------------------------------------------------------------- Socket Creation
	ConnectSocket = INVALID_SOCKET;
	// Attempt to connect to the first address returned by
	// the call to getaddrinfo
	ptr=result;

	// Create a SOCKET for connecting to server
	ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype, 
		ptr->ai_protocol);
	if (ConnectSocket == INVALID_SOCKET) {
		printf("Error at socket(): %ld\n", WSAGetLastError());
		freeaddrinfo(result);
		WSACleanup();
		return 0;//My Error is 0 - No Handle
	}
	// -------------------------------------------------------------------------------------- Connecting to Server
	iResult = connect( ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);

	if (iResult == SOCKET_ERROR) {
		closesocket(ConnectSocket);
		ConnectSocket = INVALID_SOCKET;
	}

	// Should really try the next address returned by getaddrinfo
	// if the connect call failed
	// But for this simple example we just free the resources
	// returned by getaddrinfo and print an error message

	freeaddrinfo(result);

	if (ConnectSocket == INVALID_SOCKET) {
		printf("Unable to connect to server!\n");
		WSACleanup();
		return 0;//My Error is 0 - No Handle
	}

	//On Success	ConnectSocket is usable
	return 1;
}
//--------------------------------------------------------------------------------------------------------------
Client::~Client()
{
	if(ConnectSocket == INVALID_SOCKET)
	{
		return;
	}
	//---------------------------------------------------------------------------------------- Close UP
	// shutdown the connection for sending since no more data will be sent
	// the client can still use the ConnectSocket for receiving data
	int iResult = shutdown(ConnectSocket, SD_SEND);
	if (iResult == SOCKET_ERROR) {
		printf("shutdown failed: %d\n", WSAGetLastError());
		closesocket(ConnectSocket);
		WSACleanup();
		return;
	}

	// cleanup
	closesocket(ConnectSocket);
	WSACleanup();
}
//--------------------------------------------------------------------------------------------------------------
void Client::Poll(std::string &Data_Poll,std::string vChannelId)
{
//int MySocket_Download(SOCKET ConnectSocket,char* Host_Dest,char* Target_Source,int Size)
int iResult;
	// ----------------------------------------------------------------------	Request = Poll:ChannelId;
	std::string Request = "Poll";
	if(!vChannelId.empty())
	{
		StrFixedSize(vChannelId,CHANNEL_ID_MAX_LENGTH);
		Request += ":" + vChannelId + ";";
	}
	else
	{
		Request += ":" + ChannelId + ";";
	}
	iResult = send(ConnectSocket, Request.c_str(), (int)Request.length(), 0);//------------------------	Sending Request
	if (iResult == SOCKET_ERROR)
	{
		printf("send failed: %d\n", WSAGetLastError());
		closesocket(ConnectSocket);
		WSACleanup();
	}
	//If Everything is Right we wait for Recieve
	//printf("CMD Sent...wait recv...\n");
	std::string Response;
	Response = ChannelId + ":1234:";//size == CHANNEL_ID_MAX_LENGTH + 5;
	iResult = recv(ConnectSocket, &Response[0], (int)Response.length(), MSG_WAITALL	);//"ChannelId:BlobSize:"
		
	if(Response.compare(0,ChannelId.length(),ChannelId)!=0)
	{
		cout << "Error, this is not the channel I've asked for !!! : " << Response << "in Stand of : " << ChannelId << endl;
	}
	TakeParseTo(Response,':');//remove ChannelId, left "BlobSize:"
	int PollDataSize;
	memcpy(&PollDataSize,&Response[0],4);
	if(PollDataSize > 0)
	{
		Data_Poll.resize(PollDataSize);
		iResult = recv(ConnectSocket, &Data_Poll[0], (int)Data_Poll.size(), MSG_WAITALL	);//"BlobData"
		if (iResult == SOCKET_ERROR)
		{
			printf("recv failed: %d\n", WSAGetLastError());
			closesocket(ConnectSocket);
			WSACleanup();
		}
	}
	else
	{
		Data_Poll.clear();
		NbEmptyAnswers++;
		printf("\rEmpty Blob Answers for channel (%s) : %d\n",ChannelId.c_str(),NbEmptyAnswers);
		//Data_Poll will be of size 0
	}
}
//----------------------------------------------------------------
void Client::Push(const std::string &Data_Push,std::string vChannelId)
{
int iResult;
	// ----------------------------------------------------------------------	Request = Push:ChannelId:BlobSize:BlobData
	std::string Request = "Push";
	if(!vChannelId.empty())
	{
		StrFixedSize(vChannelId,CHANNEL_ID_MAX_LENGTH);
		Request += ":" + vChannelId + ":";
	}
	else
	{
		Request += ":" + ChannelId + ":";
	}

	size_t TPStart = Request.size();
	Request.resize(TPStart + 4);//32 bits size
	int DataPushSize = (int) Data_Push.size();
	memcpy(&Request[TPStart],&DataPushSize,4);//Push:ChannelId:BlobSize
	Request.push_back(':');

	TPStart = Request.size();
	Request.resize(TPStart + Data_Push.size() );
	memcpy(&Request[TPStart],&Data_Push[0],Data_Push.size());//Push:ChannelId:BlobSize:BlobData

	iResult = send(ConnectSocket, Request.c_str(), (int)Request.length(), 0);//------------------------	Sending Request
	if (iResult == SOCKET_ERROR)
	{
		printf("send failed: %d\n", WSAGetLastError());
		closesocket(ConnectSocket);
		WSACleanup();
	}
}
//--------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//										Server
//--------------------------------------------------------------------------------------------------------------
fd_set *BufferServer::GetListenSet()
{
	FD_ZERO(&Listen_FdSet);
	FD_SET(ListenSocket, &Listen_FdSet);
	return &Listen_FdSet;
}
//--------------------------------------------------------------------------------------------------------------
fd_set *BufferServer::GetClientsSet()
{
	FD_ZERO(&Read_FdSet);
	std::list<SOCKET>::iterator it;
	for(it=Clients.begin();it!=Clients.end();it++)
	{
		FD_SET((*it), &Read_FdSet);
	}
	return &Read_FdSet;
}
//--------------------------------------------------------------------------------------------------------------
BufferServer::BufferServer()
{
}
//--------------------------------------------------------------------------------------------------------------
BufferServer::BufferServer(stringmap &config)
{
	Init(config);
}
//--------------------------------------------------------------------------------------------------------------
void BufferServer::RequestPoll(SOCKET ClientSocket,std::string &recvBuff)
{
	TakeParseTo(recvBuff,':');
	std::string RequestChannelId = TakeParseTo(recvBuff,';');//15
	RequestChannelId.resize(strlen(RequestChannelId.c_str()));//cuts down on \0
	stringlistmap::iterator it;
	it = Buffers.find(RequestChannelId);
	if(it!=Buffers.end())
	{
		if(!it->second.empty())
		{
			stringlist::reference BlobData = it->second.front();
			size_t BlobSize = BlobData.length();
			std::string Command = ChannelSize(RequestChannelId,BlobSize);
			//cout << "Sending Channel (" << RequestChannelId << ") with data (" << BlobData << ")" << endl;
			int iSendResult = send(ClientSocket, &Command[0], (int)Command.size(), 0);//<<send>> "ChannelId:BlobSize:" -----------------------
			if (iSendResult == SOCKET_ERROR)
			{
				printf("send failed: %d\n", WSAGetLastError());
				closesocket(ClientSocket);
				WSACleanup();
			}
			iSendResult = send(ClientSocket, &(BlobData[0]), (int)BlobData.length(), 0);//<<send>> "BlobData" -----------------------
			if (iSendResult == SOCKET_ERROR)
			{
				printf("send failed: %d\n", WSAGetLastError());
				closesocket(ClientSocket);
				WSACleanup();
			}
			it->second.pop_front();//----------------------------------->>>>>>>>>>>>>>>
			cout << "\r" << "- " << Buffers[RequestChannelId].size() << " ";

		}
		else
		{
			cout << "\r" << "* " << Buffers[RequestChannelId].size() << " ";
			//Send back an empty blob
			std::string Command = ChannelSize(RequestChannelId,0);
			int iSendResult = send(ClientSocket, &Command[0], (int)Command.size(), 0);//<<send>> "ChannelId:BlobSize:" -----------------------
		}
	}
	else
	{
		cout << "We don't have Channel '" << RequestChannelId << "' here" << endl;
		//Send back an empty blob
		std::string Command = ChannelSize(RequestChannelId,0);
		int iSendResult = send(ClientSocket, &Command[0], (int)Command.size(), 0);//<<send>> "ChannelId:BlobSize:" -----------------------
	}
}
//--------------------------------------------------------------------------------------------------------------
int BufferServer::RequestPush(SOCKET ClientSocket,std::string &recvBuff,int iResult)
{
	TakeParseTo(recvBuff,':');//remove Push
	std::string RequestChannelId = TakeParseTo(recvBuff,':');//15
	RequestChannelId.resize(strlen(RequestChannelId.c_str()));//cuts down on \0
	
	//since it's a Push, we need "size:"
	recvBuff.resize(5);
	iResult = recv(ClientSocket, &recvBuff[0], (int)recvBuff.size(), MSG_WAITALL);//<<recv>> -----------------------
	int BlobSize;
	memcpy(&BlobSize,&recvBuff[0],4);
	std::string BlobData;

	if(BlobSize > 0)
	{
		BlobData.resize(BlobSize);
		iResult = recv(ClientSocket, &BlobData[0], BlobSize, MSG_WAITALL);//<<recv>>-----------------------
		if (iResult == SOCKET_ERROR)
		{
			printf("send failed: %d\n", WSAGetLastError());
			closesocket(ClientSocket);
			WSACleanup();
		}
		if(Buffers[RequestChannelId].size() > MaxBlobsPerChannel)//Then Drop oldest
		{
			Buffers[RequestChannelId].pop_front();//discard
			printf(".");
		}
		Buffers[RequestChannelId].push_back(BlobData);////-----------------------------------<<<<<<<<<<<<<<<
		cout << "\r" << "+ " << Buffers[RequestChannelId].size() << " ";
	}
	else
	{
		cout << "Empty Blob from Channel (" << RequestChannelId << " );" << endl;
	}
	return iResult;
}
//--------------------------------------------------------------------------------------------------------------
int BufferServer::HandleClient(SOCKET ClientSocket)
{
	bool isDebug = true;
	std::string recvBuff;
	recvBuff.resize(21);//"Cmdr:ChannelIdSize15;" = 4 + 1 + 15 + 1 = 21
	//"Poll:ChannelIdSize15;"
	//"Push:ChannelId:BlobSize:BlobData"
	int iResult = recv(ClientSocket, &recvBuff[0], (int)recvBuff.size(), MSG_WAITALL);//<<recv>> -----------------------
	recvBuff.resize(iResult);//max 50, and we hope that the "Poll:ChannelId;" are inside else the request is broken
	if (iResult > 0)//Process the request
	{
		if(recvBuff.compare(0,5,"Poll:") == 0)
		{
			//printf("Poll:\n");
			RequestPoll(ClientSocket,recvBuff);
		}
		else if(recvBuff.compare(0,5,"Push:") == 0)//The received request = Push:ChannelId:BlobSize:BlobData
		{
			//printf("Push:\n");
			RequestPush(ClientSocket,recvBuff,iResult);
		}
		else
		{
			printf("Unckown Command. Use Push: or Poll:\n");
		}
	}
	else if (iResult == 0)
	{
		if(isDebug)printf("Connection closing...\n");
		// shutdown the send half of the connection since no more data will be sent
		iResult = shutdown(ClientSocket, SD_SEND);
		SocketCheck(ClientSocket,SOCKET_ERROR,"shutdown failed");
		if(isDebug)printf("Shutdown Send OK\n");
		// cleanup
		closesocket(ClientSocket);
	}
	else
	{
		printf("recv failed: %d\n", WSAGetLastError());
		closesocket(ClientSocket);
		WSACleanup();
	}
	return iResult;
}
//--------------------------------------------------------------------------------------------------------------
//"PORT"
// Initialize Winsock
// Server Address
// Create a Socket
// Setup listening
// listening
void BufferServer::Init(stringmap &config)
{
	MaxBlobsPerChannel = 10;
	MaxSizePerChannel = 10 * 1024 * 1024;//not beeing used yet
	MaxSizeTotal = 10 * 10 * 1024 * 1024;//not beeing used yet

	bool isDebug = true;
	int iResult;
	// ----------------------------------------------------------------------------------Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
	if (iResult != 0) {
		printf("WSAStartup failed: %d\n", iResult);
	}
	if(isDebug)printf("Initialize Winsock OK\n");

	// ---------------------------------------------------------------------------------- Server Address
	struct addrinfo *result = NULL, *ptr = NULL, hints;

	ZeroMemory(&hints, sizeof (hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;//----------------- !! STREAM !!
	hints.ai_protocol = IPPROTO_TCP;//----------------- !! TCP !!
	hints.ai_flags = AI_PASSIVE;

	PCSTR PORT = config["PORT"].c_str();//----------------- !! PORT !!

	// Resolve the local address and port to be used by the server
	iResult = getaddrinfo(NULL, PORT, &hints, &result);
	if (iResult != 0) {
		printf("getaddrinfo failed: %d\n", iResult);
		WSACleanup();
	}
	if(isDebug)printf("Server Address OK\n");

	ListenSocket = INVALID_SOCKET;

	// ----------------------------------------------------------------------------------Create a Socket
	// Create a SOCKET for the server to listen for client connections
	ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
	if (ListenSocket == INVALID_SOCKET) {
		printf("Error at socket(): %ld\n", WSAGetLastError());
		freeaddrinfo(result);
		WSACleanup();
	}
	if(isDebug)printf("Create a Socket OK\n");

	// ----------------------------------------------------------------------------------Setup listening
	// Setup the TCP listening socket
	iResult = bind( ListenSocket, result->ai_addr, (int)result->ai_addrlen);//------------ bind
	if (iResult == SOCKET_ERROR) {
		printf("bind failed: %d\n", WSAGetLastError());
		freeaddrinfo(result);
		closesocket(ListenSocket);
		WSACleanup();
	}
	freeaddrinfo(result);
	if(isDebug)printf("Setup listening OK\n");

	// ---------------------------------------------------------------------------------- listening
	if ( listen( ListenSocket, SOMAXCONN ) == SOCKET_ERROR )
	{
		printf( "Error at bind(): %ld\n", WSAGetLastError() );
		closesocket(ListenSocket);
		WSACleanup();
	}
	if(isDebug)printf("listening OK\n");
}
//--------------------------------------------------------------------------------------------------------------
// Accepting Connections 
// Send Recieve
void BufferServer::RunOneClient()
{
	bool isDebug = true;
	// ---------------------------------------------------------------------------------- Accepting Connections 
	SOCKET ClientSocket;
	ClientSocket = INVALID_SOCKET;
	// Accept a client socket
	ClientSocket = accept(ListenSocket, NULL, NULL);
	SocketCheck(	ListenSocket,INVALID_SOCKET,"accept failed");

	if(isDebug)printf("accept OK\n");
	// ---------------------------------------------------------------------------------- Send Recieve
	int iResult;
	// Receive until the peer shuts down the connection
	do
	{
		iResult = HandleClient(ClientSocket);
	} while (iResult > 0);

	if(isDebug)printf("send recieve FINISH\n");
	// ---------------------------------------------------------------------------------- CleanUp

	// No longer need server socket
	closesocket(ListenSocket);

	WSACleanup();
	if(isDebug)printf("Clean UP OK\n");
}
//--------------------------------------------------------------------------------------------------------------
void SetUpSocketBuffer(SOCKET ClientSocket,int RcvBuffSize,int SndBuffSize)
{
	bool isDebug = false;
	int iOptLen = sizeof(int);
	setsockopt(ClientSocket, SOL_SOCKET, SO_RCVBUF, (char*)&RcvBuffSize, iOptLen);
	if (getsockopt(ClientSocket, SOL_SOCKET, SO_RCVBUF, (char*)&RcvBuffSize, &iOptLen) == SOCKET_ERROR)
	{
		printf("Couldn't Set Rec Buffer to %d MBytes\n",RcvBuffSize/(1024*1024));
	}
	else if(isDebug)
	{
		printf("Set Rec Buffer to %d MBytes\n",RcvBuffSize/(1024*1024));
	}
	setsockopt(ClientSocket, SOL_SOCKET, SO_SNDBUF, (char*)&SndBuffSize, iOptLen);
	if (getsockopt(ClientSocket, SOL_SOCKET, SO_SNDBUF, (char*)&SndBuffSize, &iOptLen) == SOCKET_ERROR)
	{
		printf("couldn't Set Send Buffer to %d MBytes\n",SndBuffSize/(1024*1024));
	}
	else if(isDebug)
	{
		printf("Set Send Buffer to %d MBytes\n",SndBuffSize/(1024*1024));
	}
}
//--------------------------------------------------------------------------------------------------------------
void BufferServer::Run()
{
	bool isDebug = true;
	int MB16 = 16*1024*1024;

	//------------------------------------------------------------------- First Block until a new Client is accepted
	SOCKET ClientSocket = INVALID_SOCKET;
	ClientSocket = accept(ListenSocket, NULL, NULL);
	//SocketCheck(ListenSocket,INVALID_SOCKET,"accept failed");
	SetUpSocketBuffer(ClientSocket,MB16,MB16);
	Clients.push_back(ClientSocket);

	if(isDebug)printf("Starting the Server with the first Client\n");

	timeval TV;
	TV.tv_sec = 0;
	TV.tv_usec = 1;
	do
	{
		// ---------------------------------------------------------------------------------- Accepting New Connections
		fd_set *pListen_FdSet = GetListenSet();

		int isNewCalling = select(0,pListen_FdSet,NULL,NULL,&TV);
		if(isNewCalling == 1)
		{
			//ClientSocket = INVALID_SOCKET;
			ClientSocket = accept(ListenSocket, NULL, NULL);
			SetUpSocketBuffer(ClientSocket,MB16,MB16);
			Clients.push_back(ClientSocket);
		}
		//else if(isNewCalling == 0)//the usual case, no new connections
		else if(isNewCalling == SOCKET_ERROR)
		{
			printf("Listen select Fail\n");
			Clients.clear();
		}
		// ---------------------------------------------------------------------------------- Hanld Clients Requests
		fd_set *pRead_FdSet = GetClientsSet();

		int NewRequests = select(0,pRead_FdSet,NULL,NULL,&TV);//------------------------------ Check if any is sending a request
		if(NewRequests>0)
		{
			std::list<SOCKET>::iterator it = Clients.begin();
			std::list<SOCKET>::iterator itt;
			while(it!=Clients.end())
			{
				itt = it;
				it++;
				SOCKET ClientSocket = (*itt);
				if(FD_ISSET(ClientSocket, pRead_FdSet))
				{
					int iResult = HandleClient(ClientSocket);//------------------ Handle the Request
					if(iResult==0)
					{
						Clients.erase(itt);
					}
				}
			}
		}
		else if(isNewCalling == SOCKET_ERROR)
		{
			Clients.clear();
			printf("Request select Fail\n");
		}

	}while(!Clients.empty());

	if(isDebug)printf("Run : send recieve FINISH\n");
	

}
//--------------------------------------------------------------------------------------------------------------
// Shut Down Sending
// CleanUp
BufferServer::~BufferServer()
{

	// No longer need server socket
	closesocket(ListenSocket);
	WSACleanup();
	//if(isDebug)printf("Clean UP OK\n");
}
//--------------------------------------------------------------------------------------------------------------
