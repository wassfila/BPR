/*
	NetStream.h - File started on March 15th 2012 by wassfila
	Simple mapping for all BPR channels through the network
*/


#pragma once
#ifndef __NETSTREAM__
#define __NETSTREAM__
//#pragma message("------------------------------------NetStream.h------------------------------------")

#include <winsock2.h>

#define WIN32_LEAN_AND_MEAN
#define _WINSOCKAPI_
#include <windows.h>    // for Win32 APIs and types

#include <ws2tcpip.h>
//#include <wspiapi.h>    // for IPv6 support
//#include <strsafe.h>    // for safe versions of string functions
#include <stdio.h>
#include <time.h>

#include <iostream>
#include <string>
#include <map>
#include <list>

#ifndef CONFIGMAPS
#define CONFIGMAPS
typedef std::map<std::string,std::string> stringmap;
typedef std::list<std::string> stringlist;
typedef std::map<std::string,stringlist> stringlistmap;//a map of lists, lists contains strings
#endif

#define CHANNEL_ID_MAX_LENGTH 15

namespace sck
{

class Client
{
private:
	SOCKET ConnectSocket;
	int NbEmptyAnswers;
public:
	std::string ChannelId;
public:
	Client();//Flexible Dirty way
	Client(stringmap &config);//RAII
	~Client();
	int Init(stringmap &config);
	//----------------------------------------------------------------
	//>>"Poll:ChannelId"
	//<<"ChannelId:BlobSize:BlobData"
	void Poll(std::string &Data_Pull,std::string vChannelId = "");
	//----------------------------------------------------------------
	//>>"Push:ChannelId:BlobSize:BlobData"
	void Push(const std::string &Data_Push,std::string vChannelId = "");
	//----------------------------------------------------------------
};

class BufferServer
{
private:
	WSADATA wsaData;
	SOCKET ListenSocket;
	std::list<SOCKET>	Clients;
	stringlistmap		Buffers;
	fd_set Listen_FdSet;
	fd_set Read_FdSet;
	int MaxBlobsPerChannel;
	int MaxSizePerChannel;
	int MaxSizeTotal;
private:
	void RequestPoll(SOCKET ClientSocket,std::string &recvBuff);
	int RequestPush(SOCKET ClientSocket,std::string &recvBuff,int iResult);
public:
	// Initialize Winsock
	// Server Address
	// Create a Socket
	// Setup listening
	BufferServer();//Constructor
	BufferServer(stringmap &config);//Constructor and Initialisation
	void Init(stringmap &config);
	//----------------------------------------------------------------
	// listening
	// Accepting Connections 
	// Send Recieve
	int HandleClient(SOCKET ClientSocket);
	void RunOneClient();
	void Run();
	//----------------------------------------------------------------
	// Shut Down Sending
	// CleanUp
	~BufferServer();
	//----------------------------------------utilities
	fd_set *GetListenSet();
	fd_set *GetClientsSet();
};

#ifdef COMPARE_FUNCTIONS
//-----------------------------------------------------------------------------------------------------
//------------------------------------------	MySocket_Start	---------------------------------------
//-----------------------------------------------------------------------------------------------------
SOCKET MySocket_Start()
{
	WSADATA wsaData;
	int iResult;
	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
	if (iResult != 0) {
		printf("WSAStartup failed: %d\n", iResult);
		return 0;//My Error is 0 - No Handle
	}
	printf("WSAStartup OK\n");
	struct addrinfo *result = NULL,
					*ptr = NULL,
					hints;

	ZeroMemory( &hints, sizeof(hints) );
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	// Resolve the server address and port
	//PCSTR IPADD = "127.0.0.0";
	//PCSTR IPADD = "172.17.69.250";Home
	//PCSTR IPADD = "140.93.5.252";//OZ
	//PCSTR IPADD = "140.93.67.51";//Wassat
	PCSTR IPADD = "137.57.123.58";//StratixIII

	iResult = getaddrinfo(IPADD, DEFAULT_PORT, &hints, &result);
	if (iResult != 0) {
		printf("getaddrinfo failed: %d\n", iResult);
		WSACleanup();
		return 0;//My Error is 0 - No Handle
	}
	// -------------------------------------------------------------------------------------- Socket Creation
	SOCKET ConnectSocket = INVALID_SOCKET;
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

	//On Success
	return ConnectSocket;

}

//-----------------------------------------------------------------------------------------------------
//-----------------------------------------	MySocket_Stop	-------------------------------------------
//-----------------------------------------------------------------------------------------------------
int MySocket_Stop(SOCKET TheSocket)
{int iResult;
	//---------------------------------------------------------------------------------------- Close UP
	// shutdown the connection for sending since no more data will be sent
	// the client can still use the ConnectSocket for receiving data
	iResult = shutdown(TheSocket, SD_SEND);
	if (iResult == SOCKET_ERROR) {
		printf("shutdown failed: %d\n", WSAGetLastError());
		closesocket(TheSocket);
		WSACleanup();
		return 1;
	}

	// cleanup
	closesocket(TheSocket);
	WSACleanup();
	return 0;
}
//-----------------------------------------------------------------------------------------------------
//---------------------------------------	MySocket_Download	---------------------------------------
//-----------------------------------------------------------------------------------------------------
int MySocket_Download(SOCKET TheSocket,char* Host_Dest,char* Target_Source,int Size)
{short int PacketSize=1250,CurrentPackeSize;
int iResult;
int Target_Source_int = (int)Target_Source;

while(Size>0)
{
	if(Size>PacketSize)
	{
		CurrentPackeSize = PacketSize;
	}
	else
	{
		CurrentPackeSize = Size;
	}
	// ----------------------------------------------------------------------	Send 
	char CMDsendbuf[30];
		sprintf(CMDsendbuf,"Read");
		CMDsendbuf[4] = (Target_Source_int>>24)	&0xFF;	//MSB First
		CMDsendbuf[5] = (Target_Source_int>>16)	&0xFF;
		CMDsendbuf[6] = (Target_Source_int>>8)	&0xFF;
		CMDsendbuf[7] = (Target_Source_int)		&0xFF;
		CMDsendbuf[8] = (CurrentPackeSize>>8)	&0xFF;	//MSB First
		CMDsendbuf[9] = (CurrentPackeSize)		&0xFF;
		iResult = send(TheSocket, CMDsendbuf, 10, 0);//------------------------	Sending
		if (iResult == SOCKET_ERROR)
		{
			printf("send failed: %d\n", WSAGetLastError());
			closesocket(TheSocket);
			WSACleanup();
			return 1;
		}
		//If Everything is Right we wait for Recieve
		//printf("CMD Sent...wait recv...\n");
	    iResult = recv(TheSocket, Host_Dest, CurrentPackeSize, 0);
		if (iResult == SOCKET_ERROR)
		{
			printf("recv failed: %d\n", WSAGetLastError());
			closesocket(TheSocket);
			WSACleanup();
			return 1;
		}
		if(iResult==CurrentPackeSize)//Fine
		{
			//printf("%OK - d bytes recieved\n");
		}
		else
		{	//No Sync
			printf("ERR - NO Sync: CurrentPacketSize (%d), iResult (%d)\n",CurrentPackeSize,iResult);
		}
	Host_Dest+=iResult;
	Target_Source_int+=iResult;
	Size-=CurrentPackeSize;
}
	return 0;
}

#define DEFAULT_PORT "27035"
#define DEFAULT_BUFLEN 512

//--------------------------------- Server ---------------------------------
int ServerSetUp() {
WSADATA wsaData;

int iResult;

// ----------------------------------------------------------------------------------Initialize Winsock
iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
if (iResult != 0) {
    printf("WSAStartup failed: %d\n", iResult);
    return 1;
}
printf("Initialize Winsock OK\n");

// ---------------------------------------------------------------------------------- Server Address
struct addrinfo *result = NULL, *ptr = NULL, hints;

ZeroMemory(&hints, sizeof (hints));
hints.ai_family = AF_INET;
hints.ai_socktype = SOCK_STREAM;
hints.ai_protocol = IPPROTO_TCP;
hints.ai_flags = AI_PASSIVE;

// Resolve the local address and port to be used by the server
iResult = getaddrinfo(NULL, DEFAULT_PORT, &hints, &result);
if (iResult != 0) {
    printf("getaddrinfo failed: %d\n", iResult);
    WSACleanup();
    return 1;
}
printf("Server Address OK\n");


SOCKET ListenSocket = INVALID_SOCKET;

// ----------------------------------------------------------------------------------Create a Socket
// Create a SOCKET for the server to listen for client connections
ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
if (ListenSocket == INVALID_SOCKET) {
    printf("Error at socket(): %ld\n", WSAGetLastError());
    freeaddrinfo(result);
    WSACleanup();
    return 1;
}
printf("Create a Socket OK\n");

// ----------------------------------------------------------------------------------Setup listening
// Setup the TCP listening socket
iResult = bind( ListenSocket, 
    result->ai_addr, (int)result->ai_addrlen);
if (iResult == SOCKET_ERROR) {
    printf("bind failed: %d\n", WSAGetLastError());
    freeaddrinfo(result);
    closesocket(ListenSocket);
    WSACleanup();
    return 1;
}
freeaddrinfo(result);
printf("Setup listening OK\n");

// ---------------------------------------------------------------------------------- listening
if ( listen( ListenSocket, SOMAXCONN ) == SOCKET_ERROR ) {
    printf( "Error at bind(): %ld\n", WSAGetLastError() );
    closesocket(ListenSocket);
    WSACleanup();
    return 1;
}
printf("listening OK\n");

// ---------------------------------------------------------------------------------- Accepting Connections 
SOCKET ClientSocket;
ClientSocket = INVALID_SOCKET;
// Accept a client socket
ClientSocket = accept(ListenSocket, NULL, NULL);
if (ClientSocket == INVALID_SOCKET) {
    printf("accept failed: %d\n", WSAGetLastError());
    closesocket(ListenSocket);
    WSACleanup();
    return 1;
}
printf("accept OK\n");

// No longer need server socket
closesocket(ListenSocket);

// ---------------------------------------------------------------------------------- Send Recieve

char recvbuf[DEFAULT_BUFLEN];
int iSendResult;
int recvbuflen = DEFAULT_BUFLEN;

// Receive until the peer shuts down the connection
do {

    iResult = recv(ClientSocket, recvbuf, recvbuflen, 0);
    if (iResult > 0) {
        printf("Bytes received: %d\n", iResult);

        // Echo the buffer back to the sender
        iSendResult = send(ClientSocket, recvbuf, iResult, 0);
        if (iSendResult == SOCKET_ERROR) {
            printf("send failed: %d\n", WSAGetLastError());
            closesocket(ClientSocket);
            WSACleanup();
            return 1;
        }
        printf("Bytes sent: %d\n", iSendResult);
    } else if (iResult == 0)
        printf("Connection closing...\n");
    else {
        printf("recv failed: %d\n", WSAGetLastError());
        closesocket(ClientSocket);
        WSACleanup();
        return 1;
    }

} while (iResult > 0);

printf("send recieve FINISH\n");

// -------------------------------------------------------------------------------------- Shut Down Sending
// shutdown the send half of the connection since no more data will be sent
iResult = shutdown(ClientSocket, SD_SEND);
if (iResult == SOCKET_ERROR) {
    printf("shutdown failed: %d\n", WSAGetLastError());
    closesocket(ClientSocket);
    WSACleanup();
    return 1;
}
printf("Shutdown Send OK\n");

// ---------------------------------------------------------------------------------- CleanUp
// cleanup
closesocket(ClientSocket);
WSACleanup();
printf("Clean UP OK\n");

int i;
scanf_s("%d",&i);

return 0;
}

#endif


}

#endif /*__NETSTREAM__*/
