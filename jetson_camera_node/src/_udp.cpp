#include "_udp.h"
//#include <ws2tcpip.h>
#include <stdio.h>
#include <stdlib.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <memory.h>
#include <fcntl.h>
#include <math.h>


//#pragma comment(lib, "Ws2_32.lib")

_udp::_udp(void)
{
	/*WSADATA wsaData;
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
		//ROBO_ERROR("WinSock initialization error!");
	}*/
	GetLocalInfo();
	InitSocket();
}

_udp::~_udp(void)
{
	delete m_toAddr;
	m_toAddr = nullptr;
	close(m_socket);
	//WSACleanup();
}


void _udp::GetLocalInfo(void)
{
	/*// zjištìní názvu lokálního poèítaèe:
	gethostname(m_localName, 100);

	// zjištìní IP lokálního poèítaèe:
	struct addrinfo hints;
	struct addrinfo *servinfo;//, *p;

	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_DGRAM;
	if (getaddrinfo(m_localName, NULL, &hints, &servinfo) != 0) {}
        //ROBO_ERROR("Error getaddrinfo!");

	//char *iptext;
	sockaddr_in *ipv4 = (sockaddr_in *)servinfo->ai_addr;
	//iptext = inet_ntoa(ipv4->sin_addr);
	inet_ntop(AF_INET, &ipv4->sin_addr, m_localIP, 16);
	//strcpy_s(m_localIP, 16, iptext);
	freeaddrinfo(servinfo);*/
}

bool _udp::InitSocket(void)
{
	if ((m_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
		//ROBO_ERROR("Socket creating failed!");
		return false;
	}

	u_long nonBlocking = 1;
	//if (ioctlsocket(m_socket, FIONBIO, &nonBlocking) == -1 ) {
	fcntl(m_socket, F_SETFL, O_NONBLOCK);
	

	return true;
}

bool _udp::Server_BindToPort(int port)
{
	m_server = true;
	if (m_toAddr)
		delete m_toAddr;
	m_toAddr = new sockaddr_in;

	sockaddr_in sockName;
	sockName.sin_family = AF_INET;
    sockName.sin_port = htons(port);
	sockName.sin_addr.s_addr = INADDR_ANY;
	if (bind(m_socket, (sockaddr *)&sockName, sizeof(sockName)) == -1) {
		//ROBO_ERROR("Socket bind error!");
		return false;
	}
	return true;
}
char* itoa(int num, char* buffer, int base) {
    int curr = 0;
 
    if (num == 0) {
        // Base case
        buffer[curr++] = '0';
        buffer[curr] = '\0';
        return buffer;
    }
 
    int num_digits = 0;
 
    if (num < 0) {
        if (base == 10) {
            num_digits ++;
            buffer[curr] = '-';
            curr ++;
            // Make it positive and finally add the minus sign
            num *= -1;
        }
        else
            // Unsupported base. Return NULL
            return NULL;
    }
 
    num_digits += (int)floor(log(num) / log(base)) + 1;
 
    // Go through the digits one by one
    // from left to right
    while (curr < num_digits) {
        // Get the base value. For example, 10^2 = 1000, for the third digit
        int base_val = (int) pow(base, num_digits-1-curr);
 
        // Get the numerical value
        int num_val = num / base_val;
 
        char value = num_val + '0';
        buffer[curr] = value;
 
        curr ++;
        num -= base_val * num_val;
    }
    buffer[curr] = '\0';
    return buffer;
}
bool _udp::Client_SetServerAddress(const char * addr, int port)
{
	m_server = false;
	addrinfo hints;
	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_DGRAM;

	addrinfo * servinfo;

	char buffport[6];
	itoa(port, buffport, 10);

	if (getaddrinfo(addr, buffport, &hints, &servinfo) != 0) {
		freeaddrinfo(servinfo);
        return false;
	}
	if (m_toAddr)
		delete m_toAddr;
	m_toAddr = new sockaddr_in;
	*((sockaddr*)m_toAddr) = *servinfo->ai_addr;
	//m_toAddrLen = servinfo->ai_addrlen;

	freeaddrinfo(servinfo);
	return true;
}

bool _udp::SendPacket(const char * buf, int len)
{
	int numbytes = sendto(m_socket, buf, len, 0, (sockaddr*)m_toAddr, sizeof(sockaddr_in));
	return (len == numbytes ? true : false);
}

int _udp::ReceivePacket(char * buf, int len)
{
	int numbytes;

	if (m_server) {  // U klienta nás nezajímá odkud paket pøišel (samozøejmì ze serveru na který jsme se pøipojili). (Jinak tady ale to odlišení teoreticky nemusí být.)
		socklen_t addrLen;
		sockaddr_in theirAddr;
		memset(&theirAddr, 0,sizeof(theirAddr));
		addrLen = sizeof(theirAddr);

		numbytes = recvfrom(m_socket, buf, len, 0, (sockaddr *)&theirAddr, &addrLen);

		if (numbytes > 0) {
			//char *iptext = inet_ntoa(theirAddr.sin_addr); // IP odesílatele (jen pro vlastní potøebu)
			//strcpy_s(m_remoteIP, 16, iptext);
			inet_ntop(AF_INET, &theirAddr.sin_addr, m_remoteIP, 16);

			memcpy(m_toAddr, &theirAddr, sizeof(theirAddr));
			//m_toAddrLen = sizeof(m_toAddr);
		}
	}
	else
		numbytes = recvfrom(m_socket, buf, len, 0, NULL, NULL);

	return (numbytes < 0 ? 0 : numbytes);
}
