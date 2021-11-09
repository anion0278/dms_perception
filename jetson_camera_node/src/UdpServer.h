#pragma once
#include "Math.h"
#include "_udp.h"
//#include <wtypes.h>
#include <vector>
#include <unordered_set>

//#define DWORD unsigned int
//#define BYTE unsigned

typedef unsigned char BYTE;
typedef unsigned int DWORD;

class UdpServer : _udp
{
	private:
		static const int WATCHDOG_ALIVE = 2000;
		static const int WATCHDOG_CONNECTED = 10000;
		static const int SEND_PERIOD = 50;

		static const int MAXBUF = 60000;
		char* m_buffer;

		unsigned int m_receivedGoodCount = 0;
		unsigned int m_receivedBadCount = 0;
		unsigned int m_sentCount = 0;
		bool         m_clientConnected = false;
		bool         m_connectionAlive = false;
		DWORD        m_watchDog = 0;
		BYTE         m_cameraPacketNumber = 0;

	public:
		UdpServer();
		~UdpServer();

		const char* GetLocalIP() const { return m_localIP; };
		const char* GetLocalName() const { return m_localName; };
		const char* GetRemoteIP() const { return m_remoteIP; };

		void StartServer(int port);
		bool ProcessIncoming(float outRobotJoints[6]);
		void ProcessOutgoing(const std::vector<Vec3>& dataToSend);
		void ProcessOutgoing(const std::unordered_set<uint32_t>& dataToSend, float voxelSize);
		void ProcessOutgoing(const std::unordered_set<uint32_t>& dataToSend, float voxelSize, const Vec3& bounding_box_min, const Vec3& bounding_box_max);

		bool IsInitialized() const { return IsServerBound(); };
		bool IsConnectionAlive() const { return m_connectionAlive; };
		bool IsClientConnected() const { return m_clientConnected; };

		unsigned int GetReceivedGoodCount() const { return m_receivedGoodCount; };
		unsigned int GetReceivedBadCount() const { return m_receivedBadCount; };
		unsigned int GetSentCount() const { return m_sentCount; };

	private:
		void CheckWatchdog();
};
