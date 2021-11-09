#include "UdpServer.h"


UdpServer::UdpServer()
{
	m_buffer = new char[MAXBUF];
	//ZeroMemory(m_buffer, MAXBUF);
}

UdpServer::~UdpServer(void)
{
	delete[] m_buffer;
}
void UdpServer::StartServer(int port)
{
	Server_BindToPort(port);
}


bool UdpServer::ProcessIncoming(float outRobotJoints[6])
{
	bool ret = false;
	int numbytes = ReceivePacket(m_buffer, MAXBUF);
	if (numbytes > 0)
	{
		m_clientConnected = true;
		m_connectionAlive = true;

		BYTE command = m_buffer[0];
		if (command == 1 && numbytes == 1 + 24)
		{
			m_receivedGoodCount++;
			memcpy(outRobotJoints, m_buffer + 1, 24);
			ret = true;

			//m_watchDog = Clock::GetTimeTotal();
		}
		else
		{ // chybná velikost paketu
			m_receivedBadCount++;
		}
	}
	//CheckWatchdog();

	return ret;
}

void UdpServer::ProcessOutgoing(const std::vector<Vec3>& dataToSend)
{
	static DWORD lastTime = 0;

	if (!m_clientConnected)
		return;

	if (true)
	{//Clock::GetTimeTotal() - lastTime > SEND_PERIOD) {
		int size = 1;
		memcpy(m_buffer, &m_cameraPacketNumber, 1);
		if (dataToSend.size() > 0)
		{
			memcpy(m_buffer + 1, dataToSend.data(), dataToSend.size() * sizeof(dataToSend[0]));
			size += (int)dataToSend.size() * sizeof(Vec3);
		}
		if (SendPacket(m_buffer, size))
		{
			//lastTime = Clock::GetTimeTotal();
			m_cameraPacketNumber++;
			m_sentCount++;
		}
	}
}

void UdpServer::ProcessOutgoing(const std::unordered_set<uint32_t>& dataToSend, float voxelSize)
{
	static DWORD lastTime = 0;

	if (!m_clientConnected)
		return;

	if (true)
	{//Clock::GetTimeTotal() - lastTime > SEND_PERIOD) {
		int size = 1;
		memcpy(m_buffer, &m_cameraPacketNumber, 1);
		memcpy(m_buffer + 1, &voxelSize, sizeof(float));
		size += 4;
		char* buff = m_buffer +5;
		for (auto& voxel : dataToSend)
		{
			memcpy(buff, &voxel, sizeof(uint32_t));
			buff += sizeof(uint32_t);
		}
		size += (int)dataToSend.size() * sizeof(uint32_t);
		if (SendPacket(m_buffer, size))
		{
			//lastTime = Clock::GetTimeTotal();
			m_cameraPacketNumber++;
			m_sentCount++;
		}
	}
}


void UdpServer::ProcessOutgoing(const std::unordered_set<uint32_t>& dataToSend, float voxelSize,const Vec3& bounding_box_min, const Vec3& bounding_box_max)
{
	//static DWORD lastTime = 0;

	if (!m_clientConnected)
		return;

	if (true)
	{//Clock::GetTimeTotal() - lastTime > SEND_PERIOD) {
		int size = 1;
		memcpy(m_buffer, &m_cameraPacketNumber, 1);
		memcpy(m_buffer + 1, &voxelSize, sizeof(float));
		size += 4;
		char* buff = m_buffer + 5;

		memcpy(buff, &bounding_box_min, sizeof(Vec3));
		buff += sizeof(Vec3);
		memcpy(buff, &bounding_box_max, sizeof(Vec3));
		buff += sizeof(Vec3);

		for (auto& voxel : dataToSend)
		{
			memcpy(buff, &voxel, sizeof(uint32_t));
			buff += sizeof(uint32_t);
		}
		size += (int)dataToSend.size() * sizeof(uint32_t);
		if (SendPacket(m_buffer, size))
		{
			//lastTime = Clock::GetTimeTotal();
			m_cameraPacketNumber++;
			m_sentCount++;
		}
	}
}

void UdpServer::CheckWatchdog()
{
	if (m_clientConnected)
	{
		if (true) //Clock::GetTimeTotal() - m_watchDog > WATCHDOG_CONNECTED
		{
			m_clientConnected = false;
			//LOG("Communication - watchdog 'connection' timed out!");
		}
		else if (true) //Clock::GetTimeTotal() - m_watchDog > WATCHDOG_ALIVE
		{
			m_connectionAlive = false;
			//LOG("Communication - watchdog 'alive' timed out!");
		}
	}
}

