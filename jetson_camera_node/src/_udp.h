#pragma once

class _udp
{
	private:
		/*#if defined(_WIN64)
			typedef unsigned __int64 UINT_PTR, *PUINT_PTR;
		#else
			typedef _W64 unsigned int UINT_PTR, *PUINT_PTR;
		#endif
		typedef UINT_PTR SOCKET;*/

	protected:
		char m_localIP[16];
		char m_localName[100];
		char m_remoteIP[16];

	private:
		bool    m_server = false;     // nastav� se na true kdy� bude zavolan� funkce Server_BindToPort
		int  m_socket;
		void*   m_toAddr = nullptr; // norm�ln� je to typ sockaddr_in, ale ud�l�me m�sto toho void* abychom nemuseli v h. souboru m�t sockaddr_in definovan� pomoc� #include
		//int     m_toAddrLen;

	protected:
		_udp(void);
		~_udp(void);
	
	private:
		void GetLocalInfo(void);
		bool InitSocket(void);

	protected:
		bool Server_BindToPort(int port);                            // mus� se zavolat na serveru
		bool Client_SetServerAddress(const char * addr, int port);   // mus� se zavolat na klientovi

		bool IsServerBound() const { return m_server; };

		bool SendPacket(const char * buf, int len);
		int ReceivePacket(char * buf, int len);
};
