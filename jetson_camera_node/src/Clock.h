#pragma once
//#include <Windows.h>
#include <chrono>



class Clock
{

public:
	static void init()
	{
		start = std::chrono::high_resolution_clock::now();
		local_start = std::chrono::high_resolution_clock::now();
		//LARGE_INTEGER m_freq_proc;
		//QueryPerformanceFrequency(&m_freq_proc);
		//m_sec_per_tick = 1.f / m_freq_proc.QuadPart;
		//QueryPerformanceCounter(&m_ticks);
	};

	static float getDeltaSec() 
	{ 
		
		return m_delta_time;
	};

	static float getRunTime() { return 0/*m_ticks.QuadPart * m_sec_per_tick*/; };

	static void update()
	{
		end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<float> res = end-start;
		start = end;
		m_delta_time = (float)res.count();
		/*LARGE_INTEGER tick;
		QueryPerformanceCounter(&tick);
		__int64 dtics = tick.QuadPart - m_ticks.QuadPart;
		m_delta_time = (float)dtics * m_sec_per_tick;
		m_ticks = tick;*/
	};

	static float GetLocalTimeStep()
	{
 		local_end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<float> res = local_end-local_start;
		local_start = local_end;
		return (float)res.count();
	}
private:
	inline static std::chrono::high_resolution_clock::time_point start;
	inline static std::chrono::high_resolution_clock::time_point end;

	inline static std::chrono::high_resolution_clock::time_point local_start;
	inline static std::chrono::high_resolution_clock::time_point local_end;
	inline static float m_delta_time;
	//inline static float m_sec_per_tick;
	//inline static float m_delta_time;
	//inline static LARGE_INTEGER m_ticks;


};

