#pragma once
#include <chrono>

class Timer
{
public:
	void Start()
	{
		m_StartedTime = std::chrono::steady_clock::now();
	}

	int64 GetElapsedNanoSeconds()
	{
		auto curr = std::chrono::steady_clock::now();
		return std::chrono::duration_cast<std::chrono::nanoseconds>(curr - m_StartedTime).count();
	}

private:
	std::chrono::time_point<std::chrono::steady_clock> m_StartedTime;
};

class FixedTimeStepTime
{
public:
	void SetFixedDeltaTime(float seconds)
	{
		m_RequiredElapsedNanoSecs = SecondsToNanoSeconds(seconds);
	}

	void Start()
	{
		m_Timer.Start();
	}

	void WaitForNextUpdateTime()
	{
		while (m_Timer.GetElapsedNanoSeconds() < m_RequiredElapsedNanoSecs)
			;

		m_Timer.Start();
	}

private:
	int64 SecondsToNanoSeconds(float seconds)
	{
		return static_cast<int64>(static_cast<double>(seconds) * 1.0e+9);
	}

	int64 m_RequiredElapsedNanoSecs;
	Timer m_Timer;
};
