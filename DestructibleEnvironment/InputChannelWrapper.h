#pragma once
#include "InputChannel.h"

class InputChannelWrapper
{
public:
	InputChannelWrapper()
	{
		m_Channel = nullptr;
	}

	InputChannelWrapper(const InputChannel& channel)
	{
		m_Channel = &channel;
	}

	void Update()
	{
		auto currDown = IsDown();

		m_JustDown = currDown && !m_WasDown;
		m_JustUp = !currDown && m_WasDown;

		m_WasDown = currDown;
	}

	bool IsDown() const
	{
		return m_Channel->IsDown();
	}

	bool IsJustUp() const
	{
		return m_JustUp;
	}

	bool IsJustDown() const
	{
		return m_JustUp;
	}

private:
	const InputChannel* m_Channel;

	bool m_JustDown = false;
	bool m_JustUp = false;

	bool m_WasDown = false;
};