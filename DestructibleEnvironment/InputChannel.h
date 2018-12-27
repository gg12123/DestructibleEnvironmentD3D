#pragma once

class InputChannel
{
public:
	void OnUp()
	{
		m_IsDown = false;
		m_JustUp = true;
	}

	void OnDown()
	{
		m_IsDown = true;
		m_JustDown = true;
	}

	bool IsDown() const
	{
		return m_IsDown;
	}

	bool IsJustDown() const
	{
		return m_JustDown;
	}

	bool IsJustUp() const
	{
		return m_JustUp;
	}

	void Reset()
	{
		m_JustDown = m_JustUp = false;
	}

private:
	bool m_IsDown = false;

	bool m_JustUp = false;
	bool m_JustDown = false;
};
