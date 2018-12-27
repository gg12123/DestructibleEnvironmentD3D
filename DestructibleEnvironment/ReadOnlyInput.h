#pragma once
#include "WindowsInput.h"

class ReadOnlyInput
{
public:
	ReadOnlyInput()
	{
		m_Input = nullptr;
	}

	ReadOnlyInput(const WindowsInput ^ input)
	{
		m_Input = input;
	}

	bool GetMouseButton(int btn) const
	{
		return m_Input->GetMouseButton(btn);
	}

	bool GetMouseButtonDown(int btn) const
	{
		return m_Input->GetMouseButtonDown(btn);
	}

	bool GetMouseButtonUp(int btn) const
	{
		return m_Input->GetMouseButtonUp(btn);
	}

	auto& GetMouseChannel(int btn) const
	{
		return m_Input->GetMouseInputChannel(btn);
	}

private:
	const WindowsInput ^ m_Input;
};