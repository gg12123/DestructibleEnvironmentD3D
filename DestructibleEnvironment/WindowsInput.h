#pragma once
#include <array>
#include <vector>
#include "InputChannel.h"

ref class WindowsInput
{
public:
	void Init(_In_ Windows::UI::Core::CoreWindow^ window)
	{
		window->PointerPressed +=
			ref new Windows::Foundation::TypedEventHandler<Windows::UI::Core::CoreWindow^, Windows::UI::Core::PointerEventArgs^>(this, &WindowsInput::OnPointerPressed);

		window->PointerReleased +=
			ref new Windows::Foundation::TypedEventHandler<Windows::UI::Core::CoreWindow^, Windows::UI::Core::PointerEventArgs^>(this, &WindowsInput::OnPointerReleased);
	}

	void Reset()
	{
		for (auto i : m_NeedReseting)
			i->Reset();
	}

	const InputChannel& GetMouseInputChannel(int mouseButton) const
	{
		return m_MouseInputs[mouseButton];
	}

	bool GetMouseButton(int btn) const
	{
		return m_MouseInputs[btn].IsDown();
	}

	bool GetMouseButtonUp(int btn) const
	{
		return m_MouseInputs[btn].IsJustUp();
	}

	bool GetMouseButtonDown(int btn) const
	{
		return m_MouseInputs[btn].IsJustDown();
	}

private:
	void OnPointerPressed(_In_ Windows::UI::Core::CoreWindow^ /* sender */,
		_In_ Windows::UI::Core::PointerEventArgs^ args)
	{
		OnMouseInputDown(args->CurrentPoint->PointerId);
	}

	void OnPointerReleased(_In_ Windows::UI::Core::CoreWindow^ /* sender */,
		_In_ Windows::UI::Core::PointerEventArgs^ args)
	{
		OnMouseInputUp(args->CurrentPoint->PointerId);
	}

	void OnMouseInputDown(int mouseButton)
	{
		auto btn = &m_MouseInputs[mouseButton];
		btn->OnDown();
		m_NeedReseting.emplace_back(btn);
	}

	void OnMouseInputUp(int mouseButton)
	{
		auto btn = &m_MouseInputs[mouseButton];
		btn->OnUp();
		m_NeedReseting.emplace_back(btn);
	}

	std::array<InputChannel, 2> m_MouseInputs;
	std::vector<InputChannel*> m_NeedReseting;
};
