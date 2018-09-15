#include "DynamicGridSquare.h"

void DynamicGridSquare::AddAndDetect(Rigidbody& toAdd)
{
	for (auto it = m_Contents.begin(); it != m_Contents.end(); it++)
	{
		// detect
	}
	m_Contents.emplace_back(&toAdd);
}