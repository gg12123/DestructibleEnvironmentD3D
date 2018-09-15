#include "pch.h"
#include "DynamicGridSquare.h"
#include "CollisionDetector.h"
#include "CollisionResponder.h"
#include "Rigidbody.h"

void DynamicGridSquare::AddAndDetect(Rigidbody& toAdd)
{
	for (auto it = m_Contents.begin(); it != m_Contents.end(); it++)
	{
		auto& other = **it;

		if (other.GetLastCheckedAgainst() != &toAdd)
		{
			if (m_Detecter->FindCollision(toAdd, other, m_CollData))
				m_Responder->CalculateResponse(m_CollData, toAdd, other);

			other.SetLastCheckedAgainst(&toAdd);
		}
	}
	m_Contents.emplace_back(&toAdd);
}