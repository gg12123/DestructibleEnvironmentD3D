#pragma once
#include <vector>
#include <memory>
#include "GridPartition.h"
#include "PoolOfRecyclables.h"
#include "DynamicGridSquare.h"
#include "CollisionDetector.h"
#include "CollisionResponder.h"
#include "Rigidbody.h"
#include "BodyCollectionBounds.h"

class DynamicGridPartition
{
public:
	static constexpr uint32 XGridNumSquares = 100U;
	static constexpr uint32 YGridNumSquares = 100U;
	static constexpr uint32 ZGridNumSquares = 100U;

	DynamicGridPartition()
	{
		m_NullSquare = std::unique_ptr<DynamicGridSquare>(new DynamicGridSquare(MathU::IntMax, m_Detector, m_Responder));

		for (auto i = 0U; i < XGridNumSquares; i++)
			for (auto j = 0U; j < YGridNumSquares; j++)
				for (auto k = 0U; k < ZGridNumSquares; k++)
					m_Grid.At(i, j, k) = m_NullSquare.get();

		m_NextSquareIndex = 0U;

		auto squareCreator = [this]()
		{
			m_NextSquareIndex++;
			return std::unique_ptr<DynamicGridSquare>(new DynamicGridSquare(m_NextSquareIndex, m_Detector, m_Responder));
		};

		m_GridSquares = std::unique_ptr<PoolOfRecyclables<std::unique_ptr<DynamicGridSquare>>>(
			new PoolOfRecyclables<std::unique_ptr<DynamicGridSquare>>(XGridNumSquares * YGridNumSquares, squareCreator));
	}

	void HandleCollisions(const std::vector<std::unique_ptr<Rigidbody>>& bodies);

private:
	void CalculateGridDimensions(const std::vector<std::unique_ptr<Rigidbody>>& bodies);
	void HandleBody(Rigidbody& body);

	GridPartition<DynamicGridSquare*, XGridNumSquares, YGridNumSquares, ZGridNumSquares> m_Grid;
	std::unique_ptr<PoolOfRecyclables<std::unique_ptr<DynamicGridSquare>>> m_GridSquares;
	std::unique_ptr<DynamicGridSquare> m_NullSquare;

	GridRegion m_Region;

	CollisionDetector m_Detector;
	CollisionResponder m_Responder;

	BodyCollectionBounds m_DynamicBodiesBounds;

	uint32 m_NextSquareIndex;
};