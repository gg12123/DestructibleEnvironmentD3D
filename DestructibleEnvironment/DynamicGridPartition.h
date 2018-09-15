#pragma once
#include <vector>
#include <memory>
#include "GridPartition.h"
#include "PoolOfRecyclables.h"
#include "DynamicGridSquare.h"

class Rigidbody;

class DynamicGridPartition
{
public:
	static constexpr uint32 XGridNumSquares = 100U;
	static constexpr uint32 YGridNumSquares = 100U;
	static constexpr uint32 ZGridNumSquares = 100U;

	DynamicGridPartition()
	{
		for (auto i = 0U; i < XGridNumSquares; i++)
			for (auto j = 0U; j < YGridNumSquares; j++)
				for (auto k = 0U; k < ZGridNumSquares; k++)
					m_Grid.At(i, j, k) = nullptr;
	}

	void HandleCollisions(const std::vector<Rigidbody*>& bodies);

private:
	void CalculateGridDimensions(const std::vector<Rigidbody*>& bodies);
	void HandleBody(Rigidbody& body);

	GridPartition<DynamicGridSquare*, XGridNumSquares, YGridNumSquares, ZGridNumSquares> m_Grid;
	std::unique_ptr<PoolOfRecyclables<std::unique_ptr<DynamicGridSquare>>> m_GridSquares;

	GridRegion m_Region;
};