#include "pch.h"
#include "DynamicGridPartition.h"
#include "Rigidbody.h"
#include "MathUtils.h"

void DynamicGridPartition::CalculateGridDimensions(const std::vector<Rigidbody*>& bodies)
{
	auto origin = Vector3(MathUtils::Infinity, MathUtils::Infinity, MathUtils::Infinity);

	auto xSizeAverage = 0.0f;
	auto ySizeAverage = 0.0f;
	auto zSizeAverage = 0.0f;

	auto xMax = MathUtils::NegativeInfinity;
	auto yMax = MathUtils::NegativeInfinity;
	auto zMax = MathUtils::NegativeInfinity;

	for (auto it = bodies.begin(); it != bodies.end(); it++)
	{
		auto& body = **it;
		auto& bounds = body.GetWorldBounds();

		body.UpdateWorldBounds();

		if (bounds.GetXMin() < origin.x)
			origin.x = bounds.GetXMin();

		if (bounds.GetYMin() < origin.y)
			origin.y = bounds.GetYMin();

		if (bounds.GetZMin() < origin.z)
			origin.z = bounds.GetZMin();

		if (bounds.GetXMax() > xMax)
			xMax = bounds.GetXMax();

		if (bounds.GetYMax() > yMax)
			yMax = bounds.GetYMax();

		if (bounds.GetZMax() > zMax)
			zMax = bounds.GetZMax();

		xSizeAverage += bounds.GetXRange();
		ySizeAverage += bounds.GetYRange();
		zSizeAverage += bounds.GetZRange();
	}

	auto xRange = xMax - origin.x;
	auto yRange = yMax - origin.y;
	auto zRange = zMax - origin.z;

	auto xSizeMin = ceilf(xRange / static_cast<float>(XGridNumSquares));
	auto ySizeMin = ceilf(yRange / static_cast<float>(YGridNumSquares));
	auto zSizeMin = ceilf(zRange / static_cast<float>(ZGridNumSquares));

	auto c = static_cast<float>(bodies.size());
	xSizeAverage = MathUtils::Min(xSizeAverage / c, xSizeMin);
	ySizeAverage = MathUtils::Min(ySizeAverage / c, ySizeMin);
	zSizeAverage = MathUtils::Min(zSizeAverage / c, zSizeMin);

	m_Grid.SetSqaureDimensions(xSizeAverage, ySizeAverage, zSizeAverage, origin);
}

void DynamicGridPartition::HandleCollisions(const std::vector<Rigidbody*>& bodies)
{
	CalculateGridDimensions(bodies);

	m_GridSquares->Reset();
	m_Responder.Reset();

	for (auto it = bodies.begin(); it != bodies.end(); it++)
		HandleBody(**it);
}

void DynamicGridPartition::HandleBody(Rigidbody& body)
{
	body.SetLastCheckedAgainst(nullptr);

	m_Grid.GetRegionCovered(body.GetWorldBounds(), m_Region);

	for (auto i = m_Region.XStart; i < m_Region.XEnd; i++)
	{
		for (auto j = m_Region.YStart; j < m_Region.YEnd; j++)
		{
			for (auto k = m_Region.ZStart; k < m_Region.ZEnd; k++)
			{
				auto& square = m_Grid.At(i, j, k);

				if (!square->WasPlacedOnGridThisTick(m_GridSquares->NumRecycled()))
				{
					square = m_GridSquares->Recycle().get();
					square->Reset();
				}

				square->AddAndDetect(body);
			}
		}
	}
}